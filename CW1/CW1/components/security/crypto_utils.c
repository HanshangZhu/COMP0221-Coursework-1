#include <string.h>
#include <stddef.h>
#include <time.h>
#include <sys/time.h>
#include "crypto_utils.h"
#include "config.h"
#include "esp_log.h"
#include "mbedtls/cmac.h"
#include "lwip/def.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CRYPTO";

// Team key from config.h
static const uint8_t TEAM_KEY[16] = TEAM_KEY_BYTES;

// ============================================================
// Replay Defense: Sequence number tracking per node
// ============================================================
#ifdef DEFENSE_REPLAY_CHECK

// Track seen sequence numbers for each neighbor
typedef struct {
    uint8_t node_id[6];
    uint16_t seen_seqs[SEQ_WINDOW_SIZE];
    uint8_t seq_count;
    uint32_t last_ts_s;  // Last seen timestamp (seconds)
} replay_tracker_t;

#define MAX_TRACKED_NODES 10
static replay_tracker_t replay_table[MAX_TRACKED_NODES];
static bool replay_table_initialized = false;

static void init_replay_table(void) {
    if (!replay_table_initialized) {
        memset(replay_table, 0, sizeof(replay_table));
        replay_table_initialized = true;
        ESP_LOGI(TAG, "Replay defense table initialized");
    }
}

/**
 * Check if this packet is a replay attack
 * Uses sequence number tracking (no timestamp check - works without NTP)
 * Returns true if packet is FRESH (accept), false if REPLAY (reject)
 */
bool check_replay(const lora_packet_t *packet) {
    init_replay_table();
    
    // Find or create tracker for this node
    int idx = -1;
    int empty_idx = -1;
    
    for (int i = 0; i < MAX_TRACKED_NODES; i++) {
        if (memcmp(replay_table[i].node_id, packet->node_id, 6) == 0 && 
            replay_table[i].seq_count > 0) {
            idx = i;
            break;
        }
        if (replay_table[i].seq_count == 0 && empty_idx == -1) {
            empty_idx = i;
        }
    }
    
    // New node - accept and track
    if (idx == -1) {
        if (empty_idx == -1) empty_idx = 0;  // Overwrite oldest if full
        idx = empty_idx;
        memcpy(replay_table[idx].node_id, packet->node_id, 6);
        replay_table[idx].seq_count = 0;
        replay_table[idx].last_ts_s = 0;
    }
    
    // Check: Sequence number already seen in window?
    for (int i = 0; i < replay_table[idx].seq_count; i++) {
        if (replay_table[idx].seen_seqs[i] == packet->seq_number) {
            ESP_LOGW(TAG, "REPLAY BLOCKED: Seq %d from node already seen!", packet->seq_number);
            return false;
        }
    }
    
    // Check: Sequence number going backwards significantly? (possible replay)
    if (replay_table[idx].seq_count > 0) {
        uint16_t last_seq = replay_table[idx].seen_seqs[replay_table[idx].seq_count - 1];
        int16_t diff = (int16_t)(packet->seq_number - last_seq);
        // Allow up to 100 packets forward, or wrapping around
        if (diff < -100 && diff > -65000) {
            ESP_LOGW(TAG, "REPLAY BLOCKED: Seq %d is old (last was %d)", 
                     packet->seq_number, last_seq);
            return false;
        }
    }
    
    // Accept: Add to tracking window
    if (replay_table[idx].seq_count < SEQ_WINDOW_SIZE) {
        replay_table[idx].seen_seqs[replay_table[idx].seq_count++] = packet->seq_number;
    } else {
        // Sliding window: shift out oldest
        memmove(&replay_table[idx].seen_seqs[0], &replay_table[idx].seen_seqs[1], 
                (SEQ_WINDOW_SIZE - 1) * sizeof(uint16_t));
        replay_table[idx].seen_seqs[SEQ_WINDOW_SIZE - 1] = packet->seq_number;
    }
    replay_table[idx].last_ts_s = packet->ts_s;
    
    return true;  // Fresh packet!
}
#endif // DEFENSE_REPLAY_CHECK

void crypto_init(void) {
    ESP_LOGI(TAG, "Crypto initialized with Team Key");
#ifdef DEFENSE_REPLAY_CHECK
    ESP_LOGI(TAG, "Replay defense ENABLED (window: %d ms)", REPLAY_WINDOW_MS);
#endif
}

/**
 * Calculate CMAC - Returns 4-byte truncated tag
 */
static void calculate_cmac(const lora_packet_t *packet, uint8_t *tag_out) {
    const mbedtls_cipher_info_t *cipher_info;
    cipher_info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
    
    size_t data_len = offsetof(lora_packet_t, mac_tag);
    uint8_t output[16];
    
    mbedtls_cipher_cmac(cipher_info, TEAM_KEY, 128, (const uint8_t*)packet, data_len, output);
    memcpy(tag_out, output + 12, 4);  // Last 4 bytes
}

/**
 * Sign a packet with CMAC
 */
void crypto_sign_packet(lora_packet_t *packet) {
    memset(packet->mac_tag, 0, 4);
    calculate_cmac(packet, packet->mac_tag);
}

/**
 * Verify packet signature AND check for replay attacks (if defense enabled)
 * Returns true if valid and fresh, false otherwise
 */
bool crypto_verify_packet(const lora_packet_t *packet) {
    // Step 1: Verify CMAC signature
    uint8_t received_tag[4];
    memcpy(received_tag, packet->mac_tag, 4);
    
    uint8_t calculated_tag[4];
    calculate_cmac(packet, calculated_tag);
    
    if (memcmp(received_tag, calculated_tag, 4) != 0) {
        ESP_LOGW(TAG, "CMAC FAILED! Recv: %02X%02X%02X%02X, Expected: %02X%02X%02X%02X",
                 received_tag[0], received_tag[1], received_tag[2], received_tag[3],
                 calculated_tag[0], calculated_tag[1], calculated_tag[2], calculated_tag[3]);
        return false;
    }
    
#ifdef DEFENSE_REPLAY_CHECK
    // Step 2: Check for replay attack
    if (!check_replay(packet)) {
        return false;  // Replay detected!
    }
#endif
    
    return true;  // Valid and fresh!
}