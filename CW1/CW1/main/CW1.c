/**
 * CW1 Main Application - COMP0221 Flocking Drone System
 * 
 * Tasks:
 * - Task 1: Physics simulation (50Hz)
 * - Task 2: Flocking control (10Hz)  
 * - Task 3: Radio I/O (LoRa TX/RX + MQTT telemetry)
 * - Task 4: Attack tasks (flood, replay)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "lwip/def.h" 

#include "config.h"       // All configurable parameters are here
#include "common_types.h"
#include "lora_driver.h"
#include "wifi_mqtt.h"
#include "crypto_utils.h"
#include "esp_random.h"   // For attack mode random numbers
#include "esp_mac.h"      // For reading real MAC address

static const char *TAG = "CW1_MAIN";

// ============================================================
// Global Variables - Shared state between tasks
// ============================================================
QueueHandle_t q_current_state;     // Current drone state (position, velocity)
QueueHandle_t q_velocity_cmd;      // Velocity commands from flocking
SemaphoreHandle_t mutex_neighbor;  // Protects neighbor table access
neighbor_entry_t neighbor_table[10]; // Store up to 10 neighbors

uint8_t my_team_id = MY_TEAM_ID;   // From config.h
uint8_t my_node_id[6];             // Real ESP32 WiFi MAC address
uint16_t packet_seq = 0;           // Packet sequence counter

// ============================================================
// Latency/Jitter Statistics - For performance monitoring
// ============================================================
static uint32_t rx_count = 0;           // Total received packets
static uint32_t rx_valid_count = 0;     // Valid signature packets
static uint32_t rx_invalid_count = 0;   // Invalid signature packets
static int64_t latency_sum_ms = 0;      // Sum of latencies (for average)
static int64_t latency_sq_sum = 0;      // Sum of latency^2 (for std dev / jitter)
static uint32_t latency_count = 0;      // Count of valid latency samples
static int64_t latency_max_ms = 0;      // Max latency seen
static int64_t latency_min_ms = 999999; // Min latency seen
static uint32_t last_rx_time_ms = 0;    // Last receive time
static uint32_t timeout_count = 0;      // Neighbor timeout events
static uint32_t stats_report_count = 0; // Counter for periodic stats
static uint32_t minute_counter = 0;     // Counter for 1-minute CSV dump

// ============================================================
// Attack/Defense Statistics - For security evaluation
// ============================================================
static uint32_t replay_blocked_count = 0;   // Packets blocked by replay defense
static uint32_t replay_passed_count = 0;    // Packets that passed verification
static uint32_t cmac_fail_count = 0;        // Packets with invalid CMAC
static uint32_t attack_packets_sent = 0;    // Total attack packets transmitted

// ============================================================
// Helper Functions
// ============================================================

/**
 * @brief Update neighbor table with received packet data.
 * 
 * This function maintains a table of up to 10 neighbors. When a packet is received,
 * it either updates an existing entry (if node_id matches) or creates a new one.
 * Also tracks per-neighbor latency statistics for performance monitoring.
 * 
 * @param pkt      Pointer to the received LoRa packet containing position/velocity data
 * @param latency  One-way latency in milliseconds (calculated from packet timestamp)
 * 
 * @note Thread-safe: uses mutex_neighbor
 * @note Latency values outside 0-10000ms range are ignored (likely clock sync issues)
 */
void update_neighbor(lora_packet_t *pkt, int64_t latency) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreTake(mutex_neighbor, portMAX_DELAY);
    
    int empty_idx = -1;
    int found_idx = -1;
    
    // Search for existing entry or empty slot
    for(int i=0; i<10; i++) {
        if (neighbor_table[i].last_seen_ms > 0 && 
            memcmp(neighbor_table[i].node_id, pkt->node_id, 6) == 0) {
            found_idx = i;
            break;
        }
        if (neighbor_table[i].last_seen_ms == 0 && empty_idx == -1) {
            empty_idx = i;
        }
    }

    int idx = (found_idx != -1) ? found_idx : empty_idx;
    
    if (idx != -1) {
        // If new neighbor, initialize stats
        if (found_idx == -1) {
            neighbor_table[idx].latency_sum = 0;
            neighbor_table[idx].latency_sq_sum = 0;
            neighbor_table[idx].pkt_count = 0;
            neighbor_table[idx].latency_min = 999999;
            neighbor_table[idx].latency_max = 0;
        }
        
        memcpy(neighbor_table[idx].node_id, pkt->node_id, 6);
        // Convert mm to meters (native little-endian, no ntohl needed)
        neighbor_table[idx].state.x = (float)pkt->x_mm / 1000.0f;
        neighbor_table[idx].state.y = (float)pkt->y_mm / 1000.0f;
        neighbor_table[idx].state.z = (float)pkt->z_mm / 1000.0f;
        neighbor_table[idx].state.vx = (float)pkt->vx_mm_s / 1000.0f;
        neighbor_table[idx].state.vy = (float)pkt->vy_mm_s / 1000.0f;
        neighbor_table[idx].state.vz = (float)pkt->vz_mm_s / 1000.0f;
        neighbor_table[idx].last_seen_ms = now;
        
        // Update per-neighbor latency stats
        if (latency > 0 && latency < 10000) {
            neighbor_table[idx].latency_sum += latency;
            neighbor_table[idx].latency_sq_sum += latency * latency;
            neighbor_table[idx].pkt_count++;
            if (latency > neighbor_table[idx].latency_max) neighbor_table[idx].latency_max = latency;
            if (latency < neighbor_table[idx].latency_min) neighbor_table[idx].latency_min = latency;
        }
    }
    xSemaphoreGive(mutex_neighbor);
}

/**
 * @brief Remove stale neighbors that haven't been seen recently.
 * 
 * Iterates through the neighbor table and clears entries that haven't
 * received any packets within NEIGHBOUR_TIMEOUT_MS (default 5000ms).
 * Increments timeout_count for statistics tracking.
 * 
 * @note Thread-safe: uses mutex_neighbor
 * @note Called every LORA_LOOP_INTERVAL_MS from task_radio
 */
void cleanup_neighbors() {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreTake(mutex_neighbor, portMAX_DELAY);
    for(int i=0; i<10; i++) {
        if (neighbor_table[i].last_seen_ms > 0 && 
           (now - neighbor_table[i].last_seen_ms > NEIGHBOUR_TIMEOUT_MS)) {
            neighbor_table[i].last_seen_ms = 0; 
            timeout_count++;
            ESP_LOGW(TAG, "Neighbor index %d timed out - SAD!", i);
        }
    }
    xSemaphoreGive(mutex_neighbor);
}

/**
 * @brief Print current neighbor table to serial console.
 * 
 * Displays MAC address and position (in meters) for each active neighbor.
 * Rate-limited to print at most once every 5 seconds to avoid log spam.
 * 
 * @note Thread-safe: uses mutex_neighbor
 * @note Useful for debugging neighbor discovery issues
 */
void print_neighbor_table() {
    static uint32_t last_print = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    if (now - last_print < 5000) return;
    last_print = now;
    
    xSemaphoreTake(mutex_neighbor, portMAX_DELAY);
    
    int count = 0;
    ESP_LOGI(TAG, "=== NEIGHBOR TABLE ===");
    for(int i=0; i<10; i++) {
        if (neighbor_table[i].last_seen_ms > 0) {
            count++;
            ESP_LOGI(TAG, "[%d] MAC=%02X:%02X:%02X:%02X:%02X:%02X Pos=(%.1f, %.1f, %.1f)m",
                     i,
                     neighbor_table[i].node_id[0], neighbor_table[i].node_id[1],
                     neighbor_table[i].node_id[2], neighbor_table[i].node_id[3],
                     neighbor_table[i].node_id[4], neighbor_table[i].node_id[5],
                     neighbor_table[i].state.x, 
                     neighbor_table[i].state.y, 
                     neighbor_table[i].state.z);
        }
    }
    if (count == 0) {
        ESP_LOGW(TAG, "No neighbors found!");
    } else {
        ESP_LOGI(TAG, "Total: %d neighbors", count);
    }
    ESP_LOGI(TAG, "======================");
    
    xSemaphoreGive(mutex_neighbor);
}

/**
 * @brief Task 1: Physics Integration (50Hz)
 * 
 * Simulates drone movement using simple Euler integration:
 *   position += velocity * dt
 * 
 * Reads velocity commands from q_velocity_cmd queue (set by task_flocking)
 * and publishes updated position to q_current_state queue.
 * 
 * @param pvParameters  Unused FreeRTOS task parameter
 * 
 * @note Runs at 50Hz (20ms period) defined by PHYSICS_FREQ_HZ
 * @note Clamps position within COORD_MIN_M to COORD_MAX_M bounds
 * @note Initial position: center of simulation space (50m, 50m, 50m)
 */
void task_physics(void *pvParameters) {
    drone_state_t state = {
        .x = 50.0f, .y = 50.0f, .z = 50.0f, 
        .vx = 0, .vy = 0, .vz = 0
    };
    drone_state_t cmd_vel = {0};

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 50Hz

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Get velocity command from flocking task
        if (xQueueReceive(q_velocity_cmd, &cmd_vel, 0) == pdTRUE) {
            state.vx = cmd_vel.vx;
            state.vy = cmd_vel.vy;
            state.vz = cmd_vel.vz;
        }

        // Position integration (Euler method)
        float dt = 0.02f; 
        state.x += state.vx * dt;
        state.y += state.vy * dt;
        state.z += state.vz * dt;

        // Boundary bounce (0 to BOX_SIZE_MM/1000 meters)
        float limit_m = (float)BOX_SIZE_MM / 1000.0f;
        
        if (state.x < 0) { state.x = -state.x; state.vx *= -1; }
        if (state.x > limit_m) { state.x = 2 * limit_m - state.x; state.vx *= -1; }
        
        if (state.y < 0) { state.y = -state.y; state.vy *= -1; }
        if (state.y > limit_m) { state.y = 2 * limit_m - state.y; state.vy *= -1; }
        
        if (state.z < 0) { state.z = -state.z; state.vz *= -1; }
        if (state.z > limit_m) { state.z = 2 * limit_m - state.z; state.vz *= -1; }

        state.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        xQueueOverwrite(q_current_state, &state);
    }
}

/**
 * @brief Task 2: Flocking Control (10Hz)
 * 
 * Implements Craig Reynolds' flocking algorithm with three steering behaviors:
 *   1. Separation - Avoid crowding neighbors (strongest weight)
 *   2. Alignment  - Steer towards average heading of neighbors
 *   3. Cohesion   - Steer towards average position of neighbors
 * 
 * Reads current position from q_current_state, reads neighbor data from
 * neighbor_table, computes desired velocity, and writes to q_velocity_cmd.
 * 
 * @param pvParameters  Unused FreeRTOS task parameter
 * 
 * @note Runs at 10Hz (100ms period)
 * @note Weights defined in config.h: SEPARATION_WEIGHT, ALIGNMENT_WEIGHT, COHESION_WEIGHT
 * @note Separation radius: SEPARATION_RADIUS_MM (typically 15000mm = 15m)
 */
void task_flocking(void *pvParameters) {
    drone_state_t my_state;
    drone_state_t target_vel = {0};
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10Hz

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xQueuePeek(q_current_state, &my_state, 0) != pdTRUE) continue;

        // Accumulate neighbor data
        int64_t avg_pos_x = 0, avg_pos_y = 0, avg_pos_z = 0;
        int64_t avg_vel_x = 0, avg_vel_y = 0, avg_vel_z = 0;
        int count = 0;
        int32_t sep_x = 0, sep_y = 0, sep_z = 0;

        xSemaphoreTake(mutex_neighbor, portMAX_DELAY);
        for (int i = 0; i < 10; i++) {
            if (neighbor_table[i].last_seen_ms == 0) continue; 

            // Convert to mm for calculations
            int32_t nb_x = (int32_t)(neighbor_table[i].state.x * 1000);
            int32_t nb_y = (int32_t)(neighbor_table[i].state.y * 1000);
            int32_t nb_z = (int32_t)(neighbor_table[i].state.z * 1000);
            int32_t nb_vx = (int32_t)(neighbor_table[i].state.vx * 1000);
            int32_t nb_vy = (int32_t)(neighbor_table[i].state.vy * 1000);
            int32_t nb_vz = (int32_t)(neighbor_table[i].state.vz * 1000);

            avg_pos_x += nb_x; avg_pos_y += nb_y; avg_pos_z += nb_z;
            avg_vel_x += nb_vx; avg_vel_y += nb_vy; avg_vel_z += nb_vz;
            count++;

            int32_t my_x_mm = (int32_t)(my_state.x * 1000);
            int32_t my_y_mm = (int32_t)(my_state.y * 1000);
            int32_t my_z_mm = (int32_t)(my_state.z * 1000);

            int64_t dx = nb_x - my_x_mm;
            int64_t dy = nb_y - my_y_mm;
            int64_t dz = nb_z - my_z_mm;
            uint64_t dist_sq = dx*dx + dy*dy + dz*dz;

            // Separation: push away if too close
            if (dist_sq < (uint64_t)SEPARATION_THRESH_MM * SEPARATION_THRESH_MM && dist_sq > 0) {
                sep_x -= dx / SEPARATION_DIV;
                sep_y -= dy / SEPARATION_DIV;
                sep_z -= dz / SEPARATION_DIV;
            }
        }
        xSemaphoreGive(mutex_neighbor);

        if (count > 0) {
            avg_pos_x /= count; avg_pos_y /= count; avg_pos_z /= count;
            avg_vel_x /= count; avg_vel_y /= count; avg_vel_z /= count;

            int32_t my_x = (int32_t)(my_state.x * 1000);
            int32_t my_y = (int32_t)(my_state.y * 1000);
            int32_t my_z = (int32_t)(my_state.z * 1000);
            int32_t my_vx = (int32_t)(my_state.vx * 1000);
            int32_t my_vy = (int32_t)(my_state.vy * 1000);
            int32_t my_vz = (int32_t)(my_state.vz * 1000);

            // Cohesion: steer toward flock center
            int32_t coh_x = (avg_pos_x - my_x) / COHESION_DIV;
            int32_t coh_y = (avg_pos_y - my_y) / COHESION_DIV;
            int32_t coh_z = (avg_pos_z - my_z) / COHESION_DIV;

            // Alignment: match neighbor velocity
            int32_t ali_x = (avg_vel_x - my_vx) / ALIGNMENT_DIV;
            int32_t ali_y = (avg_vel_y - my_vy) / ALIGNMENT_DIV;
            int32_t ali_z = (avg_vel_z - my_vz) / ALIGNMENT_DIV;

            // Combine forces
            target_vel.vx = (my_vx + coh_x + ali_x + sep_x) / 1000.0f;
            target_vel.vy = (my_vy + coh_y + ali_y + sep_y) / 1000.0f;
            target_vel.vz = (my_vz + coh_z + ali_z + sep_z) / 1000.0f;
        } else {
            // No neighbors: slow decay
            target_vel.vx = my_state.vx * 0.99f;
            target_vel.vy = my_state.vy * 0.99f;
            target_vel.vz = my_state.vz * 0.99f;
        }

        // Speed limit enforcement
        float speed = sqrtf(target_vel.vx * target_vel.vx + 
                           target_vel.vy * target_vel.vy + 
                           target_vel.vz * target_vel.vz) * 1000.0f;
        if (speed > MAX_SPEED_MM_S) {
            float scale = MAX_SPEED_MM_S / speed;
            target_vel.vx *= scale;
            target_vel.vy *= scale;
            target_vel.vz *= scale;
        }

        xQueueSend(q_velocity_cmd, &target_vel, 0);
    }
}

/**
 * @brief Task 3: Radio I/O - LoRa Communication & MQTT Telemetry
 * 
 * This is the main communication task handling:
 *   - LoRa Receive: Polls for incoming packets, verifies CMAC signature,
 *                   updates neighbor table, calculates latency/jitter stats
 *   - LoRa Transmit: Broadcasts current position at configured rate (LORA_SEND_EVERY_N)
 *   - MQTT Publish: Sends JSON telemetry to broker for visualization
 * 
 * Also implements:
 *   - LoRa watchdog: Reinitializes LoRa if no valid RX for 30s
 *   - Statistics reporting: Prints latency/jitter/loss every 6 seconds
 *   - Attack modes: RANDOM_POS and FLIP_MAC modify packet before sending
 * 
 * @param pvParameters  Unused FreeRTOS task parameter
 * 
 * @note Loop interval: LORA_LOOP_INTERVAL_MS (default 100ms)
 * @note TX rate: Every LORA_SEND_EVERY_N loops (default 10 = 1Hz)
 * @note Requires WiFi/MQTT connection for telemetry
 */
void task_radio(void *pvParameters) {
    lora_init();
    crypto_init(); 

    lora_packet_t tx_pkt;
    lora_packet_t rx_pkt;
    drone_state_t my_state;
    char json_buffer[512]; 
    char node_hex[18];     
    char tag_hex[9];       
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LORA_LOOP_INTERVAL_MS);
    static int loop_count = 0;
    static uint32_t last_valid_rx_time = 0;  // Watchdog: last time we received a valid packet
    const uint32_t LORA_WATCHDOG_MS = 30000; // Reinit LoRa if no RX for 30 seconds

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // LoRa Watchdog: If no valid packets for 30s, reinitialize LoRa
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (last_valid_rx_time > 0 && (now_ms - last_valid_rx_time > LORA_WATCHDOG_MS)) {
            ESP_LOGW("LORA_WD", "No packets for %lu ms - Reinitializing LoRa!", LORA_WATCHDOG_MS);
            lora_init();
            last_valid_rx_time = now_ms;  // Reset watchdog
        }

        cleanup_neighbors();
        
        // --- Receive Logic ---
        while (lora_receive_packet(&rx_pkt)) {
            rx_count++;
            uint32_t rx_tick_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            if (crypto_verify_packet(&rx_pkt)) {
                if (rx_pkt.team_id == my_team_id || rx_pkt.team_id == 0) {
                    
                    // Check for replay attack (if defense enabled)
                    #ifdef DEFENSE_REPLAY_CHECK
                    if (!check_replay(&rx_pkt)) {
                        replay_blocked_count++;
                        ESP_LOGW("LORA_RX", "Replay attack blocked! Seq:%d", rx_pkt.seq_number);
                        continue;  // Drop the packet
                    }
                    #endif
                    
                    replay_passed_count++;
                    rx_valid_count++;
                     
                     // Calculate latency using Unix time (both sender and receiver need NTP sync)
                     uint32_t my_ts_s;
                     uint16_t my_ts_ms;
                     get_current_unix_time(&my_ts_s, &my_ts_ms);
                     int64_t my_time_ms = (int64_t)my_ts_s * 1000 + my_ts_ms;
                     int64_t tx_time_ms = (int64_t)rx_pkt.ts_s * 1000 + rx_pkt.ts_ms;
                     int64_t latency = my_time_ms - tx_time_ms;
                     
                     // Update neighbor table with latency (stores per-neighbor stats)
                     update_neighbor(&rx_pkt, latency);
                     
                     // Also update global stats
                     if (latency > 0 && latency < 10000) {
                         latency_sum_ms += latency;
                         latency_sq_sum += latency * latency;
                         latency_count++;
                         if (latency > latency_max_ms) latency_max_ms = latency;
                         if (latency < latency_min_ms) latency_min_ms = latency;
                     }
                     
                     last_rx_time_ms = rx_tick_ms;
                     
                     // Log individual latency for histogram (CSV format for Python)
                     uint32_t elapsed_s = (xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000;
                     printf("CSV_LATENCY,%lu,%lld\n", elapsed_s, latency);
                     
                     ESP_LOGI("LORA_RX", "Valid packet -> Seq:%d Z:%dmm Lat:%lldms", 
                              rx_pkt.seq_number, (int)rx_pkt.z_mm, latency);
                }
            } else {
                rx_invalid_count++;
                cmac_fail_count++;
                ESP_LOGW("LORA_RX", "Packet dropped: Invalid Signature");
            }
        }
        
        // --- Stats Report (every ~6 seconds) ---
        stats_report_count++;
        if (stats_report_count >= 60) {
            stats_report_count = 0;
            
            // Calculate global average latency
            int64_t avg_lat = (latency_count > 0) ? (latency_sum_ms / latency_count) : 0;
            
            // Calculate global jitter as std dev of latency: sqrt(E[X^2] - E[X]^2)
            float jitter = 0.0f;
            if (latency_count > 1) {
                float mean = (float)latency_sum_ms / latency_count;
                float mean_sq = (float)latency_sq_sum / latency_count;
                float variance = mean_sq - (mean * mean);
                if (variance > 0) {
                    jitter = sqrtf(variance);
                }
            }
            
            float loss_rate = (rx_count > 0) ? ((float)rx_invalid_count / rx_count * 100.0f) : 0;
            
            ESP_LOGI("STATS", "========== LORA STATS (6s) ==========");
            ESP_LOGI("STATS", "RX Total: %lu | Valid: %lu | Invalid: %lu", rx_count, rx_valid_count, rx_invalid_count);
            ESP_LOGI("STATS", "Global Latency: Avg=%lldms Min=%lldms Max=%lldms", avg_lat, latency_min_ms, latency_max_ms);
            ESP_LOGI("STATS", "Global Jitter: %.1fms | Loss: %.1f%% | Timeouts: %lu", jitter, loss_rate, timeout_count);
            
            // Per-neighbor stats
            ESP_LOGI("STATS", "--- Per-Neighbor Stats ---");
            xSemaphoreTake(mutex_neighbor, portMAX_DELAY);
            for (int i = 0; i < 10; i++) {
                if (neighbor_table[i].last_seen_ms > 0 && neighbor_table[i].pkt_count > 0) {
                    int64_t n_avg = neighbor_table[i].latency_sum / neighbor_table[i].pkt_count;
                    float n_jitter = 0.0f;
                    if (neighbor_table[i].pkt_count > 1) {
                        float n_mean = (float)neighbor_table[i].latency_sum / neighbor_table[i].pkt_count;
                        float n_mean_sq = (float)neighbor_table[i].latency_sq_sum / neighbor_table[i].pkt_count;
                        float n_var = n_mean_sq - (n_mean * n_mean);
                        if (n_var > 0) n_jitter = sqrtf(n_var);
                    }
                    ESP_LOGI("STATS", "  [%02X:%02X] Pkts:%lu Lat:Avg=%lldms Jitter:%.1fms",
                             neighbor_table[i].node_id[4], neighbor_table[i].node_id[5],
                             neighbor_table[i].pkt_count, n_avg, n_jitter);
                }
            }
            xSemaphoreGive(mutex_neighbor);
            ESP_LOGI("STATS", "=====================================");
        }
        
        // --- 1-Minute CSV Dump (every 600 loops = 60s) ---
        minute_counter++;
        if (minute_counter >= 600) {
            minute_counter = 0;
            
            // Global stats CSV line
            int64_t avg_lat = (latency_count > 0) ? (latency_sum_ms / latency_count) : 0;
            float jitter_1m = 0.0f;
            if (latency_count > 1) {
                float m = (float)latency_sum_ms / latency_count;
                float m_sq = (float)latency_sq_sum / latency_count;
                float var = m_sq - (m * m);
                if (var > 0) jitter_1m = sqrtf(var);
            }
            float loss_1m = (rx_count > 0) ? ((float)rx_invalid_count / rx_count * 100.0f) : 0;
            
            // Get elapsed time since boot
            uint32_t elapsed_s = (xTaskGetTickCount() * portTICK_PERIOD_MS) / 1000;
            
            // CSV format: elapsed_s,rx,valid,invalid,avg_lat,min_lat,max_lat,jitter,loss,timeouts
            printf("CSV_GLOBAL,%lu,%lu,%lu,%lu,%lld,%lld,%lld,%.2f,%.2f,%lu\n",
                   elapsed_s, rx_count, rx_valid_count, rx_invalid_count,
                   avg_lat, latency_min_ms, latency_max_ms,
                   jitter_1m, loss_1m, timeout_count);
            
            // Per-neighbor CSV
            xSemaphoreTake(mutex_neighbor, portMAX_DELAY);
            for (int i = 0; i < 10; i++) {
                if (neighbor_table[i].last_seen_ms > 0 && neighbor_table[i].pkt_count > 0) {
                    int64_t n_avg = neighbor_table[i].latency_sum / neighbor_table[i].pkt_count;
                    float n_jitter = 0.0f;
                    if (neighbor_table[i].pkt_count > 1) {
                        float n_mean = (float)neighbor_table[i].latency_sum / neighbor_table[i].pkt_count;
                        float n_mean_sq = (float)neighbor_table[i].latency_sq_sum / neighbor_table[i].pkt_count;
                        float n_var = n_mean_sq - (n_mean * n_mean);
                        if (n_var > 0) n_jitter = sqrtf(n_var);
                    }
                    printf("CSV_NEIGHBOR,%lu,%02X%02X%02X%02X%02X%02X,%lu,%lld,%.2f,%.2f,%.2f,%.2f\n",
                           elapsed_s,
                           neighbor_table[i].node_id[0], neighbor_table[i].node_id[1],
                           neighbor_table[i].node_id[2], neighbor_table[i].node_id[3],
                           neighbor_table[i].node_id[4], neighbor_table[i].node_id[5],
                           neighbor_table[i].pkt_count, n_avg, n_jitter,
                           neighbor_table[i].state.x, neighbor_table[i].state.y, neighbor_table[i].state.z);
                }
            }
            xSemaphoreGive(mutex_neighbor);
            
            // Flocking metrics CSV
            if (xQueuePeek(q_current_state, &my_state, 0) == pdTRUE) {
                float cx = 0, cy = 0, cz = 0;
                float min_sep = 999999.0f;
                int nb_count = 0;
                xSemaphoreTake(mutex_neighbor, portMAX_DELAY);
                for (int i = 0; i < 10; i++) {
                    if (neighbor_table[i].last_seen_ms > 0) {
                        cx += neighbor_table[i].state.x;
                        cy += neighbor_table[i].state.y;
                        cz += neighbor_table[i].state.z;
                        nb_count++;
                        float dx = my_state.x - neighbor_table[i].state.x;
                        float dy = my_state.y - neighbor_table[i].state.y;
                        float dz = my_state.z - neighbor_table[i].state.z;
                        float sep = sqrtf(dx*dx + dy*dy + dz*dz);
                        if (sep < min_sep) min_sep = sep;
                    }
                }
                xSemaphoreGive(mutex_neighbor);
                if (nb_count > 0) {
                    cx = (cx + my_state.x) / (nb_count + 1);
                    cy = (cy + my_state.y) / (nb_count + 1);
                    cz = (cz + my_state.z) / (nb_count + 1);
                    float cd = sqrtf((my_state.x-cx)*(my_state.x-cx) + 
                                     (my_state.y-cy)*(my_state.y-cy) + 
                                     (my_state.z-cz)*(my_state.z-cz));
                    printf("CSV_FLOCK,%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                           elapsed_s, nb_count, cd, min_sep, my_state.x, my_state.y, my_state.z);
                }
            }
            
            // Security stats CSV
            float block_rate = (replay_blocked_count + replay_passed_count > 0) ?
                (100.0f * replay_blocked_count / (replay_blocked_count + replay_passed_count)) : 0;
            printf("CSV_SECURITY,%lu,%lu,%lu,%lu,%.2f,%lu\n",
                   elapsed_s, replay_blocked_count, replay_passed_count, 
                   cmac_fail_count, block_rate, attack_packets_sent);
        }
        // --- Transmit Logic (every N loops) ---
        if (++loop_count >= LORA_SEND_EVERY_N) {
            loop_count = 0;
            print_neighbor_table();
            
            if (xQueuePeek(q_current_state, &my_state, 0) == pdTRUE) {
                // Build packet
                memset(&tx_pkt, 0, sizeof(tx_pkt));
                tx_pkt.version = 1;
                tx_pkt.team_id = my_team_id;
                memcpy(tx_pkt.node_id, my_node_id, 6);
                tx_pkt.seq_number = packet_seq++;
                
                uint32_t ts_s_tmp;
                uint16_t ts_ms_tmp;
                get_current_unix_time(&ts_s_tmp, &ts_ms_tmp);
                tx_pkt.ts_s = ts_s_tmp;
                tx_pkt.ts_ms = ts_ms_tmp;

#ifdef ATTACK_MODE_RANDOM_POS
                // Random position attack - for testing only!
                tx_pkt.x_mm  = esp_random() % 100000;
                tx_pkt.y_mm  = esp_random() % 100000;
                tx_pkt.z_mm  = esp_random() % 100000;
                tx_pkt.vx_mm_s = (int32_t)(esp_random() % 10000 - 5000);
                tx_pkt.vy_mm_s = (int32_t)(esp_random() % 10000 - 5000);
                tx_pkt.vz_mm_s = (int32_t)(esp_random() % 10000 - 5000);
                tx_pkt.yaw_cd = esp_random() % 36000;
                ESP_LOGW("ATTACK", "Random Position: X=%lu Y=%lu Z=%lu", tx_pkt.x_mm, tx_pkt.y_mm, tx_pkt.z_mm);
#else
                // Normal mode: use actual state
                tx_pkt.x_mm  = (uint32_t)(my_state.x * 1000);
                tx_pkt.y_mm  = (uint32_t)(my_state.y * 1000);
                tx_pkt.z_mm  = (uint32_t)(my_state.z * 1000);
                tx_pkt.vx_mm_s = (int32_t)(my_state.vx * 1000);
                tx_pkt.vy_mm_s = (int32_t)(my_state.vy * 1000);
                tx_pkt.vz_mm_s = (int32_t)(my_state.vz * 1000);
                
                // Calculate yaw from velocity direction
                if (tx_pkt.vx_mm_s == 0 && tx_pkt.vy_mm_s == 0) {
                    tx_pkt.yaw_cd = 0;
                } else {
                    float yaw_rad = atan2f(my_state.vy, my_state.vx);
                    float yaw_deg = yaw_rad * (180.0f / M_PI);
                    if (yaw_deg < 0) yaw_deg += 360.0f;
                    tx_pkt.yaw_cd = (uint16_t)(yaw_deg * 100.0f);
                }
#endif

                crypto_sign_packet(&tx_pkt);

#ifdef ATTACK_MODE_FLIP_MAC
                // Flip MAC signature for attack testing
                tx_pkt.mac_tag[0] ^= 0xFF;
                ESP_LOGW("ATTACK", "MAC Flip Attack: Signature corrupted!");
#endif

                lora_send_packet(&tx_pkt);

                ESP_LOGI("LORA_TX", "Seq:%d | Pos: X=%d Y=%d Z=%d (mm)", 
                         tx_pkt.seq_number, (int)tx_pkt.x_mm, (int)tx_pkt.y_mm, (int)tx_pkt.z_mm);
                
                // MQTT telemetry
                snprintf(node_hex, sizeof(node_hex), "%02X:%02X:%02X:%02X:%02X:%02X",
                         tx_pkt.node_id[0], tx_pkt.node_id[1], tx_pkt.node_id[2],
                         tx_pkt.node_id[3], tx_pkt.node_id[4], tx_pkt.node_id[5]);

                snprintf(tag_hex, sizeof(tag_hex), "%02X%02X%02X%02X", 
                         tx_pkt.mac_tag[0], tx_pkt.mac_tag[1], tx_pkt.mac_tag[2], tx_pkt.mac_tag[3]);

                snprintf(json_buffer, sizeof(json_buffer),
                    "{\"version\":%d,\"team_id\":%d,\"node_id\":\"%s\",\"seq_number\":%d,"
                    "\"ts_s\":%lu,\"ts_ms\":%u,\"x_mm\":%d,\"y_mm\":%d,\"z_mm\":%d,"
                    "\"vx_mm_s\":%d,\"vy_mm_s\":%d,\"vz_mm_s\":%d,\"yaw_cd\":%d,\"mac_tag\":\"%s\"}",
                    tx_pkt.version, tx_pkt.team_id, node_hex, tx_pkt.seq_number,
                    tx_pkt.ts_s, tx_pkt.ts_ms,
                    (int)tx_pkt.x_mm, (int)tx_pkt.y_mm, (int)tx_pkt.z_mm,
                    (int)tx_pkt.vx_mm_s, (int)tx_pkt.vy_mm_s, (int)tx_pkt.vz_mm_s,
                    tx_pkt.yaw_cd, tag_hex);

                mqtt_publish_telemetry(json_buffer);
            }
        }
    }
}

/**
 * @brief Task 4: Flood Attack (Security Testing Only!)
 * 
 * Sends rapid bursts of packets to test channel resilience.
 * This demonstrates a Denial-of-Service (DoS) attack vector where
 * the attacker attempts to dominate the LoRa channel.
 * 
 * Behavior:
 *   1. Sends FLOOD_PACKET_COUNT packets with FLOOD_PACKET_DELAY_MS between each
 *   2. Sleeps for FLOOD_SLEEP_MS between bursts
 *   3. Repeats forever
 * 
 * @param pvParameters  Unused FreeRTOS task parameter
 * 
 * @note Enable with #define ATTACK_MODE_FLOOD in config.h
 * @note Replaces normal flocking tasks when enabled
 * @warning FOR SECURITY TESTING ONLY - Do not use maliciously!
 */
void task_flood(void *pvParameters) {
    lora_init();
    crypto_init();
    
    lora_packet_t flood_pkt;
    memset(&flood_pkt, 0, sizeof(flood_pkt));
    
    // Initialize packet with static position
    flood_pkt.version = 1;
    flood_pkt.team_id = my_team_id;
    memcpy(flood_pkt.node_id, my_node_id, 6);
    flood_pkt.x_mm = 50000;  // Center of box
    flood_pkt.y_mm = 50000;
    flood_pkt.z_mm = 50000;
    flood_pkt.vx_mm_s = 0;
    flood_pkt.vy_mm_s = 0;
    flood_pkt.vz_mm_s = 0;
    flood_pkt.yaw_cd = 0;
    
    ESP_LOGW("ATTACK", "========== FLOOD ATTACK STARTED ==========");
    ESP_LOGW("ATTACK", "This is for TESTING ONLY. Be responsible!");
    
    while (1) {
        // Rapid fire packets
        for (int i = 0; i < FLOOD_PACKET_COUNT; i++) {
            flood_pkt.seq_number = i;
            flood_pkt.ts_s = xTaskGetTickCount() / 1000;
            flood_pkt.ts_ms = xTaskGetTickCount() % 1000;
            
            crypto_sign_packet(&flood_pkt);
            lora_send_packet(&flood_pkt);
            ESP_LOGW("ATTACK", "Flood packet #%d sent", i);
            vTaskDelay(pdMS_TO_TICKS(FLOOD_PACKET_DELAY_MS));
        }
        
        // Rest between bursts
        ESP_LOGW("ATTACK", "Flood burst complete. Sleeping %d ms...", FLOOD_SLEEP_MS);
        vTaskDelay(pdMS_TO_TICKS(FLOOD_SLEEP_MS));
    }
}

#ifdef ATTACK_MODE_REPLAY
static lora_packet_t replay_buffer[REPLAY_BUFFER_SIZE];
static int replay_buffer_count = 0;

/**
 * @brief Task 5: Replay Attack (Security Testing Only!)
 * 
 * Demonstrates a replay attack by capturing valid packets and
 * re-transmitting them later. Tests the effectiveness of replay
 * defense mechanisms (timestamp freshness, sequence number tracking).
 * 
 * Phase 1: Capture
 *   - Listens for valid packets (passes CMAC verification)
 *   - Stores REPLAY_BUFFER_SIZE packets in buffer
 * 
 * Phase 2: Replay
 *   - Continuously re-sends captured packets
 *   - Packets have OLD timestamps and DUPLICATE sequence numbers
 *   - If DEFENSE_REPLAY_CHECK is enabled on receivers, these should be blocked
 * 
 * @param pvParameters  Unused FreeRTOS task parameter
 * 
 * @note Enable with #define ATTACK_MODE_REPLAY in config.h
 * @note Replaces normal flocking tasks when enabled
 * @note Counter-defense: Enable DEFENSE_REPLAY_CHECK on receivers
 */
void task_replay(void *pvParameters) {
    lora_init();
    crypto_init();
    
    lora_packet_t rx_pkt;
    
    ESP_LOGW("ATTACK", "========== REPLAY ATTACK STARTED ==========");
    ESP_LOGW("ATTACK", "Phase 1: Capturing %d valid packets...", REPLAY_BUFFER_SIZE);
    
    // Phase 1: Capture valid packets
    while (replay_buffer_count < REPLAY_BUFFER_SIZE) {
        if (lora_receive_packet(&rx_pkt)) {
            if (crypto_verify_packet(&rx_pkt)) {
                memcpy(&replay_buffer[replay_buffer_count], &rx_pkt, sizeof(lora_packet_t));
                replay_buffer_count++;
                ESP_LOGW("ATTACK", "Captured packet %d/%d (Seq: %d)", 
                         replay_buffer_count, REPLAY_BUFFER_SIZE, rx_pkt.seq_number);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGW("ATTACK", "Phase 2: Replaying captured packets in a loop...");
    
    // Phase 2: Replay captured packets forever
    while (1) {
        for (int i = 0; i < REPLAY_BUFFER_SIZE; i++) {
            ESP_LOGW("ATTACK", "REPLAYING packet %d (Original Seq: %d, TS: %lu)", 
                     i, replay_buffer[i].seq_number, replay_buffer[i].ts_s);
            lora_send_packet(&replay_buffer[i]);
            vTaskDelay(pdMS_TO_TICKS(500));  // 2 Hz replay rate
        }
        ESP_LOGW("ATTACK", "Replay cycle complete. Starting again...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
#endif // ATTACK_MODE_REPLAY

/**
 * @brief Application Entry Point
 * 
 * Initializes the ESP32 system and starts all FreeRTOS tasks.
 * 
 * Initialization sequence:
 *   1. NVS Flash - Required for WiFi credential storage
 *   2. Read ESP32 MAC address as unique node identifier
 *   3. WiFi connection (blocks until connected)
 *   4. MQTT client start (with NTP time sync)
 *   5. Create IPC queues and mutexes
 *   6. Start appropriate tasks based on attack mode configuration
 * 
 * Task startup modes:
 *   - ATTACK_MODE_FLOOD: Only runs task_flood (no flocking)
 *   - ATTACK_MODE_REPLAY: Only runs task_replay (no flocking)
 *   - Normal: Runs task_physics + task_flocking + task_radio
 * 
 * @note This function never returns (tasks run forever)
 * @note Logs startup info including packet size validation
 */
void app_main(void) {
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Read real ESP32 WiFi MAC address
    esp_read_mac(my_node_id, ESP_MAC_WIFI_STA);
    
    // Startup info
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "CW1 Flocking System - READY TO FLY!");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Packet size: %d bytes (expected 46)", sizeof(lora_packet_t));
    ESP_LOGI(TAG, "MAC input: %zu bytes (expected 42)", offsetof(lora_packet_t, mac_tag));
    ESP_LOGI(TAG, "Team ID: %d", my_team_id);
    ESP_LOGI(TAG, "Node ID: %02X:%02X:%02X:%02X:%02X:%02X", 
             my_node_id[0], my_node_id[1], my_node_id[2],
             my_node_id[3], my_node_id[4], my_node_id[5]);
    
#ifdef ATTACK_MODE_RANDOM_POS
    ESP_LOGW(TAG, "[!] ATTACK MODE: Random Position ENABLED");
#endif
#ifdef ATTACK_MODE_FLIP_MAC  
    ESP_LOGW(TAG, "[!] ATTACK MODE: MAC Flip ENABLED");
#endif
#ifdef ATTACK_MODE_FLOOD
    ESP_LOGW(TAG, "[!] ATTACK MODE: Flood ENABLED");
#endif
#ifdef ATTACK_MODE_REPLAY
    ESP_LOGW(TAG, "[!] ATTACK MODE: Replay ENABLED");
#endif
#ifdef DEFENSE_REPLAY_CHECK
    ESP_LOGI(TAG, "[*] DEFENSE: Replay Check ENABLED (window: %d ms)", REPLAY_WINDOW_MS);
#endif
    
    ESP_LOGI(TAG, "==========================================");

    // Initialize WiFi and MQTT (MQTT starts automatically after WiFi connects)
    wifi_init_sta(); 

    // Create IPC mechanisms
    q_current_state = xQueueCreate(1, sizeof(drone_state_t));
    q_velocity_cmd = xQueueCreate(1, sizeof(drone_state_t));
    mutex_neighbor = xSemaphoreCreateMutex();
    memset(neighbor_table, 0, sizeof(neighbor_table));

#ifdef ATTACK_MODE_FLOOD
    // Flood attack mode: only start flood task
    xTaskCreate(task_flood, "FLOOD", 4096, NULL, 5, NULL);
    printf("System Started in FLOOD ATTACK Mode!\n");
#elif defined(ATTACK_MODE_REPLAY)
    // Replay attack mode: capture then replay
    xTaskCreate(task_replay, "REPLAY", 4096, NULL, 5, NULL);
    printf("System Started in REPLAY ATTACK Mode!\n");
#else
    // Normal mode: start all flocking tasks
    xTaskCreate(task_physics, "PHYSICS", 4096, NULL, 5, NULL);
    xTaskCreate(task_radio,   "RADIO",   4096, NULL, 4, NULL);
    xTaskCreate(task_flocking,"FLOCK",   4096, NULL, 3, NULL);
    printf("System Started - Flocking algorithm ACTIVE!\n");
#endif
}