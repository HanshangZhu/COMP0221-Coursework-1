#ifndef CONFIG_H
#define CONFIG_H

/**
 * ============================================================
 *  CW1 CONFIGURATION FILE
 * ============================================================
 */

// ============================================================
//  1. WiFi Configuration
// ============================================================

// Uncomment to use Eduroam
#define USE_EDUROAM

// Home WiFi (REDACTED - set your own credentials)
#define HOME_WIFI_SSID "YOUR_WIFI_SSID"    
#define HOME_WIFI_PASS "YOUR_WIFI_PASSWORD"     

// Eduroam credentials (REDACTED - set your own credentials)
#define EDUROAM_SSID        "eduroam"
#define EDUROAM_IDENTITY    "YOUR_UCL_EMAIL@ucl.ac.uk"
#define EDUROAM_USERNAME    "YOUR_UCL_EMAIL@ucl.ac.uk"
#define EDUROAM_PASSWORD    "YOUR_EDUROAM_PASSWORD"


// ============================================================
//  2. MQTT Configuration
// ============================================================

#define MQTT_BROKER_URI  "mqtt://broker.emqx.io:1883"
#define MQTT_TOPIC       "flocksim"


// ============================================================
//  3. Team Configuration
// ============================================================

#define MY_TEAM_ID          1   // Team ID (Change this to your assigned team ID)


// ============================================================
//  4. Security Key (CMAC)
// ============================================================

// Shared team key for CMAC (REDACTED - use your team's key)
#define TEAM_KEY_BYTES { \
    0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00  \
}


// ============================================================
//  5. LoRa Parameters
// ============================================================

// Pin definitions
#define PIN_NUM_MISO        19
#define PIN_NUM_MOSI        27
#define PIN_NUM_CLK         5
#define PIN_NUM_CS          18
#define PIN_NUM_RST         23
#define PIN_NUM_DIO0        26

// Modulation parameters
#define LORA_FREQ_MHZ       868.2f
#define LORA_BW_KHZ         250.0f
#define LORA_SF             7
#define LORA_CR             7
#define LORA_SYNCWORD       0x12
#define LORA_PREAMBLE       10
#define LORA_POWER_DBM      14


// ============================================================
//  6. Flocking Algorithm Parameters
// ============================================================

#define BOX_SIZE_MM             100000   // 100 meters
#define NEIGHBOUR_TIMEOUT_MS    10000    // 10 seconds
#define COHESION_DIV            100
#define ALIGNMENT_DIV           8
#define SEPARATION_DIV          0.5
#define SEPARATION_THRESH_MM    5000     // 5 meters
#define MAX_SPEED_MM_S          4900.0f  // 4.9 m/s


// ============================================================
//  7. LoRa Timing Configuration
// ============================================================

#define LORA_LOOP_INTERVAL_MS   100
#define LORA_SEND_EVERY_N       5


// ============================================================
//  8. Attack Mode Switches
// ============================================================

// Attack modes (uncomment to enable)
// #define ATTACK_MODE_RANDOM_POS
// #define ATTACK_MODE_FLIP_MAC
// #define ATTACK_MODE_FLOOD
// #define ATTACK_MODE_REPLAY

// Attack parameters
#define FLOOD_PACKET_COUNT      100
#define FLOOD_PACKET_DELAY_MS   50
#define FLOOD_SLEEP_MS          10000
#define REPLAY_BUFFER_SIZE      5


// ============================================================
//  9. Security Defense Settings
// ============================================================

// Uncomment to enable defenses
// #define DEFENSE_REPLAY_CHECK  // Sequence-based detection (no NTP needed)

// Defense parameters
#define REPLAY_WINDOW_MS        5000
#define SEQ_WINDOW_SIZE         16


#endif // CONFIG_H
