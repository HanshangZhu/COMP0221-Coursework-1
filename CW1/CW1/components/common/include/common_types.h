#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>

/**
 * LoRa Packet Structure (COMP0221 Standard - 46 bytes)
 * This is THE packet format. Perfect. No flags needed!
 */
typedef struct __attribute__((packed)) {
    uint8_t version;        // Protocol version - Currently 1. The FIRST and the BEST!
    uint8_t team_id;        // Team ID - Your crew, your channel!
    uint8_t node_id[6];     // MAC address - 6 bytes of pure identity!
    uint16_t seq_number;    // Sequence number - Counting packets like votes!
    uint32_t ts_s;          // Unix timestamp (seconds) - When it happened!
    uint16_t ts_ms;         // Milliseconds part - Precision timing!
    uint32_t x_mm;          // X position in millimeters
    uint32_t y_mm;          // Y position in millimeters
    uint32_t z_mm;          // Z position in millimeters - Altitude, baby!
    int32_t vx_mm_s;        // X velocity in mm/s
    int32_t vy_mm_s;        // Y velocity in mm/s
    int32_t vz_mm_s;        // Z velocity in mm/s
    uint16_t yaw_cd;        // Heading in centidegrees (0.01 degrees)
    uint8_t mac_tag[4];     // Security tag (4 bytes) - AUTHENTICATED!
} lora_packet_t;

/**
 * Internal Drone State - What WE know about ourselves!
 */
typedef struct {
    float x, y, z;          // Position in meters - Where are we?
    float vx, vy, vz;       // Velocity in m/s - Where are we going?
    float yaw;              // Heading
    uint32_t timestamp_ms;  // Local timestamp
} drone_state_t;

/**
 * Neighbor Table Entry - Who's flying with us?
 * We track up to 10 neighbors with their latency/jitter stats!
 */
typedef struct {
    uint8_t node_id[6];         // Their MAC address
    drone_state_t state;        // Their state - Position, velocity, etc.
    uint32_t last_seen_ms;      // When we last heard from them
    // Per-neighbor latency/jitter stats
    int64_t latency_sum;        // Sum of latencies (for avg)
    int64_t latency_sq_sum;     // Sum of latency^2 (for std dev)
    uint32_t pkt_count;         // Packets received from this neighbor
    int64_t latency_min;        // Min latency from this neighbor
    int64_t latency_max;        // Max latency from this neighbor
} neighbor_entry_t;

#endif