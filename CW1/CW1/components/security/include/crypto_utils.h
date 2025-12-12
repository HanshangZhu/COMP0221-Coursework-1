#ifndef CRYPTO_UTILS_H
#define CRYPTO_UTILS_H

#include "common_types.h"
#include <stdbool.h>

// Initialize crypto module
void crypto_init(void);

// Sign packet with CMAC
void crypto_sign_packet(lora_packet_t *packet);

// Verify packet CMAC
bool crypto_verify_packet(const lora_packet_t *packet);

// Check for replay attack (returns true if fresh, false if replay)
bool check_replay(const lora_packet_t *packet);

#endif