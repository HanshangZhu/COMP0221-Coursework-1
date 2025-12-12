#ifndef LORA_DRIVER_H
#define LORA_DRIVER_H

#include "common_types.h"

// 初始化 LoRa
void lora_init(void);

// 设置频率 (单位: MHz，例如 868, 915, 433)
void lora_set_frequency(uint32_t freq_mhz);

// 发送数据包
void lora_send_packet(lora_packet_t *packet);

// 检查并读取数据包 (返回 1 表示收到，0 表示没收到)
int lora_receive_packet(lora_packet_t *packet);

#endif