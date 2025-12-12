#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <stdint.h>
#include "config.h"  // 统一配置

// 函数声明
void wifi_init_sta(void);
void mqtt_app_start(void);
void mqtt_publish_telemetry(char *json_payload);

// 获取当前 Unix 时间戳
void get_current_unix_time(uint32_t *ts_s, uint16_t *ts_ms);

#endif