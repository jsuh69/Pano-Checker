#pragma once
#include "M5Unified.h"
using esp_err_t = int;
constexpr int ESP_OK = 0;
constexpr int ESP_SLEEP_WAKEUP_GPIO = 1, ESP_SLEEP_WAKEUP_TIMER = 2;
inline unsigned sleepCalls = 0;
inline int wakeCause = ESP_SLEEP_WAKEUP_TIMER;
inline uint64_t timerWakeUs = 0;
inline int esp_sleep_enable_gpio_wakeup() { return ESP_OK; }
inline int esp_sleep_enable_timer_wakeup(uint64_t us) { timerWakeUs = us; return ESP_OK; }
inline int esp_light_sleep_start() { ++sleepCalls; fakeUs += timerWakeUs; return ESP_OK; }
inline int esp_sleep_get_wakeup_cause() { return wakeCause; }
inline void esp_sleep_disable_wakeup_source(int) {}
