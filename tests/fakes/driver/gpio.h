#pragma once
#include "esp_sleep.h"
using gpio_num_t = int;
constexpr int GPIO_NUM_11 = 11, GPIO_NUM_12 = 12, GPIO_INTR_LOW_LEVEL = 0;
inline int gpio_wakeup_enable(int, int) { return ESP_OK; }
inline void gpio_wakeup_disable(int) {}
