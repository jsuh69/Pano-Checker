#pragma once
#include "M5Unified.h"
inline int64_t esp_timer_get_time() { return (int64_t)fakeUs; }
