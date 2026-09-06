#ifndef THREAD_SAFETY_H
#define THREAD_SAFETY_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

inline SemaphoreHandle_t i2cMutex = NULL;
inline SemaphoreHandle_t sharedDataMutex = NULL;

inline void thread_safety_init() {
    if (i2cMutex == NULL) {
        i2cMutex = xSemaphoreCreateMutex();
    }
    if (sharedDataMutex == NULL) {
        sharedDataMutex = xSemaphoreCreateMutex();
    }
}

inline bool lock_i2c(TickType_t timeout = pdMS_TO_TICKS(50)) {
    return (i2cMutex != NULL) && (xSemaphoreTake(i2cMutex, timeout) == pdTRUE);
}

inline void unlock_i2c() {
    if (i2cMutex != NULL) {
        xSemaphoreGive(i2cMutex);
    }
}

inline bool lock_shared_data(TickType_t timeout = pdMS_TO_TICKS(20)) {
    return (sharedDataMutex != NULL) && (xSemaphoreTake(sharedDataMutex, timeout) == pdTRUE);
}

inline void unlock_shared_data() {
    if (sharedDataMutex != NULL) {
        xSemaphoreGive(sharedDataMutex);
    }
}

#endif
