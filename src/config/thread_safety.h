#ifndef THREAD_SAFETY_H
#define THREAD_SAFETY_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

inline SemaphoreHandle_t i2cMutex = NULL;
inline SemaphoreHandle_t sharedDataMutex = NULL;
inline bool mutexesInitialized = false;
inline portMUX_TYPE mutexInitMux = portMUX_INITIALIZER_UNLOCKED;

inline void thread_safety_init() {
    portENTER_CRITICAL(&mutexInitMux);
    if (!mutexesInitialized) {
        i2cMutex = xSemaphoreCreateMutex();
        sharedDataMutex = xSemaphoreCreateMutex();
        mutexesInitialized = true;
    }
    portEXIT_CRITICAL(&mutexInitMux);
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
