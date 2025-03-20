#ifndef MUTEX_GUARD_HPP
#define MUTEX_GUARD_HPP

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Use RAII with a class that takes the mutex
// in the constructor, and returns the mutex
// in the destructor, ensuring the mutex is freed
// once the object goes out of scope
class MutexGuard {
public:
    // The constructor takes the desired mutex as an argument
    explicit MutexGuard(SemaphoreHandle_t &mutex) : _mutex(mutex) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
    }
    //The destructor returns the mutex
    ~MutexGuard() {
        xSemaphoreGive(_mutex);
    }
private:
    SemaphoreHandle_t &_mutex;
}

#endif