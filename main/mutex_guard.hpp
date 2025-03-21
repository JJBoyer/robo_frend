/*

File Name: mutex_guard.hpp
Author: Jacob Boyer
Description: Creates a wrapper for mutex
handling, ensuring RAII principles are applied
to taking and giving mutexes so they are
only locked when in use

*/

#ifndef MUTEX_GUARD_HPP
#define MUTEX_GUARD_HPP

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* MutexGuard class:
  Use RAII with a class that takes the mutex in
  the constructor, and returns the mutex in the
  destructor, ensuring the mutex is always
  unlocked once the object goes out of scope
*/
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