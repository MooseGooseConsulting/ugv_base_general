#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
using TickType_t = uint32_t;
using SemaphoreHandle_t = std::mutex*;
constexpr int portMAX_DELAY = -1, pdPASS = 1;
#define pdMS_TO_TICKS(ms) (ms)
extern bool failMutex, failTask;
extern std::atomic<bool> stopTask;
extern std::thread wheelThread;
struct EndTask {};
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return failMutex ? nullptr : new std::mutex; }
inline void xSemaphoreTake(SemaphoreHandle_t mutex, int) { mutex->lock(); }
inline void xSemaphoreGive(SemaphoreHandle_t mutex) { mutex->unlock(); }
inline TickType_t xTaskGetTickCount() { return 0; }
inline void vTaskDelayUntil(TickType_t*, TickType_t period) {
  if (stopTask) throw EndTask{};
  std::this_thread::sleep_for(std::chrono::milliseconds(period));
  if (stopTask) throw EndTask{};
}
inline int xTaskCreate(void (*task)(void*), const char*, int, void*, int, void*) {
  if (failTask) return 0;
  wheelThread = std::thread([task] { try { task(nullptr); } catch (const EndTask&) {} });
  return pdPASS;
}
