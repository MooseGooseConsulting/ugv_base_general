#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "wheel_control_state.h"

// Only service() (and pin initialization before task creation) writes motors.
// Command contexts publish POD state under the same short mutex. Never put
// JSON, serial, filesystem, mission, arm or gimbal work inside this mutex.
namespace WheelDriver {
WheelControl::State state(MOTION_CMD_WATCHDOG_MS, __kp, __ki, __kd, windup_limits);
SemaphoreHandle_t mutex = nullptr;
bool ready = false;
ESP32Encoder encoderA, encoderB;
PID_v2 pidA(__kp, __ki, __kd, PID::Direct);
PID_v2 pidB(__kp, __ki, __kd, PID::Direct);
double measuredLeft = 0, measuredRight = 0;
int64_t previousLeft = 0, previousRight = 0;
uint32_t previousMicros = 0;
int sampledType = 0;
uint32_t appliedResetEpoch = UINT32_MAX;
float previousKp = -1, previousKi = -1, previousKd = -1;

void hardStop() {
  ledcWrite(channel_A, 0);
  ledcWrite(channel_B, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void output(int pin1, int pin2, int channel, double value, bool reversed) {
  // Saturate before integer conversion; malformed PID output fails closed.
  int pwm = std::isfinite(value) ? lround(WheelControl::State::pwm(value)) : 0;
  if (pwm == 0) {
    ledcWrite(channel, 0);
    digitalWrite(pin1, LOW); digitalWrite(pin2, LOW);
    return;
  }
  digitalWrite(pin1, (pwm < 0) == reversed ? HIGH : LOW);
  digitalWrite(pin2, (pwm < 0) == reversed ? LOW : HIGH);
  ledcWrite(channel, abs(pwm));
}

// Called only by the wheel task. Its work is bounded: one encoder sample,
// two PID updates and two PWM writes. No command dispatch or blocking mission.
void service() {
  xSemaphoreTake(mutex, portMAX_DELAY);
  state.expire(millis());
  const auto desired = state.get();
  const bool reversed = desired.type != 1;
  const double diameter = desired.type == 3 ? .0523 : .0800;
  const int pulses = desired.type == 1 ? 2100 : desired.type == 2 ? 1650 : 1092;
  const uint32_t now = micros();
  const int64_t left = encoderA.getCount(), right = encoderB.getCount();
  const uint32_t elapsed = now - previousMicros;
  const bool typeChanged = sampledType != desired.type;
  if (typeChanged) {
    sampledType = desired.type;
    measuredLeft = measuredRight = 0;
  } else if (elapsed) {
    const double factor = (reversed ? -1 : 1) * 3.14159265359 * diameter / pulses * 1000000.0 / elapsed;
    measuredLeft = (left - previousLeft) * factor;
    measuredRight = (right - previousRight) * factor;
  }
  previousLeft = left; previousRight = right; previousMicros = now;

  if (desired.kp != previousKp || desired.ki != previousKi || desired.kd != previousKd) {
    pidA.SetTunings(desired.kp, desired.ki, desired.kd);
    pidB.SetTunings(desired.kp, desired.ki, desired.kd);
    previousKp = desired.kp; previousKi = desired.ki; previousKd = desired.kd;
  }
  if (desired.mode == WheelControl::Mode::Stopped) {
    hardStop(); // Emergency/configuration stop cannot be undone by PID state.
  } else if (desired.mode == WheelControl::Mode::Pwm) {
    output(AIN1, AIN2, channel_A, desired.left, reversed);
    output(BIN1, BIN2, channel_B, desired.right, reversed);
  } else {
    // Reset cached output/integral on entry, particularly watchdog braking.
    // Otherwise PID_v2 can replay the old nonzero output until its next sample.
    if (appliedResetEpoch != desired.resetEpoch || typeChanged) {
      pidA.SetMode(PID::Manual); pidB.SetMode(PID::Manual);
      pidA.Start(measuredLeft, 0, desired.left);
      pidB.Start(measuredRight, 0, desired.right);
    }
    pidA.Setpoint(desired.left); pidB.Setpoint(desired.right);
    double leftPwm = pidA.Run(measuredLeft), rightPwm = pidB.Run(measuredRight);
    if (abs(leftPwm) < THRESHOLD_PWM || (desired.left == 0 && measuredLeft == 0)) leftPwm = 0;
    if (abs(rightPwm) < THRESHOLD_PWM || (desired.right == 0 && measuredRight == 0)) rightPwm = 0;
    output(AIN1, AIN2, channel_A, leftPwm, reversed);
    output(BIN1, BIN2, channel_B, rightPwm, reversed);
  }
  appliedResetEpoch = desired.resetEpoch;
  xSemaphoreGive(mutex);
}

void task(void*) {
  TickType_t wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1;
  for (;;) {
    service();
    vTaskDelayUntil(&wake, period);
  }
}
} // namespace WheelDriver

void movtionPinInit() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  ledcSetup(channel_A, freq, ANALOG_WRITE_BITS); ledcAttachPin(PWMA, channel_A);
  ledcSetup(channel_B, freq, ANALOG_WRITE_BITS); ledcAttachPin(PWMB, channel_B);
  WheelDriver::hardStop();
  WheelDriver::mutex = xSemaphoreCreateMutex();
}

void initEncoders() {
  WheelDriver::encoderA.attachHalfQuad(AENCA, AENCB);
  WheelDriver::encoderB.attachHalfQuad(BENCA, BENCB);
  WheelDriver::encoderA.setCount(0); WheelDriver::encoderB.setCount(0);
}

bool startWheelControl() {
  using namespace WheelDriver;
  if (!mutex) return false; // Pin initialization already left outputs off.
  state.type(mainType, millis());
  pidA.SetOutputLimits(-255, 255); pidB.SetOutputLimits(-255, 255);
  previousMicros = micros();
  // ESP-IDF stack size is bytes. Priority 3 preempts Arduino loop's priority 1,
  // on either single- or dual-core builds. No affinity to a possibly absent core.
  if (xTaskCreate(task, "wheel-control", 4096, nullptr, 3, nullptr) != pdPASS) return false;
  xSemaphoreTake(mutex, portMAX_DELAY);
  ready = true;
  xSemaphoreGive(mutex);
  return true;
}

bool setGoalSpeed(float left, float right) {
  using namespace WheelDriver;
  if (!mutex) return false;
  xSemaphoreTake(mutex, portMAX_DELAY);
  const bool accepted = ready && state.speed(left, right, millis());
  xSemaphoreGive(mutex);
  return accepted;
}

bool rosCtrl(float x, float z) {
  using namespace WheelDriver;
  if (!mutex) return false;
  xSemaphoreTake(mutex, portMAX_DELAY);
  const bool accepted = ready && state.ros(x, z, millis());
  xSemaphoreGive(mutex);
  return accepted;
}

bool setWheelPwm(float left, float right) {
  using namespace WheelDriver;
  if (!mutex) return false;
  xSemaphoreTake(mutex, portMAX_DELAY);
  const bool accepted = ready && state.raw(left, right, millis());
  xSemaphoreGive(mutex);
  return accepted;
}

void switchEmergencyStop() {
  using namespace WheelDriver;
  if (!mutex) return;
  xSemaphoreTake(mutex, portMAX_DELAY);
  state.stop();
  xSemaphoreGive(mutex);
}

void changeHeartBeatDelay(int value) {
  using namespace WheelDriver;
  if (!mutex) return;
  xSemaphoreTake(mutex, portMAX_DELAY);
  state.timeout(value, millis());
  xSemaphoreGive(mutex);
}

void setSpdRate(float left, float right) {
  using namespace WheelDriver;
  if (!mutex) return;
  xSemaphoreTake(mutex, portMAX_DELAY);
  state.scale(left, right, millis());
  xSemaphoreGive(mutex);
}

void setPID(float p, float i, float d, float limits) {
  using namespace WheelDriver;
  if (!mutex) return;
  xSemaphoreTake(mutex, portMAX_DELAY);
  state.gains(p, i, d, limits, millis());
  xSemaphoreGive(mutex);
}

struct WheelFeedback { double left = 0, right = 0; float scaleLeft = 1, scaleRight = 1; };
WheelFeedback wheelFeedback() {
  using namespace WheelDriver;
  WheelFeedback result;
  if (!mutex) return result;
  xSemaphoreTake(mutex, portMAX_DELAY);
  result.left = measuredLeft; result.right = measuredRight;
  result.scaleLeft = state.get().scaleLeft; result.scaleRight = state.get().scaleRight;
  xSemaphoreGive(mutex);
  return result;
}

void getSpdRate() {
  const auto feedback = wheelFeedback();
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = CMD_GET_SPD_RATE;
  jsonInfoHttp["L"] = feedback.scaleLeft; jsonInfoHttp["R"] = feedback.scaleRight;
  String text;
  serializeJson(jsonInfoHttp, text); Serial.println(text);
}

void mm_settings(byte inputMain, byte inputModule) {
  if (inputMain < 1 || inputMain > 3) return;
  if (WheelDriver::mutex) {
    xSemaphoreTake(WheelDriver::mutex, portMAX_DELAY);
    WheelDriver::state.type(inputMain, millis());
    xSemaphoreGive(WheelDriver::mutex);
  }
  mainType = inputMain; moduleType = inputModule;
  // UI/config fields remain in the command context. The task uses its own copy.
  WHEEL_D = mainType == 3 ? .0523 : .0800;
  ONE_CIRCLE_PLUSES = mainType == 1 ? 2100 : mainType == 2 ? 1650 : 1092;
  TRACK_WIDTH = mainType == 1 ? .125 : mainType == 2 ? .172 : .141;
  SET_MOTOR_DIR = mainType != 1;
  screenLine_2 = mainType == 1 ? "RaspRover" : mainType == 2 ? "UGV02" : "UGV01";
  if (moduleType == 0) screenLine_2 += " Null";
  else if (moduleType == 1) screenLine_2 += " Arm";
  else if (moduleType == 2) screenLine_2 += " PT";
}
