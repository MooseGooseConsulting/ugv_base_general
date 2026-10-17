#pragma once

#include <cmath>
#include <stdint.h>

// Pure wheel-command policy. The adapter serializes access; no hardware or RTOS.
namespace WheelControl {
enum class Mode { Stopped, Pwm, Speed, Brake };
struct Desired {
  Mode mode = Mode::Stopped;
  int type = 1;
  float left = 0, right = 0;
  float scaleLeft = 1, scaleRight = 1;
  float kp = 20, ki = 2000, kd = 0, limits = 255;
  uint32_t acceptedAt = 0, timeout = 500, resetEpoch = 0;
};

class State {
 public:
  State(uint32_t timeout, float kp, float ki, float kd, float limits) {
    desired.timeout = timeout;
    desired.kp = kp; desired.ki = ki; desired.kd = kd; desired.limits = limits;
  }
  const Desired& get() const { return desired; }
  static float pwm(double value) {
    return value > 255 ? 255 : value < -255 ? -255 : static_cast<float>(value);
  }
  bool expire(uint32_t now) {
    if ((desired.mode == Mode::Speed || desired.mode == Mode::Pwm) &&
        static_cast<uint32_t>(now - desired.acceptedAt) > desired.timeout) {
      desired.mode = desired.type == 3 ? Mode::Brake : Mode::Stopped;
      desired.left = desired.right = 0;
      ++desired.resetEpoch;
      return true;
    }
    return false;
  }
  void stop() {
    desired.mode = Mode::Stopped;
    desired.left = desired.right = 0;
    ++desired.resetEpoch;
  }
  bool type(int value, uint32_t now) {
    if (value < 1 || value > 3) return false;
    expire(now);
    desired.type = value;
    stop(); // Reconfiguration never reinterprets an old moving request.
    return true;
  }
  bool timeout(int value, uint32_t now) {
    if (value <= 0) return false;
    expire(now); // Increasing the timeout cannot revive already-expired motion.
    desired.timeout = static_cast<uint32_t>(value);
    return true;
  }
  bool scale(float left, float right, uint32_t now) {
    if (!finite(left, right)) return false;
    expire(now);
    desired.scaleLeft = std::fmin(1.0f, std::fabs(left));
    desired.scaleRight = std::fmin(1.0f, std::fabs(right));
    return true;
  }
  bool gains(float kp, float ki, float kd, float limits, uint32_t now) {
    if (!finite(kp, ki) || !finite(kd, limits) || kp < 0 || ki < 0 || kd < 0) return false;
    expire(now);
    desired.kp = kp; desired.ki = ki; desired.kd = kd;
    desired.limits = limits; // Retain legacy stored value; PID output limits stay +/-255.
    return true;
  }
  bool speed(float left, float right, uint32_t now) {
    if (!finite(left, right) || (desired.type == 3 &&
        (left < -2 || left > 2 || right < -2 || right > 2))) return false;
    if (desired.type == 3)
      return accept(Mode::Speed, left * desired.scaleLeft, right * desired.scaleRight, now);
    // Validate the scaled PWM in double precision before conversion or acceptance.
    // Legacy normalized endpoints +/-0.5 map to +/-256; admit those
    // endpoints and clamp to the 8-bit duty limit before acceptance.
    // Larger requests must not renew the previous motion deadline.
    const double leftPwm = static_cast<double>(left) * 512 * desired.scaleLeft;
    const double rightPwm = static_cast<double>(right) * 512 * desired.scaleRight;
    if (!std::isfinite(leftPwm) || !std::isfinite(rightPwm) ||
        leftPwm < -256 || leftPwm > 256 || rightPwm < -256 || rightPwm > 256) return false;
    return accept(Mode::Pwm, pwm(leftPwm), pwm(rightPwm), now);
  }
  bool ros(float x, float z, uint32_t now) {
    if (!finite(x, z)) return false;
    const float track = desired.type == 1 ? .125f : desired.type == 2 ? .172f : .141f;
    return speed(x - z * track / 2, x + z * track / 2, now);
  }
  bool raw(float left, float right, uint32_t now) {
    if (!finite(left, right) || left < -255 || left > 255 || right < -255 || right > 255) return false;
    return accept(Mode::Pwm, left, right, now);
  }
 private:
  Desired desired;
  static bool finite(float left, float right) { return std::isfinite(left) && std::isfinite(right); }
  bool accept(Mode mode, float left, float right, uint32_t now) {
    if (!finite(left, right)) return false;
    expire(now); // Preserve expiry/reset even if fresh motion arrives between samples.
    if (desired.mode != mode) ++desired.resetEpoch;
    desired.mode = mode; desired.left = left; desired.right = right;
    desired.acceptedAt = now;
    return true;
  }
};
}
