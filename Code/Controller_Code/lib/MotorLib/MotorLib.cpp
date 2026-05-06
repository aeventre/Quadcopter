#include "MotorLib.h"

MotorController::MotorController(int minUs, int maxUs)
: motor_count_(0),
  min_us_(minUs),
  max_us_(maxUs) {
}

void MotorController::begin(const uint8_t *pins, size_t count) {
  if (!pins || count == 0) return;
  if (count > MAX_MOTORS) count = MAX_MOTORS;

  motor_count_ = count;

  for (size_t i = 0; i < motor_count_; ++i) {
    pins_[i] = pins[i];
    // Attach with explicit min/max pulse widths
    servos_[i].attach(pins_[i], min_us_, max_us_);
    // Start at min pulse (motors off / idle)
    servos_[i].writeMicroseconds(min_us_);
  }
}

void MotorController::stop() {
  for (size_t i = 0; i < motor_count_; ++i) {
    servos_[i].detach();
  }
  motor_count_ = 0;
}

void MotorController::setThrottle(size_t motorIndex, float throttle) {
  if (motorIndex >= motor_count_) return;

  if (throttle < 0.0f) throttle = 0.0f;
  if (throttle > 1.0f) throttle = 1.0f;

  int span    = max_us_ - min_us_;
  int pulseUs = min_us_ + static_cast<int>(throttle * span);

  servos_[motorIndex].writeMicroseconds(pulseUs);
}

void MotorController::setPulseUs(size_t motorIndex, int pulseUs) {
  if (motorIndex >= motor_count_) return;

  if (pulseUs < min_us_) pulseUs = min_us_;
  if (pulseUs > max_us_) pulseUs = max_us_;

  servos_[motorIndex].writeMicroseconds(pulseUs);
}

void MotorController::setAllThrottle(float throttle) {
  if (throttle < 0.0f) throttle = 0.0f;
  if (throttle > 1.0f) throttle = 1.0f;

  int span = max_us_ - min_us_;
  int pulseUs = min_us_ + static_cast<int>(throttle * span);

  for (size_t i = 0; i < motor_count_; ++i) {
    servos_[i].writeMicroseconds(pulseUs);
  }
}
