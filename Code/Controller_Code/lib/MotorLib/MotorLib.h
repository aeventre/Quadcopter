#pragma once

#include <Arduino.h>
#include <Servo.h>

class MotorController {
public:
  static constexpr size_t MAX_MOTORS = 8;

  // minUs / maxUs are pulse widths in microseconds, usually 1000–2000
  MotorController(int minUs = 1000, int maxUs = 2000);

  // Initialize with array of pins and number of motors.
  // Example:
  //   uint8_t pins[4] = {6, 7, 8, 9};
  //   motors.begin(pins, 4);
  void begin(const uint8_t *pins, size_t count);

  // Detach all servos (motors stop receiving PWM).
  void stop();

  // Set throttle as 0.0–1.0 (0% to 100%).
  // Values outside this range are clamped.
  void setThrottle(size_t motorIndex, float throttle);

  // Set raw pulse width in microseconds (e.g. 1000–2000).
  void setPulseUs(size_t motorIndex, int pulseUs);

  // Set same throttle for all motors (0.0–1.0).
  void setAllThrottle(float throttle);

  // Raw motor count
  size_t motorCount() const { return motor_count_; }

  int minPulseUs() const { return min_us_; }
  int maxPulseUs() const { return max_us_; }

private:
  Servo   servos_[MAX_MOTORS];
  uint8_t pins_[MAX_MOTORS];
  size_t  motor_count_;

  int min_us_;
  int max_us_;
};
