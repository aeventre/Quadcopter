#pragma once

#include <Arduino.h>
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/time.h"

class DshotController {
public:
  static constexpr size_t   MAX_MOTORS = 8;     // supports up to 8 motors
  static constexpr uint16_t DSHOT_MIN  = 48;    // avoid command range 0..47
  static constexpr uint16_t DSHOT_MAX  = 2047;

  // Create controller for a given frame rate (Hz). Protocol is DSHOT600.
  explicit DshotController(uint32_t sendRateHz = 2000);

  // Initialize with an array of GPIO pins and motor count, then launch core1 loop.
  // Example:
  //   uint pins[4] = {10, 11, 12, 13};
  //   dshot.begin(pins, 4);
  void begin(const uint *pins, size_t count);

  // Gracefully stop the sender and reset core1.
  void stop();

  // Set throttle for a motor (DSHOT values 48..2047). Values are clamped.
  void setThrottle(size_t motorIndex, uint16_t throttle);

  // Enable/disable telemetry bit for a motor.
  void setTelemetry(size_t motorIndex, bool enable);

  // Number of configured motors.
  size_t motorCount() const { return motor_count_; }

  // Frame rate (Hz).
  uint32_t sendRateHz() const { return send_rate_hz_; }

private:
  struct DshotTiming {
    uint32_t tbit;     // cycles per bit
    uint32_t one_hi;   // cycles for '1' high
    uint32_t one_lo;   // cycles for '1' low
    uint32_t zero_hi;  // cycles for '0' high
    uint32_t zero_lo;  // cycles for '0' low
  };

  // --- timing & encoding ---
  void     calcTiming600();
  uint16_t makeFrame(uint16_t throttle, bool telemetry) const;
  void     sendFrameOnPin(uint pin, uint16_t frame) const;

  // --- core1 driver ---
  static void core1Entry();     // trampoline for multicore
  void        core1Loop();      // instance method doing the work

  // --- singleton instance pointer for core1 ---
  static DshotController* s_instance_;

  // --- configuration & state ---
  uint32_t send_rate_hz_;
  size_t   motor_count_ = 0;

  uint pins_[MAX_MOTORS] = {0};

  volatile uint16_t throttle_[MAX_MOTORS]  = {0};
  volatile bool     telemetry_[MAX_MOTORS] = {false};

  volatile bool running_ = false;

  DshotTiming t_;   // computed timing values
};
