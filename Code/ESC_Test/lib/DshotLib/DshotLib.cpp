#include "DshotLib.h"

// Static instance pointer for core1 trampoline
DshotController* DshotController::s_instance_ = nullptr;

// Snapshot struct to copy commands atomically once per frame on core1
struct CmdSnapshot {
  uint16_t thr[DshotController::MAX_MOTORS];
  bool     tlm[DshotController::MAX_MOTORS];
};

DshotController::DshotController(uint32_t sendRateHz)
: send_rate_hz_(sendRateHz) {}

void DshotController::begin(const uint *pins, size_t count) {
  if (count == 0 || count > MAX_MOTORS) return;

  motor_count_ = count;
  for (size_t i = 0; i < count; ++i) {
    pins_[i] = pins[i];
    gpio_init(pins_[i]);
    gpio_set_dir(pins_[i], GPIO_OUT);
    gpio_put(pins_[i], 0);
    throttle_[i]  = 100;     // idle-ish start
    telemetry_[i] = false;
  }

  calcTiming600();

  running_    = true;
  s_instance_ = this;
  multicore_launch_core1(&DshotController::core1Entry);
}

void DshotController::stop() {
  running_ = false;
  // Allow core1 loop to exit, then reset it.
  delay(10);
  multicore_reset_core1();
  s_instance_ = nullptr;
}

void DshotController::setThrottle(size_t motorIndex, uint16_t thr) {
  if (motorIndex >= motor_count_) return;
  if (thr < DSHOT_MIN) thr = DSHOT_MIN;
  if (thr > DSHOT_MAX) thr = DSHOT_MAX;
  throttle_[motorIndex] = thr;
}

void DshotController::setTelemetry(size_t motorIndex, bool enable) {
  if (motorIndex >= motor_count_) return;
  telemetry_[motorIndex] = enable;
}

void DshotController::calcTiming600() {
  // DSHOT600 -> 600 kbit/s -> bit time ~ 1.666... us.
  // Compute from actual clk_sys to keep exact cycle counts.
  uint32_t sys_hz = clock_get_hz(clk_sys);     // e.g., 133,000,000
  uint32_t T      = sys_hz / 600000u;          // ~222 cycles/bit at 133 MHz
  if (T < 25) T = 25;

  uint32_t hi1  = (T * 3u) / 4u;               // '1' high ~75%
  uint32_t hi0  = (T * 3u) / 8u;               // '0' high ~37.5%
  if (!hi1) hi1 = 1;
  if (!hi0) hi0 = 1;

  t_.tbit    = T;
  t_.one_hi  = hi1;           t_.one_lo  = T - hi1;
  t_.zero_hi = hi0;           t_.zero_lo = T - hi0;
}

uint16_t DshotController::makeFrame(uint16_t throttle, bool telemetry) const {
  if (throttle < DSHOT_MIN)  throttle = DSHOT_MIN;
  if (throttle > DSHOT_MAX)  throttle = DSHOT_MAX;

  uint16_t payload = static_cast<uint16_t>((throttle << 1) | (telemetry ? 1u : 0u));
  uint8_t  csum    = 0;
  csum ^= (payload >> 0) & 0xF;
  csum ^= (payload >> 4) & 0xF;
  csum ^= (payload >> 8) & 0xF;

  return static_cast<uint16_t>((payload << 4) | (csum & 0xF));
}

void DshotController::sendFrameOnPin(uint pin, uint16_t frame) const {
  // Atomic ~27us per motor @ DSHOT600
  uint32_t irq = save_and_disable_interrupts();
  for (int i = 15; i >= 0; --i) {
    bool bit = (frame >> i) & 1;
    gpio_put(pin, 1);
    busy_wait_at_least_cycles(bit ? t_.one_hi  : t_.zero_hi);
    gpio_put(pin, 0);
    busy_wait_at_least_cycles(bit ? t_.one_lo  : t_.zero_lo);
  }
  // Small inter-frame gap to be safe
  busy_wait_at_least_cycles(t_.tbit * 2u);
  restore_interrupts(irq);
}

void DshotController::core1Entry() {
  if (s_instance_) s_instance_->core1Loop();
}

void DshotController::core1Loop() {
  // Prime ESCs so they latch DSHOT mode (send idle frames first)
  absolute_time_t next = make_timeout_time_us(1000000u / send_rate_hz_);
  for (int k = 0; k < 400 && running_; ++k) {
    for (size_t m = 0; m < motor_count_; ++m) {
      uint16_t idleFrame = makeFrame(DSHOT_MIN, false);
      sendFrameOnPin(pins_[m], idleFrame);
    }
    sleep_until(next);
    next = delayed_by_us(next, 1000000u / send_rate_hz_);
  }

  // Steady stream at send_rate_hz_
  next = get_absolute_time();
  while (running_) {
    // --- Snapshot all motor commands atomically relative to core1 ---
    CmdSnapshot snap;

    uint32_t irq = save_and_disable_interrupts();
    size_t   n   = motor_count_;
    for (size_t m = 0; m < n; ++m) {
      snap.thr[m] = throttle_[m];
      snap.tlm[m] = telemetry_[m];
    }
    restore_interrupts(irq);

    // --- Send using the snapshot so a whole frame is consistent ---
    for (size_t m = 0; m < n; ++m) {
      sendFrameOnPin(pins_[m], makeFrame(snap.thr[m], snap.tlm[m]));
    }

    next = delayed_by_us(next, 1000000u / send_rate_hz_);
    sleep_until(next);
  }
}
