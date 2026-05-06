#include <Arduino.h>
#include "MotorLib.h"

// ESC signal pins
uint8_t motorPins[4] = {6, 7, 8, 9};

// Use full 1000–2000us, we’ll control where we sit
MotorController motors(1000, 2000);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial.println("MotorLib PWM test with safe startup...");

  // 1) Let the ESC power up before we even attach (optional but helps
  //    if ESC and Pico power at exactly the same time).
  delay(1000);

  // 2) Attach motors and send 1000us (init range) for a few seconds.
  motors.begin(motorPins, 4);

  Serial.println("Init: 1000us for 3s (ESC sees min throttle)...");
  motors.setAllThrottle(0.0f);   // -> 1000us exactly
  delay(3000);

  // 3) Move slightly into Start Signal range (>1050us) for arming spin.
  Serial.println("Arming: low spin at ~1100us (10%) for 5s...");
  motors.setAllThrottle(0.1f);   // ~1100us
  delay(5000);

  // 4) Hold high throttle for a while.
  Serial.println("High throttle: ~70% (~1700us) for 20s...");
  motors.setAllThrottle(0.7f);   // ~1700us
  delay(20000);

  // 5) Go back to a low spin instead of full off (so they don’t stop).
  Serial.println("Back to low spin (~1100us)...");
  motors.setAllThrottle(0.1f);   // ~1100us
}

void loop() {
  // No-op in this test
}
