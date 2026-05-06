#pragma once
#include <Arduino.h>
#include <SPI.h>

// Simple container for IMU readings
struct ImuData {
  float gx, gy, gz;  // deg/s
  float ax, ay, az;  // g
};

class ImuLSM6 {
public:
  ImuLSM6(SPIClass &spi, uint8_t csPin);

  // Initialize IMU; returns true if WHO_AM_I matches expected
  bool begin();

  // Read latest gyro + accel into struct
  void read(ImuData &d);

private:
  SPIClass *spi_;
  uint8_t cs_;
  SPISettings settings_;

  uint8_t readReg(uint8_t reg);
  void writeReg(uint8_t reg, uint8_t val);
};
