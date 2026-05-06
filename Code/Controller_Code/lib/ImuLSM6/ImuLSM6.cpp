#include "ImuLSM6.h"

// LSM6DSO32 registers
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G  0x11
#define CTRL3_C  0x12
#define OUTX_L_G 0x22 // gyro start
#define OUTX_L_A 0x28 // accel start

#define WHO_VAL  0x6C

// Sensitivities (same as before)
// Accel: treat as ±2g → ~0.000244 g/LSB
static const float ACC_SENS       = 0.000244f;  // g/LSB
// Gyro: ±250 dps → 0.00875 dps/LSB
static const float GYRO_SENS_250  = 0.00875f;   // dps/LSB

ImuLSM6::ImuLSM6(SPIClass &spi, uint8_t csPin)
: spi_(&spi),
  cs_(csPin),
  settings_(1000000, MSBFIRST, SPI_MODE3)
{
}

uint8_t ImuLSM6::readReg(uint8_t reg)
{
  uint8_t v;
  digitalWrite(cs_, LOW);
  spi_->beginTransaction(settings_);
  spi_->transfer(reg | 0x80); // read
  v = spi_->transfer(0x00);
  spi_->endTransaction();
  digitalWrite(cs_, HIGH);
  return v;
}

void ImuLSM6::writeReg(uint8_t reg, uint8_t val)
{
  digitalWrite(cs_, LOW);
  spi_->beginTransaction(settings_);
  spi_->transfer(reg & 0x7F); // write
  spi_->transfer(val);
  spi_->endTransaction();
  digitalWrite(cs_, HIGH);
}

bool ImuLSM6::begin()
{
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);

  uint8_t who = readReg(WHO_AM_I);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);
  if (who != WHO_VAL) {
    Serial.println("WARNING: IMU WHO_AM_I mismatch (expected 0x6C).");
  }

  // CTRL3_C: BDU=1, IF_INC=1 → 0x44
  writeReg(CTRL3_C, 0x44);
  delay(50);

  // CTRL1_XL: ODR=104 Hz, ±4g (0x48)
  writeReg(CTRL1_XL, 0x48);

  // CTRL2_G: ODR=104 Hz, ±250 dps (0x40)
  writeReg(CTRL2_G, 0x40);

  delay(20);

  uint8_t c1 = readReg(CTRL1_XL);
  uint8_t c2 = readReg(CTRL2_G);
  uint8_t c3 = readReg(CTRL3_C);

  Serial.print("CTRL1_XL = 0x"); Serial.println(c1, HEX);
  Serial.print("CTRL2_G  = 0x"); Serial.println(c2, HEX);
  Serial.print("CTRL3_C  = 0x"); Serial.println(c3, HEX);

  return (who == WHO_VAL);
}

void ImuLSM6::read(ImuData &d)
{
  uint8_t buf[12];

  digitalWrite(cs_, LOW);
  spi_->beginTransaction(settings_);
  spi_->transfer(OUTX_L_G | 0x80); // read from gyro start, auto-increment
  for (int i = 0; i < 12; i++) {
    buf[i] = spi_->transfer(0x00);
  }
  spi_->endTransaction();
  digitalWrite(cs_, HIGH);

  int16_t gx_raw = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t gy_raw = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t gz_raw = (int16_t)((buf[5] << 8) | buf[4]);
  int16_t ax_raw = (int16_t)((buf[7] << 8) | buf[6]);
  int16_t ay_raw = (int16_t)((buf[9] << 8) | buf[8]);
  int16_t az_raw = (int16_t)((buf[11] << 8) | buf[10]);

  d.gx = gx_raw * GYRO_SENS_250;
  d.gy = gy_raw * GYRO_SENS_250;
  d.gz = gz_raw * GYRO_SENS_250;

  d.ax = ax_raw * ACC_SENS;
  d.ay = ay_raw * ACC_SENS;
  d.az = az_raw * ACC_SENS;
}
