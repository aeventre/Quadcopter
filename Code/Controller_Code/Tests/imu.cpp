#include <Arduino.h>
#include <SPI.h>

// ==== Pin + register defs ====
#define CS_PIN 13 // GPIO13 → IMU CS

// LSM6DSO32 registers
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define OUTX_L_G 0x22 // start of gyro data (6 bytes)
#define OUTX_L_A 0x28 // start of accel data (6 bytes)

// Expected WHO_AM_I value
#define WHO_VAL 0x6C

// ---- Sensitivities ----
// Treat accel as ±2g for scaling:
// ±2g : 0.061 mg/LSB → 0.000061 g/LSB
const float ACC_SENS = 0.000244f; // g/LSB

// Gyro: ±250 dps → 8.75 mdps/LSB → 0.00875 dps/LSB
const float GYRO_SENS_250 = 0.00875f; // dps/LSB

// One SPI config for all IMU transactions
SPISettings imuSPI(1000000, MSBFIRST, SPI_MODE3);

// ==== Low-level SPI helpers ====
uint8_t imuReadReg(uint8_t reg)
{
    uint8_t v;

    digitalWrite(CS_PIN, LOW);
    SPI1.beginTransaction(imuSPI);
    SPI1.transfer(reg | 0x80); // MSB=1 → read
    v = SPI1.transfer(0x00);
    SPI1.endTransaction();
    digitalWrite(CS_PIN, HIGH);

    return v;
}

void imuWriteReg(uint8_t reg, uint8_t val)
{
    digitalWrite(CS_PIN, LOW);
    SPI1.beginTransaction(imuSPI);
    SPI1.transfer(reg & 0x7F); // MSB=0 → write
    SPI1.transfer(val);
    SPI1.endTransaction();
    digitalWrite(CS_PIN, HIGH);
}

// ==== High-level IMU helpers ====

// Burst-read gyro + accel in one go (12 bytes)
void imuReadGyroAccel(float &gx, float &gy, float &gz, float &ax, float &ay, float &az)
{
    uint8_t buf[12];

    digitalWrite(CS_PIN, LOW);
    SPI1.beginTransaction(imuSPI);
    // Start at OUTX_L_G, auto-increment will step through all 6 gyro + 6 accel bytes
    SPI1.transfer(OUTX_L_G | 0x80);

    for (int i = 0; i < 12; i++)
    {
        buf[i] = SPI1.transfer(0x00);
    }

    SPI1.endTransaction();
    digitalWrite(CS_PIN, HIGH);

    // Raw little-endian 16-bit values
    int16_t gx_raw = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy_raw = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz_raw = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t ax_raw = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t ay_raw = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t az_raw = (int16_t)((buf[11] << 8) | buf[10]);

    // Convert to physical units
    gx = gx_raw * GYRO_SENS_250;
    gy = gy_raw * GYRO_SENS_250;
    gz = gz_raw * GYRO_SENS_250;

    ax = ax_raw * ACC_SENS;
    ay = ay_raw * ACC_SENS;
    az = az_raw * ACC_SENS;
}

void imuInit()
{
    // CTRL3_C: BDU=1 (block data update), IF_INC=1 (auto-increment)
    // 0b0100_0100 = 0x44
    imuWriteReg(CTRL3_C, 0x44);
    delay(50); // give it a moment to latch

    // CTRL1_XL: ODR_XL = 104 Hz (0100 << 4 = 0x40)
    // We'll *ask* for ±4g (0x48), but even if FS bits don't stick,
    // our scale constant is now based on what we actually observe.
    imuWriteReg(CTRL1_XL, 0x48);

    // CTRL2_G (gyro):
    // ODR_G  = 104 Hz → 0b0100 << 4 = 0x40
    // FS_G   = 250 dps (00 in bits [3:2])
    // Final: 0x40
    imuWriteReg(CTRL2_G, 0x40);

    delay(20);

    // Read back to confirm (for debugging / curiosity)
    uint8_t c1 = imuReadReg(CTRL1_XL);
    uint8_t c2 = imuReadReg(CTRL2_G);
    uint8_t c3 = imuReadReg(CTRL3_C);

    Serial.print("CTRL1_XL = 0x");
    Serial.println(c1, HEX);
    Serial.print("CTRL2_G  = 0x");
    Serial.println(c2, HEX);
    Serial.print("CTRL3_C  = 0x");
    Serial.println(c3, HEX);
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== LSM6DSO32 SPI Gyro+Accel Test (2g scaling) ===");

    // Configure SPI1 pins to match your PCB routing
    SPI1.setSCK(10); // GPIO10 → SCK
    SPI1.setTX(11);  // GPIO11 → MOSI
    SPI1.setRX(12);  // GPIO12 → MISO
    SPI1.begin();

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    // WHO_AM_I sanity check
    uint8_t who = imuReadReg(WHO_AM_I);
    Serial.print("WHO_AM_I = 0x");
    Serial.println(who, HEX);

    if (who != WHO_VAL)
    {
        Serial.println("WARNING: IMU WHO_AM_I mismatch (expected 0x6C).");
    }

    imuInit();
    Serial.println("IMU init done (scaled as ±2g, ±250 dps @ 104 Hz).");
}

void loop()
{
    float gx, gy, gz;
    float ax, ay, az;

    imuReadGyroAccel(gx, gy, gz, ax, ay, az);

    Serial.print("ACC [g]   ");
    Serial.print(ax, 3);
    Serial.print("   ");
    Serial.print(ay, 3);
    Serial.print("   ");
    Serial.print(az, 3);
    Serial.print("   |   ");

    Serial.print("GYR [dps] ");
    Serial.print(gx, 2);
    Serial.print("   ");
    Serial.print(gy, 2);
    Serial.print("   ");
    Serial.print(gz, 2);
    Serial.println();

    delay(50); // ~20 Hz print
}
