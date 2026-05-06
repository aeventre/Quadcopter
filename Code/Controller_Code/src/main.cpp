#include <Arduino.h>
#include <SPI.h>

#include "CRSFforArduino.hpp"
#include "DroneController.h"
#include "ImuLSM6.h"
#include "MotorLib.h"

// ======================= Motor / ESC setup =======================
uint8_t motorPins[4] = {6, 7, 8, 9};
MotorController motors(1000, 2000); // 1000–2000 us

// ======================= IMU object =======================
static const uint8_t IMU_CS_PIN = 13;
ImuLSM6 imu(SPI1, IMU_CS_PIN);

// ======================= CRSF / ELRS =======================
CRSFforArduino *crsf = nullptr;

// ======================= Controller =======================
DroneController drone(motors, imu);

// Forward declaration for CRSF callback
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000)
    {
    }
    pinMode(22, OUTPUT);
    digitalWrite(22, LOW);

    Serial.println("=== Drone main (rate + angle outer loop + RAM blackbox) ===");

    // ---- ESC init ----
    motors.begin(motorPins, 4);
    Serial.println("ESC init: sending 1000us (0% throttle) for 3s...");
    motors.setAllThrottle(0.0f); // -> 1000us
    delay(3000);

    // ---- IMU SPI init ----
    SPI1.setSCK(10); // SCK
    SPI1.setTX(11);  // MOSI
    SPI1.setRX(12);  // MISO
    SPI1.begin();

    imu.begin();
    Serial.println("IMU init done.");

    // ---- CRSF / ELRS init ----
    Serial2.setRX(21);
    Serial2.setTX(20);

    crsf = new CRSFforArduino(&Serial2);
    if (!crsf)
    {
        Serial.println("[ERROR] Failed to allocate CRSF");
        while (1)
        {
            delay(1000);
        }
    }

    if (!crsf->begin())
    {
        Serial.println("[ERROR] crsf->begin() FAILED");
        while (1)
        {
            delay(1000);
        }
    }

    drone.setCrsf(crsf);

    // RC callback forwards into DroneController
    crsf->setRcChannelsCallback(onReceiveRcChannels);

    Serial.println("Setup complete. Waiting for RC frames...");
}

void loop()
{
    drone.update();
}

// RC callback just forwards into DroneController
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    drone.handleRcUpdate(*rcChannels);
}
