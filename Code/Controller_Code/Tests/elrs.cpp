#include "Arduino.h"
#include "CRSFforArduino.hpp"

CRSFforArduino *crsf = nullptr;

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    Serial.println("=== Pico CRSF DEBUG START ===");
    delay(200);


    Serial2.setRX(21);
    Serial2.setTX(20);

    crsf = new CRSFforArduino(&Serial2);
    if (!crsf) {
        Serial.println("[ERROR] Failed to allocate CRSF object!");
        while (1) {
            Serial.println("[HALT] Out of memory?");
            delay(1000);
        }
    }


    bool ok = crsf->begin();

    if (!ok) {
        Serial.println("[ERROR] crsf->begin() FAILED");
        delete crsf;
        crsf = nullptr;
        while (1) {
            Serial.println("[HALT] CRSF failed to initialize.");
            delay(1000);
        }
    }

    crsf->setRcChannelsCallback(onReceiveRcChannels);

}

void loop()
{
    if (crsf != nullptr) {
        crsf->update();
    }

    static uint32_t lastBeat = 0;
    if (millis() - lastBeat > 2000) {
        lastBeat = millis();
        Serial.println("[DEBUG] Loop heartbeat");
    }
}

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 100) {
        return; // only print every 100 ms
    }
    lastPrint = millis();

    static bool initialised = false;
    static bool lastFailSafe = false;
    if (rcChannels->failsafe != lastFailSafe || !initialised)
    {
        initialised = true;
        lastFailSafe = rcChannels->failsafe;
        Serial.print("FailSafe: ");
        Serial.println(lastFailSafe ? "Active" : "Inactive");
    }

    if (!rcChannels->failsafe)
    {
        Serial.print("RC Channels <A: ");
        Serial.print(crsf->rcToUs(rcChannels->value[0]));
        Serial.print(", E: ");
        Serial.print(crsf->rcToUs(rcChannels->value[1]));
        Serial.print(", T: ");
        Serial.print(crsf->rcToUs(rcChannels->value[2]));
        Serial.print(", R: ");
        Serial.print(crsf->rcToUs(rcChannels->value[3]));
        Serial.print(", Aux1: ");
        Serial.print(crsf->rcToUs(rcChannels->value[4]));
        Serial.print(", Aux2: ");
        Serial.print(crsf->rcToUs(rcChannels->value[5]));
        Serial.print(", Aux3: ");
        Serial.print(crsf->rcToUs(rcChannels->value[6]));
        Serial.print(", Aux4: ");
        Serial.print(crsf->rcToUs(rcChannels->value[7]));
        Serial.println(">");
    }
}

