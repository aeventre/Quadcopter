#pragma once

#include "CRSFforArduino.hpp"
#include "ImuLSM6.h"
#include "MotorLib.h"
#include <Arduino.h>

// ===== Flight mode =====
enum FlightMode
{
    MODE_ACRO = 0,   // pure rate mode
    MODE_BALANCE = 1 // angle (self-level) mode
};

// ===== Main controller class =====
class DroneController
{
  public:
    // Rate PID structure
    struct RatePID
    {
        float kp;
        float ki;
        float kd;
        float iTerm;
        float lastErr;
    };

    // Log sample structure for RAM blackbox
    struct LogSample
    {
        uint32_t tMicros;

        int16_t rollUs;
        int16_t pitchUs;
        int16_t thrUs;
        int16_t yawUs;

        int16_t p_cmd;
        int16_t q_cmd;
        int16_t r_cmd;

        int16_t p_rate;
        int16_t q_rate;
        int16_t r_rate;

        int16_t rollOut;
        int16_t pitchOut;
        int16_t yawOut;

        uint8_t m0;
        uint8_t m1;
        uint8_t m2;
        uint8_t m3;

        int16_t rollAngle;
        int16_t pitchAngle;
    };

    enum ArmState
    {
        DISARMED = 0,
        ARMED = 1
    };

    // ===== Constructor =====
    DroneController(MotorController &motorsRef, ImuLSM6 &imuRef);

    // Set CRSF object pointer
    void setCrsf(CRSFforArduino *crsfPtr);

    // Optionally override rate PID gains at runtime
    void setRatePIDs(float rollKp, float rollKi, float rollKd, float pitchKp, float pitchKi, float pitchKd, float yawKp,
                     float yawKi, float yawKd);

    // Called from RC callback
    void handleRcUpdate(const serialReceiverLayer::rcChannels_t &rc);

    // Called repeatedly from main loop
    void update();

  private:
    // ===== Config constants (same behavior as working code) =====
    static constexpr float MAX_THROTTLE_CMD = 0.50f;
    static constexpr float ARM_THROTTLE_MAX = 0.15f; // throttle must be below this to arm

    static constexpr float GYRO_LP_ALPHA = 0.7f;
    static constexpr float RATE_DEADBAND = 2.0f;
    static constexpr float ATT_ALPHA = 0.98f;

    // Inner rate-loop PID gains (tuned)
    float ROLL_KP = 0.035f;
    float ROLL_KI = 0.01f;
    float ROLL_KD = 0.005f;

    float PITCH_KP = 0.075f;
    float PITCH_KI = 0.01f;
    float PITCH_KD = 0.0045f;

    float YAW_KP = 0.14f;
    float YAW_KI = 0.03f;
    float YAW_KD = 0.0f;

    // Stick→rate and rate limits
    static constexpr float MAX_CMD_ROLL_RATE = 120.0f; // deg/s
    static constexpr float MAX_CMD_PITCH_RATE = 120.0f;
    static constexpr float MAX_CMD_YAW_RATE = 100.0f;

    static constexpr float MAX_ROLL_RATE = 200.0f;
    static constexpr float MAX_PITCH_RATE = 200.0f;
    static constexpr float MAX_YAW_RATE = 150.0f;

    // Outer-loop (angle-mode) params
    static constexpr float MAX_ROLL_ANGLE_DEG = 35.0f;
    static constexpr float MAX_PITCH_ANGLE_DEG = 35.0f;

    // Outer (angle) loop gains
    static constexpr float ROLL_ANGLE_KP = 7.0f;
    static constexpr float PITCH_ANGLE_KP = 7.0f;

    // Start with small I so it only trims slow bias
    static constexpr float ROLL_ANGLE_KI = 0.6f;  // [1/s]
    static constexpr float PITCH_ANGLE_KI = 0.6f; // [1/s]

    // Clamp on integrator state (deg·s); KI * INT_MAX ≈ max added rate (deg/s)
    static constexpr float ANGLE_INT_MAX = 8.0f; // => up to ~30 deg/s of I term

    // ===== Helpers =====
    static float usToThrottle(int us);
    static float usToCentered(int us);
    static float clampf(float v, float lo, float hi);

    static void resetPid(RatePID &pid);
    static float pidUpdate(RatePID &pid, float err, float dt);

    void logSample(uint32_t tMicros, int rollUs, int pitchUs, int thrUs, int yawUs, float p_cmd, float q_cmd,
                   float r_cmd, float p_rate, float q_rate, float r_rate, float rollOut, float pitchOut, float yawOut,
                   float m0, float m1, float m2, float m3, float rollAngleDeg, float pitchAngleDeg);

    void dumpLogToSerial();

    // ===== Members =====
    MotorController &_motors;
    ImuLSM6 &_imu;
    CRSFforArduino *_crsf;

    FlightMode _flightMode = MODE_ACRO;

    ArmState _armState = DISARMED;

    bool _rcHaveFrame = false;
    bool _rcFailsafe = true;
    serialReceiverLayer::rcChannels_t _rc{};

    // Outer-loop integrators (for balance mode)
    float _rollAngleInt = 0.0f;  // deg·s
    float _pitchAngleInt = 0.0f; // deg·s

    uint32_t _lastMicros = 0;

    // Filtered gyro rates
    float _p_f = 0.0f;
    float _q_f = 0.0f;
    float _r_f = 0.0f;

    // Attitude estimate
    bool _attInitialized = false;
    float _rollAngleDeg = 0.0f;
    float _pitchAngleDeg = 0.0f;

    // Rate PID state
    RatePID _pidRollRate;
    RatePID _pidPitchRate;
    RatePID _pidYawRate;

    // RAM blackbox
    static constexpr size_t LOG_CAPACITY = 3000;
    LogSample _logBuf[LOG_CAPACITY];
    size_t _logIndex = 0;
    bool _loggingEnabled = true;
};