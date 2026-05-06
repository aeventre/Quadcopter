#include "DroneController.h"
#include <math.h>

// ========== ctor / config ==========

DroneController::DroneController(MotorController &motorsRef, ImuLSM6 &imuRef)
    : _motors(motorsRef), _imu(imuRef), _crsf(nullptr)
{
    // initialize PID structs with current gains
    _pidRollRate  = {ROLL_KP,  ROLL_KI,  ROLL_KD,  0.0f, 0.0f};
    _pidPitchRate = {PITCH_KP, PITCH_KI, PITCH_KD, 0.0f, 0.0f};
    _pidYawRate   = {YAW_KP,   YAW_KI,   YAW_KD,   0.0f, 0.0f};
}

void DroneController::setCrsf(CRSFforArduino *crsfPtr)
{
    _crsf = crsfPtr;
}

void DroneController::setRatePIDs(float rollKp, float rollKi, float rollKd,
                                  float pitchKp, float pitchKi, float pitchKd,
                                  float yawKp, float yawKi, float yawKd)
{
    ROLL_KP  = rollKp;
    ROLL_KI  = rollKi;
    ROLL_KD  = rollKd;
    PITCH_KP = pitchKp;
    PITCH_KI = pitchKi;
    PITCH_KD = pitchKd;
    YAW_KP   = yawKp;
    YAW_KI   = yawKi;
    YAW_KD   = yawKd;

    _pidRollRate.kp  = ROLL_KP;
    _pidRollRate.ki  = ROLL_KI;
    _pidRollRate.kd  = ROLL_KD;
    _pidPitchRate.kp = PITCH_KP;
    _pidPitchRate.ki = PITCH_KI;
    _pidPitchRate.kd = PITCH_KD;
    _pidYawRate.kp   = YAW_KP;
    _pidYawRate.ki   = YAW_KI;
    _pidYawRate.kd   = YAW_KD;

    Serial.println("[PID] Gains updated from main.");
}

void DroneController::handleRcUpdate(const serialReceiverLayer::rcChannels_t &rc)
{
    _rc         = rc;
    _rcFailsafe = rc.failsafe;
    _rcHaveFrame = true;
}

// ========== static helpers ==========

float DroneController::usToThrottle(int us)
{
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;

    float t = (us - 1000) / 1000.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t;
}

float DroneController::usToCentered(int us)
{
    float v = (us - 1500) / 500.0f;
    if (v < -1.0f) v = -1.0f;
    if (v > 1.0f)  v =  1.0f;
    return v;
}

float DroneController::clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void DroneController::resetPid(RatePID &pid)
{
    pid.iTerm   = 0.0f;
    pid.lastErr = 0.0f;
}

float DroneController::pidUpdate(RatePID &pid, float err, float dt)
{
    const float I_MAX   = 200.0f;
    const float OUT_MAX = 300.0f;

    // Integrator
    pid.iTerm += pid.ki * err * dt;
    if (pid.iTerm >  I_MAX) pid.iTerm =  I_MAX;
    if (pid.iTerm < -I_MAX) pid.iTerm = -I_MAX;

    // Derivative
    float dTerm = 0.0f;
    if (dt > 0.0f)
        dTerm = pid.kd * (err - pid.lastErr) / dt;
    pid.lastErr = err;

    // Output
    float out = pid.kp * err + pid.iTerm + dTerm;
    if (out >  OUT_MAX) out =  OUT_MAX;
    if (out < -OUT_MAX) out = -OUT_MAX;
    return out;
}

// ========== logging ==========

void DroneController::logSample(uint32_t tMicros,
                                int rollUs, int pitchUs, int thrUs, int yawUs,
                                float p_cmd, float q_cmd, float r_cmd,
                                float p_rate, float q_rate, float r_rate,
                                float rollOut, float pitchOut, float yawOut,
                                float m0, float m1, float m2, float m3,
                                float rollAngleDeg, float pitchAngleDeg)
{
    if (!_loggingEnabled)      return;
    if (_logIndex >= LOG_CAPACITY) return;

    LogSample &s = _logBuf[_logIndex++];

    s.tMicros = tMicros;

    s.rollUs  = (int16_t)rollUs;
    s.pitchUs = (int16_t)pitchUs;
    s.thrUs   = (int16_t)thrUs;
    s.yawUs   = (int16_t)yawUs;

    s.p_cmd = (int16_t)(p_cmd * 10.0f);
    s.q_cmd = (int16_t)(q_cmd * 10.0f);
    s.r_cmd = (int16_t)(r_cmd * 10.0f);

    s.p_rate = (int16_t)(p_rate * 10.0f);
    s.q_rate = (int16_t)(q_rate * 10.0f);
    s.r_rate = (int16_t)(r_rate * 10.0f);

    s.rollOut  = (int16_t)(rollOut  * 10.0f);
    s.pitchOut = (int16_t)(pitchOut * 10.0f);
    s.yawOut   = (int16_t)(yawOut   * 10.0f);

    s.m0 = (uint8_t)clampf(m0 * 255.0f, 0.0f, 255.0f);
    s.m1 = (uint8_t)clampf(m1 * 255.0f, 0.0f, 255.0f);
    s.m2 = (uint8_t)clampf(m2 * 255.0f, 0.0f, 255.0f);
    s.m3 = (uint8_t)clampf(m3 * 255.0f, 0.0f, 255.0f);

    s.rollAngle  = (int16_t)(rollAngleDeg  * 10.0f);
    s.pitchAngle = (int16_t)(pitchAngleDeg * 10.0f);
}

void DroneController::dumpLogToSerial()
{
    if (_logIndex == 0)
    {
        Serial.println("[LOG] No samples to dump.");
        return;
    }

    Serial.println("=== LOG START (CSV) ===");
    Serial.println(
        "t_us,rollUs,pitchUs,thrUs,yawUs,"
        "p_cmd,q_cmd,r_cmd,p_rate,q_rate,r_rate,"
        "rollOut,pitchOut,yawOut,"
        "m0,m1,m2,m3,rollDeg,pitchDeg"
    );

    for (size_t i = 0; i < _logIndex; ++i)
    {
        const LogSample &s = _logBuf[i];

        Serial.print(s.tMicros);  Serial.print(',');

        Serial.print(s.rollUs);   Serial.print(',');
        Serial.print(s.pitchUs);  Serial.print(',');
        Serial.print(s.thrUs);    Serial.print(',');
        Serial.print(s.yawUs);    Serial.print(',');

        Serial.print(s.p_cmd / 10.0f); Serial.print(',');
        Serial.print(s.q_cmd / 10.0f); Serial.print(',');
        Serial.print(s.r_cmd / 10.0f); Serial.print(',');

        Serial.print(s.p_rate / 10.0f); Serial.print(',');
        Serial.print(s.q_rate / 10.0f); Serial.print(',');
        Serial.print(s.r_rate / 10.0f); Serial.print(',');

        Serial.print(s.rollOut  / 10.0f); Serial.print(',');
        Serial.print(s.pitchOut / 10.0f); Serial.print(',');
        Serial.print(s.yawOut   / 10.0f); Serial.print(',');

        Serial.print((int)s.m0); Serial.print(',');
        Serial.print((int)s.m1); Serial.print(',');
        Serial.print((int)s.m2); Serial.print(',');
        Serial.print((int)s.m3); Serial.print(',');

        Serial.print(s.rollAngle  / 10.0f); Serial.print(',');
        Serial.println(s.pitchAngle / 10.0f);
    }

    Serial.println("=== LOG END ===");
    _logIndex = 0;
}

// ========== main control update ==========

void DroneController::update()
{
    // ===== dt calculation =====
    uint32_t nowMicros = micros();
    float dt;
    if (_lastMicros == 0)
    {
        dt = 0.005f;
    }
    else
    {
        dt = (nowMicros - _lastMicros) / 1e6f;
        if (dt <= 0.0f || dt > 0.05f)
            dt = 0.005f;
    }
    _lastMicros = nowMicros;

    if (_crsf != nullptr)
    {
        _crsf->update();
    }

    // If we haven't seen any RC frame yet, keep motors off
    if (!_rcHaveFrame)
    {
        _motors.setAllThrottle(0.0f);
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 500)
        {
            lastPrint = millis();
            Serial.println("[INFO] Waiting for first RC frame...");
        }
        delay(5);
        return;
    }

    // ===== RC inputs =====
    int rollUs  = _crsf->rcToUs(_rc.value[0]);
    int pitchUs = _crsf->rcToUs(_rc.value[1]);
    int thrUs   = _crsf->rcToUs(_rc.value[2]);
    int yawUs   = _crsf->rcToUs(_rc.value[3]);
    int armUs   = _crsf->rcToUs(_rc.value[4]); // AUX1 – arm

    int modeUs  = _crsf->rcToUs(_rc.value[5]); // AUX2 – flight mode (3-pos)
    // AUX3, unused currently
    int aux4Us = _crsf->rcToUs(_rc.value[7]); // AUX4
    int logUs   = _crsf->rcToUs(_rc.value[8]); // AUX5 – blackbox dump

    float throttleCmdRaw = usToThrottle(thrUs);

    if (aux4Us > 1500) {
    digitalWrite(22, HIGH);
} else {
    digitalWrite(22, LOW);
}


    // ===== Safe arming logic =====
    ArmState prevState = _armState;
    if (armUs > 1500 && throttleCmdRaw < ARM_THROTTLE_MAX && !_rcFailsafe)
    {
        _armState = ARMED;
    }
    else if (armUs < 1300)
    {
        _armState = DISARMED;
    }

    if (_armState != prevState)
    {
        Serial.print("[INFO] ");
        Serial.print(_armState == ARMED ? "ARMED" : "DISARMED");
        Serial.print(" (throttle=");
        Serial.print(throttleCmdRaw, 2);
        Serial.println(")");
    }

    // ===== Flight mode selection via AUX2 (3-position) =====
    FlightMode prevMode = _flightMode;

    if (modeUs < 1300)
    {
        _flightMode = MODE_ACRO;      // low → acro / rate
    }
    else if (modeUs < 1700)
    {
        _flightMode = MODE_BALANCE;   // mid → angle / balance
    }
    else
    {
        _flightMode = MODE_BALANCE;   // high → same as mid for now
        // later: third mode here if you want
    }

    if (_flightMode != prevMode)
    {
        Serial.print("[MODE] ");
        if (_flightMode == MODE_ACRO)
            Serial.println("ACRO");
        if (_flightMode == MODE_BALANCE)
            Serial.println("BALANCE (angle)");
    }

    // ===== Blackbox dump via AUX5 when DISARMED =====
    if (_armState == DISARMED && !_rcFailsafe && logUs > 1500 && _logIndex > 0)
    {
        Serial.println("[LOG] Dump requested via AUX5.");
        dumpLogToSerial();
        delay(1000);
    }

    float throttleCmd = throttleCmdRaw;

    // If disarmed or failsafe, kill throttle and reset PIDs and attitude
    if (_armState == DISARMED || _rcFailsafe)
    {
        throttleCmd = 0.0f;
        resetPid(_pidRollRate);
        resetPid(_pidPitchRate);
        resetPid(_pidYawRate);
        _attInitialized = false;
        _rollAngleInt  = 0.0f;
        _pitchAngleInt = 0.0f;
    }

    // Apply hard max throttle cap
    if (throttleCmd > MAX_THROTTLE_CMD)
    {
        throttleCmd = MAX_THROTTLE_CMD;
    }

    // ===== Read IMU =====
    ImuData imuData;
    _imu.read(imuData);

    // Gyro: IMU Y = forward, IMU X = left, IMU Z = down
    // Body: Xfwd, Yright, Zdown
    float p = imuData.gy;  // roll rate about X_body
    float q = imuData.gx;  // pitch rate about Y_body
    float r = imuData.gz;  // yaw rate about Z_body

    // Low-pass filter the rates
    _p_f = GYRO_LP_ALPHA * _p_f + (1.0f - GYRO_LP_ALPHA) * p;
    _q_f = GYRO_LP_ALPHA * _q_f + (1.0f - GYRO_LP_ALPHA) * q;
    _r_f = GYRO_LP_ALPHA * _r_f + (1.0f - GYRO_LP_ALPHA) * r;

    // Deadband
    if (fabsf(_p_f) < RATE_DEADBAND) _p_f = 0.0f;
    if (fabsf(_q_f) < RATE_DEADBAND) _q_f = 0.0f;
    if (fabsf(_r_f) < RATE_DEADBAND) _r_f = 0.0f;

    // ===== Attitude estimate (for outer loop + logging) =====
    // Map accel to body frame: Xfwd, Yright, Zdown
    float aXb = imuData.ay;   // IMU +Y forward
    float aYb = -imuData.ax;  // IMU +X left  → body +Y right
    float aZb = -imuData.az;  // sign chosen to match your current working attitude

    float rollAccDeg  = atan2(aYb, aZb) * 180.0f / PI;
    float pitchAccDeg = atan2(-aXb, sqrt(aYb * aYb + aZb * aZb)) * 180.0f / PI;

    if (!_attInitialized)
    {
        _rollAngleDeg  = rollAccDeg;
        _pitchAngleDeg = pitchAccDeg;
        _attInitialized = true;
    }
    else
    {
        _rollAngleDeg  = ATT_ALPHA * (_rollAngleDeg  + _p_f * dt)
                       + (1.0f - ATT_ALPHA) * rollAccDeg;
        _pitchAngleDeg = ATT_ALPHA * (_pitchAngleDeg + _q_f * dt)
                       + (1.0f - ATT_ALPHA) * pitchAccDeg;
    }

    // ===== Sticks =====
    float rollStick  = usToCentered(rollUs);   // -1..+1
    float pitchStick = usToCentered(pitchUs);  // -1..+1
    float yawStick   = usToCentered(yawUs);    // -1..+1

    // small yaw deadband
    const float YAW_DB = 0.08f;   // ~8% stick
    if (fabsf(yawStick) < YAW_DB) yawStick = 0.0f;

    // ===== Inner-loop rate commands (with outer loop when in BALANCE) =====
    float p_cmd = 0.0f;
    float q_cmd = 0.0f;
    float r_cmd = 0.0f;

    float rollAngleCmdDeg  = 0.0f;
    float pitchAngleCmdDeg = 0.0f;

    if (_flightMode == MODE_ACRO)
    {
        // Pure rate mode (acro)
        p_cmd = rollStick  * MAX_CMD_ROLL_RATE;
        q_cmd = pitchStick * MAX_CMD_PITCH_RATE;
    }
    else // MODE_BALANCE
    {
        // Angle mode: stick -> angle -> rate
        rollAngleCmdDeg  = rollStick  * MAX_ROLL_ANGLE_DEG;
        pitchAngleCmdDeg = pitchStick * MAX_PITCH_ANGLE_DEG;

        float rollAngleErrDeg  = rollAngleCmdDeg  - _rollAngleDeg;
        float pitchAngleErrDeg = pitchAngleCmdDeg - _pitchAngleDeg;

        // ---- Outer-loop angle integrator (only in BALANCE mode) ----
        _rollAngleInt  += ROLL_ANGLE_KI  * rollAngleErrDeg  * dt;
        _pitchAngleInt += PITCH_ANGLE_KI * pitchAngleErrDeg * dt;

        // Clamp integrator to avoid huge commands
        _rollAngleInt  = clampf(_rollAngleInt,  -ANGLE_INT_MAX, ANGLE_INT_MAX);
        _pitchAngleInt = clampf(_pitchAngleInt, -ANGLE_INT_MAX, ANGLE_INT_MAX);

        // Stick movement & very low throttle: bleed the integrator instead of hard reset
        if (fabsf(rollStick) > 0.2f || fabsf(pitchStick) > 0.2f || throttleCmdRaw < 0.15f)
        {
            _rollAngleInt  *= 0.97f;  // slow decay
            _pitchAngleInt *= 0.97f;
        }

        // Outer loop output → rate command (deg/s)
        p_cmd = rollAngleErrDeg  * ROLL_ANGLE_KP  + _rollAngleInt;
        q_cmd = pitchAngleErrDeg * PITCH_ANGLE_KP + _pitchAngleInt;
    }

    // Yaw is always rate mode
    r_cmd = yawStick * MAX_CMD_YAW_RATE;

    // Safety clamps
    if (p_cmd >  MAX_ROLL_RATE)  p_cmd =  MAX_ROLL_RATE;
    if (p_cmd < -MAX_ROLL_RATE)  p_cmd = -MAX_ROLL_RATE;
    if (q_cmd >  MAX_PITCH_RATE) q_cmd =  MAX_PITCH_RATE;
    if (q_cmd < -MAX_PITCH_RATE) q_cmd = -MAX_PITCH_RATE;
    if (r_cmd >  MAX_YAW_RATE)   r_cmd =  MAX_YAW_RATE;
    if (r_cmd < -MAX_YAW_RATE)   r_cmd = -MAX_YAW_RATE;

    if (_armState == DISARMED || _rcFailsafe)
    {
        p_cmd = q_cmd = r_cmd = 0.0f;
    }

    // ===== Rate PID loops =====
    float p_err = p_cmd - _p_f;
    float q_err = q_cmd - _q_f;
    float r_err = r_cmd - _r_f;

    float rollOut  = pidUpdate(_pidRollRate,  p_err, dt);
    float pitchOut = pidUpdate(_pidPitchRate, q_err, dt);
    float yawOut   = pidUpdate(_pidYawRate,   r_err, dt);

    // Avoid integrator buildup at very low throttle for rate PIDs
    if (_armState == ARMED && !_rcFailsafe && throttleCmdRaw < 0.05f)
    {
        resetPid(_pidRollRate);
        resetPid(_pidPitchRate);
        resetPid(_pidYawRate);
    }

    // ===== Mix to motors =====

    // Scale attitude authority with throttle so landings aren't violent
    float attScale = throttleCmd / MAX_THROTTLE_CMD; // 0..1
    attScale = clampf(attScale, 0.0f, 1.0f);
    attScale = attScale * attScale; // fade faster near ground

    const float MIX_SCALE = 1.0f / 300.0f;
    float rollTerm  = clampf(rollOut  * MIX_SCALE * attScale, -0.5f, 0.5f);
    float pitchTerm = clampf(pitchOut * MIX_SCALE * attScale, -0.5f, 0.5f);
    float yawTerm   = clampf(yawOut   * MIX_SCALE * attScale, -0.5f, 0.5f);

    // 0: back right (CCW)
    // 1: front right (CW)
    // 2: back left  (CW)
    // 3: front left (CCW)
    float m0 = throttleCmd - rollTerm + pitchTerm + yawTerm; // back right, CCW
    float m1 = throttleCmd - rollTerm - pitchTerm - yawTerm; // front right, CW
    float m2 = throttleCmd + rollTerm + pitchTerm - yawTerm; // back left,  CW
    float m3 = throttleCmd + rollTerm - pitchTerm + yawTerm; // front left, CCW

    m0 = clampf(m0, 0.0f, 1.0f);
    m1 = clampf(m1, 0.0f, 1.0f);
    m2 = clampf(m2, 0.0f, 1.0f);
    m3 = clampf(m3, 0.0f, 1.0f);

    if (_armState == DISARMED || _rcFailsafe)
    {
        m0 = m1 = m2 = m3 = 0.0f;
    }

    // ===== Log sample while ARMED =====
    if (_armState == ARMED && !_rcFailsafe)
    {
        logSample(nowMicros,
                  rollUs, pitchUs, thrUs, yawUs,
                  p_cmd, q_cmd, r_cmd,
                  _p_f, _q_f, _r_f,
                  rollOut, pitchOut, yawOut,
                  m0, m1, m2, m3,
                  _rollAngleDeg, _pitchAngleDeg);
    }

    _motors.setThrottle(0, m0);
    _motors.setThrottle(1, m1);
    _motors.setThrottle(2, m2);
    _motors.setThrottle(3, m3);

    delay(2);
}
