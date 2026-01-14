/*
  Robotics Lab 2 — Human augmentation device (1-DoF linear actuator + IMU gesture)
  Hardware:
   - Arduino UNO
   - DRV8825 stepper driver + NEMA17
   - Grove 9-DoF IMU (ICM20600 accel+gyro, optional AK09918 mag)

  Notes (per lab statement):
   - 1 step = 1 pulse on STEP pin
   - Typical pulse HIGH/LOW (T1/T2): 700..1500 us  :contentReference[oaicite:1]{index=1}
*/

#include <Wire.h>
#include <math.h>
#include "ICM20600.h"
#include "AK09918.h"

// ============================== CONFIG ==============================
// --- Pins (match your wiring) ---
static const uint8_t STEP_PIN   = 8;
static const uint8_t DIR_PIN    = 9;
static const uint8_t SW_LEFT    = 11;   // end-switch left
static const uint8_t SW_RIGHT   = 10;   // end-switch right

// --- End-switch electrical logic ---
// If using INPUT_PULLUP: pressed -> LOW
static const bool     USE_INTERNAL_PULLUPS = true;
static const uint8_t  PRESSED_LEVEL        = LOW;

// --- DRV8825 pulse timing (typical 700..1500 us per lab) ---
static const uint16_t STEP_HIGH_US = 1000;
static const uint16_t STEP_LOW_US  = 1000;

// --- Serial ---
static const uint32_t BAUDRATE          = 115200;
static const bool     STREAM_DEFAULT_ON = true;
static const uint16_t STREAM_PERIOD_MS  = 20;      // 50 Hz CSV

// --- IMU filter ---
static const float COMP_ALPHA = 0.98f;   // complementary filter weight (gyro)

// --- Gesture on X (roll) to trigger "push" ---
static const float    ANGLE_X_DEADBAND_DEG   = 90.0f;
static const uint16_t ANGLE_X_HOLD_MS        = 120;
static const uint32_t GESTURE_REFRACTORY_MS  = 700;

// --- Y proportional position control (pitch) ---
static const float Y_MAX_DEG      = 89.0f;
static const float Y_DEADBAND_DEG = 8.0f;

// Extra smoothing on angleY (reduce jitter while tilted)
static const float Y_ANGLE_LPF_ALPHA = 0.99f;  // 0..1 higher=smoother

// Hysteresis on target steps (reduce chatter)
static const int32_t Y_TARGET_HYST_STEPS = 14; // increase if it still chatters

// Enable Y control only when X is near 0 deg (helps decouple axes)
static const float X_Y_ENABLE_DEG = 10.0f;

// Step pacing for smooth motion (in addition to STEP_HIGH/LOW delays)
static const uint32_t STEP_INTERVAL_US = 2500;

// --- Button push sequence (Task 4) ---
static const int8_t   BUTTON_DIR     = -1;    // +1 toward SW_RIGHT, -1 toward SW_LEFT (set to match your mechanics)
static const int32_t  PRESS_STEPS    = 200;
static const uint16_t PRESS_HOLD_MS  = 1500;
static const uint16_t POST_PUSH_FREEZE_MS = 600;

// Safety
static const uint32_t MAX_TRAVEL_STEPS = 8000;

// Optional magnetometer (not needed for tasks; keep off by default)
#define ENABLE_MAG 0

// ============================== UTIL ==============================
static inline int32_t iabs32(int32_t v) { return (v >= 0) ? v : -v; }

static inline bool switchPressed(uint8_t pin) {
  return (digitalRead(pin) == PRESSED_LEVEL);
}

// ============================== MODULE: Stepper Linear Axis ==============================
class StepperLinearAxis {
public:
  void begin() {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    if (USE_INTERNAL_PULLUPS) {
      pinMode(SW_LEFT,  INPUT_PULLUP);
      pinMode(SW_RIGHT, INPUT_PULLUP);
    } else {
      pinMode(SW_LEFT,  INPUT);
      pinMode(SW_RIGHT, INPUT);
    }

    posSteps_    = 0;
    maxPosSteps_ = 0;
    travelSteps_ = 0;
    lastStepUs_  = 0;
  }

  // One physical step pulse
  void stepOnce(int dir) {
    digitalWrite(DIR_PIN, (dir > 0) ? HIGH : LOW);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_LOW_US);
  }

  // Move N steps or stop if safety switch is pressed
  int32_t moveWithSafety(int32_t steps, uint8_t safetyPin) {
    int dir = (steps >= 0) ? +1 : -1;
    uint32_t n = (uint32_t)iabs32(steps);
    int32_t moved = 0;

    for (uint32_t i = 0; i < n; i++) {
      if (switchPressed(safetyPin)) break;
      stepOnce(dir);
      moved += dir;
    }
    posSteps_ += moved;
    return moved;
  }

  // Run in a direction until target switch pressed or maxSteps reached
  int32_t runUntilSwitch(int dir, uint8_t targetSwitchPin, uint32_t maxStepsRun) {
    int32_t moved = 0;
    for (uint32_t i = 0; i < maxStepsRun; i++) {
      if (switchPressed(targetSwitchPin)) break;
      stepOnce(dir);
      moved += dir;
    }
    posSteps_ += moved;
    return moved;
  }

  // Homing: go LEFT to SW_LEFT, zero; go RIGHT to SW_RIGHT measuring travel; go back to center
  void homeAndCenter() {
    Serial.println("\n[HOMING] Starting...");

    // 1) Go to left end-stop
    Serial.println("[HOMING] Going LEFT until SW_LEFT...");
    runUntilSwitch(-1, SW_LEFT, MAX_TRAVEL_STEPS);

    // Optional tiny backoff to release switch (helps bounce)
    if (switchPressed(SW_LEFT)) {
      moveWithSafety(+10, SW_RIGHT);
    }

    // Define left as zero
    posSteps_ = 0;

    // 2) Go to right end-stop and measure travel
    Serial.println("[HOMING] Going RIGHT until SW_RIGHT...");
    int32_t movedToRight = runUntilSwitch(+1, SW_RIGHT, MAX_TRAVEL_STEPS);

    // Optional backoff
    if (switchPressed(SW_RIGHT)) {
      moveWithSafety(-10, SW_LEFT);
      movedToRight = movedToRight - 10;
    }

    travelSteps_ = iabs32(movedToRight);
    maxPosSteps_ = travelSteps_ / 2;

    Serial.print("[HOMING] Travel steps = "); Serial.println(travelSteps_);
    Serial.print("[HOMING] Max pos steps = "); Serial.println(maxPosSteps_);

    // 3) Go to center from right-ish position: current pos is ~travelSteps
    Serial.print("[HOMING] Going to center ("); Serial.print(maxPosSteps_); Serial.println(" steps) ...");
    moveWithSafety(-maxPosSteps_, SW_LEFT);

    // Define center as zero
    posSteps_ = 0;

    Serial.println("[HOMING] Done. At center.\n");
  }

  // Smoothly service motion towards a target using step interval pacing
  void serviceToTarget(int32_t targetSteps) {
    if (maxPosSteps_ <= 0) return;

    int32_t err = targetSteps - posSteps_;
    if (err == 0) return;

    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastStepUs_) < STEP_INTERVAL_US) return;
    lastStepUs_ = nowUs;

    int dir = (err > 0) ? +1 : -1;
    uint8_t safety = (dir > 0) ? SW_RIGHT : SW_LEFT;

    // Hard safety clamp if we hit a switch
    if (switchPressed(safety)) {
      posSteps_ = (dir > 0) ? +maxPosSteps_ : -maxPosSteps_;
      return;
    }

    stepOnce(dir);
    posSteps_ += dir;
  }

  // Force-set center reference (used after push action)
  void setCenterNow() { posSteps_ = 0; }

  int32_t pos() const { return posSteps_; }
  int32_t maxPos() const { return maxPosSteps_; }

private:
  int32_t  posSteps_;
  int32_t  maxPosSteps_;
  int32_t  travelSteps_;
  uint32_t lastStepUs_;
};

// ============================== MODULE: IMU (ICM20600) ==============================
class IMUCompFilter {
public:
  void begin() {
    icm_.initialize();
#if ENABLE_MAG
    magErr_ = mag_.initialize();
#endif
    reset();
  }

  void calibrateGyroBias(uint16_t ms = 2000) {
    Serial.println("[IMU] Keep still. Calibrating gyro bias...");
    uint32_t t0 = millis();
    uint32_t n = 0;
    double sx = 0, sy = 0, sz = 0;

    while (millis() - t0 < ms) {
      sx += icm_.getGyroscopeX(); // library returns dps (as in read_imu.ino)
      sy += icm_.getGyroscopeY();
      sz += icm_.getGyroscopeZ();
      n++;
      delay(5);
    }

    gyroBiasX_ = (float)(sx / n);
    gyroBiasY_ = (float)(sy / n);
    gyroBiasZ_ = (float)(sz / n);

    Serial.print("[IMU] Gyro bias (dps): ");
    Serial.print(gyroBiasX_); Serial.print(", ");
    Serial.print(gyroBiasY_); Serial.print(", ");
    Serial.println(gyroBiasZ_);

    reset();
  }

  void update() {
    int16_t ax = icm_.getAccelerationX();
    int16_t ay = icm_.getAccelerationY();
    int16_t az = icm_.getAccelerationZ();

    int16_t gx = icm_.getGyroscopeX();
    int16_t gy = icm_.getGyroscopeY();
    int16_t gz = icm_.getGyroscopeZ();

    // store raw for streaming/debug
    ax_ = ax; ay_ = ay; az_ = az;
    gx_ = gx; gy_ = gy; gz_ = gz;

    updateAnglesXY(ax, ay, az, gx, gy);
  }

  float angleXdeg() const { return angleXdeg_; }        // roll
  float angleYdeg() const { return angleYdeg_; }        // pitch
  float angleYsmooth() const { return angleYsmooth_; }

  int16_t ax() const { return ax_; }
  int16_t ay() const { return ay_; }
  int16_t az() const { return az_; }
  int16_t gx() const { return gx_; }
  int16_t gy() const { return gy_; }
  int16_t gz() const { return gz_; }

private:
  void reset() {
    angleXdeg_ = 0.0f;
    angleYdeg_ = 0.0f;
    angleYsmooth_ = 0.0f;
    lastMs_ = 0;
  }

  static float rollFromAccelDeg(int16_t ax, int16_t ay, int16_t az) {
    (void)ax;
    return atan2((float)ay, (float)az) * 180.0f / PI;
  }

  static float pitchFromAccelDeg(int16_t ax, int16_t ay, int16_t az) {
    float denom = sqrtf((float)ay * (float)ay + (float)az * (float)az);
    return atan2(-(float)ax, denom) * 180.0f / PI;
  }

  void updateAnglesXY(int16_t ax, int16_t ay, int16_t az, int16_t gx_raw, int16_t gy_raw) {
    uint32_t now = millis();
    float dt = 0.02f;

    if (lastMs_ != 0) {
      dt = (now - lastMs_) * 1e-3f;
      if (dt <= 0.0f) dt = 0.02f;
      if (dt > 0.2f)  dt = 0.02f;
    }
    lastMs_ = now;

    // gyro in dps (per library), subtract bias then integrate
    float gx = (float)gx_raw - gyroBiasX_;
    float gy = (float)gy_raw - gyroBiasY_;

    float rollAcc  = rollFromAccelDeg(ax, ay, az);
    float pitchAcc = pitchFromAccelDeg(ax, ay, az);

    float gyroAngleX = angleXdeg_ + gx * dt;
    float gyroAngleY = angleYdeg_ + gy * dt;

    angleXdeg_ = COMP_ALPHA * gyroAngleX + (1.0f - COMP_ALPHA) * rollAcc;
    angleYdeg_ = COMP_ALPHA * gyroAngleY + (1.0f - COMP_ALPHA) * pitchAcc;

    // Extra smoothing for control
    angleYsmooth_ = Y_ANGLE_LPF_ALPHA * angleYsmooth_ + (1.0f - Y_ANGLE_LPF_ALPHA) * angleYdeg_;
  }

private:
  ICM20600 icm_{true};

#if ENABLE_MAG
  AK09918 mag_;
  AK09918_err_type_t magErr_;
#endif

  float gyroBiasX_ = 0, gyroBiasY_ = 0, gyroBiasZ_ = 0;

  float angleXdeg_ = 0, angleYdeg_ = 0, angleYsmooth_ = 0;
  uint32_t lastMs_ = 0;

  // raw cache
  int16_t ax_ = 0, ay_ = 0, az_ = 0;
  int16_t gx_ = 0, gy_ = 0, gz_ = 0;
};

// ============================== MODULE: Gesture Detector (X) ==============================
class XGestureDetector {
public:
  void reset(uint32_t nowMs) {
    wasOutside_ = false;
    outsideStartMs_ = 0;
    lastGestureMs_ = nowMs;
  }

  bool update(float angleXdeg) {
    uint32_t now = millis();

    // refractory
    if (now - lastGestureMs_ < GESTURE_REFRACTORY_MS) {
      bool outside = (fabs(angleXdeg) > ANGLE_X_DEADBAND_DEG);
      wasOutside_ = outside;
      outsideStartMs_ = 0;
      return false;
    }

    bool outside = (fabs(angleXdeg) > ANGLE_X_DEADBAND_DEG);

    if (!outside) {
      wasOutside_ = false;
      outsideStartMs_ = 0;
      return false;
    }

    if (!wasOutside_) {
      wasOutside_ = true;
      outsideStartMs_ = now;
      return false;
    }

    if (outsideStartMs_ != 0 && (now - outsideStartMs_ >= ANGLE_X_HOLD_MS)) {
      lastGestureMs_ = now;
      outsideStartMs_ = 0;
      return true;
    }

    return false;
  }

private:
  bool     wasOutside_ = false;
  uint32_t outsideStartMs_ = 0;
  uint32_t lastGestureMs_ = 0;
};

// ============================== MODULE: Y Target Mapping + Hysteresis ==============================
class YTargetController {
public:
  void reset() { targetSteps_ = 0; rawTargetSteps_ = 0; }

  int32_t target() const { return targetSteps_; }
  int32_t rawTarget() const { return rawTargetSteps_; }

  void computeAndUpdate(float angleYdeg_smooth, int32_t maxPosSteps) {
    rawTargetSteps_ = computeTargetStepsFromAngleY(angleYdeg_smooth, maxPosSteps);

    // hysteresis on target update
    if (iabs32(rawTargetSteps_ - targetSteps_) >= Y_TARGET_HYST_STEPS) {
      targetSteps_ = rawTargetSteps_;
    }
  }

  // Freeze target at current pos to "hold still"
  void holdAt(int32_t posSteps) {
    rawTargetSteps_ = targetSteps_;
    targetSteps_ = posSteps;
  }

private:
  static int32_t computeTargetStepsFromAngleY(float aDeg, int32_t maxPosSteps) {
    if (maxPosSteps <= 0) return 0;

    if (fabs(aDeg) < Y_DEADBAND_DEG) aDeg = 0.0f;

    if (aDeg >  Y_MAX_DEG) aDeg =  Y_MAX_DEG;
    if (aDeg < -Y_MAX_DEG) aDeg = -Y_MAX_DEG;

    float ratio = aDeg / Y_MAX_DEG; // -1..+1
    return (int32_t)lroundf(ratio * (float)maxPosSteps);
  }

private:
  int32_t targetSteps_    = 0;
  int32_t rawTargetSteps_ = 0;
};

// ============================== SYSTEM STATE / GLOBALS ==============================
static StepperLinearAxis  axis;
static IMUCompFilter      imu;
static XGestureDetector   gestureX;
static YTargetController  yCtrl;

static bool     streamEnabled = STREAM_DEFAULT_ON;
static uint32_t lastStreamMs  = 0;
static uint32_t yControlResumeMs = 0;

// ============================== APP ACTIONS ==============================
static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  H  -> home + center");
  Serial.println("  P  -> button push (manual trigger)");
  Serial.println("  S  -> start streaming CSV");
  Serial.println("  s  -> stop streaming CSV");
  Serial.println("  C  -> calibrate gyro bias (keep still)");
  Serial.println("  ?  -> help");
  Serial.println();
}

static void printCsvHeader() {
  Serial.println("t_ms,ax,ay,az,gx,gy,gz,angleX_deg,angleY_deg,angleY_smooth,rawTarget,targetSteps,yEnabled,gestureX,posSteps");
}

static void doButtonPush() {
  Serial.println("[ACTION] Button push!");

  // Temporarily freeze Y-control so it doesn't fight the action
  yControlResumeMs = millis() + POST_PUSH_FREEZE_MS;
  yCtrl.reset();

  // Move towards the button
  if (BUTTON_DIR > 0) axis.moveWithSafety(+PRESS_STEPS, SW_RIGHT);
  else                axis.moveWithSafety(-PRESS_STEPS, SW_LEFT);

  // Keep contact
  delay(PRESS_HOLD_MS);

  // Return to center (based on current pos estimate)
  int32_t back = -axis.pos();
  if (back > 0) axis.moveWithSafety(back, SW_RIGHT);
  else          axis.moveWithSafety(back, SW_LEFT);

  // Re-zero center
  axis.setCenterNow();
  yCtrl.reset();

  // restart gesture refractory
  gestureX.reset(millis());

  Serial.println("[ACTION] Done. Back to center.");
}

// ============================== ARDUINO ==============================
void setup() {
  Wire.begin();
  Serial.begin(BAUDRATE);

  axis.begin();
  imu.begin();

  printHelp();

  imu.calibrateGyroBias(2000);
  gestureX.reset(millis());
  yCtrl.reset();
  yControlResumeMs = millis();

  axis.homeAndCenter();

  if (streamEnabled) printCsvHeader();
}

void loop() {
  // ---------- Serial UI ----------
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'H') { axis.homeAndCenter(); axis.setCenterNow(); yCtrl.reset(); }
    else if (c == 'P') doButtonPush();
    else if (c == 'S') { streamEnabled = true; printCsvHeader(); }
    else if (c == 's') streamEnabled = false;
    else if (c == 'C') { imu.calibrateGyroBias(2000); gestureX.reset(millis()); }
    else if (c == '?') printHelp();
  }

  // ---------- IMU update ----------
  imu.update();

  // ---------- Gesture on X -> push ----------
  bool trigX = gestureX.update(imu.angleXdeg());
  if (trigX) {
    doButtonPush();
  }

  // ---------- Y proportional control gated by X alignment ----------
  bool yEnabled = (fabs(imu.angleXdeg()) < X_Y_ENABLE_DEG) && (millis() >= yControlResumeMs);

  if (yEnabled) {
    yCtrl.computeAndUpdate(imu.angleYsmooth(), axis.maxPos());
  } else {
    // Hold current position
    yCtrl.holdAt(axis.pos());
  }

  // ---------- Stepper service ----------
  axis.serviceToTarget(yCtrl.target());

  // ---------- CSV streaming ----------
  uint32_t nowMs = millis();
  if (streamEnabled && (nowMs - lastStreamMs >= STREAM_PERIOD_MS)) {
    lastStreamMs = nowMs;

    Serial.print(nowMs); Serial.print(",");
    Serial.print(imu.ax()); Serial.print(",");
    Serial.print(imu.ay()); Serial.print(",");
    Serial.print(imu.az()); Serial.print(",");
    Serial.print(imu.gx()); Serial.print(",");
    Serial.print(imu.gy()); Serial.print(",");
    Serial.print(imu.gz()); Serial.print(",");
    Serial.print(imu.angleXdeg(), 2); Serial.print(",");
    Serial.print(imu.angleYdeg(), 2); Serial.print(",");
    Serial.print(imu.angleYsmooth(), 2); Serial.print(",");
    Serial.print(yCtrl.rawTarget()); Serial.print(",");
    Serial.print(yCtrl.target()); Serial.print(",");
    Serial.print(yEnabled ? 1 : 0); Serial.print(",");
    Serial.print(trigX ? 1 : 0); Serial.print(",");
    Serial.println(axis.pos());
  }
}
