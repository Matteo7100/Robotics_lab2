/*
  Robotics Lab 2 — Human augmentation device (1-DoF linear actuator + IMU gesture)
*/

#include <Wire.h>
#include <math.h>
#include "ICM20600.h"
#include "AK09918.h"

// ============================== CONFIG ==============================
static const uint8_t STEP_PIN  = 8;
static const uint8_t DIR_PIN   = 9;
static const uint8_t SW_LEFT   = 11;
static const uint8_t SW_RIGHT  = 10;

static const bool    USE_INTERNAL_PULLUPS = true;
static const uint8_t PRESSED_LEVEL        = LOW;

static const uint16_t STEP_HIGH_US = 1000;
static const uint16_t STEP_LOW_US  = 1000;

static const uint32_t BAUDRATE         = 115200;
static const bool     STREAM_DEFAULT_ON = true;
static const uint16_t STREAM_PERIOD_MS  = 20;  

static const float COMP_ALPHA = 0.98f;

// Gesture on X (roll) to trigger push
static const float    ANGLE_X_DEADBAND_DEG  = 90.0f;
static const uint16_t ANGLE_X_HOLD_MS       = 120;
static const uint32_t GESTURE_REFRACTORY_MS = 700;

static const float Y_ANGLE_LPF_ALPHA = 0.90f;

static const int8_t   BUTTON_DIR     = -1;
static const int32_t  PRESS_STEPS    = 200;
static const uint16_t PRESS_HOLD_MS  = 1500;

static const uint32_t MAX_TRAVEL_STEPS = 8000;

#define ENABLE_MAG 0

// ============================== UTIL ==============================
static inline int32_t iabs32(int32_t v) { return (v >= 0) ? v : -v; }
static inline bool switchPressed(uint8_t pin) { return (digitalRead(pin) == PRESSED_LEVEL); }

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

    posSteps_ = 0;
    maxPosSteps_ = 0;
    travelSteps_ = 0;
  }

  void stepOnce(int dir) {
    digitalWrite(DIR_PIN, (dir > 0) ? HIGH : LOW);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_LOW_US);
  }

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

  void homeAndCenter() {
    Serial.println("\n[HOMING] Starting...");

    Serial.println("[HOMING] Going LEFT until SW_LEFT...");
    runUntilSwitch(-1, SW_LEFT, MAX_TRAVEL_STEPS);

    if (switchPressed(SW_LEFT)) moveWithSafety(+10, SW_RIGHT);
    posSteps_ = 0;

    Serial.println("[HOMING] Going RIGHT until SW_RIGHT...");
    int32_t movedToRight = runUntilSwitch(+1, SW_RIGHT, MAX_TRAVEL_STEPS);

    if (switchPressed(SW_RIGHT)) {
      moveWithSafety(-10, SW_LEFT);
      movedToRight -= 10;
    }

    travelSteps_ = iabs32(movedToRight);
    maxPosSteps_ = travelSteps_ / 2;

    Serial.print("[HOMING] Travel steps = "); Serial.println(travelSteps_);
    Serial.print("[HOMING] Max pos steps = "); Serial.println(maxPosSteps_);

    Serial.print("[HOMING] Going to center (");
    Serial.print(maxPosSteps_);
    Serial.println(" steps) ...");

    moveWithSafety(-maxPosSteps_, SW_LEFT);
    posSteps_ = 0;

    Serial.println("[HOMING] Done. At center.\n");
  }

  void setCenterNow() { posSteps_ = 0; }

  int32_t pos() const { return posSteps_; }
  int32_t maxPos() const { return maxPosSteps_; }

private:
  int32_t posSteps_ = 0;
  int32_t maxPosSteps_ = 0;
  int32_t travelSteps_ = 0;
};

// ============================== MODULE: IMU (ICM20600) ==============================
class IMUCompFilter {
public:
  void begin() {
    icm_.initialize();
    reset();
  }

  void calibrateGyroBias(uint16_t ms = 2000) {
    Serial.println("[IMU] Keep still. Calibrating gyro bias...");
    uint32_t t0 = millis();
    uint32_t n = 0;
    double sx = 0, sy = 0, sz = 0;

    while (millis() - t0 < ms) {
      sx += icm_.getGyroscopeX();
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
    ax_ = icm_.getAccelerationX();
    ay_ = icm_.getAccelerationY();
    az_ = icm_.getAccelerationZ();

    gx_ = icm_.getGyroscopeX();
    gy_ = icm_.getGyroscopeY();
    gz_ = icm_.getGyroscopeZ();

    updateAnglesXY(ax_, ay_, az_, gx_, gy_);
  }

  float angleXdeg() const { return angleXdeg_; }     // roll
  float angleYdeg() const { return angleYdeg_; }     // pitch
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

    float gx = (float)gx_raw - gyroBiasX_;
    float gy = (float)gy_raw - gyroBiasY_;

    float rollAcc  = rollFromAccelDeg(ax, ay, az);
    float pitchAcc = pitchFromAccelDeg(ax, ay, az);

    float gyroAngleX = angleXdeg_ + gx * dt;
    float gyroAngleY = angleYdeg_ + gy * dt;

    angleXdeg_ = COMP_ALPHA * gyroAngleX + (1.0f - COMP_ALPHA) * rollAcc;
    angleYdeg_ = COMP_ALPHA * gyroAngleY + (1.0f - COMP_ALPHA) * pitchAcc;

    // smoothing for visualization only
    angleYsmooth_ = Y_ANGLE_LPF_ALPHA * angleYsmooth_ + (1.0f - Y_ANGLE_LPF_ALPHA) * angleYdeg_;
  }

private:
  ICM20600 icm_{true};

  float gyroBiasX_ = 0, gyroBiasY_ = 0, gyroBiasZ_ = 0;
  float angleXdeg_ = 0, angleYdeg_ = 0, angleYsmooth_ = 0;
  uint32_t lastMs_ = 0;

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
  bool wasOutside_ = false;
  uint32_t outsideStartMs_ = 0;
  uint32_t lastGestureMs_ = 0;
};

// ============================== GLOBALS ==============================
static StepperLinearAxis axis;
static IMUCompFilter imu;
static XGestureDetector gestureX;

static bool streamEnabled = STREAM_DEFAULT_ON;
static uint32_t lastStreamMs = 0;

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
  Serial.println("t_ms,ax,ay,az,gx,gy,gz,angleX_deg,angleY_deg,angleY_smooth,gestureX,posSteps");
}

static void doButtonPush() {
  Serial.println("[ACTION] Button push!");

  if (BUTTON_DIR > 0) axis.moveWithSafety(+PRESS_STEPS, SW_RIGHT);
  else                axis.moveWithSafety(-PRESS_STEPS, SW_LEFT);

  delay(PRESS_HOLD_MS);

  int32_t back = -axis.pos();
  if (back > 0) axis.moveWithSafety(back, SW_RIGHT);
  else          axis.moveWithSafety(back, SW_LEFT);

  axis.setCenterNow();
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

  axis.homeAndCenter();

  if (streamEnabled) printCsvHeader();
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'H') { axis.homeAndCenter(); axis.setCenterNow(); }
    else if (c == 'P') doButtonPush();
    else if (c == 'S') { streamEnabled = true; printCsvHeader(); }
    else if (c == 's') streamEnabled = false;
    else if (c == 'C') { imu.calibrateGyroBias(2000); gestureX.reset(millis()); }
    else if (c == '?') printHelp();
  }

  imu.update();

  bool trigX = gestureX.update(imu.angleXdeg());
  if (trigX) doButtonPush();

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
    Serial.print(trigX ? 1 : 0); Serial.print(",");
    Serial.println(axis.pos());
  }
}
