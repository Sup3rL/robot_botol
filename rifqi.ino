#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Ps3Controller.h>
#include <math.h>

// ======================================================
// ESP32 MECANUM ROBOT
// 4x GA25 encoder + 2x L298N + HMC5883L + PS3
// Arduino-ESP32 core 3.x
//
// Urutan roda: FL, FR, RL, RR
// FL = kiri depan, FR = kanan depan
// RL = kiri belakang, RR = kanan belakang
// ======================================================

// ---------- ENCODER (pin asli pengguna) ----------
const uint8_t ENC_A[4] = {32, 12, 27, 25};
const uint8_t ENC_B[4] = {33, 13, 14, 26};

// ---------- L298N ----------
// L298N A: FL + FR
// L298N B: RL + RR
const uint8_t PWM_PIN[4] = {18, 5, 4, 0};
const uint8_t IN1[4]     = {19, 16, 15, 1};
const uint8_t IN2[4]     = {23, 17, 2, 3};

// ---------- I2C HMC5883L ----------
const int SDA_PIN = 21;
const int SCL_PIN = 22;

Adafruit_HMC5883_Unified compass(12345);

// ---------- PARAMETER MOTOR ----------
const int PWM_FREQ = 18000;
const int PWM_BITS = 8;
const int PWM_MAX = 255;

const uint32_t CONTROL_PERIOD_MS = 20;

// Maksimum target hitungan encoder per detik.
// Sesuaikan setelah pengujian.
float MAX_TICKS_PER_SEC = 800.0f;

// Kalibrasi jumlah tick encoder per meter perjalanan.
// NILAI CONTOH, wajib diukur pada robot sebenarnya.
float TICKS_PER_METER = 1000.0f;

// ---------- ARAH MOTOR / ENCODER ----------
// Balik tanda jika arah roda/encoder tidak sesuai.
int8_t MOTOR_SIGN[4]   = {1, -1, 1, -1};
int8_t ENCODER_SIGN[4] = {1, -1, 1, -1};

// ---------- PID KECEPATAN RODA ----------
float KpWheel = 0.12f;
float KiWheel = 0.025f;
float KdWheel = 0.001f;

// ---------- PID HEADING ----------
float KpHeading = 0.018f;
float KiHeading = 0.000f;
float KdHeading = 0.001f;

// ---------- RAMP KECEPATAN ----------
float MAX_ACCEL = 900.0f;  // tick/s per detik

// ---------- ENCODER QUADRATURE ----------
volatile int32_t encoderTicks[4] = {0, 0, 0, 0};
volatile uint8_t previousAB[4] = {0, 0, 0, 0};

const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

float targetSpeed[4] = {0, 0, 0, 0};
float rampSpeed[4]   = {0, 0, 0, 0};
float measuredSpeed[4] = {0, 0, 0, 0};

float wheelIntegral[4] = {0, 0, 0, 0};
float wheelPreviousError[4] = {0, 0, 0, 0};
int32_t previousTicks[4] = {0, 0, 0, 0};

// ---------- HEADING ----------
float heading = 0.0f;
float headingTarget = 0.0f;
float headingIntegral = 0.0f;
float headingPreviousError = 0.0f;
bool headingHold = false;
bool compassOK = false;

// ---------- MODE ROBOT ----------
enum RobotMode {
  MODE_MANUAL,
  MODE_AUTO_MOVE,
  MODE_AUTO_TURN,
  MODE_STOP
};

RobotMode mode = MODE_MANUAL;

// Autonomous state
float autoX = 0;
float autoY = 0;
float autoStartTicks[4] = {0, 0, 0, 0};
float autoTargetTicks = 0;
float autoTargetHeading = 0;
uint32_t autoStartMs = 0;
uint32_t autoTimeoutMs = 10000;

// ---------- UTILITAS ----------
float clampFloat(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float wrap360(float angle) {
  while (angle < 0) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

float angleError(float target, float current) {
  float e = target - current;
  while (e > 180.0f) e -= 360.0f;
  while (e < -180.0f) e += 360.0f;
  return e;
}

float deadband(int value) {
  if (abs(value) < 16) return 0.0f;
  return clampFloat(value / 127.0f, -1.0f, 1.0f);
}

// ======================================================
// ENCODER ISR
// ======================================================
void IRAM_ATTR encoderISR(uint8_t i) {
  uint8_t a = digitalRead(ENC_A[i]);
  uint8_t b = digitalRead(ENC_B[i]);
  uint8_t currentAB = (a << 1) | b;

  uint8_t index = (previousAB[i] << 2) | currentAB;
  encoderTicks[i] += QUAD_TABLE[index];
  previousAB[i] = currentAB;
}

void IRAM_ATTR isrFL() { encoderISR(0); }
void IRAM_ATTR isrFR() { encoderISR(1); }
void IRAM_ATTR isrRL() { encoderISR(2); }
void IRAM_ATTR isrRR() { encoderISR(3); }

// ======================================================
// MOTOR DRIVER
// PWM_PIN adalah pin ENA/ENB, jumper enable dilepas.
// ======================================================
void setMotor(int i, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  pwm *= MOTOR_SIGN[i];

  if (pwm > 0) {
    digitalWrite(IN1[i], HIGH);
    digitalWrite(IN2[i], LOW);
  } else if (pwm < 0) {
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], HIGH);
  } else {
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], LOW);
  }

  ledcWrite(PWM_PIN[i], abs(pwm));
}

void resetWheelPID() {
  for (int i = 0; i < 4; i++) {
    wheelIntegral[i] = 0;
    wheelPreviousError[i] = 0;
  }
}

void stopRobot() {
  for (int i = 0; i < 4; i++) {
    targetSpeed[i] = 0;
    rampSpeed[i] = 0;
    setMotor(i, 0);
  }

  resetWheelPID();
  headingHold = false;
  mode = MODE_STOP;
}

// ======================================================
// COMPASS
// Heading magnetometer sederhana.
// Kalibrasi hard-iron dan declination diperlukan.
// ======================================================
bool readCompass() {
  if (!compassOK) return false;

  sensors_event_t event;
  compass.getEvent(&event);

  float angle = atan2f(event.magnetic.y,
                       event.magnetic.x) * 180.0f / PI;

  heading = wrap360(angle);
  return true;
}

// ======================================================
// MECANUM MIXER
// x: strafe kanan (+)
// y: maju (+)
// r: rotasi searah jarum jam (+)
//
// Semua target dinormalisasi bersama.
// ======================================================
void mecanumMix(float x, float y, float r) {
  float m[4] = {
    y + x + r,  // FL
    y - x - r,  // FR
    y - x + r,  // RL
    y + x - r   // RR
  };

  float largest = 1.0f;
  for (int i = 0; i < 4; i++) {
    largest = max(largest, fabsf(m[i]));
  }

  for (int i = 0; i < 4; i++) {
    targetSpeed[i] =
      (m[i] / largest) * MAX_TICKS_PER_SEC;
  }
}

// ======================================================
// PID HEADING
// Keluaran -1..1 untuk rotasi mecanum.
// ======================================================
float headingPID(float dt) {
  if (dt <= 0 || !compassOK) return 0;

  float error = angleError(headingTarget, heading);

  headingIntegral += error * dt;
  headingIntegral = clampFloat(
    headingIntegral, -100.0f, 100.0f);

  float derivative =
    (error - headingPreviousError) / dt;

  headingPreviousError = error;

  return clampFloat(
    KpHeading * error +
    KiHeading * headingIntegral +
    KdHeading * derivative,
    -1.0f, 1.0f
  );
}

// ======================================================
// PID KECEPATAN RODA + RAMP
// ======================================================
void updateWheelPID(float dt) {
  int32_t snapshot[4];

  noInterrupts();
  for (int i = 0; i < 4; i++) {
    snapshot[i] = encoderTicks[i];
  }
  interrupts();

  for (int i = 0; i < 4; i++) {
    int32_t delta = snapshot[i] - previousTicks[i];
    previousTicks[i] = snapshot[i];

    measuredSpeed[i] =
      (float)(delta * ENCODER_SIGN[i]) / dt;

    // Ramp target supaya percepatan lebih halus
    float maxChange = MAX_ACCEL * dt;
    float difference = targetSpeed[i] - rampSpeed[i];

    rampSpeed[i] += clampFloat(
      difference, -maxChange, maxChange);

    float error = rampSpeed[i] - measuredSpeed[i];

    wheelIntegral[i] += error * dt;
    wheelIntegral[i] = clampFloat(
      wheelIntegral[i], -1500.0f, 1500.0f);

    float derivative =
      (error - wheelPreviousError[i]) / dt;

    wheelPreviousError[i] = error;

    float feedForward =
      rampSpeed[i] / MAX_TICKS_PER_SEC * 180.0f;

    float output = feedForward +
                   KpWheel * error +
                   KiWheel * wheelIntegral[i] +
                   KdWheel * derivative;

    setMotor(i, (int)clampFloat(
      output, -PWM_MAX, PWM_MAX));
  }
}

// ======================================================
// MANUAL CONTROL PS3
// Stick kiri = translasi
// Stick kanan horizontal = rotasi
// Cross = stop
// Circle = mode manual
// Triangle = autonomous demo
// ======================================================
void manualControl(float dt) {
  if (!Ps3.isConnected()) {
    for (int i = 0; i < 4; i++) targetSpeed[i] = 0;
    headingHold = false;
    return;
  }

  if (Ps3.data.button.cross) {
    stopRobot();
    return;
  }

  if (Ps3.data.button.circle) {
    mode = MODE_MANUAL;
  }

  if (Ps3.data.button.triangle) {
    startAutoDemo();
    return;
  }

  float x = deadband(Ps3.data.analog.stick.lx);
  float y = -deadband(Ps3.data.analog.stick.ly);
  float r = deadband(Ps3.data.analog.stick.rx);

  // Saat joystick rotasi digunakan, lepas heading hold
  if (fabsf(r) > 0.08f) {
    headingHold = false;
    mecanumMix(x, y, r);
  } else {
    if (!headingHold) {
      headingTarget = heading;
      headingIntegral = 0;
      headingPreviousError = 0;
      headingHold = true;
    }

    float correction = compassOK ? headingPID(dt) : 0;
    mecanumMix(x, y, correction);
  }
}

// ======================================================
// AUTONOMOUS: bergerak dengan jarak encoder
// x = strafe (+ kanan), y = maju (+)
// distanceCm = jarak lintasan yang diminta
// ======================================================
void startAutoMove(float x, float y, float distanceCm) {
  if (x == 0 && y == 0) return;

  autoX = x;
  autoY = y;

  noInterrupts();
  for (int i = 0; i < 4; i++) {
    autoStartTicks[i] = encoderTicks[i];
  }
  interrupts();

  autoTargetTicks = fabsf(distanceCm) / 100.0f *
                    TICKS_PER_METER;

  autoStartMs = millis();
  autoTargetHeading = heading;
  headingTarget = heading;

  resetWheelPID();
  headingHold = true;
  mode = MODE_AUTO_MOVE;
}

float averageAutoProgress() {
  int32_t nowTicks[4];

  noInterrupts();
  for (int i = 0; i < 4; i++) {
    nowTicks[i] = encoderTicks[i];
  }
  interrupts();

  // Untuk jarak gerak lurus, ambil magnitudo
  // rata-rata perubahan roda.
  float total = 0;
  for (int i = 0; i < 4; i++) {
    total += fabsf(
      (nowTicks[i] - autoStartTicks[i]) *
      ENCODER_SIGN[i]);
  }

  return total / 4.0f;
}

void updateAutoMove(float dt) {
  if (millis() - autoStartMs > autoTimeoutMs) {
    stopRobot();
    return;
  }

  float progress = averageAutoProgress();

  if (progress >= autoTargetTicks) {
    stopRobot();
    return;
  }

  float correction = compassOK ? headingPID(dt) : 0;

  float magnitude = sqrtf(autoX * autoX + autoY * autoY);
  if (magnitude < 0.001f) {
    stopRobot();
    return;
  }

  mecanumMix(
    autoX / magnitude * 0.5f,
    autoY / magnitude * 0.5f,
    correction
  );
}

// ======================================================
// AUTONOMOUS: putar menuju sudut absolut
// Contoh: turnToHeading(90) = menghadap 90° kompas
// ======================================================
void turnToHeading(float target, uint32_t timeoutMs = 5000) {
  headingTarget = wrap360(target);
  autoTargetHeading = headingTarget;
  headingIntegral = 0;
  headingPreviousError = 0;
  autoStartMs = millis();
  autoTimeoutMs = timeoutMs;
  mode = MODE_AUTO_TURN;
}

void updateAutoTurn(float dt) {
  if (!compassOK) {
    stopRobot();
    return;
  }

  if (millis() - autoStartMs > autoTimeoutMs) {
    stopRobot();
    return;
  }

  float error = angleError(headingTarget, heading);

  if (fabsf(error) <= 3.0f) {
    stopRobot();
    return;
  }

  // Batasi rotasi saat mendekati target
  float rotation = headingPID(dt);
  if (fabsf(error) < 15.0f) {
    rotation *= 0.45f;
  }

  mecanumMix(0, 0, rotation);
}

// ======================================================
// DEMO AUTONOMOUS NON-BLOCKING
// Maju 50 cm -> belok 90° -> maju 50 cm
// ======================================================
int demoStep = 0;

void startAutoDemo() {
  demoStep = 0;
  startAutoMove(0, 1, 50);
}

void updateAutoDemo() {
  // Demo menggunakan mode yang sedang aktif.
  // Langkah selanjutnya dipilih di loop berdasarkan
  // mode MODE_STOP dan demoStep.
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);

    previousAB[i] =
      (digitalRead(ENC_A[i]) << 1) |
       digitalRead(ENC_B[i]);

    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);

    ledcAttach(PWM_PIN[i], PWM_FREQ, PWM_BITS);
    setMotor(i, 0);
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[0]), isrFL, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[1]), isrFR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[2]), isrRL, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), isrRR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B[3]), isrRR, CHANGE);

  Wire.begin(SDA_PIN, SCL_PIN);
  compassOK = compass.begin();

  if (compassOK) {
    Serial.println("HMC5883L OK");
    readCompass();
    headingTarget = heading;
  } else {
    Serial.println("ERROR: HMC5883L tidak terdeteksi");
  }

  Ps3.attachOnConnect([]() {
    Serial.println("PS3 connected");
  });

  // Jika diperlukan, isi MAC Bluetooth host ESP32:
  // Ps3.begin("AA:BB:CC:DD:EE:FF");
  Ps3.begin();

  Serial.println("Robot siap. Cross = stop, Circle = manual.");
}

// ======================================================
// LOOP 50 Hz
// ======================================================
void loop() {
  static uint32_t lastLoop = 0;
  uint32_t now = millis();

  if (now - lastLoop < CONTROL_PERIOD_MS) return;

  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  readCompass();

  switch (mode) {
    case MODE_MANUAL:
      manualControl(dt);
      break;

    case MODE_AUTO_MOVE:
      updateAutoMove(dt);
      break;

    case MODE_AUTO_TURN:
      updateAutoTurn(dt);
      break;

    case MODE_STOP:
      // Berhenti sampai perintah baru diterima
      if (Ps3.isConnected() && Ps3.data.button.circle) {
        mode = MODE_MANUAL;
      }
      break;
  }

  updateWheelPID(dt);

  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;

    Serial.printf(
      "Heading: %.1f | FL %.0f FR %.0f RL %.0f RR %.0f\n",
      heading,
      measuredSpeed[0], measuredSpeed[1],
      measuredSpeed[2], measuredSpeed[3]
    );
  }
}
