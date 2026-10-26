#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

// =====================================================================
// ---------- Konfigurasi Pin & Komunikasi UART ----------
// =====================================================================
#define UART_RX 23
#define UART_TX 19

// Buffer penerimaan data JSON UART
unsigned long lastSend = 0;
uint32_t counter = 0;
String rxBuffer = "";

// Driver 1 (Motor Depan)
const int Dp_kn_speed = 13; // Speed Kanan Depan
const int Dp_kn_mj    = 12;
const int Dp_kn_md    = 14;

const int Dp_kr_speed = 15; // Speed Kiri Depan
const int Dp_kr_mj    = 27;
const int Dp_kr_md    = 26;

// Driver 2 (Motor Belakang)
const int Bl_kn_speed = 25; // Speed Kanan Belakang
const int Bl_kn_mj    = 4;
const int Bl_kn_md    = 2;

const int Bl_kr_speed = 5;  // Speed Kiri Belakang
const int Bl_kr_mj    = 17;
const int Bl_kr_md    = 16;

// =====================================================================
// ---------- Objek & Instance Eksternal ----------
// =====================================================================
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); // PCA9685 di 0x40

String uartLine;
unsigned long lastRemoteMs = 0;
const unsigned long REMOTE_TIMEOUT_MS = 250;

// =====================================================================
// ---------- Konfigurasi PWM & Tuning Joystick ----------
// =====================================================================
int pwm = 0;

const int STICK_DEADZONE = 120;
const int PWM_MIN        = 130;
const int frequency      = 30000;
const int resolution     = 8;

// PWM Channels
const int CH_ENA_Dp_kn = 0;
const int CH_ENB_Dp_kr = 1;
const int CH_ENA_Bl_kn = 2;
const int CH_ENB_Bl_kr = 3;
const int CH_ena       = 4;

int channelFor(int pin) {
  switch (pin) {
    case Dp_kn_speed: return CH_ENA_Dp_kn;
    case Dp_kr_speed: return CH_ENB_Dp_kr;
    case Bl_kn_speed: return CH_ENA_Bl_kn;
    case Bl_kr_speed: return CH_ENB_Bl_kr;
    default:          return CH_ena;
  }
}

// =====================================================================
// ---------- Motor & Movement Helper ----------
// =====================================================================
void motor(int en, int pin_maju, int pin_mundur, int duty, int status_maju, int status_mundur) {
  ledcWrite(channelFor(en), duty);
  digitalWrite(pin_maju, status_maju);
  digitalWrite(pin_mundur, status_mundur);
}

void maju() {
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, pwm, HIGH, LOW);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, pwm, HIGH, LOW);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, pwm, HIGH, LOW);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, pwm, HIGH, LOW);
}

void mundur() {
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, pwm, LOW, HIGH);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, pwm, LOW, HIGH);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, pwm, LOW, HIGH);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, pwm, LOW, HIGH);
}

void bel_ki() { // Kanan maju, Kiri mundur
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, pwm, HIGH, LOW);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, pwm, LOW, HIGH);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, pwm, HIGH, LOW);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, pwm, LOW, HIGH);
}

void bel_ka() { // Kiri maju, Kanan mundur
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, pwm, LOW, HIGH);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, pwm, HIGH, LOW);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, pwm, LOW, HIGH);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, pwm, HIGH, LOW);
}

void geser_kiri() { // Depan Kanan Maju, Depan Kiri Mundur, Belakang Kanan Mundur, Belakang Kiri Maju
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, pwm, HIGH, LOW);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, pwm, LOW, HIGH);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, pwm, LOW, HIGH);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, pwm, HIGH, LOW);
}

void geser_kanan() { // Depan Kiri Maju, Depan Kanan Mundur, Belakang Kiri Mundur, Belakang Kanan Maju
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, pwm, LOW, HIGH);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, pwm, HIGH, LOW);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, pwm, HIGH, LOW);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, pwm, LOW, HIGH);
}

void diagonal_kanan_maju() {
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, 0,   LOW,  LOW);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, pwm, HIGH, LOW);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, pwm, HIGH, LOW);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, 0,   LOW,  LOW);
}

void diagonal_kiri_maju() {
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, pwm, HIGH, LOW);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, 0,   LOW,  LOW);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, 0,   LOW,  LOW);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, pwm, HIGH, LOW);
}

void stopped() {
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, 0, LOW, LOW);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, 0, LOW, LOW);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, 0, LOW, LOW);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, 0, LOW, LOW);
}

// =====================================================================
// ---------- Lifter Subsystem ----------
// =====================================================================
void lifter_stopped() {
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, HIGH);
}

void lifter_down() {
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
}

void lifter_up() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
}

// =====================================================================
// ---------- Gripper Subsystem: MG996R Servos ----------
// =====================================================================
const bool GRIPPER_360 = false;

const int CH_A = 14;
const int CH_B = 15;

const int OPEN_A_US   = 1000;
const int CLOSE_A_US  = 2800;
const int OPEN_B_US   = 2000;
const int CLOSE_B_US  = 200;
const float START_GRIP = 0.0;
const float GRIP_SPEED = 1.0;

const int NEUTRAL_A_US = 1500;
const int NEUTRAL_B_US = 1500;
const int SPEED_US     = 150;
const int DIR_A        = 1;
const int DIR_B        = -1;

const int SERVO_FREQ   = 50;
float grip             = START_GRIP;
int lastUsA            = -1;
int lastUsB            = -1;
int gripDir            = 0;
unsigned long gripLastMs = 0;

void writeUs(int ch, int us) {
  servo.writeMicroseconds(ch, us);
}

void servoOff(int ch) {
  servo.setPWM(ch, 0, 4096);
}

void writeJaws(bool force) {
  int usA = OPEN_A_US + (int)((CLOSE_A_US - OPEN_A_US) * grip);
  int usB = OPEN_B_US + (int)((CLOSE_B_US - OPEN_B_US) * grip);
  if (force || usA != lastUsA) {
    writeUs(CH_A, usA);
    lastUsA = usA;
  }
  if (force || usB != lastUsB) {
    writeUs(CH_B, usB);
    lastUsB = usB;
  }
}

void gripper_stop() {
  if (GRIPPER_360) {
    servoOff(CH_A);
    servoOff(CH_B);
  }
  gripDir = 0;
}

void updateGripper(bool closeBtn, bool openBtn) {
  int want = 0;
  if (closeBtn && !openBtn) want = +1;
  if (openBtn && !closeBtn) want = -1;

  unsigned long now = millis();
  float dt = (now - gripLastMs) / 1000.0f;
  gripLastMs = now;
  if (dt > 0.05f) dt = 0.05f;

  if (!GRIPPER_360 && want != 0) {
    grip = constrain(grip + want * GRIP_SPEED * dt, 0.0f, 1.0f);
    writeJaws(false);
  }

  if (want == gripDir) return;
  gripDir = want;

  if (want == 0) {
    gripper_stop();
    if (GRIPPER_360) {
      Serial.println("gripper stop");
    } else {
      Serial.printf("gripper stop at %d%% closed (A=%d us, B=%d us)\n",
                    (int)(grip * 100), lastUsA, lastUsB);
    }
  } else {
    if (GRIPPER_360) {
      writeUs(CH_A, NEUTRAL_A_US + want * DIR_A * SPEED_US);
      writeUs(CH_B, NEUTRAL_B_US + want * DIR_B * SPEED_US);
    }
    Serial.println(want > 0 ? "gripper closing" : "gripper opening");
  }
}

// =====================================================================
// ---------- Remote Control Receiver (from ESP32 #2) ----------
// =====================================================================
void applyRemoteControl(bool up, bool down, bool left, bool right,
                        bool l1, bool r1, bool y, bool a,
                        bool closeGrip, bool openGrip,
                        int mag, bool dpadUsed) {
  updateGripper(closeGrip, openGrip);

  int drivePwm = 255;
  if (!dpadUsed && mag > STICK_DEADZONE) {
    drivePwm = map(constrain(mag, STICK_DEADZONE, 512), STICK_DEADZONE, 512, PWM_MIN, 255);
  }

  if (up && left) {
    pwm = drivePwm;
    diagonal_kiri_maju();
  } else if (up && right) {
    pwm = drivePwm;
    diagonal_kanan_maju();
  } else if (up) {
    pwm = drivePwm;
    maju();
  } else if (left) {
    pwm = drivePwm;
    bel_ki();
  } else if (right) {
    pwm = drivePwm;
    bel_ka();
  } else if (down) {
    pwm = drivePwm;
    mundur();
  } else if (l1) {
    pwm = 255;
    geser_kiri();
  } else if (r1) {
    pwm = 255;
    geser_kanan();
  } else if (y) {
    lifter_up();
  } else if (a) {
    lifter_down();
  } else {
    pwm = 0;
    lifter_stopped();
    stopped();
  }
}

void handleRemoteSerial() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (uartLine.length() == 0) continue;

      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, uartLine);
      uartLine = "";
      if (err) continue;

      bool up    = doc["up"]   | false;
      bool down  = doc["down"] | false;
      bool left  = doc["left"] | false;
      bool right = doc["right"]| false;
      bool l1    = doc["l1"]   | false;
      bool r1    = doc["r1"]   | false;
      bool y     = doc["y"]    | false;
      bool a     = doc["a"]    | false;
      bool x     = doc["x"]    | false;
      bool b     = doc["b"]    | false;
      bool dpad  = doc["dpad"] | false;
      int mag    = doc["mag"]  | 0;

      mag = constrain(mag, 0, 512);
      applyRemoteControl(up, down, left, right, l1, r1, y, a, x, b, mag, dpad);
      lastRemoteMs = millis();
    } else {
      if (uartLine.length() < 255) uartLine += c;
    }
  }
}

// =====================================================================
// ---------- Utilities & Diagnostics ----------
// =====================================================================
const char* resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power on (normal)";
    case ESP_RST_SW:        return "software reset";
    case ESP_RST_PANIC:     return "CRASH (panic)";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (power dip!)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:       return "watchdog";
    default:                return "other";
  }
}

void handleSerialTuning() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("off")) {
    servoOff(CH_A);
    servoOff(CH_B);
    lastUsA = lastUsB = -1;
    gripDir = 0;
    Serial.println("servos off");
    return;
  }

  int sp = line.indexOf(' ');
  if (sp < 0) {
    Serial.println("Format: 14 1500   or   off");
    return;
  }

  int ch = line.substring(0, sp).toInt();
  int us = line.substring(sp + 1).toInt();
  if (ch < 0 || ch > 15 || us < 500 || us > 2500) {
    Serial.println("Channel must be 0-15, value 500-2500 us");
    return;
  }

  gripDir = 0;
  if (ch == CH_A) lastUsA = us;
  if (ch == CH_B) lastUsB = us;
  writeUs(ch, us);
  Serial.printf("ch%d -> %d us\n", ch, us);
}

void komunikasi_test() {
    // KIRIM JSON setiap 1 detik
  if (millis() - lastSend >= 1000) {
    lastSend = millis();

    JsonDocument doc;
    doc["device"] = "ESP32_1";
    doc["counter"] = counter++;
    doc["value"] = 100;
    doc["status"] = "online";

    serializeJson(doc, Serial1);
    Serial1.println();

    Serial.print("KIRIM: ");
    serializeJson(doc, Serial);
    Serial.println();
  }

  // TERIMA JSON dari ESP32 2
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      JsonDocument doc;
      DeserializationError err =
          deserializeJson(doc, rxBuffer);

      if (!err) {
        Serial.print("TERIMA: ");
        serializeJson(doc, Serial);
        Serial.println();
      } else {
        Serial.println("JSON tidak valid");
      }

      rxBuffer = "";
    } else if (c != '\r') {
      if (rxBuffer.length() < 250) {
        rxBuffer += c;
      } else {
        rxBuffer = "";
      }
    }
  }
}

// =====================================================================
// ---------- Setup & Loop ----------
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("ESP32 1 siap mengirim JSON");
  Serial.printf("\n=== BOOT === reset reason: %s\n", resetReasonName());

  servo.begin();
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(SERVO_FREQ);
  delay(10);

  Wire.beginTransmission(0x40);
  Serial.println(Wire.endTransmission() == 0 ? "PCA9685 found at 0x40"
                                            : "PCA9685 NOT FOUND - check SDA/SCL/VCC/GND");

  if (GRIPPER_360) {
    gripper_stop();
  } else {
    writeJaws(true);
  }
  gripLastMs = millis();

  // Direction pins setup
  int outPins[] = {
    Dp_kn_mj, Dp_kn_md, Dp_kr_mj, Dp_kr_md,
    Bl_kn_mj, Bl_kn_md, Bl_kr_mj, Bl_kr_md,
    IN1, IN2
  };
  for (int p : outPins) pinMode(p, OUTPUT);

  // PWM setup
  ledcSetup(CH_ENA_Dp_kn, frequency, resolution);
  ledcSetup(CH_ENB_Dp_kr, frequency, resolution);
  ledcSetup(CH_ENA_Bl_kn, frequency, resolution);
  ledcSetup(CH_ENB_Bl_kr, frequency, resolution);

  ledcAttachPin(Dp_kn_speed, CH_ENA_Dp_kn);
  ledcAttachPin(Dp_kr_speed, CH_ENB_Dp_kr);
  ledcAttachPin(Bl_kn_speed, CH_ENA_Bl_kn);
  ledcAttachPin(Bl_kr_speed, CH_ENB_Bl_kr);

  ledcWrite(CH_ena, 255);

  stopped();
  lifter_stopped();
  Serial.println("Ready. Waiting remote control from ESP32 #2.");
}

void loop() {
  handleSerialTuning();
  handleRemoteSerial();

  if (millis() - lastRemoteMs > REMOTE_TIMEOUT_MS) {
    stopped();
    lifter_stopped();
    updateGripper(false, false);
  }

  delay(5);
}