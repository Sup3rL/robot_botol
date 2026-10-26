#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();  // PCA9685 at 0x40

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ---------- Pins ----------

// DRIVER 1 (MOTOR DEPAN)
const int Dp_kn_speed = 13; // Speed Kanan Depan (13)
const int Dp_kn_mj = 12;
const int Dp_kn_md = 14;

const int Dp_kr_speed = 15; // Speed Kiri Depan (Diubah jadi 15)
const int Dp_kr_mj = 27;
const int Dp_kr_md = 26;

// DRIVER 2 (MOTOR BELAKANG)
const int Bl_kn_speed = 25; // Speed Kanan Belakang (Diubah jadi 25)
const int Bl_kn_mj = 4;
const int Bl_kn_md = 2; 

const int Bl_kr_speed = 5;  // Speed Kiri Belakang (5)
const int Bl_kr_mj = 17; 
const int Bl_kr_md = 16; 

// LIFTER
const int rem = 18;
const int ena = 32;  
const int IN2 = 19;
const int IN1 = 23;

int pwm = 0;

// ----- Joystick tuning -----
const int STICK_DEADZONE = 120;  
const int PWM_MIN = 130;         
const int frequency = 30000;
const int resolution = 8;

// PWM channels
const int CH_ENA_Dp_kn = 0;
const int CH_ENB_Dp_kr = 1;
const int CH_ENA_Bl_kn = 2;
const int CH_ENB_Bl_kr = 3;
const int CH_ena = 4;

int channelFor(int pin) {
  switch (pin) {
    case Dp_kn_speed: return CH_ENA_Dp_kn;
    case Dp_kr_speed: return CH_ENB_Dp_kr;
    case Bl_kn_speed: return CH_ENA_Bl_kn;
    case Bl_kr_speed: return CH_ENB_Bl_kr;
    default: return CH_ena;
  }
}

// ---------- Helper ----------
// Sets one motor: enable pin duty + two direction pins
void motor(int en, int pin_maju, int pin_mundur, int duty, int status_maju, int status_mundur) {
  ledcWrite(channelFor(en), duty);
  digitalWrite(pin_maju, status_maju);
  digitalWrite(pin_mundur, status_mundur);
}

// ---------- Movement ----------
// HIGH, LOW = Maju
// LOW, HIGH = Mundur
// LOW, LOW  = Berhenti

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
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, 0, LOW, LOW);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, pwm, HIGH, LOW);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, pwm, HIGH, LOW);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, 0, LOW, LOW);
}

void diagonal_kiri_maju() {
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, pwm, HIGH, LOW);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, 0, LOW, LOW);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, 0, LOW, LOW);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, pwm, HIGH, LOW);
}

void stopped() {
  motor(Dp_kn_speed, Dp_kn_mj, Dp_kn_md, 0, LOW, LOW);
  motor(Dp_kr_speed, Dp_kr_mj, Dp_kr_md, 0, LOW, LOW);
  motor(Bl_kn_speed, Bl_kn_mj, Bl_kn_md, 0, LOW, LOW);
  motor(Bl_kr_speed, Bl_kr_mj, Bl_kr_md, 0, LOW, LOW);
}

// ---------- Lifter ----------
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
// ---------- Gripper: two identical MG996R servos ----------
const bool GRIPPER_360 = false;

const int CH_A = 14;  
const int CH_B = 15;  

const int OPEN_A_US = 1000;    
const int CLOSE_A_US = 2800;   
const int OPEN_B_US = 2000;    
const int CLOSE_B_US = 200;    
const float START_GRIP = 0.0;  
const float GRIP_SPEED = 1.0;  

const int NEUTRAL_A_US = 1500;  
const int NEUTRAL_B_US = 1500;  
const int SPEED_US = 150;       
const int DIR_A = 1;            
const int DIR_B = -1;           

const int SERVO_FREQ = 50;  
float grip = START_GRIP;    
int lastUsA = -1, lastUsB = -1;
int gripDir = 0;  
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
    if (GRIPPER_360) Serial.println("gripper stop");
    else Serial.printf("gripper stop at %d%% closed (A=%d us, B=%d us)\n",
                       (int)(grip * 100), lastUsA, lastUsB);
  } else {
    if (GRIPPER_360) {
      writeUs(CH_A, NEUTRAL_A_US + want * DIR_A * SPEED_US);
      writeUs(CH_B, NEUTRAL_B_US + want * DIR_B * SPEED_US);
    }
    Serial.println(want > 0 ? "gripper closing" : "gripper opening");
  }
}

// ---------- Bluepad32 callbacks ----------
void onConnectedController(ControllerPtr ctl) {
  for (int k = 0; k < BP32_MAX_GAMEPADS; k++) {
    if (myControllers[k] == nullptr) {
      myControllers[k] = ctl;
      ControllerProperties p = ctl->getProperties();
      Serial.printf("Controller connected: %s, VID=0x%04x PID=0x%04x\n",
                    ctl->getModelName().c_str(), p.vendor_id, p.product_id);
      return;
    }
  }
  Serial.println("Controller connected, but no free slot");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int k = 0; k < BP32_MAX_GAMEPADS; k++) {
    if (myControllers[k] == ctl) {
      myControllers[k] = nullptr;
      Serial.println("Controller disconnected");
    }
  }
  pwm = 0;
  stopped();
  lifter_stopped();
  updateGripper(false, false);
}

// ---------- Button handling ----------
void processGamepad(ControllerPtr ctl) {
  updateGripper(ctl->x(), ctl->b());

  uint8_t d = ctl->dpad();
  bool dUp = d & DPAD_UP;
  bool dDown = d & DPAD_DOWN;
  bool dLeft = d & DPAD_LEFT;
  bool dRight = d & DPAD_RIGHT;
  bool dpadUsed = dUp || dDown || dLeft || dRight;

  int x = ctl->axisX();
  int y = ctl->axisY();
  int ax = abs(x), ay = abs(y);
  int mag = max(ax, ay);

  bool jUp = false, jDown = false, jLeft = false, jRight = false;
  if (mag > STICK_DEADZONE) {
    int minAxis = mag * 4 / 10;
    if (ay > STICK_DEADZONE && ay >= minAxis) {
      jUp = (y < 0);
      jDown = (y > 0);
    }
    if (ax > STICK_DEADZONE && ax >= minAxis) {
      jLeft = (x < 0);
      jRight = (x > 0);
    }
  }

  bool up = dUp || jUp;
  bool down = dDown || jDown;
  bool left = dLeft || jLeft;
  bool right = dRight || jRight;

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
  } else if (ctl->l1()) { 
    pwm = 255;
    geser_kiri();
  } else if (ctl->r1()) { 
    pwm = 255;
    geser_kanan();
  } else if (ctl->y()) {
    lifter_up();
  } else if (ctl->a()) {
    lifter_down();
  } else {
    pwm = 0;
    lifter_stopped();
    stopped();
  }
}

const char* resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power on (normal)";
    case ESP_RST_SW: return "software reset";
    case ESP_RST_PANIC: return "CRASH (panic)";
    case ESP_RST_BROWNOUT: return "BROWNOUT (power dip!)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    default: return "other";
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

void setup() {
  Serial.begin(115200);
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
  int outPins[] = { Dp_kn_mj, Dp_kn_md, Dp_kr_mj, Dp_kr_md, 
                    Bl_kn_mj, Bl_kn_md, Bl_kr_mj, Bl_kr_md, 
                    IN1, IN2 };
  for (int p : outPins) pinMode(p, OUTPUT);

  // PWM setup
  ledcSetup(CH_ENA_Dp_kn, frequency, resolution);
  ledcSetup(CH_ENB_Dp_kr, frequency, resolution);
  ledcSetup(CH_ENA_Bl_kn, frequency, resolution);
  ledcSetup(CH_ENB_Bl_kr, frequency, resolution);
  ledcSetup(CH_ena, frequency, resolution);

  ledcAttachPin(Dp_kn_speed, CH_ENA_Dp_kn);
  ledcAttachPin(Dp_kr_speed, CH_ENB_Dp_kr);
  ledcAttachPin(Bl_kn_speed, CH_ENA_Bl_kn);
  ledcAttachPin(Bl_kr_speed, CH_ENB_Bl_kr);
  ledcAttachPin(ena, CH_ena);
  
  ledcWrite(CH_ena, 255);  

  stopped();
  lifter_stopped();

  // Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);
  Serial.println("Ready. Put the G90 in Bluetooth pairing mode.");
}

void loop() {
  BP32.update();
  handleSerialTuning();

  bool anyConnected = false;
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->isGamepad()) {
      processGamepad(ctl);
      anyConnected = true;
      break; 
    }
  }

  if (!anyConnected) {
    stopped();
    lifter_stopped();
    updateGripper(false, false);
  }

  delay(5);
}