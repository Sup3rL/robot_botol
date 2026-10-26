#include <Arduino.h>
#include <ArduinoJson.h>
#include <Bluepad32.h>

// =====================================================================
// ---------- Konfigurasi Pin & Objek Gamepad ----------
// =====================================================================
#define UART_RX 19
#define UART_TX 23

// Buffer penerimaan data JSON UART
unsigned long lastSend = 0;
uint32_t counter = 0;
String rxBuffer = "";

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

const int STICK_DEADZONE = 120;
unsigned long lastControlTxMs = 0;

// =====================================================================
// ---------- Pengiriman Paket Kontrol (UART JSON) ----------
// =====================================================================
void sendControlPacket(bool up, bool down, bool left, bool right, bool l1, bool r1, bool y, bool a, bool x, bool b, bool dpadUsed, int mag) {
  JsonDocument doc;
  doc["src"]   = "esp2";
  doc["up"]    = up;
  doc["down"]  = down;
  doc["left"]  = left;
  doc["right"] = right;
  doc["l1"]    = l1;
  doc["r1"]    = r1;
  doc["y"]     = y;
  doc["a"]     = a;
  doc["x"]     = x;
  doc["b"]     = b;
  doc["dpad"]  = dpadUsed;
  doc["mag"]   = constrain(mag, 0, 512);

  serializeJson(doc, Serial1);
  Serial1.println();
}

void sendStopPacket() {
  sendControlPacket(false, false, false, false,
                    false, false, false, false,
                    false, false, false, 0);
}

// =====================================================================
// ---------- Bluepad32 Callbacks & Handler ----------
// =====================================================================
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      ControllerProperties p = ctl->getProperties();
      Serial.printf("Controller on ESP2: %s, VID=0x%04x PID=0x%04x\n",
                    ctl->getModelName().c_str(), p.vendor_id, p.product_id);
      return;
    }
  }
  Serial.println("Controller connected, no free slot");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      Serial.println("Controller disconnected from ESP2");
      sendStopPacket();
      return;
    }
  }
}

void processAndSendGamepad(ControllerPtr ctl) {
  uint8_t d = ctl->dpad();
  bool dUp    = d & DPAD_UP;
  bool dDown  = d & DPAD_DOWN;
  bool dLeft  = d & DPAD_LEFT;
  bool dRight = d & DPAD_RIGHT;
  bool dpadUsed = dUp || dDown || dLeft || dRight;

  int x = ctl->axisX();
  int y = ctl->axisY();
  int ax = abs(x);
  int ay = abs(y);
  int mag = max(ax, ay);

  bool jUp = false, jDown = false, jLeft = false, jRight = false;
  if (mag > STICK_DEADZONE) {
    int minAxis = mag * 4 / 10;
    if (ay > STICK_DEADZONE && ay >= minAxis) {
      jUp   = (y < 0);
      jDown = (y > 0);
    }
    if (ax > STICK_DEADZONE && ax >= minAxis) {
      jLeft  = (x < 0);
      jRight = (x > 0);
    }
  }

  bool up    = dUp || jUp;
  bool down  = dDown || jDown;
  bool left  = dLeft || jLeft;
  bool right = dRight || jRight;

  sendControlPacket(up, down, left, right, ctl->l1(), ctl->r1(), ctl->y(), ctl->a(), ctl->x(), ctl->b(), dpadUsed, mag);
}

// =====================================================================
// ---------- Pin Encoder & Lifter ----------
// =====================================================================

// Kanan depan
#define En_Kn_dp_A 12
#define En_Kn_dp_B 13

// Kiri depan
#define En_kr_dp_A 32
#define En_kr_dp_B 33

// Kanan belakang
#define En_kn_bl_A 25
#define En_kn_bl_B 26

// Kiri belakang
#define En_kr_bl_A 27
#define En_kr_bl_B 14

// Pin Lifter (Up/Down/Right/Left)
#define LIFTER_UP    16
#define LIFTER_DOWN  17
#define LIFTER_RIGHT 18
#define LIFTER_LEFT  4

// =====================================================================
// ---------- Counter & Quadrature Decoder ----------
// =====================================================================
volatile long count_Kn_dp = 0;
volatile long count_kr_dp = 0;
volatile long count_kn_bl = 0;
volatile long count_kr_bl = 0;

// State quadrature sebelumnya
volatile uint8_t state_Kn_dp = 0;
volatile uint8_t state_kr_dp = 0;
volatile uint8_t state_kn_bl = 0;
volatile uint8_t state_kr_bl = 0;

// Tabel pembacaan quadrature
const int8_t qdec[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// =====================================================================
// ---------- Fungsi Interrupt (ISR) ----------
// =====================================================================
void IRAM_ATTR isr_Kn_dp() {
  uint8_t s = (digitalRead(En_Kn_dp_A) << 1) | digitalRead(En_Kn_dp_B);
  count_Kn_dp += qdec[(state_Kn_dp << 2) | s];
  state_Kn_dp = s;
}

void IRAM_ATTR isr_kr_dp() {
  uint8_t s = (digitalRead(En_kr_dp_A) << 1) | digitalRead(En_kr_dp_B);
  count_kr_dp += qdec[(state_kr_dp << 2) | s];
  state_kr_dp = s;
}

void IRAM_ATTR isr_kn_bl() {
  uint8_t s = (digitalRead(En_kn_bl_A) << 1) | digitalRead(En_kn_bl_B);
  count_kn_bl += qdec[(state_kn_bl << 2) | s];
  state_kn_bl = s;
}

void IRAM_ATTR isr_kr_bl() {
  uint8_t s = (digitalRead(En_kr_bl_A) << 1) | digitalRead(En_kr_bl_B);
  count_kr_bl += qdec[(state_kr_bl << 2) | s];
  state_kr_bl = s;
}

// =====================================================================
// ---------- Setup ----------
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("ESP32 2 siap: gamepad -> UART + encoder");

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  pinMode(En_Kn_dp_A, INPUT);
  pinMode(En_Kn_dp_B, INPUT);
  pinMode(En_kr_dp_A, INPUT);
  pinMode(En_kr_dp_B, INPUT);
  pinMode(En_kn_bl_A, INPUT);
  pinMode(En_kn_bl_B, INPUT);
  pinMode(En_kr_bl_A, INPUT);
  pinMode(En_kr_bl_B, INPUT);

  // Simpan kondisi awal semua channel
  state_Kn_dp = (digitalRead(En_Kn_dp_A) << 1) | digitalRead(En_Kn_dp_B);
  state_kr_dp = (digitalRead(En_kr_dp_A) << 1) | digitalRead(En_kr_dp_B);
  state_kn_bl = (digitalRead(En_kn_bl_A) << 1) | digitalRead(En_kn_bl_B);
  state_kr_bl = (digitalRead(En_kr_bl_A) << 1) | digitalRead(En_kr_bl_B);

  // Interrupt kedua channel setiap encoder
  attachInterrupt(digitalPinToInterrupt(En_Kn_dp_A), isr_Kn_dp, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_Kn_dp_B), isr_Kn_dp, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_kr_dp_A), isr_kr_dp, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_kr_dp_B), isr_kr_dp, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_kn_bl_A), isr_kn_bl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_kn_bl_B), isr_kn_bl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_kr_bl_A), isr_kr_bl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_kr_bl_B), isr_kr_bl, CHANGE);

  Serial.println("ESP32 4 Motor Encoder Counter");
}

// =====================================================================
// ---------- Uji Komunikasi Serial ESP32 ----------
// =====================================================================
void komunikasi_test() {
  if (millis() - lastSend >= 1000) {
    lastSend = millis();

    JsonDocument doc;
    doc["device"] = "ESP32_2";
    doc["counter"] = counter++;
    doc["value"] = 200;
    doc["status"] = "online";

    serializeJson(doc, Serial1);
    Serial1.println();

    Serial.print("KIRIM: ");
    serializeJson(doc, Serial);
    Serial.println();
  }

  // TERIMA JSON dari ESP32 1
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
// ---------- Main Loop ----------
// =====================================================================
void loop() {
  BP32.update();

  // Proses penerimaan data serial UART non-blocking
  komunikasi_test();

  bool sent = false;
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->isGamepad()) {
      if (millis() - lastControlTxMs >= 20) {
        processAndSendGamepad(ctl);
        lastControlTxMs = millis();
      }
      sent = true;
      break;
    }
  }

  if (!sent && millis() - lastControlTxMs >= 100) {
    sendStopPacket();
    lastControlTxMs = millis();
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();

    long kn_dp, kr_dp, kn_bl, kr_bl;

    // Baca counter dengan aman
    noInterrupts();
    kn_dp = count_Kn_dp;
    kr_dp = count_kr_dp;
    kn_bl = count_kn_bl;
    kr_bl = count_kr_bl;
    interrupts();

    Serial.printf(
      "Kn_dp: %ld | kr_dp: %ld | kn_bl: %ld | kr_bl: %ld\n",
      kn_dp, kr_dp, kn_bl, kr_bl
    );
  }
}