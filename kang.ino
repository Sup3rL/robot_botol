#include <Arduino.h>

// ===================================
// PIN ENCODER
// ===================================

// Kanan depan
#define En_Kn_dp_A 12
#define En_Kn_dp_B 13

// Kiri depan
#define En_kr_dp_A 33
#define En_kr_dp_B 32

// Kanan belakang
#define En_kn_bl_A 25
#define En_kn_bl_B 26

// Kiri belakang
#define En_kr_bl_A 14
#define En_kr_bl_B 27

// ===================================
// COUNTER
// ===================================

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

// ===================================
// FUNGSI INTERRUPT
// ===================================

void IRAM_ATTR isr_Kn_dp() {
  uint8_t s = (digitalRead(En_Kn_dp_A) << 1)
            | digitalRead(En_Kn_dp_B);
  count_Kn_dp += qdec[(state_Kn_dp << 2) | s];
  state_Kn_dp = s;
}

void IRAM_ATTR isr_kr_dp() {
  uint8_t s = (digitalRead(En_kr_dp_A) << 1)
            | digitalRead(En_kr_dp_B);
  count_kr_dp += qdec[(state_kr_dp << 2) | s];
  state_kr_dp = s;
}

void IRAM_ATTR isr_kn_bl() {
  uint8_t s = (digitalRead(En_kn_bl_A) << 1)
            | digitalRead(En_kn_bl_B);
  count_kn_bl += qdec[(state_kn_bl << 2) | s];
  state_kn_bl = s;
}

void IRAM_ATTR isr_kr_bl() {
  uint8_t s = (digitalRead(En_kr_bl_A) << 1)
            | digitalRead(En_kr_bl_B);
  count_kr_bl += qdec[(state_kr_bl << 2) | s];
  state_kr_bl = s;
}

// ===================================
// SETUP
// ===================================

void setup() {
  Serial.begin(115200);

  pinMode(En_Kn_dp_A, INPUT);
  pinMode(En_Kn_dp_B, INPUT);
  pinMode(En_kr_dp_A, INPUT);
  pinMode(En_kr_dp_B, INPUT);
  pinMode(En_kn_bl_A, INPUT);
  pinMode(En_kn_bl_B, INPUT);
  pinMode(En_kr_bl_A, INPUT);
  pinMode(En_kr_bl_B, INPUT);

  // Simpan kondisi awal semua channel
  state_Kn_dp = (digitalRead(En_Kn_dp_A) << 1)
              | digitalRead(En_Kn_dp_B);
  state_kr_dp = (digitalRead(En_kr_dp_A) << 1)
              | digitalRead(En_kr_dp_B);
  state_kn_bl = (digitalRead(En_kn_bl_A) << 1)
              | digitalRead(En_kn_bl_B);
  state_kr_bl = (digitalRead(En_kr_bl_A) << 1)
              | digitalRead(En_kr_bl_B);

  // Interrupt kedua channel setiap encoder
  attachInterrupt(digitalPinToInterrupt(En_Kn_dp_A),
                  isr_Kn_dp, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_Kn_dp_B),
                  isr_Kn_dp, CHANGE);

  attachInterrupt(digitalPinToInterrupt(En_kr_dp_A),
                  isr_kr_dp, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_kr_dp_B),
                  isr_kr_dp, CHANGE);

  attachInterrupt(digitalPinToInterrupt(En_kn_bl_A),
                  isr_kn_bl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_kn_bl_B),
                  isr_kn_bl, CHANGE);

  attachInterrupt(digitalPinToInterrupt(En_kr_bl_A),
                  isr_kr_bl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(En_kr_bl_B),
                  isr_kr_bl, CHANGE);

  Serial.println("ESP32 4 Motor Encoder Counter");
  Serial.println("Putar encoder untuk menguji.");
}

// ===================================
// LOOP
// ===================================

void loop() {
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