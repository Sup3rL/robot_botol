#include <Arduino.h>

// ESP1 link test (monitor di ESP1)
// ESP1 RX = 21, TX = 22
// Pasangan ESP2: RX = 19, TX = 23
#define UART_RX 21
#define UART_TX 22

unsigned long lastSendMs = 0;
uint32_t seqTx = 0;
String line;
unsigned long lastRxMs = 0;
unsigned long lastStatusMs = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  Serial.println("[ESP1] UART 2-board link test");
  Serial.println("[ESP1] RX=21 TX=22 baud=115200");
  Serial.println("[ESP1] Wiring: TX22->ESP2 RX19, RX21<-ESP2 TX23, GND common");
}

void loop() {
  // Kirim PING ke ESP2 tiap 1000 ms
  if (millis() - lastSendMs >= 1000) {
    lastSendMs = millis();
    seqTx++;

    // Format: PING,<seq>,<millis>
    Serial1.printf("PING,%lu,%lu\n", (unsigned long)seqTx, (unsigned long)millis());
    Serial.printf("[ESP1 TX] PING seq=%lu\n", (unsigned long)seqTx);
  }

  // Baca data masuk per baris
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (line.length() > 0) {
        Serial.print("[ESP1 RX] ");
        Serial.println(line);
        if (line.startsWith("ACK_FROM_ESP2,")) {
          lastRxMs = millis();
          Serial.println("[ESP1 OK] Link to ESP2 works");
        }
      }
      line = "";
    } else {
      if (line.length() < 180) line += c;
      else line = "";
    }
  }

  // Status link tiap 1 detik
  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    if (lastRxMs > 0 && (millis() - lastRxMs) <= 2500) {
      Serial.println("[ESP1 STATUS] LINK UP");
    } else {
      Serial.println("[ESP1 STATUS] LINK DOWN (check wiring / power ESP2)");
    }
  }
}
