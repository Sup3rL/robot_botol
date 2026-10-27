#include <Arduino.h>

// ESP2 responder for 2-board link test
// ESP2 RX = 19, TX = 23
#define UART_RX 19
#define UART_TX 23

String line;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  Serial.println("[ESP2] UART responder start");
  Serial.println("[ESP2] RX=19 TX=23 baud=115200");
}

void loop() {
  // Baca data masuk per baris
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (line.length() > 0) {
        Serial.print("[ESP2 RX] ");
        Serial.println(line);

        // Balas ACK jika terima PING dari ESP1
        if (line.startsWith("PING")) {
          Serial1.print("ACK_FROM_ESP2,");
          Serial1.println(millis());
          Serial.println("[ESP2 TX] ACK_FROM_ESP2");
        }
      }
      line = "";
    } else {
      if (line.length() < 180) line += c;
      else line = "";
    }
  }
}
