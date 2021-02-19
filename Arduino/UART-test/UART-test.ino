#include <SoftwareSerial.h>

SoftwareSerial serial(2, 3); // RX, TX

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Ready");

  serial.begin(115200);
}

void loop() {
  if (serial.available()) {
    Serial.print("0x");
    Serial.print(String(serial.read(), HEX));
    Serial.println();
  }
}
