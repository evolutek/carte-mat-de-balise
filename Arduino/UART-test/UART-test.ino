#include <SoftwareSerial.h>

#define NUCLEO Serial1
#define LIDAR Serial2

void setup() {
  Serial.begin(115200);
  NUCLEO.begin(115200);
  LIDAR.begin(115200)
  while (!Serial);
  while (!NUCLEO);
  while (!LIDAR);

  Serial.println("Ready");
}

void loop() {
  if (NUCLEO.available()) {
    NUCLEO.print("Nucleo: 0x");
    NUCLEO.print(String(Serial.read(), HEX));
    NUCLEO.println();
  }
  if (LIDAR.available()) {
    LIDAR.print("Lidar: 0x");
    LIDAR.print(String(Serial.read(), HEX));
    LIDAR.println();
  }
}
