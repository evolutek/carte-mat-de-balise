#include <Wire.h>
#include <VL53L0X.h>
#include <FastLED.h>

#define BRIGHTNESS      128
#define COLOR_ORDER     RGB
#define LED_PIN         36
#define LED_TYPE        WS2812
#define MAX_DIST        2000
#define NB_SENSORS      2
#define REFRESH         100
#define SENSOR_ADDR     0x52



float   coef = 255.0 / (MAX_DIST / 2);
CRGB    leds[NB_SENSORS];
VL53L0X sensors[NB_SENSORS];

void setup() {

  // Init Led strip
  FastLED.addLeds<LED_TYPE, LED_PIN, GRB>(leds, NB_SENSORS);
  FastLED.setBrightness(BRIGHTNESS);

  Serial.begin(9600);
  Wire.begin();

  // Init xshut pins
  for (int i = 0; i < NB_SENSORS; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  // Init sensors
  for (int i = NB_SENSORS -1; i >= 0; i--) {
    digitalWrite(i, HIGH);
    init_sensor(i);
  }

}

void init_sensor(int i) {
  Serial.print("Init sensor nb: ");
  Serial.println(i);
  sensors[i].init();
  sensors[i].setTimeout(500);
  sensors[i].setAddress(SENSOR_ADDR + i);
  sensors[i].setSignalRateLimit(0.1);
  sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

void loop() {
  Serial.print("---- STARTING SCAN -----\n");

  for (int i = 0; i < NB_SENSORS; i++) {
    Serial.print("Sensor nb : ");
    Serial.print(i + 1);
    Serial.print(" dist :");

    int dist = sensors[i].readRangeSingleMillimeters();

    Serial.print(dist);

    if (sensors[i].timeoutOccurred())
      Serial.print(" TIMEOUT");
    Serial.println();

    if (dist > MAX_DIST)
      leds[i] = CRGB::Black;
    else {
      int r = 256 - dist * coef;
      int g = 256 - abs(MAX_DIST / 2 - dist) * coef;
      int b = 256 - (MAX_DIST - dist) * coef;
      leds[i].red = r >= 0 ? r : 0;
      leds[i].green = g >= 0 ? g : 0;
      leds[i].blue = b >= 0 ? b : 0;
    }
  }

  Serial.print("---- END SCAN -----\n");

  FastLED.show();
  delay(REFRESH);

}
