/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>
#include <FastLED.h>

#define LED_PIN     5
#define NUM_LEDS    3
#define BRIGHTNESS  12
#define LED_TYPE    WS2812
#define COLOR_ORDER RGB
#define XSHUT2 2

CRGB leds[NUM_LEDS];
VL53L0X sensor1;
VL53L0X sensor2;

void setup()
{
  delay(2000);
  FastLED.addLeds<LED_TYPE, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  Serial.begin(9600);
  Wire.begin();
  
  pinMode(XSHUT2, OUTPUT);
  digitalWrite(XSHUT2, LOW);

  init_sensor_1();
  init_sensor_2();
}

void init_sensor_1()
{
  sensor1.init();
  sensor1.setTimeout(500);
  sensor1.setAddress(0x53);
  sensor1.setSignalRateLimit(0.1);
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

void init_sensor_2()
{
  digitalWrite(XSHUT2, HIGH);
  sensor2.init();
  sensor2.setTimeout(500);
  sensor2.setSignalRateLimit(0.1);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

void read_sensor_1()
{
  Serial.print("Sensor 1 : ");
  int dist = sensor1.readRangeSingleMillimeters();

  if (dist < 250)
    leds[0] = CRGB::Red;
   else if (dist < 500)
    leds[0] = CRGB::Green;
   else if (dist < 750)
    leds[0] = CRGB::Blue;
   else
    leds[0] = CRGB::Black;
  
  Serial.print(dist);
  if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
}

void read_sensor_2()
{
  Serial.print("Sensor 2 : ");
  int dist = sensor2.readRangeSingleMillimeters();

  if (dist < 250)
    leds[1] = CRGB::Red;
   else if (dist < 500)
    leds[1] = CRGB::Green;
   else if (dist < 750)
    leds[1] = CRGB::Blue;
   else
    leds[1] = CRGB::Black;
 
  Serial.print(dist);
  if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
}

void loop()
{
  read_sensor_1();
  read_sensor_2();
  FastLED.show();
  delay(100);
}
