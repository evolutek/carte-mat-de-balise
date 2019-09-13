/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor1;
VL53L0X sensor2;

int xshut2 = 2;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  pinMode(xshut2, OUTPUT);
  digitalWrite(xshut2, LOW);

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
  digitalWrite(xshut2, HIGH);
  sensor2.init();
  sensor2.setTimeout(500);
  sensor2.setSignalRateLimit(0.1);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

void read_sensor_1()
{
  Serial.print("Sensor 1 : ");
  Serial.print(sensor1.readRangeSingleMillimeters());
  if (sensor1.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
}

void read_sensor_2()
{
  Serial.print("Sensor 2 : ");
  Serial.print(sensor2.readRangeSingleMillimeters());
  if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
}

void loop()
{
  read_sensor_1();
  read_sensor_2();
  delay(250);
}
