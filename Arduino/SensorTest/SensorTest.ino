#include <Wire.h>
#include <VL53L0X.h>

#define I2C_SENSOR_SPEED  400000
#define SENSOR_ADDR       0x29
#define TIMEOUT_SENSOR    500
#define GPIO              2

VL53L0X sensor;

void setup() {

  delay(1000); // Without this, the first serial messages are not received
  
  Serial.begin(9600);
  Serial.println("Started serial communication");
  pinMode(LED_BUILTIN, OUTPUT);
  
  Wire.setClock(I2C_SENSOR_SPEED);
  Wire.begin();

  pinMode(GPIO, OUTPUT);
  digitalWrite(GPIO, LOW);
  digitalWrite(GPIO, HIGH);

  sensor.setTimeout(TIMEOUT_SENSOR);
  sensor.setAddress(SENSOR_ADDR);
  
  if ( ! sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    error();
  }
  if( ! sensor.setSignalRateLimit(0.1)) {
    Serial.println("Failed to set signal rate limit!");
    error();
  }
  if( ! sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18)) {
    Serial.println("Failed to set Vcsel pulse period pre range!");
    error();
  }
  if( ! sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14)) {
    Serial.println("Failed to set Vcsel pulse period final range!");
    error();
  }

  sensor.startContinuous();
}

void error() {
  while(true) {
    Serial.println("Error");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  delay(50);
}
