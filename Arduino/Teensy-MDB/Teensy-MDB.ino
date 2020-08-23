#include <Wire.h>
#include <VL53L0X.h>
#include <FastLED.h>
#include <Adafruit_MCP23017.h>


// Sensors
#define AVOID_RANGE       500
#define I2C_SENSOR_SPEED  400000
#define NB_SENSORS        16
#define REFRESH           100
#define ROBOT_RANGE       1000
#define SENSOR_ADDR       0x29
#define TIMEOUT_SENSOR    500
#define XSHUT_START_GPIO  0
#define LONG_RANGE

VL53L0X sensors[NB_SENSORS];
int     scan[NB_SENSORS];
int     front_zone[]  = {15, 16, 1, 2, 3};
int     back_zone[]   = {7, 8, 9, 10, 11};
bool    is_front    = false;
bool    is_back     = false;
bool    is_robot    = false;


// Led strips
#define BRIGHTNESS        128
#define COLOR_ORDER       RGB
#define LED_PIN           29
#define LED_TYPE          WS2812

enum debug_modes {
  DIST,
  ZONES
};

enum debug_modes debug_mode = DIST;
float   coef = 255.0 / (ROBOT_RANGE / 2);
CRGB    leds[NB_SENSORS];


// Communication
#define FRONT_GPIO        32
#define BACK_GPIO         31
#define ROBOT_GPIO        30
#define DEBUG_SERIAL


// Remove
Adafruit_MCP23017 mcp_xshut;

void init_sensor(int i) {
  #ifdef DEBUG_SERIAL
    Serial.print("Init sensor nb: ");
    Serial.println(i + 1);
  #endif

  sensors[i].init();
  sensors[i].setTimeout(TIMEOUT_SENSOR);
  sensors[i].setAddress(SENSOR_ADDR + i);

  // Enable Long Range mode
  #ifdef LONG_RANGE
    sensors[i].setSignalRateLimit(0.1);
    sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif

  // Start continous mode
  sensors[i].startContinuous();
}


void setup() {
  // Init communication
  Wire.setClock(I2C_SENSOR_SPEED);
  Wire.begin();

  #ifdef DEBUG_SERIAL
    Serial.begin(9600);
  #endif

  pinMode(FRONT_GPIO, OUTPUT);
  digitalWrite(FRONT_GPIO, OUTPUT);
  pinMode(BACK_GPIO, OUTPUT);
  digitalWrite(BACK_GPIO, OUTPUT);
  pinMode(ROBOT_GPIO, OUTPUT);
  digitalWrite(ROBOT_GPIO, OUTPUT);

  // Init Led strip
  FastLED.addLeds<LED_TYPE, LED_PIN, GRB>(leds, NB_SENSORS);
  FastLED.setBrightness(BRIGHTNESS);


  // Init XSHUTs
  #ifdef DEBUG_SERIAL
    Serial.println("Init XSHUTs");
  #endif

  // Remove
  mcp_xshut.begin();

  // Set every pin to output
  for (int i = 0; i < 15; ++i)
  {
    // Remove
    mcp_xshut.pinMode(i, OUTPUT);
    mcp_xshut.digitalWrite(i, LOW);

    //pinMode(XSHUT_START_GPIO + i, OUTPUT);
    //digitalWrite(XSHUT_START_GPIO + i, LOW);
    //
  }

  // Init sensors
  for (int i = NB_SENSORS -1; i >= 0; i--) {

    // Remove
    mcp_xshut.digitalWrite(i, HIGH);

    //digitalWrite(XSHUT_START_GPIO + i, HIGH);
    init_sensor(i);
  }
}

bool is_in_array(int n, int array[]) {
  for (int i = 0; i < sizeof(array) / sizeof(int); i++) {
    if (array[i] == n)
      return true;
  }

  return false;
}

void manage_leds() {
  for (int i = 0; i < NB_SENSORS; i++) {

    switch(debug_mode) {

      case DIST:
        if (scan[i] > ROBOT_RANGE)
          leds[i] = CRGB::Black;
        else {
          int r = 255 - scan[i] * coef;
          int g = 255 - abs(ROBOT_RANGE / 2 - scan[i]) * coef;
          int b = 255 - (ROBOT_RANGE - scan[i]) * coef;
          leds[i].red = r >= 0 ? r : 0;
          leds[i].green = g >= 0 ? g : 0;
          leds[i].blue = b >= 0 ? b : 0;
        }
        break;

      case ZONES:
        if (!is_robot)
          leds[i] = CRGB::Green;
        else {
          if (is_front && is_in_array(i, front_zone))
            leds[i] = CRGB::Red;
          else if (is_back && is_in_array(i, back_zone))
            leds[i] = CRGB::Red;
          else
            leds[i] = CRGB::Orange;
        }
        break;

      default:
        break;
    }
  }
}

void loop() {
  #ifdef DEBUG_SERIAL
    Serial.print("---- STARTING SCAN -----\n");
    int start = millis();
  #endif

  bool _is_front = false;
  bool _is_back = false;
  bool _is_robot = false;

  for (int i = 0; i < NB_SENSORS; i++) {
    #ifdef DEBUG_SERIAL
      Serial.print("Sensor nb : ");
      Serial.print(i + 1);
      Serial.print(" dist :");
    #endif

    scan[i] = sensors[i].readRangeContinuousMillimeters();

    // Front detection
    if (scan[i] <= AVOID_RANGE && is_in_array(i, front_zone))
      _is_front = true;

    // Back detection
    if (scan[i] <= AVOID_RANGE && is_in_array(i, back_zone))
      _is_back = true;

    if (scan[i] <= ROBOT_RANGE)
      _is_robot = true;

    #ifdef DEBUG_SERIAL
      Serial.print(scan[i]);
      if (sensors[i].timeoutOccurred())
        Serial.print(" TIMEOUT");
      Serial.println();
    #endif

  }

  #ifdef DEBUG_SERIAL
    Serial.print("---- END SCAN -----\n");
    Serial.print("Ellipsed time: ");
    Serial.println(millis() - start);
  #endif

  is_front = _is_front;
  is_back  = _is_back;
  is_robot = _is_robot;

  manage_leds();
  delay(REFRESH);
}
