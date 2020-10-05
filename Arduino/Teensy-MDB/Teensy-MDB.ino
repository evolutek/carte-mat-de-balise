#include <Wire.h>
#include <VL53L0X.h> // By Pololu v1.2.0
#include <FastLED.h> // By Daniel Garcia v3.3.3

// Sensors
#define AVOID_RANGE       500
#define I2C_SENSOR_SPEED  400000
#define NB_SENSORS        16
#define NB_SAMPLES        5
#define ROBOT_RANGE       1000
#define SENSOR_ADDR       0x30
#define TIMEOUT_SENSOR    500
#define XSHUT_START_GPIO  0
#define LONG_RANGE
#define SENSORS_I2C Wire

VL53L0X sensors[NB_SENSORS];
int     scan[NB_SENSORS];
int     front_zone[]  = {15, 16, 1, 2, 3};
int     back_zone[]   = {7, 8, 9, 10, 11};
int     no_zone[]     = {4, 5, 6, 12, 13, 14};
bool    is_front    = false;
bool    is_back     = false;
bool    is_robot    = false;
int*    samples[NB_SAMPLES];
int     currentSample = 0;
bool    enableSensors = true;
bool    failedSensors[NB_SENSORS];

#define FRONT_PIN 32
#define BACK_PIN 31
#define IS_ROBOT_PIN 30

// Led strips
#define DEFAULT_BRIGHTNESS 1
#define COLOR_ORDER        GRB
#define LED_PIN            29
#define LED_TYPE           WS2812
#define LED_LOADING_U      5
#define LED_LOADING_DELAY  100
#define LED_DISABLED_DELAY 400
#define LED_ORANGE CRGB(255, 35, 0)

enum leds_modes {
  DIST,
  ZONES,
  LOADING,
  DISABLED
};

enum leds_modes leds_mode = LOADING;
float coef = 255.0 / (ROBOT_RANGE / 2);
CRGB led_loading_color = CRGB::Orange; //CRGB::Blue;
unsigned int led_timer = 0;
int led_loading_current = 0;
bool led_disabled_even = true;
CRGB leds[NB_SENSORS];

#define DEBUG_SERIAL

bool enableErrorMode = false;

// In waiting mode by default. The RaspberryPI disables
// it with I2C on startup (if the I2C bus works)
bool enableWaitingMode = true;

// RaspberryPi communication
#define RASPI_I2C Wire1
#define SLAVE_ADDRESS 0x42
int data_type = 0;

bool init_sensor(int i) {
    
  #ifdef DEBUG_SERIAL
    Serial.print("Init sensor nb: ");
    Serial.println(i + 1);
  #endif

  delay(100);

  sensors[i].setTimeout(TIMEOUT_SENSOR);
  sensors[i].setAddress(SENSOR_ADDR + i);
  if( ! sensors[i].init()) return false;
  sensors[i].setTimeout(TIMEOUT_SENSOR);
  sensors[i].setAddress(SENSOR_ADDR + i);

  // Enable Long Range mode
  #ifdef LONG_RANGE
    if( ! sensors[i].setSignalRateLimit(0.1)) return false;
    if( ! sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18)) return false;
    if( ! sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14)) return false;
  #endif

  // Start continous mode
  sensors[i].startContinuous();

  return true;
}

void setup() {

  #ifdef DEBUG_SERIAL
    Serial.begin(9600);
    delay(1000);
    Serial.println("Started");
  #endif
  
  // Init communication
  SENSORS_I2C.setClock(I2C_SENSOR_SPEED);
  SENSORS_I2C.begin();

  // RaspberryPi communication
  RASPI_I2C.begin(SLAVE_ADDRESS);
  RASPI_I2C.onRequest(handleRequest);
  RASPI_I2C.onReceive(handleReceive); 

  // Init Led strip
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NB_SENSORS);
  FastLED.setBrightness(DEFAULT_BRIGHTNESS);

  pinMode(FRONT_PIN, OUTPUT);
  pinMode(BACK_PIN, OUTPUT);
  pinMode(IS_ROBOT_PIN, OUTPUT);
  digitalWrite(FRONT_PIN, LOW);
  digitalWrite(BACK_PIN, LOW);
  digitalWrite(IS_ROBOT_PIN, LOW);

  // Set every pin to output
  for (int i = NB_SENSORS -1; i >= 0; i--) {
    pinMode(XSHUT_START_GPIO + i, OUTPUT);
    digitalWrite(XSHUT_START_GPIO + i, LOW);
  }

  // Init sensors
  bool failed = false;
  for (int i = NB_SENSORS -1; i >= 0; i--) {
    digitalWrite(XSHUT_START_GPIO + i, HIGH);
    failedSensors[i] = !init_sensor(i);
    if(failedSensors[i]) {
      #ifdef DEBUG_SERIAL
        Serial.println("FAILED");
      #endif
      failed = true;
    }
  }
  if(failed) errorMode(true);

  for(int i = 0; i < NB_SAMPLES; i++) {
    samples[i] = (int*) malloc(sizeof(int)*NB_SENSORS); 
    doSampleScan();
  }
  
  if(leds_mode == LOADING)
    enableSensors = false;
}

void handleRequest() {

  #ifdef DEBUG_SERIAL
    Serial.println("Received data request from RaspberryPi");
    Serial.print("Current data_type: "); Serial.println(data_type);
  #endif

  // Scan request
  if(data_type == 0) {
    for(int i = 0; i < NB_SENSORS; i++) {
      int ret = scan[i];
      if(ret >= 0 && ret <= 8191) {
        RASPI_I2C.write(ret/256);
        RASPI_I2C.write(ret%256);
      }
      else {
        RASPI_I2C.write(255);
        RASPI_I2C.write(255);
      }
    }
  }

  // Zones request
  if(data_type == 1) {
    RASPI_I2C.write(is_front ? 1 : 0);
    RASPI_I2C.write(is_back  ? 1 : 0);
    RASPI_I2C.write(is_robot ? 1 : 0);
  }
}

void handleReceive(int nBytes) {

  #ifdef DEBUG_SERIAL
    Serial.println("Received data from Raspberrypi");
  #endif

  if(nBytes < 1) {
    #ifdef DEBUG_SERIAL
      Serial.println("ERROR Received receive event with no data");
    #endif
    return;
  }
  char type = RASPI_I2C.read();

  #ifdef DEBUG_SERIAL
    Serial.print("Type: ");
    Serial.println(type);
  #endif

  // Enable/Disable sensors
  if(type == 'e')
    enableSensors = RASPI_I2C.read() != 0;
  
  // LEDs mode
  if(type == 'l') {
    byte mode = RASPI_I2C.read();
    if(mode == 0) leds_mode = DIST;
    if(mode == 1) leds_mode = ZONES;
    if(mode == 2) leds_mode = LOADING;
    if(mode == 3) leds_mode = DISABLED;
  }

  // Data type (for the next request)
  if(type == 't')
    data_type = RASPI_I2C.read();

  // Loading leds color
  if(type == 'c')
    led_loading_color = RASPI_I2C.read() == 0 ? CRGB::Orange : CRGB::Blue;

  // Change brightness
  if(type == 'b')
    FastLED.setBrightness(RASPI_I2C.read());

  // Enable/Disable error mode (Also disables waiting mode)
  if(type == 'r') {
    int enable_ = RASPI_I2C.read() != 0;
    enableErrorMode = enable_;
    if(!enable_)
      enableWaitingMode = false;
  }
}

void manage_leds() {
  
  switch(leds_mode) {
    case DIST:
      for (int i = 0; i < NB_SENSORS; i++) {
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
      }
      break;

    case ZONES: 
      if(is_robot) {
        for(unsigned int i = 0; i < NB_SENSORS; i++)
          leds[i] = CRGB::Orange;
      } else {
        for(unsigned int i = 0; i < NB_SENSORS; i++)
          leds[i] = CRGB::Green;
      }   
      if(is_front) {
        for(unsigned int i = 0; i < sizeof(front_zone)/sizeof(int); i++)
          leds[front_zone[i]-1] = CRGB::Red;
      }
      if(is_back) {
        for(unsigned int i = 0; i < sizeof(back_zone)/sizeof(int); i++)
          leds[back_zone[i]-1] = CRGB::Red;
      }
      break;

    case LOADING:
      if(millis() > led_timer) {
        leds[led_loading_current] = CRGB::Black;
        leds[(led_loading_current + LED_LOADING_U) % NB_SENSORS] = led_loading_color;
        led_loading_current = (led_loading_current + 1) % NB_SENSORS;
        led_timer = millis() + LED_LOADING_DELAY;
      }
      break;

    case DISABLED:
      if(millis() > led_timer) {
        led_timer = millis() + LED_DISABLED_DELAY;
        led_disabled_even = !led_disabled_even;
        for(int i = 0; i < NB_SENSORS; i++)
          leds[i] = (i%2 == 0) == led_disabled_even ? CRGB::Black : LED_ORANGE;
      }

    default:
      break;
  }
   
  FastLED.show();
}

inline void updateFlags(int* _sensors, int nbSensors, bool* obstacle, bool* isRobot) {

  for (int index = 0; index < nbSensors; index++) {
    int i = _sensors[index] -1; // Index of the sensor in the sensors array
    if (obstacle && scan[i] <= AVOID_RANGE)
      *obstacle = true;
    if (scan[i] <= ROBOT_RANGE)
      *isRobot = true;
  }
}

void doSampleScan() {
  for (int sensorIndex = 0; sensorIndex < NB_SENSORS; sensorIndex++) {
      
    #ifdef DEBUG_SERIAL
      Serial.print("Sensor nb : ");
      Serial.print(sensorIndex + 1);
      Serial.print(" dist :");
    #endif

    samples[currentSample][sensorIndex] = 
      sensors[sensorIndex].readRangeContinuousMillimeters();

    #ifdef DEBUG_SERIAL
      Serial.print(samples[currentSample][sensorIndex]);
      if (sensors[sensorIndex].timeoutOccurred())
        Serial.print(" TIMEOUT");
      Serial.println();
    #endif
  }

  currentSample = (currentSample + 1)%NB_SAMPLES;
}

void loop() {

  if(enableErrorMode)
    errorMode(false);

  if(enableWaitingMode)
    waitingMode();

  if(!enableSensors) {
    manage_leds();
    return;
  }
  
  #ifdef DEBUG_SERIAL
    Serial.print("---- STARTING SCAN -----\n");
    int start = millis();
  #endif

  doSampleScan();

  #ifdef DEBUG_SERIAL
    Serial.print("---- END SCAN -----\n");
    Serial.print("Elapsed time: ");
    Serial.println(millis() - start);
  #endif

  for(int i = 0; i < NB_SENSORS; i++) {
    scan[i] = 0;
    #ifdef DEBUG_SERIAL
      Serial.print("Sensor id: ");
      Serial.print(i);
      Serial.print("; values:");
    #endif
    for(int j = 0; j < NB_SAMPLES; j++) {
      scan[i] += samples[j][i];
      #ifdef DEBUG_SERIAL
        Serial.print(" ");
        Serial.print(samples[j][i]);
      #endif
    }
    scan[i] /= NB_SAMPLES;
    #ifdef DEBUG_SERIAL
      Serial.print("; Average: ");
      Serial.print(scan[i]);
      Serial.println();
    #endif
  }

  bool _is_front = false;
  bool _is_back = false;
  bool _is_robot = false;

  updateFlags(front_zone, sizeof(front_zone)/sizeof(int), &_is_front, &_is_robot);
  updateFlags(back_zone, sizeof(back_zone)/sizeof(int), &_is_back, &_is_robot);
  updateFlags(no_zone, sizeof(no_zone)/sizeof(int), NULL, &_is_robot);

  is_front = _is_front;
  is_back  = _is_back;
  is_robot = _is_robot;

  digitalWrite(FRONT_PIN, is_front);
  digitalWrite(BACK_PIN, is_back);
  digitalWrite(IS_ROBOT_PIN, is_robot);

  manage_leds();
}

// Error mode
// When the MDB blinks in red it means an error occured
// The blinking will occur in this order:
// Full red to show it's an error (500ms)
// If known, the sensors that failed in yellow (500ms)

// This function stays in error mode forever if the specificSensors parameter is ON (one or more 
// sensors couldn't be initialised). Or until the raspberrypi disables it by I2C
void errorMode(bool specificSensors) {
  while(!specificSensors && enableErrorMode) {

    #ifdef DEBUG_SERIAL
      Serial.println("ERROR MODE #################");
    #endif

    // First blink
    for(unsigned int i = 0; i < sizeof(leds)/sizeof(CRGB); i++)
      leds[i] = CRGB::Yellow;
    FastLED.show();
    delay(500);
    for(unsigned int i = 0; i < sizeof(leds)/sizeof(CRGB); i++)
      leds[i] = CRGB::Black;
    FastLED.show();
    delay(500);

    // Sensor id
    if(specificSensors) {
      for(unsigned int i = 0; i < sizeof(leds)/sizeof(CRGB); i++)
        leds[i] = failedSensors[i] ? CRGB::Yellow : CRGB::Black;
      FastLED.show();
      delay(500);
      for(unsigned int i = 0; i < sizeof(leds)/sizeof(CRGB); i++)
        leds[i] = CRGB::Black;
      FastLED.show();
      delay(500);
    }
  }
}

// This function stays in waiting mode until the raspberrypi disables it by I2C
void waitingMode() {
  while(enableWaitingMode) {

    #ifdef DEBUG_SERIAL
      Serial.println("WAITING MODE #################");
    #endif

    // First blink
    for(unsigned int i = 0; i < sizeof(leds)/sizeof(CRGB); i++)
      leds[i] = CRGB::Green;
    FastLED.show();
    delay(500);
    for(unsigned int i = 0; i < sizeof(leds)/sizeof(CRGB); i++)
      leds[i] = CRGB::Black;
    FastLED.show();
    delay(500);
  }
}
