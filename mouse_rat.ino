#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include "Adafruit_MPR121.h"

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

// LSM9DS0, use I2C, ID #1005
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1005);

// MPR121, use I2C, ID #1000 
Adafruit_MPR121 cap = Adafruit_MPR121();

float xNormal = -3.0,
      yNormal = 1,
      // Clamp inputs to this range (m/s^2)
      accelMax =  20,
      accelMin = -20,
      // Map inputs to this range (pixels)
      outMin   =  200,
      outMax   = -200;

int calibrationCounter = 0;

float xRange[] = {0,0};
float yRange[] = {0,0};

const int MOUSE_LEFT_PIN = 10,
          MOUSE_MID_PIN   = 8,
          MOUSE_RIGHT_PIN = 6;
    
uint16_t lasttouched = 0,
         currtouched = 0;


// Configures the gain and integration time for the TSL2561
void configureSensor(void) {
  // Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}


void setup(void) {
  Serial.begin(9600);
  
  dof_setup();
  mpr_setup();
}


void loop(void) {
  dof_loop();
  mpr_loop();
}


void mpr_setup() {
  // needed to keep leonardo/micro from starting too fast!
  while (!Serial);
  Serial.begin(9600);    Serial.println("Adafruit MPR121 Capacitive Touch sensor test"); 
  
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A))  Serial.println("MPR121 not found");
  else                   Serial.println("MPR121 found!");
}


void dof_setup() {
  if(!lsm.begin()) { while(1); }

  // Setup the sensor gain and integration time
  configureSensor();
  
  // Set a max and min value to not do anything
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);  
}


void dof_loop() {
  /* Get a new sensor event */
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);

  float mx, my = 0.0;
  float accelX = clamp(accel.acceleration.x, accelMin, accelMax);
  float accelY = clamp(accel.acceleration.y, accelMin, accelMax);

  if (calibrationCounter > 5) {
  
    if (accel.acceleration.x < xRange[0] || accel.acceleration.x > xRange[1]) {
      mx = map(accelY - yNormal, accelMin, accelMax, outMax, outMin);
    }
    
    if (accel.acceleration.y < yRange[0] || accel.acceleration.y > yRange[1]) {
      my = map(accelX - xNormal, accelMin, accelMax, outMax, outMin);
    }
  
    Mouse.move(mx, my, 0);
  }
  else {
    calibrationCounter++;
    calibrate(accel.acceleration.x, accel.acceleration.y);
  }
  delay(7);
}


void mpr_loop() {
  // Get the currently touched pads
  currtouched = cap.touched();
  checkTouch();
  // reset our state
  lasttouched = currtouched;
}


void checkTouch() {
  for (uint8_t i = 0; i < 12; i++) {
    // it if /is/ touched and /wasn't/ touched before, alert!
    if ( (currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) touchHandler(i);

    // if it /was/ touched and now /isn't/, alert!
    if ( !(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) releaseHandler(i);
  } 
}


void calibrate(float x, float y) {
  xRange[0] = min(xRange[0], x);
  xRange[1] = max(xRange[1], x);
   
  yRange[0] = min(yRange[0], y);
  yRange[1] = max(yRange[1], y);
   
  xNormal = (xRange[0] + xRange[1])/2;
  yNormal = (yRange[0] + yRange[1])/2;
   
  Serial.print(xRange[0]); Serial.print(", "); Serial.println(xRange[1]);
  Serial.print(yRange[0]); Serial.print(", "); Serial.println(yRange[1]);
}


void touchHandler(int id) {
  Serial.print(id); Serial.println(" touched");
  
  switch(id) {
    case MOUSE_LEFT_PIN:
      Mouse.press(MOUSE_LEFT);
      break;
    case MOUSE_MID_PIN:
      Mouse.press(MOUSE_MIDDLE);
      break;
    case MOUSE_RIGHT_PIN:
      Mouse.press(MOUSE_RIGHT);
      break;
  }
}


void releaseHandler(int id) {
  Serial.print(id); Serial.println(" released");
    
  switch(id) {
    case MOUSE_LEFT_PIN:
      Mouse.release(MOUSE_LEFT);
      break;
    case MOUSE_MID_PIN:
      Mouse.release(MOUSE_MIDDLE);
      break;
    case MOUSE_RIGHT_PIN:
      Mouse.release(MOUSE_RIGHT);
      break;
  }
}


float clamp(float value, float min_, float max_) {
  return (value < min_) ? min_ : (value > max_) ? max_ : value;
}


void displaySensorDetails(void) {
  sensor_t accel, mag, gyro, temp;
  lsm.getSensor(&accel, &mag, &gyro, &temp);

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  delay(500);
}
