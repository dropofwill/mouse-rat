#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections (For default I2C)
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GROUND to common ground

   History
   =======
   2014/JULY/25  - First version (KTOWN)
*/

/* Assign a unique base ID for this sensor */
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

float xNormal = -3.0;
float yNormal = 1;

float xRange[2] = [0,0]
float yRange[2] = [0,0]

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

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void) {
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) {
  Serial.begin(9600);

  /* Initialise the sensor */
  if(!lsm.begin()) { while(1); }

  /* Setup the sensor gain and integration time */
  configureSensor();
}

void loop(void) {
  /* Get a new sensor event */
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp);

  float mx, my = 0.0;

  if (accel.acceleration.x > xNormal || accel.acceleration.x > -xNormal) {
    my = map(accel.acceleration.x - xNormal, -20, 20, 200, -200);
    mx = map(accel.acceleration.y - yNormal, -20, 20, 200, -200);
  }
  else if (accel.acceleration.y > yNormal || accel.acceleration.y > -yNormal) {
    my = map(accel.acceleration.x - xNormal, -20, 20, 200, -200);
    mx = map(accel.acceleration.y - yNormal, -20, 20, 200, -200);
  }

  calibrate(accel.acceleration.x, accel.acceleration.y);

  Mouse.move(mx, my, 0);

  // print out accelleration data
//  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
//  Serial.print("  \tY: "); Serial.print(accel.acceleration.y);   Serial.print(" ");
//  Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);   Serial.println("  \tm/s^2");
//  Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
//  Serial.print("  \tY: "); Serial.print(mag.magnetic.y);       Serial.print(" ");
//  Serial.print("  \tZ: "); Serial.print(mag.magnetic.z);     Serial.println("  \tgauss");
//  Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
//  Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
//  Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");
//  Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");

  delay(16);
}

void calibrate(float x, float y) {
   xRange[0] = min(xRange[0], x);
   xRange[1] = max(xRange[1], x);
   
   yRange[0] = min(yRange[0], y);
   yRange[1] = max(yRange[1], y);
   
   Serial.print(xRange[0]); Serial.print(", "); Serial.println(xRange[1]);
   Serial.print(yRange[0]); Serial.print(", "); Serial.println(yRange[1]);
 }
