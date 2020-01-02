#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>
#include <SPI.h>

static NMEAGPS gps;
static gps_fix  fix;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

imu::Vector<3> euler;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> linacc;
imu::Vector<3> grav;
imu::Vector<3> mag;
int8_t temp;

// SD card vars
File myFile;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// Teensy audio board: pin 10
// Teensy 3.5 & 3.6 on-board: BUILTIN_SDCARD
// Wiz820+SD board: pin 4
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
const int chipSelect = 4;

void serialprint() {
  // euler
  Serial.print("Euler: ");
  Serial.print(euler.x());
  Serial.print(" ");
  Serial.print(euler.y());
  Serial.print(" ");
  Serial.println(euler.z());
  // gyro
  Serial.print("Gyroscope: ");
  Serial.print(gyro.x());
  Serial.print(" ");
  Serial.print(gyro.y());
  Serial.print(" ");
  Serial.println(gyro.z());
  // accel
  Serial.print("Accelerometer: ");
  Serial.print(accel.x());
  Serial.print(" ");
  Serial.print(accel.y());
  Serial.print(" ");
  Serial.println(accel.z());
  // linacc
  Serial.print("Linear Accel: ");
  Serial.print(linacc.x());
  Serial.print(" ");
  Serial.print(linacc.y());
  Serial.print(" ");
  Serial.println(linacc.z());
  // grav
  Serial.print("Gravity: ");
  Serial.print(grav.x());
  Serial.print(" ");
  Serial.print(grav.y());
  Serial.print(" ");
  Serial.println(grav.z());
  // mag
  Serial.print("Magnetometer: ");
  Serial.print(mag.x());
  Serial.print(" ");
  Serial.print(mag.y());
  Serial.print(" ");
  Serial.println(mag.z());
  // temp
  Serial.println("Temperature: " + temp);
  // GPS
  Serial.print("Longitude: ");
  Serial.println(fix.longitude());
  Serial.print("Latitude: ");
  Serial.println(fix.latitude());
  Serial.print("Altitude: ");
  Serial.println(fix.altitude_cm());
}

void sdprint() {
  // euler
  myFile.print("Euler: ");
  myFile.print(euler.x());
  myFile.print(" ");
  myFile.print(euler.y());
  myFile.print(" ");
  myFile.println(euler.z());
  // gyro
  myFile.print("Gyroscope: ");
  myFile.print(gyro.x());
  myFile.print(" ");
  myFile.print(gyro.y());
  myFile.print(" ");
  myFile.println(gyro.z());
  // accel
  myFile.print("Accelerometer: ");
  myFile.print(accel.x());
  myFile.print(" ");
  myFile.print(accel.y());
  myFile.print(" ");
  myFile.println(accel.z());
  // linacc
  myFile.print("Linear Accel: ");
  myFile.print(linacc.x());
  myFile.print(" ");
  myFile.print(linacc.y());
  myFile.print(" ");
  myFile.println(linacc.z());
  // grav
  myFile.print("Gravity: ");
  myFile.print(grav.x());
  myFile.print(" ");
  myFile.print(grav.y());
  myFile.print(" ");
  myFile.println(grav.z());
  // mag
  myFile.print("Magnetometer: ");
  myFile.print(mag.x());
  myFile.print(" ");
  myFile.print(mag.y());
  myFile.print(" ");
  myFile.println(mag.z());
  // temp
  myFile.println("Temperature: " + temp);
  // GPS
  myFile.print("Longitude: ");
  myFile.println(fix.longitude());
  myFile.print("Latitude: ");
  myFile.println(fix.latitude());
  myFile.print("Altitude: ");
  myFile.println(fix.altitude_cm());
}

void setup() {
  // set data rate for ports
  Serial.begin(9600);
  gpsPort.begin(9600);  // Serial1

  // IMU
  if (!bno.begin()) {
    Serial.print("No BNO055 detected.");
    while(1);
  }

  delay(1000);
    
  bno.setExtCrystalUse(true);

  if (!SD.begin(chipSelect)) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD initialization done.");

  myFile = SD.open("data.txt", FILE_WRITE);
  
}

void loop() {
  // IMU
  sensors_event_t event; 
  bno.getEvent(&event);

  euler   = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro    = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  linacc  = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  grav    = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  mag     = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  temp = bno.getTemp();

  if (gps.available( gpsPort )) {
    fix = gps.read();
  }

  serialprint();
  sdprint();
  myFile.close();
  
}
