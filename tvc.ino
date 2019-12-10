#include <NMEAGPS.h>
#include <GPSport.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

static NMEAGPS gps;
static gps_fix  fix;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

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
  
}

void loop() {
  // IMU
  sensors_event_t event; 
  bno.getEvent(&event);

  imu::Vector<3> euler   = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gryo    = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> linacc  = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> grav    = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> mag     = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  // imu::Quaternion quat = bno.getQuat();
  int8_t temp = bno.getTemp();

  // GPS
  if (gps.available( gpsPort )) {
    fix = gps.read();
  }
}
