#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>
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
  imu::Vector<3> gyro    = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> linacc  = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> grav    = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> mag     = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  // imu::Quaternion quat = bno.getQuat();
  int8_t temp = bno.getTemp();

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
  if (gps.available( gpsPort )) {
    fix = gps.read();
    Serial.print("Longitude: ");
    Serial.println(fix.longitude());
    Serial.print("Latitude: ");
    Serial.println(fix.latitude());
    Serial.print("Altitude: ");
    Serial.println(fix.altitude_cm());
  }
}
