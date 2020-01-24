#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>
#include <SPI.h>


// -- GPS --
static NMEAGPS gps;
static gps_fix fix;

// -- BNO055 --
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
imu::Vector<3> euler;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> linacc;
imu::Vector<3> grav;
imu::Vector<3> mag;
int8_t temp;

// -- SD --
#define LOGNAME "log.txt"
File myFile;
const int chipSelect = BUILTIN_SDCARD;
// text buffers
char eulerbuf[64];
char gyrobuf[64];
char accelbuf[64];
char linaccbuf[64];
char gravbuf[64];
char magbuf[64];
char tempbuf[64];
char gpsbuf[64];

// Writes all variables to buffers
void writebuf(){
  sprintf(eulerbuf, "Euler: %e %e %e", euler.x(), euler.y(), euler.z());
  sprintf(gyrobuf, "Gyroscope: %e %e %e", gyro.x(), gyro.y(), gyro.z());
  sprintf(accelbuf, "Accelerometer: %e %e %e", accel.x(), accel.y(), accel.z());
  sprintf(linaccbuf, "Linear Accel: %e %e %e", linacc.x(), linacc.y(), linacc.z());
  sprintf(gravbuf, "Gravity: %e %e %e", grav.x(), grav.y(), grav.z());
  sprintf(magbuf, "Magnetometer: %e %e %e", mag.x(), mag.y(), mag.z());
  sprintf(tempbuf, "Temperature: %d", temp);
  sprintf(gpsbuf, "Longitude: %e, Latitude: %e, Altitude: %ld",fix.longitude(), fix.latitude(), fix.altitude_cm());
}

// Prints all buffers to Serial 
void serialprint() {
  Serial.println(eulerbuf);
  Serial.println(gyrobuf);
  Serial.println(accelbuf);
  Serial.println(linaccbuf);
  Serial.println(gravbuf);
  Serial.println(magbuf);
  Serial.println(tempbuf);
  Serial.println(gpsbuf);
}

// Prints all buffers to SD
void sdprint() {
  // print data
  myFile.println(eulerbuf);
  myFile.println(gyrobuf);
  myFile.println(accelbuf);
  myFile.println(linaccbuf);
  myFile.println(gravbuf);
  myFile.println(magbuf);
  myFile.println(tempbuf);
  myFile.println(gpsbuf);
}


void setup() {
  // Serial
  Serial.begin(9600);

  // GPS - Serial1
  gpsPort.begin(9600);



  // IMU
  if (!bno.begin()) {
    Serial.print("No BNO055 detected.");
    while(1);
  }
  delay(1000); 
  bno.setExtCrystalUse(true);

  // SD
  if (!SD.begin(chipSelect)) {
    Serial.println("SD initialization failed");
    return;
  }
  File myFile = SD.open(LOGNAME, FILE_WRITE);
  Serial.println("SD initialization done.");
  delay(1000);
  Serial.println("Setup done.");
}

void loop() {
  // BNO055 - IMU and temperature
  sensors_event_t event; 
  bno.getEvent(&event);
  euler   = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro    = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel   = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  linacc  = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  grav    = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  mag     = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  temp    = bno.getTemp();

  // GPS
  long currentime = millis();
  while(!gps.available( gpsPort )){}
  if (gps.available( gpsPort )) {
    fix = gps.read();
    trace_all( Serial, gps, fix );
    Serial.println(millis()-currentime);
  }

  writebuf();
  serialprint();
  sdprint(); 
}
