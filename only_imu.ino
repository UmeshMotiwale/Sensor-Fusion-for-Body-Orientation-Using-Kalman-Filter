#include <Wire.h> //for the I2C commumination
#include <Adafruit_LSM9DS1.h> // to understand the lsm9ds1
#include <Adafruit_Sensor.h> 

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); // creating an object so that we can take information from this
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //to start the conversation betweent the esp and the computer
  Wire.begin(21, 22);  // SDA=21, SCL=22 for ESP32 
  if (!lsm.begin()) {
    Serial.println("Failed to find LSM9DS1 chip");
    while (1);
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  Serial.println("LSM9DS1 ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
  lsm.read(); // update all sensor data (accel, gyro, mag)
  const float SENSITIVITY_MG_PER_LSB = 0.061;
  float ax= lsm.accelData.x* SENSITIVITY_MG_PER_LSB / 100.0;
  float ay= lsm.accelData.y* SENSITIVITY_MG_PER_LSB / 100.0;
  float az= lsm.accelData.z* SENSITIVITY_MG_PER_LSB / 100.0;
  float theta = atan2(ay, sqrt(ax*ax+az*az))*180/PI;
  Serial.println(theta);
  


}
