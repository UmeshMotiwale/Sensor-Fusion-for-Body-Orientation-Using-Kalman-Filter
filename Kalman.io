#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
float Rate_Roll;
float Rate_callibration_roll;
float angle_x =0;
int callibration_number;
float AccX,AccY,AccZ;
float AngleRoll;
float kalman_roll = 0, kalman_uncertainity_roll = 2*2;
float kalman_output[] = {0,0};
unsigned long lastTime =0;
float dt = 0.004;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); // IMU object

void kalman1d(float kalman_state, float Kalman_uncertainity, float kalman_input, float kalman_measurement){
  kalman_state = kalman_state + dt*kalman_input;
  Kalman_uncertainity = Kalman_uncertainity + dt*dt*4*4;
  float kalman_gain = Kalman_uncertainity/(Kalman_uncertainity+ dt*dt*20*20);
  kalman_state = kalman_state + kalman_gain*(kalman_measurement-kalman_state);
  Kalman_uncertainity = (1-kalman_gain)*Kalman_uncertainity;
  kalman_output[0] = kalman_state;
  kalman_output[1] = Kalman_uncertainity;
}

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
  lsm.read();
  float gx_dps = lsm.gyroData.x * 0.00875;
  const float SENSITIVITY_MG_PER_LSB = 0.061;
  float ax= lsm.accelData.x* SENSITIVITY_MG_PER_LSB / 100.0;
  float ay= lsm.accelData.y* SENSITIVITY_MG_PER_LSB / 100.0;
  float az= lsm.accelData.z* SENSITIVITY_MG_PER_LSB / 100.0;
  float theta = atan2(ay, sqrt(ax*ax+az*az))*180/PI;
  angle_x += gx_dps * dt;  
  kalman1d(kalman_roll,kalman_uncertainity_roll,gx_dps,theta);
  kalman_roll = kalman_output[0];
  kalman_uncertainity_roll = kalman_output[1];
  Serial.print(kalman_roll);
  Serial.print(" ");
  Serial.println(angle_x);
}
