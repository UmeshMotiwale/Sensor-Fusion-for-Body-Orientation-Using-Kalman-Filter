#include <Arduino.h>
#include <Wire.h>

#include <Arduino_LSM9DS1.h>   // Use Arduino's built-in library

float Rate_Roll;
float Rate_callibration_roll;
float angle_x = 0;
int callibration_number;
float AccX, AccY, AccZ;
float AngleRoll;
float kalman_roll = 0, kalman_uncertainity_roll = 2*2;
float kalman_output[] = {0, 0};
unsigned long lastTime = 0;
float dt = 0.004;

void kalman1d(float kalman_state, float Kalman_uncertainity, float kalman_input, float kalman_measurement) {
  kalman_state = kalman_state + kalman_input * dt;
  Kalman_uncertainity = Kalman_uncertainity + dt * dt * 0.037;
  float kalman_gain = Kalman_uncertainity / (Kalman_uncertainity + dt * dt * 53);
  kalman_state = kalman_state + kalman_gain * (kalman_measurement - kalman_state);
  Kalman_uncertainity = (1 - kalman_gain) * Kalman_uncertainity;
  kalman_output[0] = kalman_state;
  kalman_output[1] = Kalman_uncertainity;
}

void setup() {
  Serial.begin(115200);

  if (!IMU.begin()) {
    while (1);
  }
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }

  // Convert accel to roll angle
  float theta = atan2(-ay, sqrt(ax * ax + az * az)) * 180 / PI;

  // Gyro X in degrees/sec
  float gx_dps = gx;   // Already in dps with Arduino_LSM9DS1
  angle_x += gx_dps * dt;

  // Kalman filter
  kalman1d(kalman_roll, kalman_uncertainity_roll, gx_dps, theta);
  kalman_roll = kalman_output[0];
  kalman_uncertainity_roll = kalman_output[1];

  Serial.println(kalman_roll);
  
}
