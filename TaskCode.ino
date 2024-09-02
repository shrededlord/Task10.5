#include <Wire.h>

// MPU6050 I2C address and register addresses
#define MPU_ADDR 0x68
#define PWR_MGMT 0x6B
#define GYRO_Z_H 0x47

// Gyroscope data and yaw angle
int16_t gx, gy, gz;
float gz_offset, yaw = 0.0;
unsigned long lastTime;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT);
  Wire.write(0);
  Wire.endTransmission();

  // Gyro calibration
  calibrateGyro();
  lastTime = millis();
}

void loop() {
  getGyroData();

  updateYaw();

  // Display yaw angle
  Serial.print("Yaw: ");
  Serial.println(yaw);

  delay(100);
}

void getGyroData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_Z_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

void updateYaw() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Convert gyro data to degrees per second
  float gz_rate = (gz - gz_offset) / 131.0;

  // Integrate to get yaw angle
  yaw += gz_rate * dt;

  // Keep yaw within 0-360 degrees
  if (yaw >= 360) yaw -= 360;
  if (yaw < 0) yaw += 360;
}

void calibrateGyro() {
  int samples = 1000;

  for (int i = 0; i < samples; i++) {
    getGyroData();
    gz_offset += gz;
    delay(3);
  }

  gz_offset /= samples;
}
