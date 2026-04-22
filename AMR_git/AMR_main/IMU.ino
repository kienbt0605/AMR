MPU6050 mpu;

extern float roll, pitch, yaw;
float gyroX_offset = 0.0, gyroY_offset = 0.0, gyroZ_offset = 0.0;
float gyroZ_rate = 0.0;  // Tốc độ góc quay quanh trục Z (°/s) - dùng cho PID
unsigned long imu_prev_time = 0;

void IMU_Init() {
  Wire.begin(IMU_SDA, IMU_SCL);
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    return;
  }
  Serial.println("MPU6050 connected!");

  // Calibrate gyro X, Y, Z - lấy offset khi đứng yên
  Serial.println("Calibrating gyro...");
  long sumX = 0, sumY = 0, sumZ = 0;
  int samples = 500;
  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(3);
  }
  gyroX_offset = (float)sumX / samples;
  gyroY_offset = (float)sumY / samples;
  gyroZ_offset = (float)sumZ / samples;
  Serial.println("Gyro calibration done!");

  imu_prev_time = millis();
}

void IMU_Update() {
  unsigned long now = millis();
  float dt = (now - imu_prev_time) / 1000.0;
  imu_prev_time = now;

  int16_t gx_raw, gy_raw, gz_raw;
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

  float gx = (gx_raw - gyroX_offset) / 131.0;  // °/s
  float gy = (gy_raw - gyroY_offset) / 131.0;
  float gz = (gz_raw - gyroZ_offset) / 131.0;

  gyroZ_rate = gz;  // Lưu tốc độ góc Z hiện tại cho PID

  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;
}
