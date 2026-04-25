// ==================== MPU6050 YAW ====================
// Đọc góc Yaw từ gyro tích phân + calibrate offset
// Chân I2C: SDA = IMU_SDA (21), SCL = IMU_SCL (22)
// =====================================================

MPU6050 mpu;

float yaw = 0.0;
unsigned long imu_prevTime = 0;
float gyroZOffset = 0.0;
const float YAW_DEAD_ZONE = 0.4;

void IMU_init() 
{
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  mpu.initialize();

  if (mpu.testConnection()) 
  {
    // MPU6050 OK
  }
  else 
  {
    // MPU6050 FAIL
  }

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // Calibrate gyro Z offset
  // Calibrate gyro Z offset
  long sumGz = 0;
  int16_t ax0, ay0, az0, gx0, gy0, gz0;
  for (int i = 0; i < 100; i++) {
    mpu.getMotion6(&ax0, &ay0, &az0, &gx0, &gy0, &gz0);
    delay(2);
  }
  for (int i = 0; i < 1000; i++) {
    mpu.getMotion6(&ax0, &ay0, &az0, &gx0, &gy0, &gz0);
    sumGz += gz0;
    delay(2);
  }
  gyroZOffset = (float)sumGz / 1000.0;


  imu_prevTime = micros();
}

void IMU_read() 
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = micros();
  float dt = (now - imu_prevTime) / 1000000.0;
  imu_prevTime = now;
  if (dt <= 0 || dt > 0.5) dt = 0.02;

  float gyroZ = ((float)gz - gyroZOffset) / 131.0;
  if (fabs(gyroZ) < YAW_DEAD_ZONE) gyroZ = 0.0;
  yaw += gyroZ * dt;

  if (yaw > 180.0)  yaw -= 360.0;
  if (yaw < -180.0) yaw += 360.0;
}

void IMU_print() 
{
  Serial.print("yaw: ");
  Serial.println(yaw);
}

void IMU_reset() { yaw = 0.0; }
float getYaw() { return yaw; }
