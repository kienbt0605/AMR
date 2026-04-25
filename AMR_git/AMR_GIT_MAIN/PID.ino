// ==================== PID CONTROLLER ====================
// 1. PID Xoay góc: setpoint = odom_theta, actual = yaw (IMU)
// 2. PID Giữ hướng 0° khi đi thẳng: setpoint = 0, actual = yaw (IMU)
// 3. PID Đi thẳng (encoder balance): setpoint = 0, actual = enc1 - enc2
// ========================================================

// ---- PID Xoay góc (theta) ----
// Dùng khi xoay tại chỗ để hướng về mục tiêu (Bước 1)
float pidTheta_Kp = 10.0;
float pidTheta_Ki = 0.0;
float pidTheta_Kd = 0.0;

float pidTheta_prevError = 0.0;
float pidTheta_integral  = 0.0;

// ---- PID Giữ hướng 0° khi đi thẳng (heading) ----
// Dùng khi đi thẳng để giữ yaw = 0 (Bước 2)
// Error tính bằng rad (nhỏ, ~0.01-0.1), Kp phải lớn để output ra PWM correction
// Ví dụ: lệch 3° ≈ 0.05 rad → correction = 500 * 0.05 = 25 PWM
float pidHeading_Kp = 5.0;
float pidHeading_Ki = 0.0;
float pidHeading_Kd = 0.0;

float pidHeading_prevError = 0.0;
float pidHeading_integral  = 0.0;

// ---- PID Đi thẳng (encoder balance) ----
float pidStraight_Kp = 5.0;
float pidStraight_Ki = 0.0;
float pidStraight_Kd = 0.0;

float pidStraight_prevError = 0.0;
float pidStraight_integral  = 0.0;

// Giới hạn tích phân (chống windup)
const float INTEGRAL_LIMIT = 100.0;

// Encoder tích lũy khi bắt đầu đi thẳng
int straightStartEnc1 = 0;
int straightStartEnc2 = 0;

// =====================================================
// Reset PID - gọi khi chuyển trạng thái
// =====================================================
void PID_resetTheta()
{
  pidTheta_prevError = 0.0;
  pidTheta_integral  = 0.0;
}

void PID_resetHeading()
{
  pidHeading_prevError = 0.0;
  pidHeading_integral  = 0.0;
}

void PID_resetStraight()
{
  pidStraight_prevError = 0.0;
  pidStraight_integral  = 0.0;
  straightStartEnc1 = encoder1_value;
  straightStartEnc2 = encoder2_value;
}

// =====================================================
// PID Xoay góc
//
// setpoint: targetAngle (góc mục tiêu, rad)
// actual:   yaw từ IMU (chuyển sang rad)
//
// Trả về: w (rad/s) để truyền vào driveVW()
// Dùng cho: Bước 1 - xoay tại chỗ
// =====================================================
float PID_theta(float setpoint_rad, float dt)
{
  // Chuyển yaw IMU từ độ sang rad
  float actual_rad = (float)yaw * PI / 180.0;

  // Tính sai số góc (chuẩn hoá [-PI, PI])
  float error = setpoint_rad - actual_rad;
  error = atan2(sin(error), cos(error));

  // Tích phân
  pidTheta_integral += error * dt;
  pidTheta_integral = constrain(pidTheta_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Đạo hàm
  float derivative = 0.0;
  if (dt > 0) derivative = (error - pidTheta_prevError) / dt;
  pidTheta_prevError = error;

  // Output PID
  float output = pidTheta_Kp * error 
               + pidTheta_Ki * pidTheta_integral 
               + pidTheta_Kd * derivative;

  // Giới hạn output (rad/s)
  output = constrain(output, -3.0, 3.0);

  return output;
}

// =====================================================
// PID Giữ hướng 0° khi đi thẳng
//
// setpoint: 0 (giữ yaw = 0 sau khi reset IMU)
// actual:   yaw từ IMU (chuyển sang rad)
//
// Trả về: correction PWM để kết hợp với PID encoder
// Dùng cho: Bước 2 - đi thẳng, giữ hướng không lệch
// =====================================================
float PID_heading(float dt)
{
  // Setpoint = 0 (giữ hướng thẳng)
  float error = 0.0 - yaw;
  //error = atan2(sin(error), cos(error));

  // Tích phân
  pidHeading_integral += error * dt;
  pidHeading_integral = constrain(pidHeading_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Đạo hàm
  float derivative = 0.0;
  if (dt > 0) derivative = (error - pidHeading_prevError) / dt;
  pidHeading_prevError = error;

  // Output PID
  float output = pidHeading_Kp * error 
               + pidHeading_Ki * pidHeading_integral 
               + pidHeading_Kd * derivative;

  // Giới hạn output (PWM correction)
  output = constrain(output, -100.0, 100.0);

  return output;
}

// =====================================================
// PID Đi thẳng
//
// setpoint: 0 (2 bánh phải đi bằng nhau)
// actual:   (encoder1 - encoder2) tích lũy từ lúc bắt đầu đi thẳng
//           Nếu > 0: bánh trái đi nhiều hơn → lệch phải
//           Nếu < 0: bánh phải đi nhiều hơn → lệch trái
//
// Trả về: correction để bù vào PWM
//         Motor(basePWM + correction, basePWM - correction)
// =====================================================
float PID_straight(float dt)
{
  // Tính delta encoder từ lúc bắt đầu đi thẳng
  int encL = encoder1_value - straightStartEnc1;
  int encR = encoder2_value - straightStartEnc2;

  // Sai số = hiệu 2 encoder (setpoint = 0)
  float error = (float)(encL - encR);

  // Tích phân
  pidStraight_integral += error * dt;
  pidStraight_integral = constrain(pidStraight_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Đạo hàm
  float derivative = 0.0;
  if (dt > 0) derivative = (error - pidStraight_prevError) / dt;
  pidStraight_prevError = error;

  // Output PID
  float output = pidStraight_Kp * error 
               + pidStraight_Ki * pidStraight_integral 
               + pidStraight_Kd * derivative;

  // Giới hạn correction
  output = constrain(output, -100.0, 100.0);

  return output;
}