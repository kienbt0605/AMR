// ==================== ODOMETRY ====================
// Tính toán odometry từ encoder cho differential drive robot
//
// Công thức Velocity:
//   v_left  = (delta_tickL * PI * D) / (N * dt)     [mm/s]
//   v_right = (delta_tickR * PI * D) / (N * dt)     [mm/s]
//   v       = (v_left + v_right) / 2                 [mm/s]
//   w       = (v_left - v_right) / L                 [rad/s]
//
// Công thức Odometry:
//   SL          = delta_tickL * d_tick               [mm]
//   SR          = delta_tickR * d_tick               [mm]
//   S           = (SL + SR) / 2                      [mm]
//   delta_theta = (SL - SR) / L                      [rad]
//   x          += S * cos(theta + delta_theta / 2)
//   y          += S * sin(theta + delta_theta / 2)
//   theta      += delta_theta
// ===================================================

// Chu vi bánh xe (mm)
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Quãng đường mỗi tick encoder (mm)
const float DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_PPR;

// Vị trí & hướng robot
float odom_x     = 0.0;  // mm
float odom_y     = 0.0;  // mm
float odom_theta = 0.0;  // rad
float x_cood;
float y_cood;
// Lưu encoder cũ để tính delta
int prev_encoder1 = 0;
int prev_encoder2 = 0;

// Thời gian trước để tính dt
unsigned long odom_prevTime = 0;

// =====================================================
// Khởi tạo Odometry - gọi trong setup()
// =====================================================
void Odom_init()
{
  odom_x     = 0.0;
  odom_y     = 0.0;
  odom_theta = 0.0;
  prev_encoder1 = encoder1_value;
  prev_encoder2 = encoder2_value;
  odom_prevTime = micros();
}

// =====================================================
// Cập nhật Odometry - gọi khi odomFlag == true
// Tính v, w, x, y, theta từ encoder ticks
// =====================================================
void Odom_update()
{
  // Tính dt (giây)
  unsigned long now = micros();
  float dt = (now - odom_prevTime) / 1000000.0;
  odom_prevTime = now;

  if (dt <= 0 || dt > 1.0) dt = 0.02;  // Bảo vệ

  // Tính delta ticks từ lần đọc trước
  int delta_tickL = (encoder1_value - prev_encoder1);
  int delta_tickR = (encoder2_value - prev_encoder2);
  prev_encoder1 = encoder1_value;
  prev_encoder2 = encoder2_value;

  // ---- Velocity Formulas ----
  // v_left  = (delta_tickL * PI * D) / (N * dt)
  // v_right = (delta_tickR * PI * D) / (N * dt)
  vLeft  = (delta_tickL * PI * WHEEL_DIAMETER) / (ENCODER_PPR * dt);
  vRight = (delta_tickR * PI * WHEEL_DIAMETER) / (ENCODER_PPR * dt);

  // v = (v_left + v_right) / 2
  v = (vLeft + vRight) / 2.0;

  // w = (v_left - v_right) / L
  w = (vLeft - vRight) / WHEEL_BASE;

  // ---- Odometry Formulas ----
  // Quãng đường từng bánh
  float SL = delta_tickL * DISTANCE_PER_TICK;
  float SR = delta_tickR * DISTANCE_PER_TICK;

  // Quãng đường trung bình
  float S = (SL + SR) / 2.0;

  // Thay đổi góc
  float delta_theta = (SL - SR) / WHEEL_BASE;

  // Cập nhật toạ độ (mid-point approximation)
  odom_x += S * cos(odom_theta + delta_theta / 2.0);
  odom_y += S * sin(odom_theta + delta_theta / 2.0);
  odom_theta += delta_theta;
  x_cood = -odom_x;
  y_cood = -odom_y;

  // Chuẩn hoá theta về [-PI, PI]
  odom_theta = atan2(sin(odom_theta), cos(odom_theta));
}

// =====================================================
// In v, w, x, y, theta ra Serial
// =====================================================
void Odom_print()
{
  Serial.print("X: ");
  Serial.print(odom_x);
  Serial.print(" Y: ");
  Serial.print(odom_y);
  Serial.print("  ");
}

// =====================================================
// Reset odometry về gốc
// =====================================================
void Odom_reset()
{
  odom_x     = 0.0;
  odom_y     = 0.0;
  odom_theta = 0.0;
  prev_encoder1 = encoder1_value;
  prev_encoder2 = encoder2_value;
  v = 0.0;
  w = 0.0;
  vLeft  = 0.0;
  vRight = 0.0;
}

// =====================================================
// Chuyển đổi vận tốc (mm/s) → PWM (0-255)
// =====================================================
// Vận tốc tối đa đo được: 594 mm/s ở PWM 255
const float MAX_VELOCITY = 594.0;  // mm/s tại PWM 255

// Chuyển vận tốc 1 bánh (mm/s) → PWM (-255 đến 255)
// Có PWM tối thiểu để motor không bị kẹt (dead zone)
const int MIN_PWM = 50;  // PWM tối thiểu để motor quay được

int velocityToPWM(float velocity_mms)
{
  // Nếu vận tốc gần 0 → không chạy
  if (fabs(velocity_mms) < 5.0) return 0;

  int pwm = (int)(velocity_mms * 255.0 / MAX_VELOCITY);

  // Đảm bảo PWM >= MIN_PWM khi có lệnh chạy
  if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
  if (pwm < 0 && pwm > -MIN_PWM) pwm = -MIN_PWM;

  return constrain(pwm, -255, 255);
}

// =====================================================
// Điều khiển robot bằng v (mm/s) và w (rad/s)
//
// Công thức differential drive (ngược):
//   v_left  = v - (w * L) / 2
//   v_right = v + (w * L) / 2
//
// Sau đó chuyển v_left, v_right → PWM → Motor()
// =====================================================
void driveVW(float v_cmd, float w_cmd)
{
  // Tính vận tốc từng bánh từ v, w
  float vL = v_cmd - (w_cmd * WHEEL_BASE) / 2.0;
  float vR = v_cmd + (w_cmd * WHEEL_BASE) / 2.0;

  // Chuyển sang PWM
  int pwmL = velocityToPWM(vL);
  int pwmR = velocityToPWM(vR);

  // Gọi hàm Motor
  Motor(pwmL, pwmR);
}

// =====================================================
// Di chuyển robot theo quãng đường từ (0,0) đến (x_target, y_target)
//
// Bước 1: Xoay tại chỗ hướng về góc atan2(y, x)
// Bước 2: Đi thẳng quãng đường = sqrt(x² + y²)
//         Đếm quãng đường bằng encoder ticks
// Gọi liên tục trong loop(), trả về true khi đi đủ quãng đường
// =====================================================
const float DIST_THRESHOLD  = 2.0;    // mm - chừa 2mm cho quán tính (trước là 15mm nên bị thiếu)
const float DRIVE_SPEED     = 200.0;  // mm/s
const float TURN_KP         = 3.0;    // Hệ số P xoay
const float ANGLE_THRESHOLD = 0.02;   // rad (~1°) - ngưỡng xoay xong

// Biến theo dõi quãng đường
float totalDistTarget  = 0.0;   // Quãng đường cần đi (mm)
float totalDistTraveled = 0.0;  // Quãng đường đã đi (mm)
float targetAngle = 0.0;        // Góc cần hướng tới (rad)
bool  rotationDone = false;     // Đã xoay xong chưa
int   prevTickL_nav = 0;        // Encoder trước đó (cho navigation)
int   prevTickR_nav = 0;

void goToXY_init(float x_target, float y_target)
{
  // Tính quãng đường cần đi
  totalDistTarget = sqrt(x_target * x_target + y_target * y_target);
  totalDistTraveled = 0.0;

  // Tính góc cần hướng tới
  targetAngle = atan2(y_target, x_target);

  // Reset trạng thái
  rotationDone = false;
  prevTickL_nav = encoder1_value;
  prevTickR_nav = encoder2_value;


}

bool goToXY(float x_target, float y_target)
{
  float dt = 0.02;  // ~50Hz loop

  // ---- BƯỚC 1: Xoay tại chỗ hướng về mục tiêu ----
  // PID_theta: setpoint = targetAngle, actual = yaw (IMU)
  if (!rotationDone)
  {
    // Kiểm tra sai số góc (dùng IMU yaw)
    float yaw_rad = yaw * PI / 180.0;
    float angle_error = targetAngle - yaw_rad;
    angle_error = atan2(sin(angle_error), cos(angle_error));

    if (fabs(angle_error) > ANGLE_THRESHOLD)
    {
      // Dùng PID xoay góc
      float w_turn = PID_theta(targetAngle, dt);
      driveVW(0, w_turn);



      // Reset encoder count khi đang xoay
      prevTickL_nav = encoder1_value;
      prevTickR_nav = encoder2_value;
      return false;
    }

    // Đã xoay xong
    rotationDone = true;
    prevTickL_nav = encoder1_value;
    prevTickR_nav = encoder2_value;

    // Reset toạ độ odometry — chỉ đếm quãng đường đi thẳng
    odom_x = 0.0;
    odom_y = 0.0;
    prev_encoder1 = encoder1_value;
    prev_encoder2 = encoder2_value;

    // ★ Reset yaw IMU về 0 → hướng hiện tại = trục X mới
    IMU_reset();

    // Reset PID cho phase đi thẳng
    PID_resetStraight();
    PID_resetHeading();
    PID_resetTheta();


  }

  // ---- BƯỚC 2: Đi thẳng ----
  // Đếm quãng đường bằng encoder
  int dTickL = -(encoder1_value - prevTickL_nav);
  int dTickR = -(encoder2_value - prevTickR_nav);
  prevTickL_nav = encoder1_value;
  prevTickR_nav = encoder2_value;

  float dSL = dTickL * DISTANCE_PER_TICK;
  float dSR = dTickR * DISTANCE_PER_TICK;
  float dS  = (dSL + dSR) / 2.0;
  totalDistTraveled += dS;

  // Đã đi đủ quãng đường → dừng
  if (totalDistTraveled >= totalDistTarget - DIST_THRESHOLD)
  {
    Motor(0, 0);

    return true;
  }

  // Tính PWM cơ bản từ vận tốc
  float remaining = totalDistTarget - totalDistTraveled;
  float v_cmd = DRIVE_SPEED;
  if (remaining < 80.0)
    v_cmd = DRIVE_SPEED * (remaining / 80.0);
  int basePWM = velocityToPWM(v_cmd);

  // ★ KẾT HỢP 2 PID cho đi thẳng:
  // 1. PID encoder: giữ 2 bánh đi bằng nhau (setpoint=0, actual=enc1-enc2)
  float corrEncoder = PID_straight(dt);

  // 2. PID yaw IMU: giữ hướng 0° (setpoint=0, actual=yaw)
  //    Sau khi reset yaw=0, nếu xe bị lệch, yaw ≠ 0 → PID kéo về 0
  //    PID_heading output trực tiếp là PWM correction
  float corrYaw = -PID_heading(dt);

  // Tổng correction
  float totalCorr = corrEncoder + corrYaw;

  // Áp dụng: bánh nào đi nhiều hơn / lệch hướng → điều chỉnh
  int pwmL = basePWM + (int)totalCorr;
  int pwmR = basePWM - (int)totalCorr;
  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);

  Motor(pwmL, pwmR);
  return false;
}
