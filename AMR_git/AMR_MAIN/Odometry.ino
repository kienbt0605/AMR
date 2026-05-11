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

// Vị trí & hướng robot (local — bị reset khi goToXY)
float odom_x     = 0.0;  // mm
float odom_y     = 0.0;  // mm
float odom_theta = 0.0;  // rad
float x_cood;
float y_cood;

// Toạ độ global (KHÔNG bao giờ reset, dùng cho telemetry về Python)
float global_x     = 0.0;  // mm
float global_y     = 0.0;  // mm
float global_theta = 0.0;  // rad
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

  // === Cập nhật toạ độ GLOBAL (encoder distance + IMU yaw tuyệt đối) ===
  // Dùng S từ encoder + getGlobalYaw() (hướng tuyệt đối, không bị IMU_reset ảnh hưởng)
  extern float getGlobalYaw();
  float global_yaw_deg = getGlobalYaw();
  float yaw_rad = global_yaw_deg * PI / 180.0;
  global_x += S * cos(yaw_rad);
  global_y += S * sin(yaw_rad);
  global_theta = yaw_rad;
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

// Reset toạ độ global (chỉ gọi khi cần reset hoàn toàn)
void Odom_resetGlobal()
{
  global_x     = 0.0;
  global_y     = 0.0;
  global_theta = 0.0;
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
const float GYRO_RATE_THRESHOLD = 1;      // °/s — robot dừng quay khi |gyroZ| < giá trị này
const unsigned long SETTLE_DURATION_MS = 150; // ms — thời gian chờ ổn định sau xoay

// Biến theo dõi quãng đường
float totalDistTarget  = 0.0;   // Quãng đường cần đi (mm)
float totalDistTraveled = 0.0;  // Quãng đường đã đi (mm)
float targetAngle = 0.0;        // Góc cần hướng tới (rad)
bool  rotationDone = false;     // Đã xoay xong chưa
int   prevTickL_nav = 0;        // Encoder trước đó (cho navigation)
int   prevTickR_nav = 0;
float crossTrackError = 0.0;    // Lệch ngang tích lũy (mm) cho cross-track PID
float headingSetpoint_deg = 0.0; // Góc tuyệt đối (°) cần giữ khi đi thẳng (lưu từ yaw sau khi xoay xong)
bool  settlingDone = false;             // Đã chờ ổn định xong chưa
unsigned long settleStartTime = 0;      // Thời điểm bắt đầu settling (ms)
unsigned long straightStartMs = 0;      // Thời điểm bắt đầu đi thẳng (ms) - cho timeout

void goToXY_init(float target_x, float target_y)
{
  // Dừng motor hoàn toàn trước khi bắt đầu segment mới
  Motor(0, 0);

  // ★ TARGET_X, TARGET_Y giờ là toạ độ GLOBAL (Python world frame)
  // Vị trí robot trong Python frame: (-global_x, -global_y)
  // Vector từ robot → đích: (target_x - (-global_x), target_y - (-global_y))
  float dx = target_x + global_x;   // = target_x - (-global_x)
  float dy = target_y + global_y;   // = target_y - (-global_y)

  // Tính quãng đường cần đi
  totalDistTarget = sqrt(dx * dx + dy * dy);
  totalDistTraveled = 0.0;

  // Tính góc cần hướng tới (TUYỆT ĐỐI trong yaw frame)
  // atan2(dy, dx) trả về hướng tuyệt đối trong Python/yaw frame
  // → KHÔNG cần cộng yaw nữa (đã là góc tuyệt đối)
  targetAngle = atan2(dy, dx);
  // Chuẩn hoá về [-PI, PI]
  targetAngle = atan2(sin(targetAngle), cos(targetAngle));

  // Reset trạng thái navigation
  rotationDone = false;
  settlingDone = false;
  settleStartTime = 0;
  prevTickL_nav = encoder1_value;
  prevTickR_nav = encoder2_value;

  // ★ Reset tất cả PID cho segment mới — tránh state cũ gây nhiễu
  PID_resetTheta();
  PID_resetStraight();
  PID_resetHeading();
  PID_resetCross();
  crossTrackError = 0.0;
  straightStartMs = 0;  // Reset timeout cho phase đi thẳng

  // ★ DEBUG — in khi nhận lệnh GO mới
  Serial.printf("INIT: target=(%.1f,%.1f) dx=%.1f dy=%.1f dist=%.1f  yaw=%.1f  tgtAngle=%.1f deg  gPos=(%.1f,%.1f)\n",
    target_x, target_y, dx, dy, totalDistTarget,
    yaw, targetAngle * 180.0 / PI, -global_x, -global_y);
}

bool goToXY(float x_target, float y_target, float dt)
{

  // ---- BƯỚC 1: Xoay tại chỗ hướng về mục tiêu ----
  // PID_theta: setpoint = targetAngle, actual = yaw (IMU)
  // ★ Điều kiện xoay xong: |angle_error| < threshold VÀ |gyroZ| < threshold
  extern float currentGyroZ;  // Vận tốc góc từ IMU (°/s)

  if (!rotationDone)
  {
    float yaw_rad = yaw * PI / 180.0;
    float angle_error = targetAngle - yaw_rad;
    angle_error = atan2(sin(angle_error), cos(angle_error));

    // ★ DEBUG ROTATION — in mỗi 100ms
    static unsigned long lastRotDbg = 0;
    if (millis() - lastRotDbg >= 100) {
      lastRotDbg = millis();
      Serial.printf("ROT: yaw=%.1f tgt=%.1f err=%.3f(%.1f°) gyroZ=%.2f rotDone=%d\n",
        yaw, targetAngle * 180.0 / PI,
        angle_error, angle_error * 180.0 / PI,
        currentGyroZ, rotationDone);
    }

    // ★ Kiểm tra CẢ góc VÀ vận tốc góc (gyro rate)
    // Robot chỉ "xoay xong" khi đã đến đúng góc VÀ đã dừng quay thực sự
    if (fabs(angle_error) > ANGLE_THRESHOLD || fabs(currentGyroZ) > GYRO_RATE_THRESHOLD)
    {
      if (fabs(angle_error) > ANGLE_THRESHOLD) {
        // Chưa đến góc — tiếp tục xoay bằng PID
        float w_turn = PID_theta(targetAngle, dt);
        driveVW(0, w_turn);

        // ★ DEBUG: in w_turn và PWM
        if (millis() - lastRotDbg < 20) {
          float vL_dbg = -(w_turn * WHEEL_BASE) / 2.0;
          float vR_dbg = +(w_turn * WHEEL_BASE) / 2.0;
          Serial.printf("  w_turn=%.2f vL=%.1f vR=%.1f\n", w_turn, vL_dbg, vR_dbg);
        }
      } else {
        // Đã đến góc nhưng vẫn đang quay (quán tính) — phanh chờ dừng
        Motor(0, 0);
      }

      prevTickL_nav = encoder1_value;
      prevTickR_nav = encoder2_value;
      return false;
    }

    // ★ Cả góc và vận tốc góc đều OK — bắt đầu pha settling
    rotationDone = true;
    settlingDone = false;
    settleStartTime = millis();
    Motor(0, 0);  // Dừng motor hoàn toàn
    prevTickL_nav = encoder1_value;
    prevTickR_nav = encoder2_value;

    Serial.printf("ROTATE DONE: yaw=%.1f  target=%.1f°  gyroZ=%.2f\n",
      yaw, targetAngle * 180.0 / PI, currentGyroZ);
    return false;  // Chưa đi thẳng — chờ settling
  }

  // ---- PHA SETTLING: Chờ robot ổn định hoàn toàn sau xoay ----
  if (!settlingDone)
  {
    Motor(0, 0);  // Giữ dừng
    prevTickL_nav = encoder1_value;
    prevTickR_nav = encoder2_value;

    if (millis() - settleStartTime >= SETTLE_DURATION_MS)
    {
      settlingDone = true;

      // Reset toạ độ odometry — chỉ đếm quãng đường đi thẳng
      odom_x = 0.0;
      odom_y = 0.0;
      prev_encoder1 = encoder1_value;
      prev_encoder2 = encoder2_value;

      // Dùng targetAngle (góc chính xác) làm setpoint cho PID_heading
      headingSetpoint_deg = targetAngle * 180.0 / PI;

      // Reset PID cho phase đi thẳng
      PID_resetStraight();
      PID_resetHeading();
      PID_resetCross();
      crossTrackError = 0.0;

      Serial.printf("SETTLE DONE: yaw=%.1f  hSet=%.1f  settleMs=%lu\n",
        yaw, headingSetpoint_deg, SETTLE_DURATION_MS);
    }
    return false;
  }

  // ---- BƯỚC 2: Đi thẳng ----
  // Đếm quãng đường bằng encoder (encoder đếm ÂM khi tiến → negate để dS dương)
  int dTickL = -(encoder1_value - prevTickL_nav);
  int dTickR = -(encoder2_value - prevTickR_nav);
  prevTickL_nav = encoder1_value;
  prevTickR_nav = encoder2_value;

  float dSL = dTickL * DISTANCE_PER_TICK;
  float dSR = dTickR * DISTANCE_PER_TICK;
  float dS  = (dSL + dSR) / 2.0;
  totalDistTraveled += dS;

  // ★ DEBUG — in mỗi 500ms qua Serial (xóa sau khi xác nhận đúng)
  static unsigned long lastDebugMs = 0;
  if (millis() - lastDebugMs >= 500) {
    lastDebugMs = millis();
    Serial.printf("NAV: dist=%.1f/%.1f  dTL=%d dTR=%d  e1=%d e2=%d  yaw=%.1f hSet=%.1f\n",
      totalDistTraveled, totalDistTarget,
      dTickL, dTickR,
      encoder1_value, encoder2_value,
      yaw, headingSetpoint_deg);
  }

  // Đã đi đủ quãng đường → dừng
  if (totalDistTraveled >= totalDistTarget - DIST_THRESHOLD)
  {
    Motor(0, 0);
    Serial.printf("ARRIVED: dist=%.1f/%.1f\n", totalDistTraveled, totalDistTarget);
    return true;
  }

  // ★ Timeout an toàn: nếu đi quá lâu (>15s) → dừng cưỡng bức
  if (straightStartMs == 0) straightStartMs = millis();
  if (millis() - straightStartMs > 15000) {
    Motor(0, 0);
    Serial.printf("TIMEOUT: dist=%.1f/%.1f\n", totalDistTraveled, totalDistTarget);
    straightStartMs = 0;
    return true;
  }

  // Tính PWM cơ bản từ vận tốc
  float remaining = totalDistTarget - totalDistTraveled;
  float v_cmd = DRIVE_SPEED;
  if (remaining < 80.0)
    v_cmd = DRIVE_SPEED * (remaining / 80.0);

  // ★ Đảm bảo v_cmd đủ lớn để basePWM >= MIN_PWM (tránh motor stall)
  float v_min = MIN_PWM * MAX_VELOCITY / 255.0;  // ~116 mm/s tại MIN_PWM=50
  if (v_cmd < v_min && v_cmd > 5.0)
    v_cmd = v_min;

  int basePWM = velocityToPWM(v_cmd);

  // ★ KẾT HỢP 3 PID cho đi thẳng:
  // 1. PID encoder: giữ 2 bánh đi bằng nhau (setpoint=0, actual=enc1-enc2)
  float corrEncoder = PID_straight(dt);

  // 2. PID yaw IMU: giữ hướng tuyệt đối (setpoint=headingSetpoint_deg, actual=yaw)
  //    Nếu quán tính từ xoay gây lệch, PID tự kéo về đúng targetAngle
  float corrYaw = -PID_heading(headingSetpoint_deg, dt);

  // 3. PID cross-track: sửa lệch ngang (kéo robot về đường thẳng lý tưởng)
  //    Lệch ngang = dS * sin(yaw - headingSetpoint) — lệch so với hướng mục tiêu
  float heading_error_rad = (yaw - headingSetpoint_deg) * PI / 180.0;
  crossTrackError += dS * sin(heading_error_rad);
  float corrCross = PID_crossTrack(crossTrackError, dt);

  // Tổng correction
  float totalCorr = corrEncoder + corrYaw + corrCross;

  // Áp dụng: bánh nào đi nhiều hơn / lệch hướng → điều chỉnh
  int pwmL = basePWM + (int)totalCorr;
  int pwmR = basePWM - (int)totalCorr;
  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);

  Motor(pwmL, pwmR);
  return false;
}
