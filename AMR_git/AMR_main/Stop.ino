// Cấu hình mục tiêu
float targetDist = 1800.0;   // mm
float maxSpd     = 300.0;    // mm/s
float accel      = 100.0;    // gia tốc tăng
float decel      = 200.0;    // gia tốc giảm
float minSpd     = 10.0;

float dt = 0.02;             // 20 ms

bool reached = false;
float currentSpeed = 0;

void Execute_Stop_At_Target() {

  if (reached) return;

  // --- 1. Tính quãng đường ---
  float d1 = (float)(-encoder1_value);
  float d2 = (float)encoder2_value;

  float dist1 = (d1 / 495.0) * PI * 66.0;
  float dist2 = (d2 / 495.0) * PI * 66.0;

  float currentAvg = (dist1 + dist2) / 2.0;

  // --- 2. Khoảng cách còn lại ---
  float remaining = targetDist - currentAvg;

  // --- 3. Dừng ---
  if (remaining <= 0) {
    currentSpeed = 0;
    reached = true;
    Odometry(0,0);
    Serial.println("STOP");
    return;
  }

  // --- 4. Tính khoảng cách cần để dừng ---
  float stopDist = (currentSpeed * currentSpeed) / (2 * decel);

  // --- 5. Chọn chế độ ---
  if (remaining <= stopDist) {
    // GIẢM TỐC
    currentSpeed -= decel * dt;
  }
  else if (currentSpeed < maxSpd) {
    // TĂNG TỐC
    currentSpeed += accel * dt;
  }

  // --- 6. Giới hạn tốc độ ---
  if (currentSpeed > maxSpd) currentSpeed = maxSpd;
  if (currentSpeed < minSpd) currentSpeed = minSpd;

  // --- 7. Điều khiển robot ---
  Odometry(currentSpeed, 0);

  // Debug
  Serial.print("Dist: "); Serial.print(currentAvg);
  Serial.print(" | Rem: "); Serial.print(remaining);
  Serial.print(" | V: "); Serial.println(currentSpeed);
}