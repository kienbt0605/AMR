const float R_wheel = WHEEL_DIAMETER / 2.0;  // Bán kính bánh xe (mm)
const float DT = 0.02;                   // Chu kỳ lấy mẫu (s) = 1000ms

/*float posX  = 0.0;  // mm
float posY  = 0.0;  // mm
float theta = 0.0;  // rad*/

float Kp =0.5 ,Kd = 0.0;     //Cty 6 
float Kpw = 2, Kdw = 0.0;
float error_prev = 0;
float error_w_prev = 0;

int prev_encoder1 = 0;
int prev_encoder2 = 0;

#define MAX_SPEED_MMS 449.0

int velocityToPWM(float v_mms) {
  float pwm = (v_mms / MAX_SPEED_MMS) * 255.0;
  return constrain((int)pwm, -255, 255);
}

float PID_Correction(float error) {

  float derivative = (error - error_prev) / DT;
  error_prev = error;

  float output = Kp * error + Kd * derivative;
  return constrain(output, -100.0, 100.0);
}

float PID_w_Correction(float error_w)
{
  float derivative = (error_w - error_w_prev) / DT;
  error_w_prev = error_w;

  float output = Kpw * error_w + Kdw * derivative;
  return constrain(output, -100.0, 100.0);
  
}

void Odometry(float v, float w)
{

  vRight = v + (w * WHEEL_BASE) / 2.0;
  vLeft  = v - (w * WHEEL_BASE) / 2.0;

  // Setpoint đi thẳng: w = 0 → yaw phải giữ = 0
  float setpoint = w;       // w = 0 khi đi thẳng
  float actual_w = roll;     // Góc heading thực tế từ IMU (trục Z)

  int basePWM_L = velocityToPWM(vLeft);
  int basePWM_R = velocityToPWM(vRight);

  noInterrupts();
  int enc1_now = encoder1_value;
  int enc2_now = encoder2_value;
  interrupts();

  int delta_encL = enc1_now - prev_encoder1;
  int delta_encR = enc2_now - prev_encoder2;
  prev_encoder1 = enc1_now;
  prev_encoder2 = enc2_now;

  // PID Encoder: setpoint = 0 (hiệu 2 encoder = 0 → 2 bánh quay đều)
  float enc_error = (float)(delta_encL - delta_encR);  // setpoint = 0
  float correction = PID_Correction(enc_error);

  // PID IMU: setpoint = 0 (yaw = 0 → giữ hướng ban đầu)
  float w_error = setpoint - actual_w;                 // setpoint = 0
  float w_correction = PID_w_Correction(w_error);

  // Kết hợp cả 2 correction: encoder chỉnh nhanh, IMU chỉnh drift dài hạn
  float total_correction = correction + w_correction;

  int pwmL = constrain(basePWM_L - (int)total_correction, -255, 255);
  int pwmR = constrain(basePWM_R + (int)total_correction, -255, 255);
  Motor(pwmL, pwmR);

  // Serial.println(encoder1_value);
  // Serial.print("   ");
  // Serial.print(encoder2_value);
}