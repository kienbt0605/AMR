// ★ Debug: lưu PWM gần nhất để gửi qua telemetry
int lastPWM_L = 0;
int lastPWM_R = 0;

void Motor(int LPWM, int RPWM) 
{
  digitalWrite(STBY, 1);
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  // ★ Lưu lại để telemetry đọc
  lastPWM_L = LPWM;
  lastPWM_R = RPWM;

  if (LPWM >= 0) 
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } 
  else 
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  // ★ ĐẢO CHIỀU motor phải — sửa lỗi CW không xoay
  if (RPWM >= 0) 
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } 
  else 
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  analogWrite(ENA, abs(LPWM));
  analogWrite(ENB, abs(RPWM));
}