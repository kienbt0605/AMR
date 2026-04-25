void Motor(int LPWM, int RPWM) 
{
  digitalWrite(STBY, 1);
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

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

  if (RPWM >= 0) 
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } 
  else 
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, abs(LPWM));
  analogWrite(ENB, abs(RPWM));
}