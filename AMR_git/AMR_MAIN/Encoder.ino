void encoder1_isr() 
{
  int B = digitalRead(ENCODER_1B);

  if (B == LOW) 
  {
    encoder1_value++; 
  } 
  else 
  {
    encoder1_value--; 
  }
}

void encoder2_isr() 
{ 
  int B = digitalRead(ENCODER_2B);

  if (B == LOW) 
  {
    encoder2_value++;
  } 
  else 
  {
    encoder2_value--; 
  }
}