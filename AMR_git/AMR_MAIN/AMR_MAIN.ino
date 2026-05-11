#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <MPU6050.h>

#define IMU_SDA 21
#define IMU_SCL 22

// Thông số robot
#define WHEEL_DIAMETER  66.0    // Đường kính bánh xe (mm)        
#define WHEEL_BASE      205.0  // Khoảng cách giữa 2 bánh (mm)   
#define ENCODER_PPR     495.0   // Số xung encoder mỗi vòng quay 

#define IN1 19
#define IN2 18
#define IN3 14
#define IN4 27
#define ENA 25
#define ENB 26 
#define STBY 15 

//Encoder
#define ENCODER_1A 34 // Pin for Encoder A
#define ENCODER_1B 35 // Pin for Encoder B

#define ENCODER_2A 33 // Pin for Encoder A
#define ENCODER_2B 32 // Pin for Encoder B

#define PI  3.141592653589

volatile bool odomFlag = false;
unsigned long startTime = 0;
bool timerStarted = false;
unsigned long goToXY_prevTime = 0;  // Để tính dt thực cho PID trong goToXY

volatile int encoder1_value = 0;
volatile int encoder2_value = 0;
volatile float vRight;
volatile float vLeft;
volatile float v;
volatile float w;

// Extern: biến từ Wifi.ino (compile sau theo alphabet)
extern float wifi_targetX;
extern float wifi_targetY;
extern bool  wifi_hasTarget;

hw_timer_t *timer0 = NULL;
void IRAM_ATTR timer0_ISR() {
  odomFlag = true;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENCODER_1A, INPUT_PULLUP);
  pinMode(ENCODER_1B, INPUT_PULLUP);

  pinMode(ENCODER_2A, INPUT_PULLUP);
  pinMode(ENCODER_2B, INPUT_PULLUP);

  // Attaching the ISR to encoder 1A
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A), encoder1_isr, RISING); 

  // Attaching the ISR to encoder 2A
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A), encoder2_isr, RISING);

  timer0 = timerBegin(1000000);
  // Gắn ISR
  timerAttachInterrupt(timer0, &timer0_ISR);

  // Set alarm sau 1 giây (1,000,000 microseconds)
  timerAlarm(timer0, 20, true, 0); //20ms

  // Khởi tạo MPU6050
  IMU_init();

  // Khởi tạo Odometry
  Odom_init();

  // Khởi tạo WiFi AP + TCP Server
  WiFi_init();
  
  // Bật driver motor
  digitalWrite(STBY, HIGH);

  // Không goToXY_init ở đây — chờ lệnh từ Python qua WiFi
}

bool arrived = true;  // Bắt đầu ở trạng thái đã dừng, chờ lệnh

void loop() {
  // Đọc lệnh từ Python qua WiFi (GO,X,Y) — không cần đồng bộ chặt
  WiFi_readCommand();

  // Tất cả xử lý cần đồng bộ 20ms → đặt trong odomFlag
  if (odomFlag) {
    odomFlag = false;

    // Đọc và cập nhật góc từ MPU6050 (đồng bộ với Odometry)
    IMU_read();

    // Cập nhật Odometry
    Odom_update();

    // Di chuyển đến mục tiêu (nếu có)
    if (!arrived && wifi_hasTarget) {
      // Tính dt thực cho PID (không hardcode 0.02)
      unsigned long now_us = micros();
      float dt_real = (now_us - goToXY_prevTime) / 1000000.0f;
      if (dt_real <= 0 || dt_real > 0.1f) dt_real = 0.02f;  // Bảo vệ
      goToXY_prevTime = now_us;

      arrived = goToXY(wifi_targetX, wifi_targetY, dt_real);
      if (arrived) {
        wifi_hasTarget = false;
      }
    }

    // Gửi trạng thái robot qua WiFi cho Python
    WiFi_sendStatus();

    // In v, w, x, y, theta
    Odom_print();
    IMU_print();

  }
}
