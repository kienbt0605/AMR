// ==================== WIFI COMMUNICATION ====================
// ESP32 phát WiFi (AP mode) + TCP Server
// Nhận toạ độ mục tiêu (X, Y) từ Python simulation
// Gửi trạng thái robot (x, y, yaw) lên Python
//
// Kết nối:
//   - SSID: AMR_Robot_WiFi
//   - Password: 12345678
//   - IP: 192.168.4.1
//   - Port: 8080
// ============================================================

#include <WiFi.h>

// Extern: biến từ các file .ino khác
extern float yaw;
extern float odom_x, odom_y;
extern float x_cood, y_cood;
extern float odom_theta;
extern volatile int encoder1_value, encoder2_value;
extern bool arrived;

// Extern: hàm từ Odometry.ino
extern void goToXY_init(float x, float y);

// WiFi AP config
const char* WIFI_SSID = "AMR_Robot_WiFi";
const char* WIFI_PASS = "12345678";

// TCP Server
WiFiServer wifiServer(8080);
WiFiClient wifiClient;

// Biến lưu toạ độ mục tiêu nhận từ Python
float wifi_targetX = 0.0;
float wifi_targetY = 0.0;
bool  wifi_hasTarget = false;   // true khi có lệnh mới từ Python

// =====================================================
// Khởi tạo WiFi AP + TCP Server - gọi trong setup()
// =====================================================
void WiFi_init()
{
  // Phát WiFi Access Point
  WiFi.softAP(WIFI_SSID, WIFI_PASS);

  // Khởi động TCP Server
  wifiServer.begin();
}

// =====================================================
// Đọc lệnh từ Python qua TCP
// Format nhận: "GO,X,Y\n"
// Ví dụ: "GO,500.0,300.0\n"
// =====================================================
void WiFi_readCommand()
{
  // Kiểm tra client mới kết nối
  if (!wifiClient || !wifiClient.connected()) {
    WiFiClient newClient = wifiServer.available();
    if (newClient) {
      wifiClient = newClient;
    }
    return;
  }

  // Đọc dữ liệu từ client
  if (wifiClient.available()) {
    String cmd = wifiClient.readStringUntil('\n');
    cmd.trim();

    // Parse lệnh GO,X,Y
    if (cmd.startsWith("GO,")) {
      int comma1 = cmd.indexOf(',');
      int comma2 = cmd.indexOf(',', comma1 + 1);

      if (comma1 > 0 && comma2 > comma1) {
        wifi_targetX = cmd.substring(comma1 + 1, comma2).toFloat();
        wifi_targetY = cmd.substring(comma2 + 1).toFloat();
        wifi_hasTarget = true;

        // Khởi tạo navigation
        goToXY_init(wifi_targetX, wifi_targetY);
        arrived = false;

        // Phản hồi cho Python
        wifiClient.println("OK");
      }
    }
    else if (cmd == "STOP") {
      // Dừng robot
      arrived = true;
      wifi_hasTarget = false;
      wifiClient.println("STOPPED");
    }
    else if (cmd == "RESET") {
      // Reset odometry
      arrived = true;
      wifi_hasTarget = false;
      wifiClient.println("RESET_OK");
    }
  }
}

// =====================================================
// Gửi trạng thái robot lên Python
// Format gửi: "STATUS,x,y,yaw,arrived\n"
// =====================================================
void WiFi_sendStatus()
{
  if (!wifiClient || !wifiClient.connected()) return;

  char buf[128];
  snprintf(buf, sizeof(buf),
    "STATUS,%.1f,%.1f,%.2f,%d\n",
    x_cood, y_cood, yaw, arrived ? 1 : 0
  );
  wifiClient.print(buf);
}
