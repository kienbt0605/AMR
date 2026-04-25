// ==================== BLUETOOTH DEBUG (Classic SPP) ====================
// Truyền Serial debug lên điện thoại qua Bluetooth Classic
//
// Yêu cầu: Tools → Partition Scheme → "Huge APP (3MB No OTA/1MB SPIFFS)"
//
// Cách dùng trên điện thoại:
//   1. Pair với "AMR_Robot" trong Bluetooth Settings
//   2. Mở app "Serial Bluetooth Terminal"
//   3. Devices → Bluetooth Classic → chọn "AMR_Robot"
//   4. Connect → nhận data debug realtime
// ======================================================================

#include "BluetoothSerial.h"

// Extern: biến từ các file .ino khác
extern float yaw;
extern float x_cood, y_cood;
extern float odom_theta;
extern volatile int encoder1_value, encoder2_value;

BluetoothSerial SerialBT;

// =====================================================
// Khởi tạo Bluetooth - gọi trong setup()
// =====================================================
void BT_init()
{
  SerialBT.begin("AMR_Robot");
}

// =====================================================
// In toàn bộ thông tin debug qua Bluetooth
// Gọi trong loop()
// =====================================================
void BT_debugAll()
{
  char buf[128];
  snprintf(buf, sizeof(buf),
    "yaw:%.1f x:%.1f y:%.1f th:%.1f e1:%d e2:%d\n",
    yaw, x_cood, y_cood,
    odom_theta * 180.0 / PI,
    encoder1_value, encoder2_value
  );
  SerialBT.print(buf);
}
