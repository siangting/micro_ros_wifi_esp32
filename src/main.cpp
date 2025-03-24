#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// micro_ros_arduino 的核心函式庫
#include <micro_ros_arduino.h>

// ROS 2 / Micro-ROS 基礎
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h> 

// ====== 使用者需自行修改的 Wi-Fi 連線資訊 ======
const char* ssid     = "screamlab";
const char* password = "s741852scream";

// ====== Micro-ROS Agent 的 IP 與 Port ======
IPAddress agent_ip(192, 168, 75, 2); // 請改成你的電腦(Agent) IP
uint16_t  agent_port = 8888;         // 與 Agent 監聽的 Port 一致

// 建立全域 UDP 物件，用於傳送/接收 Micro-ROS 封包
WiFiUDP udp;

// ====== ROS 2 相關物件 ======
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rcl_timer_t timer;
rclc_executor_t executor;

/* =====================
    自訂運輸層 (Custom Transport)
   ===================== */

// 1) 開啟連線
extern "C" bool arduino_wifi_transport_open(struct uxrCustomTransport * transport) {
  // UDP 一般不需要特別連線
  Serial.println("[DEBUG] transport_open() called.");
  return true;
}

// 2) 關閉連線
extern "C" bool arduino_wifi_transport_close(struct uxrCustomTransport * transport) {
  Serial.println("[DEBUG] transport_close() called. Stopping UDP...");
  udp.stop();
  return true;
}

// 3) 寫入資料（傳送到 Agent）
extern "C" size_t arduino_wifi_transport_write(struct uxrCustomTransport* transport, 
                                               const uint8_t* buf, 
                                               size_t len, 
                                               uint8_t* err) 
{
  (void) transport;
  (void) err;

  // Debug：顯示即將傳送的資料量
  Serial.print("[DEBUG] transport_write() - length: ");
  Serial.println(len);

  udp.beginPacket(agent_ip, agent_port);
  size_t written = udp.write(buf, len);
  udp.endPacket();

  Serial.print("[DEBUG] transport_write() - actually written: ");
  Serial.println(written);

  return written;
}

// 4) 讀取資料（從 Agent 收到的封包）
extern "C" size_t arduino_wifi_transport_read(struct uxrCustomTransport* transport, 
                                              uint8_t* buf, 
                                              size_t len, 
                                              int timeout, 
                                              uint8_t* err) 
{
  (void) transport;
  (void) timeout;
  (void) err;

  size_t packet_size = udp.parsePacket();
  size_t received = 0;

  if (packet_size > 0) {
    received = udp.read(buf, len);
    Serial.print("[DEBUG] transport_read() - packet_size: ");
    Serial.print(packet_size);
    Serial.print(", received: ");
    Serial.println(received);
  }

  return received;
}

/* =====================
    Timer Callback
   ===================== */
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void) last_call_time; // 未使用參數
  if (timer != nullptr) {
    msg.data++;
    // 若要嚴謹，可檢查 rcl_publish() 回傳值
    rcl_publish(&publisher, &msg, nullptr);

    Serial.print("[DEBUG] Published: ");
    Serial.println(msg.data);
  }
}

/* =====================
    setup()
   ===================== */
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("[DEBUG] Setup start.");

  // 連線 Wi‑Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("[DEBUG] Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[DEBUG] WiFi connected.");
  Serial.print("[DEBUG] ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  // 啟動 UDP（在 ESP32 端的接聽 port，可與 agent_port 相同或不同）
  Serial.println("[DEBUG] Begin UDP on port: " + String(agent_port));
  udp.begin(agent_port);

  // 設定自訂運輸層
  Serial.println("[DEBUG] Setting up custom transport...");
  rmw_uros_set_custom_transport(
    true,
    NULL,
    arduino_wifi_transport_open,
    arduino_wifi_transport_close,
    arduino_wifi_transport_write,
    arduino_wifi_transport_read
  );

  // 在 setup() 連線 WiFi + 自訂 Transport 完成後：
  if (rmw_uros_ping_agent(2000, 1) == RMW_RET_OK) {
    Serial.println("[DEBUG] Ping Agent success!");
  } else {
    Serial.println("[ERROR] Ping Agent failed!");
  }

  // 建立 Micro-ROS 基本結構
  Serial.println("[DEBUG] Initializing rcl stuff...");
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) {
    Serial.print("[ERROR] rclc_support_init failed with code ");
    Serial.println(rc);
  }

  // 建立 ROS 2 Node
  Serial.println("[DEBUG] Creating node: esp32_wifi_node");
  rcl_node_t node;
  rc = rclc_node_init_default(&node, "esp32_wifi_node", "", &support);
  if (rc != RCL_RET_OK) {
    Serial.print("[ERROR] rclc_node_init_default failed with code ");
    Serial.println(rc);
  }

  // 建立 Publisher
  Serial.println("[DEBUG] Creating publisher: esp32_topic");
  rc = rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "esp32_topic"
  );
  if (rc != RCL_RET_OK) {
    Serial.print("[ERROR] rclc_publisher_init_default failed with code ");
    Serial.println(rc);
  }

  // 建立 Timer，每 1000 ms (1 秒) 呼叫一次
  Serial.println("[DEBUG] Creating timer, period = 1000 ms");
  const unsigned int timer_timeout = 1000; // 毫秒
  rc = rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback
  );
  if (rc != RCL_RET_OK) {
    Serial.print("[ERROR] rclc_timer_init_default failed with code ");
    Serial.println(rc);
  }

  // 建立 Executor 並加入 Timer
  Serial.println("[DEBUG] Creating executor...");
  rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (rc != RCL_RET_OK) {
    Serial.print("[ERROR] rclc_executor_init failed with code ");
    Serial.println(rc);
  }

  rc = rclc_executor_add_timer(&executor, &timer);
  if (rc != RCL_RET_OK) {
    Serial.print("[ERROR] rclc_executor_add_timer failed with code ");
    Serial.println(rc);
  }

  // 初始化訊息
  msg.data = 0;

  Serial.println("[DEBUG] Setup done.");
}

/* =====================
    loop()
   ===================== */
void loop() {
  // 執行 Executor
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  // 這個 delay(10) 是放慢 spin 的頻率
  delay(10);
}
