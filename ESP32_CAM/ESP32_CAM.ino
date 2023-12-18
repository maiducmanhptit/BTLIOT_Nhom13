#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Base64.h"
#include "esp_camera.h"
#include <WebServer.h>
WebServer server(80);

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED2_PIN 14
#define FLASH_LED_PIN 4
#define PIR_SENSOR_PIN 13

const char* ssid = "Manh";
const char* password = "9999999900";
//======================================== 
void handleRoot() {
  String html = "<html><body><h1>ESP32-CAM Stream</h1><img src=\"/stream\"></body></html>";
  server.send(200, "text/html", html);
}

void handleStream() {
  WiFiClient client = server.client();
  camera_fb_t * fb = NULL;

  String header = "HTTP/1.1 200 OK\r\n";
  header += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(header);

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }
    
    // Bắt đầu một khung hình mới
    server.sendContent("--frame\r\n");
    server.sendContent("Content-Type: image/jpeg\r\n\r\n");

    // Gửi hình ảnh JPEG
    client.write(fb->buf, fb->len);
    server.sendContent("\r\n");

    esp_camera_fb_return(fb);

    // Kiểm tra nếu kết nối đã bị đóng
    if (!client.connected()) {
      break;
    }
  }
}
void startCameraServer() {
  server.on("/", handleRoot);
  server.on("/stream", HTTP_GET, handleStream);
  server.begin();
}
//======================================== Replace with your "Deployment ID" and Folder Name.
String myDeploymentID = "AKfycbw9bfd6ONJw4BppFdz8_l3Y5k79DH8f8KPlvB3HjWjMxRNy0QqyCRKb1bjbu5hDsmfzbQ";
String myMainFolderName = "ESP32-CAM";

bool LED_Flash_ON = true;

WiFiClientSecure client;
void startCameraServer();

void Test_Con() {
  const char* host = "script.google.com";
  while(1) {
    Serial.println("-----------");
    Serial.println("Kiểm tra kết nối...");
    Serial.println("Kết nối đến " + String(host));
  
    client.setInsecure();
  
    if (client.connect(host, 443)) {
      Serial.println("Kết nối thành công.");
      Serial.println("-----------");
      client.stop();
      break;
    } else {
      Serial.println("Kết nối đến " + String(host) + " thất bại.");
      Serial.println("Chờ một lát để kết nối lại.");
      Serial.println("-----------");
      client.stop();
    }
  
    delay(1000);
  }
}

void SendCapturedPhotos() {
  const char* host = "script.google.com";
  Serial.println();
  Serial.println("-----------");
  Serial.println("Kết nối đến " + String(host));
  
  client.setInsecure();

  //---------------------------------------- Đèn LED nhấp nháy một lần để báo hiệu bắt đầu kết nối.
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(100);
  digitalWrite(FLASH_LED_PIN, LOW);
  delay(100);
  //---------------------------------------- 

  //---------------------------------------- Quá trình kết nối, chụp ảnh và gửi lên Google Drive.
  if (client.connect(host, 443)) {
    Serial.println("Kết nối thành công.");
    
    if (LED_Flash_ON == true) {
      digitalWrite(FLASH_LED_PIN, HIGH);
      delay(100);
    }

    //.............................. Chụp ảnh.
    Serial.println();
    Serial.println("Đang chụp ảnh...");
    
    for (int i = 0; i <= 3; i++) {
      camera_fb_t * fb = NULL;
      fb = esp_camera_fb_get();
       if(!fb) {
          Serial.println("Camera capture failed");
          Serial.println("Restarting the ESP32 CAM.");
          delay(1000);
          ESP.restart();
          return;
        } 
      esp_camera_fb_return(fb);
      delay(200);
    }
  
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();
    if(!fb) {
      Serial.println("Camera capture failed");
      Serial.println("Restarting the ESP32 CAM.");
      delay(1000);
      ESP.restart();
      return;
    } 
  
    if (LED_Flash_ON == true) digitalWrite(FLASH_LED_PIN, LOW);
    
    Serial.println("Chụp ảnh thành công.");
    //.............................. 

    //.............................. Sending image to Google Drive.
    Serial.println();
    Serial.println("Đang gửi ảnh lên Google Drive.");
    Serial.println("Size: " + String(fb->len) + "byte");
    
    String url = "/macros/s/" + myDeploymentID + "/exec?folder=" + myMainFolderName;

    client.println("POST " + url + " HTTP/1.1");
    client.println("Host: " + String(host));
    client.println("Transfer-Encoding: chunked");
    client.println();

    int fbLen = fb->len;
    char *input = (char *)fb->buf;
    int chunkSize = 3 * 1000; //--> must be multiple of 3.
    int chunkBase64Size = base64_enc_len(chunkSize);
    char output[chunkBase64Size + 1];

    Serial.println();
    int chunk = 0;
    for (int i = 0; i < fbLen; i += chunkSize) {
      int l = base64_encode(output, input, min(fbLen - i, chunkSize));
      client.print(l, HEX);
      client.print("\r\n");
      client.print(output);
      client.print("\r\n");
      delay(100);
      input += chunkSize;
      Serial.print(".");
      chunk++;
      if (chunk % 50 == 0) {
        Serial.println();
      }
    }
    client.print("0\r\n");
    client.print("\r\n");

    esp_camera_fb_return(fb);
    //.............................. 

    //.............................. Waiting for response.
    Serial.println("Đang chờ phản hồi...");
    long int StartTime = millis();
    while (!client.available()) {
      Serial.print(".");
      delay(100);
      if ((StartTime + 10 * 1000) < millis()) {
        Serial.println();
        Serial.println("No response.");
        break;
      }
    }
    Serial.println();
    while (client.available()) {
      Serial.print(char(client.read()));
    }

    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(500);
    //.............................. 
  }
  else {
    Serial.println("Kết nối đến " + String(host) + " thất bại.");
    
    //.............................. Flash LED blinks twice as a failed connection indicator.
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(500);
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(500);
    //.............................. 
  }
  //---------------------------------------- 

  Serial.println("-----------");

  client.stop();
}

void setup() {
  // put your setup code here, to run once:
  
  // Disable brownout detector.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);
  Serial.println();
  delay(1000);
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(FLASH_LED_PIN, OUTPUT);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // Setting the ESP32 WiFi to station mode.
  Serial.println();
  Serial.println("Setting the ESP32 WiFi to station mode.");
  WiFi.mode(WIFI_STA);

  //---------------------------------------- The process of connecting ESP32 CAM with WiFi Hotspot / WiFi Router.
  Serial.println();
  Serial.print("Connecting to : ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.print("Assigned IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi not connected!");
  }

  int connecting_process_timed_out = 20; //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(250);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(250);
    if(connecting_process_timed_out > 0) connecting_process_timed_out--;
    if(connecting_process_timed_out == 0) {
      Serial.println();
      Serial.print("Failed to connect to ");
      Serial.println(ssid);
      Serial.println("Restarting the ESP32 CAM.");
      delay(1000);
      ESP.restart();
    }
  }

  digitalWrite(FLASH_LED_PIN, LOW);
  
  Serial.println();
  Serial.print("Successfully connected to ");
  Serial.println(ssid);
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());
  //---------------------------------------- 

  //---------------------------------------- Set the camera ESP32 CAM.
  Serial.println();
  Serial.println("Set the camera ESP32 CAM...");
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
                                                                                                                                                                                                                                                  
  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 8;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial.println();
    Serial.println("Restarting the ESP32 CAM.");
    delay(1000);
    ESP.restart();
  }

  sensor_t * s = esp_camera_sensor_get();

  // Selectable camera resolution details :
  // -UXGA   = 1600 x 1200 pixels
  // -SXGA   = 1280 x 1024 pixels
  // -XGA    = 1024 x 768  pixels
  // -SVGA   = 800 x 600   pixels
  // -VGA    = 640 x 480   pixels
  // -CIF    = 352 x 288   pixels
  // -QVGA   = 320 x 240   pixels
  // -HQVGA  = 240 x 160   pixels
  // -QQVGA  = 160 x 120   pixels
  s->set_framesize(s, FRAMESIZE_SXGA);  //--> UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA

  Serial.println("Setting the camera successfully.");
  Serial.println();

  delay(1000);

  Test_Con();
  Serial.println();
  delay(2000);
  startCameraServer();
}

//________________________________________________________________________________ VOID LOOP()
void loop() {
  server.handleClient();

  if (digitalRead(PIR_SENSOR_PIN) == HIGH) {
    Serial.println("Chuyển động phát hiện - Chụp ảnh");
    digitalWrite(FLASH_LED_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH); // Bật đèn LED 2
    SendCapturedPhotos();
    digitalWrite(FLASH_LED_PIN, LOW);
    digitalWrite(LED2_PIN, LOW); // Tắt đèn LED 2
    delay(20000); // Đợi 20 giây
  } else {
    Serial.println("Không có chuyển động");
    delay(500); // Kiểm tra cảm biến mỗi 0.5 giây
  }
}
