#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "PIAL";
const char* password = "Pi@ldas164";

// ===========================
// LED flash setup (if your board has one)
// ===========================
#if defined(LED_GPIO_NUM)
void setupLedFlash() {
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW); // Turn off initially
}

void ledFlashOn() {
  digitalWrite(LED_GPIO_NUM, HIGH);
}

void ledFlashOff() {
  digitalWrite(LED_GPIO_NUM, LOW);
}
#endif

// ===========================
// Web server
// ===========================
WebServer server(80);
bool ledState = false;

// ===========================
// Camera Stream
// ===========================
void handleJPGStream(void) {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }

    response = "--frame\r\n";
    response += "Content-Type: image/jpeg\r\n\r\n";
    server.sendContent(response);

    client.write(fb->buf, fb->len);
    server.sendContent("\r\n");
    esp_camera_fb_return(fb);

    if (!client.connected()) break;
  }
}

// ===========================
// Web Interface
// ===========================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>ESP32-CAM</title>";
  html += "<style>";
  html += "body { font-family: Arial; text-align: center; background: #f4f4f4; }";
  html += "h1 { color: #333; }";
  html += "img { border: 2px solid #ccc; margin: 10px 0; }";
  html += "button { padding: 10px 20px; margin: 5px; font-size: 16px; cursor: pointer; }";
  html += "</style></head><body>";
  html += "<h1>ESP32-CAM Live Stream</h1>";
  html += "<img id='stream' src='/stream' width='320' height='240'><br>";
  html += "<button onclick=\"toggleLED()\">Toggle LED Flash</button>";
  html += "<button onclick=\"capturePhoto()\">Capture Photo</button><br>";
  html += "<label for='resolution'>Resolution: </label>";
  html += "<select id='resolution' onchange='changeResolution()'>";
  html += "<option value='FRAMESIZE_QVGA'>QVGA</option>";
  html += "<option value='FRAMESIZE_VGA'>VGA</option>";
  html += "<option value='FRAMESIZE_SVGA'>SVGA</option>";
  html += "</select>";
  html += "<script>";
  html += "function toggleLED() { fetch('/led_toggle'); }";
  html += "function capturePhoto() { window.open('/capture', '_blank'); }";
  html += "function changeResolution() { let res = document.getElementById('resolution').value; fetch('/set_res?res=' + res); }";
  html += "setInterval(()=>{document.getElementById('stream').src='/stream?'+Date.now();},1000);";
  html += "</script>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ===========================
// LED Toggle
// ===========================
void handleLEDToggle() {
#if defined(LED_GPIO_NUM)
  ledState = !ledState;
  digitalWrite(LED_GPIO_NUM, ledState ? HIGH : LOW);
#endif
  server.send(200, "text/plain", ledState ? "ON" : "OFF");
}

// ===========================
// Capture Photo
// ===========================
void handleCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// ===========================
// Set Resolution
// ===========================
void handleSetResolution() {
  String res = server.arg("res");
  sensor_t *s = esp_camera_sensor_get();

  if (res == "FRAMESIZE_QVGA") s->set_framesize(s, FRAMESIZE_QVGA);
  else if (res == "FRAMESIZE_VGA") s->set_framesize(s, FRAMESIZE_VGA);
  else if (res == "FRAMESIZE_SVGA") s->set_framesize(s, FRAMESIZE_SVGA);

  server.send(200, "text/plain", "OK");
}

// ===========================
// Start Web Server
// ===========================
void startCameraServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/stream", HTTP_GET, handleJPGStream);
  server.on("/led_toggle", HTTP_GET, handleLEDToggle);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/set_res", HTTP_GET, handleSetResolution);
  server.begin();
  Serial.println("HTTP server started");
}

// ===========================
// Setup
// ===========================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Camera config
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    config.fb_count = 2;
    config.jpeg_quality = 10;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.frame_size = FRAMESIZE_SVGA;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  // WiFi connection
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("Connecting to WiFi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("Camera ready! Connect to: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi failed. Starting Access Point...");
    WiFi.softAP("ESP32-CAM", "12345678");
    Serial.print("AP IP: http://");
    Serial.println(WiFi.softAPIP());
  }

  startCameraServer();
}

// ===========================
// Loop
// ===========================
void loop() {
  server.handleClient();
}
