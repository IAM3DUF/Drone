#include "esp_camera.h"
#include <WiFi.h>
#include <SD_MMC.h>

// Define the SSID and Password for the Wi-Fi Access Point
const char* ssid = "GatorCam";
const char* password = "Gator1234";

// Create a WiFiServer object to listen for incoming connections
WiFiServer server(80);

// Select camera model
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Task handles
TaskHandle_t StreamTaskHandle;
TaskHandle_t RecordTaskHandle;

// Function prototypes
void stream_task(void *pvParameters);
void record_task(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Serial.println();

  // Start the Wi-Fi access point
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP started");

  // Initialize the camera
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
  config.frame_size = FRAMESIZE_QVGA;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Initialize the camera server
  server.begin();
  Serial.println("HTTP server started");

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("' to connect");

  // Initialize SD card
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return;
  }

  // Create tasks
  xTaskCreatePinnedToCore(
    stream_task,        // Function to implement the task
    "Stream Task",      // Name of the task
    10000,              // Stack size in words
    NULL,               // Task input parameter
    1,                  // Priority of the task
    &StreamTaskHandle,  // Task handle
    0                   // Core where the task should run
  );

  xTaskCreatePinnedToCore(
    record_task,        // Function to implement the task
    "Record Task",      // Name of the task
    10000,              // Stack size in words
    NULL,               // Task input parameter
    1,                  // Priority of the task
    &RecordTaskHandle,  // Task handle
    1                   // Core where the task should run
  );
}

void loop() {
  // Main loop does nothing, tasks run independently
}

void stream_task(void *pvParameters) {
  while (true) {
    WiFiClient client = server.available();
    if (client) {
      Serial.println("New client connected");
      String req = client.readStringUntil('\r');
      client.flush();

      if (req.indexOf("GET /") != -1) {
        // Send the initial HTTP headers
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
        client.println();

        while (client.connected()) {
          camera_fb_t *fb = esp_camera_fb_get();
          if (!fb) {
            Serial.println("Camera capture failed");
            client.stop();
            return;
          }

          // Send the current frame
          client.print("--frame\r\n");
          client.print("Content-Type: image/jpeg\r\n");
          client.print("Content-Length: ");
          client.print(fb->len);
          client.print("\r\n\r\n");
          client.write(fb->buf, fb->len);
          client.print("\r\n");

          esp_camera_fb_return(fb);

          // Small delay to prevent watchdog timer reset and control frame rate
          delay(10);
        }
      }

      // Close the connection
      client.stop();
      Serial.println("Client disconnected");
    }
  }
}

void record_task(void *pvParameters) {
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      continue;
    }

    // Save frame to SD card
    String path = "/video" + String(millis()) + ".jpg";
    File file = SD_MMC.open(path, FILE_WRITE);
    if (file) {
      file.write(fb->buf, fb->len);
      file.close();
      Serial.printf("Saved frame to %s\n", path.c_str());
    } else {
      Serial.println("Failed to open file in writing mode");
    }

    esp_camera_fb_return(fb);

    // Small delay to control recording rate
    delay(100);
  }
}
