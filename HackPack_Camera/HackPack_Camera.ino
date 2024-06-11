//20240516
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ### Project Summary: Vision and control of Automated Nerf Ball Launcher
//
// #### Overview:
//          This project uses an ESP32 cam module to host a web interface that sends commands to control an automated Nerf ball launcher.
//
// #### Setup:
//          This project relies on a number of libraries to function.
//            1. Install the follow libraries from the lebrary manager:
//
//          This project relies on the ESP32 boards. Install the following:
//            1. Under file/preferences add the following link to the "Additional board manager URLs" : https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
//            2. Install the board library "ESP32" by Espressif from the board manager
//
//          Select "AI Thinker ESP32-CAM" as the target board
//
//          connect your PC/mobile to the "HackPack" Wifi network
//
//          Using a web browser navigate to http://hackpack.local or 192.168.4. wait for webserver to load.
//
//
// #### Key Features of the Code:
//      1. **Private access point**: This system hosts a private WiFi access point.
//      2. **Web interface**: There is a web server for diplay of the video feed and control buttons for system motion.
//      3. **Communication**: system communicates with simple uart packet to move launcher to new coordinates.
//      5. **Facial recognition**: uses machine vision to track location of a person's face with in the frame
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Include necessary libraries
#include "esp_camera.h"
#include "img_converters.h"
#include "soc/soc.h"           // disable brownout problems
#include "soc/rtc_cntl_reg.h"  // disable brownout problems
#include "esp_http_server.h"
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_system.h>
#include "esp_wifi.h"
#include "fb_gfx.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "sensor.h"
#include "sys/time.h"
#include "sdkconfig.h"

// Pin definitions and other constants
#define PART_BOUNDARY "123456789000000000000987654321"
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define LED_OUTPUT 4

// Version control
const String Version = "V0.2.0";

// Global variables for HTTP server instances
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

// ================================================================
// ===                Face detection stuff                      ===
// ================================================================

#define CONFIG_ESP_FACE_DETECT_ENABLED 1


#include <vector>
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"

#define TWO_STAGE 1 /*<! 1: detect by two-stage which is more accurate but slower(with keypoints). */
                    /*<! 0: detect by one-stage which is less accurate but faster(without keypoints). */

#define FACE_COLOR_WHITE 0x00FFFFFF
#define FACE_COLOR_BLACK 0x00000000
#define FACE_COLOR_RED 0x000000FF
#define FACE_COLOR_GREEN 0x0000FF00
#define FACE_COLOR_BLUE 0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

// ================================================================
// ===                         Web Page                         ===
// ================================================================

// Web page (HTML, CSS, JavaScript) for controlling the robot
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
  <html>
  <head>
    <title>HackPack</title>
    <meta name="viewport" content="width=device-width, height=device-height, initial-scale=1" >
    <style>
      body { font-family: Arial; text-align: center; margin:0 auto; padding-top: 30px;}
      .button {
        background-color: #2f4468;
        width: 100px;
        height: 80px;
        border: none;
        color: white;
        font-size: 20px;
        font-weight: bold;
        text-align: center;
        text-decoration: none;
        border-radius: 10px;
        display: inline-block;
        margin: 6px 6px;
        cursor: pointer;
        -webkit-tap-highlight-color: rgba(0,0,0,0);
        -webkit-user-select: none; /* Chrome, Safari, Opera */
        -moz-user-select: none; /* Firefox all */
        -ms-user-select: none; /* IE 10+ */
        user-select: none; /* Likely future */
      }
      img { width: auto; max-width: 100%; height: auto; }
      #buttons { text-align: center; }
    </style>
  </head>
  <body style="background-color:black;" oncontextmenu="return false;">
    <h1 style="color:white">HackPack</h1>
    <img src="" id="photo">
    <div id="buttons">
      <button class="button" onpointerdown="sendData('target')" onpointerup="releaseData()">Target</button>
      <button class="button" onpointerdown="sendData('up')" onpointerup="releaseData()">Up</button>
      <button class="button" onpointerdown="sendData('shoot')" onpointerup="releaseData()">Shoot</button><br>
      <button class="button" onpointerdown="sendData('left')" onpointerup="releaseData()">Left</button>
      <button class="button" onpointerdown="sendData('stop')" onpointerup="releaseData()">Stop</button>
      <button class="button" onpointerdown="sendData('right')" onpointerup="releaseData()">Right</button><br>
      <button class="button" onpointerdown="sendData('ledon')" onpointerup="releaseData()">LED ON</button>
      <button class="button" onpointerdown="sendData('down')" onpointerup="releaseData()">Down</button>
      <button class="button" onpointerdown="sendData('ledoff')" onpointerup="releaseData()">LED OFF</button>
 </div>
    <script>
      var isButtonPressed = false; // Add this flag

      function sendData(x) {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/action?go=" + x, true);
        xhr.send();
      }

      function releaseData() {
        isButtonPressed = false; // A button has been released
        sendData('stop');
      }

      const keyMap = {
        'ArrowUp': 'up',
        'ArrowLeft': 'left',
        'ArrowDown': 'down',
        'ArrowRight': 'right',
        'KeyW': 'up',
        'KeyA': 'left',
        'KeyS': 'down',
        'KeyD': 'right',
        'KeyL': 'ledon',
        'KeyO': 'ledoff'
      };

      document.addEventListener('keydown', function(event) {
        if (!isButtonPressed) { // Only send data if no button is being pressed
          const action = keyMap[event.code];
          if (action) sendData(action);
          isButtonPressed = true; // A button has been pressed
        }
      });

      document.addEventListener('keyup', function(event) {
         releaseData();
      });

      window.onload = function() {
        document.getElementById("photo").src = window.location.href.slice(0, -1) + ":81/stream";
      }
    </script>
  </body>
</html>
)rawliteral";

// ================================================================
// ===                       Server                             ===
// ================================================================

// Function to start the camera server
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri = "/action",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
  }
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// ================================================================
// ===                    INDEX Handler                         ===
// ================================================================

// HTTP handler for serving the web page
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

// ================================================================
// ===                    Draw Face Box                         ===
// ================================================================

static void draw_face_boxes(fb_data_t *fb, std::list<dl::detect::result_t> *results, int face_id) {
  int x, y, w, h;
  uint32_t color = FACE_COLOR_GREEN;
  
  if (fb->bytes_per_pixel == 2) {
    color = ((color >> 8) & 0xF800) | ((color >> 3) & 0x07E0) | (color & 0x001F);
    // color = ((color >> 16) & 0x001F) | ((color >> 3) & 0x07E0) | ((color << 8) & 0xF800);
  }
  int i = 0;
  for (std::list<dl::detect::result_t>::iterator prediction = results->begin(); prediction != results->end(); prediction++, i++) {
    // rectangle box
    x = (int)prediction->box[0];
    y = (int)prediction->box[1];
    if (x < 1) x = 1;
    if (y < 1) y = 1;

    w = (int)prediction->box[2] - x + 1;
    h = (int)prediction->box[3] - y + 1;
    if (w < 1) w = 1;
    if (h < 1) h = 1;
//The center coordinates of the bounding box are calculated and printed to the Serial port.
    int Center_X = (x + (w / 2)) - 80;
    int Center_Y = (y + (h / 2)) - 60;

    Serial.printf("X%d\tY%d\n", Center_X, Center_Y);

//Ensure the bounding box does not exceed the framebuffer dimensions
    if ((x + w) > fb->width) {
      w = fb->width - x;
    }
    if ((y + h) > fb->height) {
      h = fb->height - y;
    }

//Draw the bounding box on the framebuffer using horizontal and vertical lines.
    fb_gfx_drawFastHLine(fb, x, y, w, color);
    fb_gfx_drawFastHLine(fb, x, y + h - 1, w, color);
    fb_gfx_drawFastVLine(fb, x, y, h, color);
    fb_gfx_drawFastVLine(fb, x + w - 1, y, h, color);

    // #if TWO_STAGE
    //     // landmarks (left eye, mouth left, nose, right eye, mouth right)
    //     int x0, y0, j;
    //     for (j = 0; j < 10; j += 2) {
    //       x0 = (int)prediction->keypoint[j];
    //       y0 = (int)prediction->keypoint[j + 1];
    //       fb_gfx_fillRect(fb, x0, y0, 3, 3, color);
    //     }
    // #endif
  }
}

// ================================================================
// ===                   STREAM Handler                         ===
// ================================================================

// HTTP handler for streaming the camera feed
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->width > 159) {
        if (fb->format != PIXFORMAT_JPEG) {

          // #if TWO_STAGE
          //           HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
          //           HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
          //           std::list<dl::detect::result_t> &candidates = s1.infer((uint16_t *)fb->buf, { (int)fb->height, (int)fb->width, 3 });
          //           std::list<dl::detect::result_t> &results = s2.infer((uint16_t *)fb->buf, { (int)fb->height, (int)fb->width, 3 }, candidates);
          // #else
          HumanFaceDetectMSR01 s1(0.05F, 0.3F, 1, 0.5F);
          std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)fb->buf, { (int)fb->height, (int)fb->width, 3 });
          // #endif

          if (results.size() > 0) {
            fb_data_t rfb;
            rfb.width = fb->width;
            rfb.height = fb->height;
            rfb.data = fb->buf;
            rfb.bytes_per_pixel = 2;
            rfb.format = FB_RGB565;

            // Draw face boxes / target
            draw_face_boxes(&rfb, &results, 0);
          }
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);

          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    vTaskDelay(40 / portTICK_PERIOD_MS);
    //vTaskDelay(50);
  }
  return res;
}

// ================================================================
// ===                  COMMAND Handler                         ===
// ================================================================

// HTTP handler for processing robot movement commands
static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf;
  size_t buf_len;
  char variable[32] = {
    0,
  };

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int res = 0;
  int LED_Max = 50;
  int StepSize = 10;
  if (!strcmp(variable, "up")) {
    Serial.println("U\t" + String(StepSize));
  } else if (!strcmp(variable, "down")) {
    Serial.println("D\t" + String(StepSize));
  } else if (!strcmp(variable, "left")) {
    Serial.println("L\t" + String(StepSize));
  } else if (!strcmp(variable, "right")) {
    Serial.println("R\t" + String(StepSize));
  } else if (!strcmp(variable, "stop")) {
    Serial.println("stop");
  } else if (!strcmp(variable, "target")) {
    Serial.println("Target");
  } else if (!strcmp(variable, "shoot")) {
    Serial.println("Shoot");
  } else if (!strcmp(variable, "ledon")) {
    Serial.println("xON");
    analogWrite(LED_OUTPUT, LED_Max);
  } else if (!strcmp(variable, "ledoff")) {
    Serial.println("xOFF");
    analogWrite(LED_OUTPUT, 0);
  } else {
    // Serial.println("Stop");
    res = -1;
  }
  //Serial.println("\n");

  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// ================================================================
// ===                        WiFi                              ===
// ================================================================

void WifiSetup() {
  // Wi-Fi connection
  WiFi.mode(WIFI_AP);
  WiFi.softAP("HackPack");

  // Set up mDNS responder
  if (!MDNS.begin("hackpack")) Serial.println("Error setting up MDNS responder!");
  MDNS.addService("http", "tcp", 80);
}

void Camera_Setup() {
  // Configure the camera
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
  config.pixel_format = PIXFORMAT_RGB565;


  // Set frame size and quality
  config.frame_size = FRAMESIZE_QQVGA;  // FRAMESIZE_ + 96X96|QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA|
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 20;  //10-63 lower number means higher quality
  config.fb_count = 1;       // frame buffer count

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Get camera sensor settings
  sensor_t *s = esp_camera_sensor_get();
  // Set camera sensor parameters
  s->set_brightness(s, 0);                  // -2 to 2
  s->set_contrast(s, 0);                    // -2 to 2
  s->set_saturation(s, 0);                  // -2 to 2
  s->set_special_effect(s, 0);              // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);                    // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);                    // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);                     // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);               // 0 = disable , 1 = enable
  s->set_aec2(s, 1);                        // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);                    // -2 to 2
  s->set_aec_value(s, 300);                 // 0 to 1200
  s->set_gain_ctrl(s, 1);                   // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);                    // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 1);                         // 0 = disable , 1 = enable
  s->set_wpc(s, 1);                         // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);                     // 0 = disable , 1 = enable
  s->set_lenc(s, 1);                        // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);                     // 0 = disable , 1 = enable
  s->set_vflip(s, 0);                       // 0 = disable , 1 = enable
  s->set_dcw(s, 1);                         // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);                    // 0 = disable , 1 = enable
}
// ================================================================
// ===                         SETUP                            ===
// ================================================================

// Arduino setup function
void setup() {
  // Setup serial comms
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //disable brownout detector

  // Set up LED output pin
  pinMode(LED_OUTPUT, OUTPUT);



  //start Camera system
  Camera_Setup();

  // Start WIFI system
  WifiSetup();

  // Start streaming web server
  startCameraServer();

  Serial.println("HackPack Camera Ready");
}

// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================

void loop() {
  // Loop function here...
}
