/*
	Felicia's ScareCrow Monitor. 
	
	Arduino Source code for ESP32-CAM
	
	ESP32-CAM Connections:
		TPIO pin 2: Serial TX to ScareCrow Controller RX
		TPIO pin 3: Serial RX to ScareCrow Controller TX
		GND	: connect to ScareCrow Controller GND
		Vin	: for power distributor 5V
		GND	: for power distributor GND
	
	Note: While the web control interface is fully reconstructed, and the servo control part removed,
	this code still has conceptually and paritally reference from webserver example of:
	https://randomnerdtutorials.com/esp32-cam-pan-and-tilt-2-axis/
	
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems
#include "esp_http_server.h"
#include <ESP32Servo.h>
#include "IPAddress.h"

// Replace with your network credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

#define PART_BOUNDARY "123456789000000000000987654321"

#define CAMERA_MODEL_AI_THINKER
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

#define FLASH_GPIO_NUM    4
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t httpdCamera = NULL;
httpd_handle_t httpdStream = NULL;

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
<head>
  <title>Felicia's Patrolling-Robot</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-weight: bold; font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;}
    table { font-weight: bold; margin-left: auto; margin-right: auto; text-align: center;}
    td { padding: 8 px; }
    .button {
      background-color: #44682f;
      color: yellow;
      padding: 10px 20px;
      text-align: center;
    }
    img {  width: 480 ;
      max-width: 100% ;
      height: 320 ; 
    }
  </style>
</head>
<body>
  <h2>Felicia's Patroling-Robot</h2>
  <table border=1>
  <tr><td>Monitor</td><td>Camera Angle</td></tr>
  <tr><td><table><tr><td><img src="" id="photo" width="480" height="320"></td></tr></table></td><td>
    <table>
    <tr><td colspan="3"><button class="button" onmousedown="toggleCheckbox('TU');" ontouchstart="toggleCheckbox('TU');">&uarr;</button></td></tr>
    <tr><td><button class="button" onmousedown="toggleCheckbox('PL');" ontouchstart="toggleCheckbox('PL');">&larr;</button></td>
      <td><button class="button" onmousedown="toggleCheckbox('PR');" ontouchstart="toggleCheckbox('PR');">&rarr;</button></td></tr>
    <tr><td colspan="3"><button class="button" onmousedown="toggleCheckbox('TD');" ontouchstart="toggleCheckbox('TD');">&darr;</button></td></tr>
    </table>
  </td></tr>
  <tr><td>Drive Chain (dual motors)</td><td>Action</td></tr>
  <tr><td>
    <table>
    <tr><td><button class="button" onmousedown="toggleCheckbox('FS');" ontouchstart="toggleCheckbox('FS');">&uarr;&otimes;</button></td>
      <td><button class="button" onmousedown="toggleCheckbox('FF');" ontouchstart="toggleCheckbox('FF');">&uarr;&uarr;</butto</td>
      <td><button class="button" onmousedown="toggleCheckbox('SF');" ontouchstart="toggleCheckbox('SF');">&otimes;&uarr;</button></td></tr>
    <tr><td colspan="3"><button class="button" onmousedown="toggleCheckbox('SS');" ontouchstart="toggleCheckbox('SS');">&otimes;&otimes;</button></td></tr>
    <tr><td><button class="button" onmousedown="toggleCheckbox('RS');" ontouchstart="toggleCheckbox('RS');">&darr;&otimes;</button></td>
      <td><button class="button" onmousedown="toggleCheckbox('RR');" ontouchstart="toggleCheckbox('RR');">&darr;&darr;</butto</td>
      <td><button class="button" onmousedown="toggleCheckbox('SR');" ontouchstart="toggleCheckbox('SR');">&otimes;&darr;</button></td></tr>
    </table>
  </td><td>
  <table><tr><td><button class="button" onmousedown="toggleCheckbox('WV');" ontouchstart="toggleCheckbox('WV');">Flag</button></td></tr>
    <tr><td><button class="button" onmousedown="toggleCheckbox('HN');" ontouchstart="toggleCheckbox('HN');">Horn</button></td></tr>
    <tr><td><button class="button" onmousedown="toggleCheckbox('LG');" ontouchstart="toggleCheckbox('LG');">Flash</button></td></tr>
  </table>
  </td></tr>
  </table>
  <script>
  function toggleCheckbox(x) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/action?go=" + x, true);
    xhr.send();
  }
  window.onload = document.getElementById("photo").src = window.location.href.slice(0, -1) + ":81/stream";
  </script>
</body>
</html>
)rawliteral";

#define REUSE_PAN_THRESHOLD  100
#define REUSE_TILT_THRESHOLD 100
#define REUSE_DRIVE_THRESHOLD 100
#define REUSE_FLAG_THRESHOLD  6000
#define REUSE_HORN_THRESHOLD  1000
#define REUSE_FLASH_THRESHOLD  1000

unsigned long last_pan_used =0;
unsigned long last_tilt_used =0;
unsigned long last_drive_used =0;
unsigned long last_flag_used =0;
unsigned long last_horn_used =0;
unsigned long last_flash_used =0;

static esp_err_t handlerIndex(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t handlerStream(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

static esp_err_t handlerCommand(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
  
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if(!buf){
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

  sensor_t * s = esp_camera_sensor_get();
  //flip the camera vertically
  //s->set_vflip(s, 1);          // 0 = disable , 1 = enable
  // mirror effect
  //s->set_hmirror(s, 1);          // 0 = disable , 1 = enable
  int res = 0;
  if(!strcmp(variable, "TU") || !strcmp(variable, "TD")) {
    if(millis() - last_tilt_used > REUSE_TILT_THRESHOLD) {
      Serial.println(variable);
      last_tilt_used = millis();
    } else { 
      res=-1;
    }
  } else if(!strcmp(variable, "PL") || !strcmp(variable, "PR")) {
    if(millis() - last_pan_used > REUSE_PAN_THRESHOLD) {
      Serial.println(variable);
      last_pan_used = millis();
    } else { 
      res=-1;
    }
  } else if(!strcmp(variable, "FS") || !strcmp(variable, "FF") || !strcmp(variable, "SF") || !strcmp(variable, "SS")
            || !strcmp(variable, "RS") || !strcmp(variable, "RR") || !strcmp(variable, "SR")) {
    if(millis() - last_drive_used > REUSE_DRIVE_THRESHOLD) {
      Serial.println(variable);  
      last_drive_used = millis();
    } else { 
      res=-1;
    }
  } else if(!strcmp(variable, "WV")) {
    if(millis() - last_flag_used > REUSE_FLAG_THRESHOLD) {
      Serial.println(variable);  
      last_flag_used = millis();
    } else { 
      res=-1;
    }
  } else if(!strcmp(variable, "HN")) {
    if(millis() - last_horn_used > REUSE_HORN_THRESHOLD) {
      Serial.println(variable);  
      last_horn_used = millis();
    } else { 
      res=-1;
    }
  } else if(!strcmp(variable, "LG")) {
    doFlash();
  } else {
    res = -1;
  }
  
  if(res){
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  httpd_uri_t uriIndex = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = handlerIndex,
    .user_ctx  = NULL
  };
  httpd_uri_t uriCommand = {
    .uri       = "/action",
    .method    = HTTP_GET,
    .handler   = handlerCommand,
    .user_ctx  = NULL
  };
  httpd_uri_t uriStream = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = handlerStream,
    .user_ctx  = NULL
  };
  if (httpd_start(&httpdCamera, &config) == ESP_OK) {
    httpd_register_uri_handler(httpdCamera, &uriIndex);
    httpd_register_uri_handler(httpdCamera, &uriCommand);
  }
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&httpdStream, &config) == ESP_OK) {
    httpd_register_uri_handler(httpdStream, &uriStream);
  }
}

void doFlash() {
  digitalWrite(FLASH_GPIO_NUM, HIGH);
  delay(2000);
  digitalWrite(FLASH_GPIO_NUM, LOW);
  delay(2000);
}

void setupSerials() {
  Serial.begin(9600);
  Serial.setDebugOutput(false);
}

void setupCamera() {
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
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  } 
}

void setupWiFi() {
  // Wi-Fi connection
  IPAddress ip(192,168,1,74);
  IPAddress gateway(192,168,1,1);
  IPAddress subnet(255,255,255,0);
  WiFi.config(ip, gateway,subnet);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());
}


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  //setupServos();
  pinMode(FLASH_GPIO_NUM,OUTPUT);
  setupSerials();
  setupCamera();
  setupWiFi();
  // Start streaming web server
  startCameraServer();
}

void loop() {
  
}