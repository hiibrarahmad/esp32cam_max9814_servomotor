#include "esp_camera.h"
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ezButton.h>
#include <Servo.h>
#include "driver/adc.h"

#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"
ezButton button(14);
const int servoPin = 15; // Pin for the servo

// WiFi and Websockets configuration
const char* ssid = "ibrar";
const char* password = "ibrarahmad";
const char* websockets_server_host = "192.168.4.1";
const uint16_t websockets_server_port = 8888;
using namespace websockets;
WebsocketsClient client;

Servo servo;
bool rotateServo = false;       // Flag to indicate servo rotation
unsigned long rotationStartTime; // Time when rotation started
bool buttonPressed = false;
bool buttonState = false;
bool lastButtonState = false;

// ADC configuration
#define ADC_PIN 33

TaskHandle_t adcHandler = NULL;

// Function prototypes
void start_to_connect();
void adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len);
static void adc_task(void* arg);
void onEventsCallback(WebsocketsEvent event, String data);

void setup() {
  Serial.begin(115200);
  button.setDebounceTime(50); // set debounce time to 50 milliseconds
  Serial.setDebugOutput(true);
  Serial.println();
  servo.attach(servoPin);

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
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA; // 320x240
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }


  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // WiFi connection...
  start_to_connect();

  // Socket connection...

  // Websockets client message callback...
  client.onMessage([](WebsocketsMessage msg) {
    Serial.println("Got Message: " + msg.data());

    // Check if the message is "Button pressed!"
    if (msg.data() == "Button pressed!") {
      Serial.println("Rotating servo...");
      rotateServo = true;
      rotationStartTime = millis(); // Record the start time of rotation
    }
  });

  // Websockets client event callback...
  client.onEvent(onEventsCallback);

  // Task creation for sending audio from ADC mic
  xTaskCreatePinnedToCore(adc_task, "adc_task", 4096, NULL, 1, &adcHandler, 1);
}

void loop() {
  button.loop();
  buttonState = button.isPressed();

  // Check if the button state has changed
  if (buttonState != lastButtonState) {
    if (buttonState) {
      buttonPressed = true;
    }
    lastButtonState = buttonState;
  }

  if (buttonPressed) {
    captureAndSendImage();
    buttonPressed = false; // Reset the button state
  }

  if (rotateServo) {
    // Check if 10 seconds have passed since rotation started
    if (millis() - rotationStartTime >= 10000) {
      Serial.println("Returning servo to initial position...");
      servo.write(0); // Set the servo to the initial position
      rotateServo = false; // Reset the flag
    } else {
      // Rotate the servo to the desired position
      servo.write(90);
    }
  }

  client.poll();

  delay(100);
}

void captureAndSendImage() {
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Send the image over the websocket
  client.sendBinary((const char*)fb->buf, fb->len);
  

  esp_camera_fb_return(fb);
}

void start_to_connect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("Socket Connecting");

  while (!client.connect(websockets_server_host, websockets_server_port, "/")) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Socket Connected!");
}

void adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len) {
  uint32_t j = 0;
  for (int i = 0; i < len; i++) {
    d_buff[j++] = 0;
    d_buff[j++] = s_buff[i];
  }
}

static void adc_task(void* arg) {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_0);

  int adc_read_len = 4096;
  uint8_t* adc_read_buff = (uint8_t*)calloc(adc_read_len, sizeof(char));
  uint8_t* scaled_buff = (uint8_t*)calloc(adc_read_len * 2, sizeof(char));

  while (1) {
    for (int i = 0; i < adc_read_len; i++) {
      adc_read_buff[i] = adc1_get_raw(ADC1_CHANNEL_5) >> 4;
    }
    adc_data_scale(scaled_buff, adc_read_buff, adc_read_len);
    client.sendBinary((const char*)scaled_buff, adc_read_len * 2);
    Serial.print(scaled_buff);
    serial.print(adc_read_len*2);
    ets_printf("Never Used Stack Size: %u\n", uxTaskGetStackHighWaterMark(NULL));
  }

  free(adc_read_buff);
  adc_read_buff = NULL;
  free(scaled_buff);
  scaled_buff = NULL;
}

void onEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connnection Opened");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("Connnection Closed");
    ESP.restart();
  }
}
