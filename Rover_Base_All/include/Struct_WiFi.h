#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct Joystick {
  int x1;
  int y1;
  int btn1;
  int x2;
  int y2;
  int btn2;
  int interruptBtn;
} Joystick;

// Create a struct_message called myData
Joystick myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    vTaskDelay(50 / portTICK_PERIOD_MS);
}