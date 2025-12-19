#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "Struct_WiFi.h"

#define JOYSTICK1_BTN_PIN 32
#define JOYSTICK1_X_PIN 36
#define JOYSTICK1_Y_PIN 39
#define JOYSTICK2_BTN_PIN 33
#define JOYSTICK2_X_PIN 34
#define JOYSTICK2_Y_PIN 35

#define BTN_CHANGE_1_PIN 19
#define BTN_CHANGE_2_PIN 18
#define BTN_CHANGE_3_PIN 17

// Define Analog Joystick correction data
#define ANALOG_X_CORRECTION 128
#define ANALOG_Y_CORRECTION 128

esp_now_peer_info_t peerInfo;
long BTN_ChangeMode_Time, BTN_ChangeMode_Last;
TaskHandle_t dataChangeHandle = NULL;

//  Button changer for mode selection.
//  There is a delay function to prevent the button being pressed too fast
//  or giving out a double tap
void changeMode1()
{
    BTN_ChangeMode_Time = millis();
    if (BTN_ChangeMode_Time - BTN_ChangeMode_Last > 500)
    {
        myData.interruptBtn = 1;
    }
    BTN_ChangeMode_Last = BTN_ChangeMode_Time;
}
void changeMode2()
{
    BTN_ChangeMode_Time = millis();
    if (BTN_ChangeMode_Time - BTN_ChangeMode_Last > 500)
    {
        myData.interruptBtn = 2;
    }
    BTN_ChangeMode_Last = BTN_ChangeMode_Time;
}
void changeMode3()
{
    BTN_ChangeMode_Time = millis();
    if (BTN_ChangeMode_Time - BTN_ChangeMode_Last > 500)
    {
        myData.interruptBtn = 3;
    }
    BTN_ChangeMode_Last = BTN_ChangeMode_Time;
}

void dataChange(void *parameter){
    for(;;){
        //  Move joystick inputs to myData connection
        myData.x1 = map(analogRead(JOYSTICK1_X_PIN), 0, 4096, 0, 256) - ANALOG_X_CORRECTION;
        myData.y1 = map(analogRead(JOYSTICK1_Y_PIN), 0, 4096, 0, 256) - ANALOG_Y_CORRECTION;
        myData.btn1 = digitalRead(JOYSTICK1_BTN_PIN) == 0;
        myData.x2 = map(analogRead(JOYSTICK2_X_PIN), 0, 4096, 0, 256) - ANALOG_X_CORRECTION;
        myData.y2 = map(analogRead(JOYSTICK2_Y_PIN), 0, 4096, 0, 256) - ANALOG_Y_CORRECTION;
        myData.btn2 = digitalRead(JOYSTICK2_BTN_PIN) == 0;
        
        //  Send the data to the other ESP-32
        esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup() {
    pinMode(JOYSTICK1_BTN_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK2_BTN_PIN, INPUT_PULLUP);
    
    pinMode(BTN_CHANGE_1_PIN, INPUT_PULLUP);
    pinMode(BTN_CHANGE_2_PIN, INPUT_PULLUP);
    pinMode(BTN_CHANGE_3_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_CHANGE_1_PIN), changeMode1, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_CHANGE_2_PIN), changeMode2, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_CHANGE_3_PIN), changeMode3, FALLING);

    WiFi.mode(WIFI_STA);
    esp_now_init();
    
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        return;
    }

    xTaskCreatePinnedToCore(dataChange, "dataChange", 4000, NULL, 1, &dataChangeHandle, 1);
}

void loop() {
}