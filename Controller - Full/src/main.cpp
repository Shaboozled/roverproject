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

// Define Analog Joystick correction data
#define ANALOG_X_CORRECTION 128
#define ANALOG_Y_CORRECTION 128

esp_now_peer_info_t peerInfo;
long BTN_ChangeMode_Time, BTN_ChangeMode_Last;

//  Button changer for mode selection.
//  There is a delay function to prevent the button being pressed too fast
//  or giving out a double tap
void changeMode()
{
    BTN_ChangeMode_Time = millis();
    if (BTN_ChangeMode_Time - BTN_ChangeMode_Last > 500)
    {
    if (myData.interruptBtn == 0)
        myData.interruptBtn = 1;
    else if (myData.interruptBtn == 1)
        myData.interruptBtn = 2;
    else if (myData.interruptBtn == 2)
        myData.interruptBtn = 3;
    else if (myData.interruptBtn == 3)
        myData.interruptBtn = 1;
    }
    BTN_ChangeMode_Last = BTN_ChangeMode_Time;
}

void setup() {
    Serial.begin(115200);
    
    Serial.println("Starting");
    delay(2500);
    pinMode(JOYSTICK1_BTN_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK2_BTN_PIN, INPUT_PULLUP);
    
    pinMode(25, INPUT_PULLUP);
    pinMode(26, INPUT_PULLUP);
    pinMode(27, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(25), changeMode, FALLING);
    attachInterrupt(digitalPinToInterrupt(26), changeMode, FALLING);
    attachInterrupt(digitalPinToInterrupt(27), changeMode, FALLING);

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
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    //  Move joystick inputs to myData connection
    myData.x2 = map(analogRead(JOYSTICK2_X_PIN), 0, 4096, 0, 256);
    myData.y2 = map(analogRead(JOYSTICK2_Y_PIN), 0, 4096, 0, 256);
    myData.btn2 = digitalRead(JOYSTICK2_BTN_PIN) == 0;

    //  Send the data to the other ESP-32
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    delay(10);
}