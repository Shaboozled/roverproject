#include "freertos/FreeRTOS.h"
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
//#include <Wire.h>
//#include "Joystick.h"
//#include "PWMBoard.h"
//#include "Buttons.h"
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

//PWMBoard structPWM;
//ButtonSwitch btns;
//Joystick joystick;

esp_now_peer_info_t peerInfo;
int BTN_ChangeMode = 0;
long BTN_ChangeMode_Time, BTN_ChangeMode_Last;

void changeMode()
{
    Serial.println("Button pressed");
    BTN_ChangeMode_Time = millis();
    if (BTN_ChangeMode_Time - BTN_ChangeMode_Last > 500)
    {
    if (BTN_ChangeMode == 0)
        BTN_ChangeMode = 1;
    else if (BTN_ChangeMode == 1)
        BTN_ChangeMode = 2;
    else if (BTN_ChangeMode == 2)
        BTN_ChangeMode = 3;
    else if (BTN_ChangeMode == 3)
        BTN_ChangeMode = 1;
    }
    BTN_ChangeMode_Last = BTN_ChangeMode_Time;
}

void setup() {
    Serial.begin(9600);
    
    Serial.println("Starting");
    delay(2500);
    pinMode(JOYSTICK1_BTN_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK2_BTN_PIN, INPUT_PULLUP);
    
    pinMode(25, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(25), changeMode, FALLING);

    WiFi.mode(WIFI_STA);
    esp_now_init();

    //btns.btnSetup();
    //structPWM.PWM_Setup();
    //joystick.JoyStick_Setup();
    
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
    myData.x2 = map(analogRead(JOYSTICK2_X_PIN), 0, 4096, 0, 256);
    myData.y2 = map(analogRead(JOYSTICK2_Y_PIN), 0, 4096, 0, 256);
    myData.btn2 = digitalRead(JOYSTICK2_BTN_PIN) == 0;
    myData.interruptBtn = BTN_ChangeMode;

    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    delay(100);
    /*
    //  Joystick movement -- right and left only
    if (analog.x1 <= -100){ structPWM.servoClockwise(0);}
    else if (analog.x1 >= 100){ structPWM.servoCounterClockwise(0);}
    else if (analog.x2 <= -100){ structPWM.servoClockwise(1);}
    else if (analog.x2 >= 100){ structPWM.servoCounterClockwise(1);}
    
    //  Joystick buttons
    if (analog.pressed1){ structPWM.servoClockwise(2);}
    else if (analog.pressed2){ structPWM.servoCounterClockwise(2);}
    */
}