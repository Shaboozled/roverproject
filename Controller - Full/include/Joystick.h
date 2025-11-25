#include <Arduino.h>

// Define Pins
#define JOYSTICK1_BTN_PIN 4
#define JOYSTICK1_X_PIN 16
#define JOYSTICK1_Y_PIN 17
#define JOYSTICK2_BTN_PIN 25
#define JOYSTICK2_X_PIN 26
#define JOYSTICK2_Y_PIN 27

// Define Analog Joystick correction data
#define ANALOG_X_CORRECTION 128
#define ANALOG_Y_CORRECTION 128

struct Joystick
{
  // Variables for positioning
  short x1, y1, x2, y2;
  byte pressed1 = 0, pressed2 = 0;
  
  void JoyStick_Setup(){
    pinMode(JOYSTICK1_BTN_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK2_BTN_PIN, INPUT_PULLUP);
    analogSetAttenuation(ADC_11db);
  }

  byte readAnalogAxisLevel(int pin){
    return map(analogRead(pin), 0, 4096, 0, 256);
  }

  bool isAnalogButtonPressed(int pin){
    return digitalRead(pin) == 0;
  }

  void readAnalogPositions1(){
    x1 = readAnalogAxisLevel(JOYSTICK1_X_PIN) - ANALOG_X_CORRECTION;
    y1 = readAnalogAxisLevel(JOYSTICK1_Y_PIN) - ANALOG_Y_CORRECTION;
    pressed1 = isAnalogButtonPressed(JOYSTICK1_BTN_PIN);
  }

  void readAnalogPositions2(){
    x2 = readAnalogAxisLevel(JOYSTICK2_X_PIN) - ANALOG_X_CORRECTION;
    y2 = readAnalogAxisLevel(JOYSTICK2_Y_PIN) - ANALOG_Y_CORRECTION;
    pressed2 = isAnalogButtonPressed(JOYSTICK2_BTN_PIN);
  }
};