#include <Arduino.h>

// Define Pins
#define BTN_MANUEL_PIN 5
#define BTN_AUTO_PIN 18
#define BTN_ROBOARM_PIN 19

struct ButtonSwitch
{
    void btnSetup(){
        pinMode(BTN_MANUEL_PIN, INPUT_PULLUP);
        pinMode(BTN_AUTO_PIN, INPUT_PULLUP);
        pinMode(BTN_ROBOARM_PIN, INPUT_PULLUP);
    }
};