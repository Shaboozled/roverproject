#include <Arduino.h>
#include <Wire.h>
#include "Joystick.h"
#include "PWMBoard.h"
#include "Buttons.h"

Joystick analog;
PWMBoard structPWM;
ButtonSwitch btns;

void setup() {
  Serial.begin(9600);
  btns.btnSetup();
  structPWM.PWM_Setup();
  analog.JoyStick_Setup();
}

void loop() {
  pulseleng = SERVOSTOP;
  pwm.setPWM(0,0,SERVOSTOP);
  pwm.setPWM(1,0,SERVOSTOP);
  pwm.setPWM(2,0,SERVOSTOP);

  //  Joystick readers
  analog.readAnalogPositions1();
  analog.readAnalogPositions2();
  
  //  Joystick movement -- right and left only
  if (analog.x1 <= -100){ structPWM.servoClockwise(0);}
  else if (analog.x1 >= 100){ structPWM.servoCounterClockwise(0);}
  else if (analog.x2 <= -100){ structPWM.servoClockwise(1);}
  else if (analog.x2 >= 100){ structPWM.servoCounterClockwise(1);}
  
  //  Joystick buttons
  if (analog.pressed1){ structPWM.servoClockwise(2);}
  else if (analog.pressed2){ structPWM.servoCounterClockwise(2);}
}