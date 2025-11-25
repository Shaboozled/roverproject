#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// Pre-definition of variables used in structs
Adafruit_PWMServoDriver pwm;
int pulseleng;

// Define the servo parameters
#define SERVO_OPENING_DEADZONE 297    // Deadzone start (312)
#define SERVOSTOP 350                 // Complete deadzone
#define SERVO_CLOSING_DEADZONE 425    // Deadzone start (410)
#define SERVO_FREQ 50                 // Analog servos run at ~50 Hz

struct PWMBoard
{

  void PWM_Setup(){
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ); // Set PWM frequency to 50 Hz
  }

  void servoClockwise(int chan){
    pulseleng = SERVO_CLOSING_DEADZONE;
    pwm.setPWM(chan, 0, pulseleng);
    pulseleng++;
    delay(10);
  }
  void servoCounterClockwise(int chan){
    pulseleng = SERVO_OPENING_DEADZONE;
    pwm.setPWM(chan, 0, pulseleng);
    pulseleng--;
    delay(10);
  }
};