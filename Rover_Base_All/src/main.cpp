#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "Struct_WiFi.h"

/*                          ___________________________
       BATTERY POWER --     3.3V                    GND     -- BATTERY GROUND
                            EN                      23
                            VP                      22      -- SCK
       ULS_ECHO_LEFT --     39                      TX
      ULS_ECHO_RIGHT --     34                      RX
      ULS_ECHO_FRONT --     35                      21      -- SDA
    ULS_TRIGGER_LEFT --     32                      GND
   ULS_TRIGGER_RIGHT --     33                      19      -- MOTOR A-1A
   ULS_TRIGGER_FRONT --     25                      18      -- MOTOR A-1B
                            26                      5       -- BUZZER
                            27                      17      -- MOTOR B-1B
                            14                      16      -- MOTOR B-1A
                            12                      4
                            GND                     0
                            13                      2       
                            D2                      15
                            D3                      D1
                            CMD        ____         D0
                            5V_________|  |_________CLK
        
*/

//  PIN configurations
#define ULS_ECHO_LEFT_PIN 39
#define ULS_ECHO_FRONT_PIN 35
#define ULS_ECHO_RIGHT_PIN 34
#define ULS_TRIGGER_LEFT_PIN 32
#define ULS_TRIGGER_FRONT_PIN 25
#define ULS_TRIGGER_RIGHT_PIN 33

#define motorA1 18
#define motorA2 19
#define motorB1 16
#define motorB2 17

#define BUZZER 5

// DRIVE States
#define DRIVE_FORWARD 2
#define DRIVE_BACKWARD 1
#define DRIVE_RIGHT 3
#define DRIVE_LEFT 4
#define DRIVE_STOP 0

//  Class constructers
Adafruit_PWMServoDriver pwm;

// Define the servo parameters
#define SERVO_OPENING_DEADZONE 280    // Deadzone start (312)
#define SERVOSTOP 350                 // Complete deadzone
#define SERVO_CLOSING_DEADZONE 440    // Deadzone start (410)
#define SERVO_FREQ 50                 // Analog servos run at ~50 Hz

//  Define different tasks handlers
TaskHandle_t ButtonTaskHandle = NULL;
TaskHandle_t ManualTaskHandle = NULL;
TaskHandle_t AutoTaskHandle = NULL;
TaskHandle_t RobotTaskHandle = NULL;
TaskHandle_t SpinningTaskHandle = NULL;

//  Used variables
int btnSettings = 0,  BTN_Mode = 0, pulseleng;
long BTN_ChangeMode_Time = 0, BTN_ChangeMode_Last = 0;

long ULS_Duration_LEFT = 0, ULS_Distance_LEFT = 0;
long ULS_Duration_FRONT = 0, ULS_Distance_FRONT = 0;
long ULS_Duration_RIGHT = 0, ULS_Distance_RIGHT = 0;

static TickType_t buzzerOffTime = 0;
static bool buzzerActive = false;

int motorPins[4] = {motorA1, motorA2, motorB1, motorB2};//  Motor definition
                                                        //  DRIVE MODES
int driveModes[5][4] = {{LOW,   LOW,    LOW,    LOW},   //  STOP
                        {LOW,   HIGH,   LOW,    HIGH},  //  BACKWARDS
                        {HIGH,  LOW,    HIGH,   LOW},   //  FORWARDS
                        {HIGH,  LOW,    LOW,    HIGH},  //  RIGHT
                        {LOW,   HIGH,   HIGH,   LOW},   //  LEFT
};

//  Creating the different structs with a Variable creator at the end
//  HBRO - Wheel controls
struct Hbro{
    //  PIN Setups
    void setupPins(){
        for (size_t i = 0; i < 4; i++)
        {
            pinMode(motorPins[i], OUTPUT);
        }
    }
    //  DRIVE function with a stopper at the end
    //  Goes through the different DRIVE MODES with a function variable
    void driveFunction(int driveMode){
        for (size_t i = 0; i < 4; i++)
        {
            digitalWrite(motorPins[i], driveModes[driveMode][i]);
        }
    }
};
Hbro alleHjul;

//  Range Sensor (ULS) setup only
struct RangeSensors{
    void ULSSetup(){
        pinMode(ULS_TRIGGER_LEFT_PIN, OUTPUT);
        pinMode(ULS_TRIGGER_RIGHT_PIN, OUTPUT);
        pinMode(ULS_TRIGGER_FRONT_PIN, OUTPUT);
        pinMode(ULS_ECHO_LEFT_PIN, INPUT);
        pinMode(ULS_ECHO_FRONT_PIN, INPUT);
        pinMode(ULS_ECHO_RIGHT_PIN, INPUT);
    }
};
RangeSensors range;

//  PWM Board for Servo motor control
struct PWMBoard{
    //  Controls the clockwise motion of the servo
    void servoClockwise(int chan, int speed, int data){
        pwm.setPWM(chan, 0, SERVO_CLOSING_DEADZONE - data);
        vTaskDelay(speed / portTICK_PERIOD_MS);
        servoStop(chan);
    }
    //  Controls the counter clockwise motion of the servo
    void servoCounterClockwise(int chan, int speed, int data){
        pwm.setPWM(chan, 0, SERVO_OPENING_DEADZONE - data);
        vTaskDelay(speed / portTICK_PERIOD_MS);
        servoStop(chan);
    }
    //  Stopper to control the servos better
    void servoStop(int chan){
        pwm.setPWM(chan, 0, SERVOSTOP);
    };
};
PWMBoard structPWM;

void updateBuzzer(){
    if (buzzerActive && xTaskGetTickCount() > buzzerOffTime)
    {
        digitalWrite(BUZZER, LOW);
        buzzerActive = false;
    }
}

//  Task creations
//  btnTask controls what mode we are currently in
//  0   -   Default, does nothing
//  1   -   Manual controls over the rover
//  2   -   Auto driving, with range measuring
//  3   -   Robot Arm, with manual controls
//
//  Disables the tasks that is not being used, and enables only the one needed task
void btnTask(void *paramter){
    for(;;){
        while(myData.interruptBtn == 1){
            vTaskSuspend(AutoTaskHandle);
            vTaskSuspend(RobotTaskHandle);
            vTaskSuspend(SpinningTaskHandle);
            vTaskResume(ManualTaskHandle);
        }
        while(myData.interruptBtn == 2){
            vTaskSuspend(ManualTaskHandle);
            vTaskSuspend(RobotTaskHandle);
            vTaskSuspend(SpinningTaskHandle);
            vTaskResume(AutoTaskHandle);
        }
        while(myData.interruptBtn == 3){
            vTaskSuspend(AutoTaskHandle);
            vTaskSuspend(ManualTaskHandle);
            vTaskSuspend(SpinningTaskHandle);
            vTaskResume(RobotTaskHandle);
        }
        while(myData.interruptBtn == 4){
            vTaskSuspend(AutoTaskHandle);
            vTaskSuspend(ManualTaskHandle);
            vTaskSuspend(RobotTaskHandle);
            vTaskResume(SpinningTaskHandle);

        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

//  Manual task
//  Controls the rover from the Joysticks
void ManualTask(void *parameter){
    while(myData.interruptBtn == 1){
        //  Checks the different positions from the joysticks and translates them to movement
        if (myData.x2 <= -75){
            alleHjul.driveFunction(DRIVE_RIGHT);
        }
        else if (myData.x2 >= 75){
            alleHjul.driveFunction(DRIVE_LEFT);
        }
        else if (myData.y2 <= -75){
            alleHjul.driveFunction(DRIVE_BACKWARD);
            if (!buzzerActive)
            {
                digitalWrite(BUZZER, HIGH);
                buzzerOffTime = xTaskGetTickCount() + pdMS_TO_TICKS(200);
                buzzerActive = true;
            }
        }
        else if (myData.y2 >= 75){
            alleHjul.driveFunction(DRIVE_FORWARD);
        }
        else{
            alleHjul.driveFunction(DRIVE_STOP);
        }
        updateBuzzer();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

//  Autopilot task
//  Self driving mode
void AutoTask(void *parameter){
    while(myData.interruptBtn == 2){
        digitalWrite(ULS_TRIGGER_LEFT_PIN, HIGH);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(ULS_TRIGGER_LEFT_PIN, LOW);
        ULS_Duration_LEFT = pulseInLong(ULS_ECHO_LEFT_PIN, HIGH);
        ULS_Distance_LEFT = (ULS_Duration_LEFT/2.0) / 29.1;
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(ULS_TRIGGER_RIGHT_PIN, HIGH);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(ULS_TRIGGER_RIGHT_PIN, LOW);
        ULS_Duration_RIGHT = pulseInLong(ULS_ECHO_RIGHT_PIN, HIGH);
        ULS_Distance_RIGHT = (ULS_Duration_RIGHT/2.0) / 29.1;
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(ULS_TRIGGER_FRONT_PIN, HIGH);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(ULS_TRIGGER_FRONT_PIN, LOW);
        ULS_Duration_FRONT = pulseInLong(ULS_ECHO_FRONT_PIN, HIGH);
        ULS_Distance_FRONT = (ULS_Duration_FRONT/2.0) / 29.1;

        if (ULS_Distance_FRONT > 30){
            if (ULS_Distance_LEFT > 55){
                alleHjul.driveFunction(DRIVE_LEFT);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                alleHjul.driveFunction(DRIVE_STOP);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (ULS_Distance_RIGHT > 55){
                alleHjul.driveFunction(DRIVE_RIGHT);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                alleHjul.driveFunction(DRIVE_STOP);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            alleHjul.driveFunction(DRIVE_FORWARD);
        }
        else {
            alleHjul.driveFunction(DRIVE_BACKWARD);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            alleHjul.driveFunction(DRIVE_STOP);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            alleHjul.driveFunction(DRIVE_RIGHT);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        vTaskDelay(75 / portTICK_PERIOD_MS);
    }
}

//  Robot task
//  Control the robot arm from the Joysticks
void RobotTask(void *parameter){
    while(myData.interruptBtn == 3){
        if (myData.x1 <= -75){
            structPWM.servoClockwise(0, 20, myData.x1);
        }
        if (myData.x1 >= 75){
            structPWM.servoCounterClockwise(0, 20, myData.x1);
        }
        if (myData.y1 <= -75){
            structPWM.servoClockwise(1, 50, myData.y1);
        }
        if (myData.y1 >= 75){
            structPWM.servoCounterClockwise(1, 50, myData.y1);
        }
        if (myData.x2 <= -75){
            structPWM.servoClockwise(2, 50, myData.x2);
            structPWM.servoClockwise(3, 50, myData.y2);
        }
        if (myData.x2 >= 75){
            structPWM.servoCounterClockwise(2, 50, myData.x2);
            structPWM.servoCounterClockwise(3, 50, myData.y2);
        }
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);
}

void SpinningTask(void *parameter){
    while(myData.interruptBtn == 4){
        alleHjul.driveFunction(DRIVE_RIGHT);
        vTaskDelay(75 / portTICK_PERIOD_MS);
    }
}

//  Default setup
void setup(){
    Wire.begin();

    alleHjul.setupPins();
    range.ULSSetup();

    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ); // Set PWM frequency to 50 Hz
    delay(50);

    pinMode(BUZZER, OUTPUT);

    // Create tasks
    xTaskCreatePinnedToCore(AutoTask, "AutoTask", 6000, NULL, 7, &AutoTaskHandle, 0);
    vTaskSuspend(AutoTaskHandle);
    xTaskCreatePinnedToCore(ManualTask, "ManualTask", 6000, NULL, 6, &ManualTaskHandle, 0);
    vTaskSuspend(ManualTaskHandle);
    xTaskCreatePinnedToCore(RobotTask, "RobotTask", 6000, NULL, 5, &RobotTaskHandle, 0);
    vTaskSuspend(RobotTaskHandle);
    xTaskCreatePinnedToCore(SpinningTask, "AutoSpin", 6000, NULL, 8, &SpinningTaskHandle, 0);
    vTaskSuspend(SpinningTaskHandle);
    xTaskCreatePinnedToCore(btnTask, "ButtonTask", 4096, NULL, 1, &ButtonTaskHandle, 1);
    delay(50);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    esp_now_init();
    if (esp_now_init() != ESP_OK) {
        return;
    }
    delay(50);
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

//  Default loop
void loop(){
}