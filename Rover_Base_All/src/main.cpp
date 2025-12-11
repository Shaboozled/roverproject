#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_VL53L0X.h>
#include <U8g2lib.h>

#include "Struct_WiFi.h"

/*                          ___________________________
       BATTERY POWER --     3.3V                    GND     -- BATTERY GROUND
                            EN                      23
         BATTERY PIN --     VP                      22      -- SCK
                            VN                      TX
                            34                      RX
            ULS ECHO --     35                      21      -- SDA
         ULS TRIGGER --     32                      GND
            MOTOR A1 --     33                      19
            MOTOR A2 --     25                      18      -- MOTOR C2
                            26                      5       -- MOTOR C1
            MOTOR B1 --     27                      17      -- MOTOR D2
            MOTOR B2 --     14                      16      -- MOTOR D1
                            12                      4
                            GND                     0
                            13                      2       -- X-SHUT TOF1
                            D2                      15
                            D3                      D1
                            CMD        ____         D0
                            5V_________|  |_________CLK
        
*/

//  PIN configurations
#define trigPin 32
#define echoPin 35
#define xShut_Pin 2
#define batteryLevelPin 36

#define motorA1 18
#define motorA2 19
#define motorB1 16
#define motorB2 17

//  Class constructers
Adafruit_PWMServoDriver pwm;
U8G2_SH1106_128X32_VISIONOX_F_HW_I2C Display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Adafruit_VL53L0X ToFSensor1;
Adafruit_VL53L0X ToFSensor2;
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

// Define the servo parameters
#define SERVO_OPENING_DEADZONE 297    // Deadzone start (312)
#define SERVOSTOP 350                 // Complete deadzone
#define SERVO_CLOSING_DEADZONE 435    // Deadzone start (410)
#define SERVO_FREQ 50                 // Analog servos run at ~50 Hz

//  Define different tasks handlers
TaskHandle_t ButtonTaskHandle = NULL;
TaskHandle_t ManualTaskHandle = NULL;
TaskHandle_t AutoTaskHandle = NULL;
TaskHandle_t RobotTaskHandle = NULL;

//  Used variables
int btnSettings = 0,  BTN_Mode = 0, pulseleng, rangeMeasure1 = 0, rangeMeasure2 = 0;
long BTN_ChangeMode_Time = 0, BTN_ChangeMode_Last = 0, duration = 0, distance = 0;
char oledInput[11];
int motorPins[4] = {motorA1, motorA2, motorB1, motorB2};//  Motor definition
                                                        //  DRIVE MODES
int driveModes[5][4] = {{LOW,   LOW,    LOW,    LOW},   //  STOP
                        {HIGH,  LOW,    HIGH,   LOW},   //  FREMAD
                        {LOW,   HIGH,   LOW,    HIGH},  //  BAGUD
                        {HIGH,  LOW,    LOW,    HIGH},  //  VENSTRE
                        {LOW,   HIGH,   HIGH,   LOW},   //  HØJRE
};

//  Creating the different structs with a Variable creator at the end
struct Hbro{
    void setupPins(){
        for (size_t i = 0; i < 4; i++)
        {
            pinMode(motorPins[i], OUTPUT);
        }
    }
    void driveFunction(int driveMode){
        for (size_t i = 0; i < 4; i++)
        {
            digitalWrite(motorPins[i], driveModes[driveMode][i]);
        }
    }
};
Hbro alleHjul;

struct OLED{
    void setup(){
        Display1.begin();
    }
    void OLEDWrite(char* Text){
        Display1.clearBuffer();                 // clear the internal memory
        Display1.setFont(u8g2_font_4x6_mf);     // choose a suitable font
        Display1.drawStr(2,5, Text);            // write something to the internal memory
        Display1.sendBuffer();                  // transfer internal memory to the display
        delay(1000);
    }
};
OLED display;

struct Battery{
  void SetupBattery(){
      pinMode(batteryLevelPin, INPUT);
  }
  char* readBatteryLevel(){
      int analogInput = analogRead(batteryLevelPin);
      float rawVolts = analogInput * 3.3/4096;

      dtostrf(rawVolts, 10, 8, oledInput);

      return oledInput;
  }
};
Battery batteri;

struct PWMBoard{
    void servoClockwise(int chan){
        pwm.setPWM(chan, 0, SERVO_CLOSING_DEADZONE);
        delay(150);
        pwm.setPWM(chan, 0, SERVOSTOP);
    }
    void servoCounterClockwise(int chan){
        pwm.setPWM(chan, 0, SERVO_OPENING_DEADZONE);
        delay(150);
        pwm.setPWM(chan, 0, SERVOSTOP);
    }
};
PWMBoard structPWM;

//  Task creations
//  btnTask controls what mode we are currently in
//  0   -   Default, does nothing
//  1   -   Manual controls over the rover
//  2   -   Auto driving, with range measuring
//  3   -   Robot Arm, with manual controls
void btnTask(void *paramter){
    for(;;){
        while(myData.interruptBtn == 1){
            vTaskSuspend(RobotTaskHandle);
            vTaskResume(ManualTaskHandle);
        }
        while(myData.interruptBtn == 2){
            vTaskSuspend(ManualTaskHandle);
            vTaskResume(AutoTaskHandle);
        }
        while(myData.interruptBtn == 3){
            vTaskSuspend(AutoTaskHandle);
            vTaskResume(RobotTaskHandle);
        }
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

//  Manual task
//  Controls the rover from the Joysticks
void ManualTask(void *parameter){
    // Task is activated, and the rover is now controlled manually
    // Manual motor control goes here!!!
    while(myData.interruptBtn == 1){
        if (myData.x2 >= 200){alleHjul.driveFunction(1);}           //  FREMAD
        else if (myData.x2 <= 50){alleHjul.driveFunction(2);}       //  BAGUD
        else if (myData.y2 <= 50){alleHjul.driveFunction(3);}       //  VENSTRE
        else if (myData.y2 >= 200){alleHjul.driveFunction(4);}      //  HØJRE
        else{alleHjul.driveFunction(0);}                            //  STOP
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

//  Autopilot task
//  Self driving mode
void AutoTask(void *parameter){
    while(myData.interruptBtn == 2){
        //  Send pulse
        digitalWrite(trigPin, LOW);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        digitalWrite(trigPin, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(trigPin, LOW);

        // Read echo
        duration = pulseInLong(echoPin, HIGH);
        distance = (duration/2.0) / 29.1;
/*
        // Read ToFs
        rangeMeasure1 = ToFSensor1.rangingTest(&measure1, false);
        rangeMeasure2 = ToFSensor2.rangingTest(&measure2, false);
*/
        if (distance > 25){
            alleHjul.driveFunction(1);      //  FREMAD
            Serial.println("No obstacles, moving forward");
        }
        else if (distance > 0 && distance <= 25){
            alleHjul.driveFunction(2);      //  BAGUD
            Serial.println("Too close, reversing!");
            delay(50);
            alleHjul.driveFunction(0);      //  STOP MOTOR
        }
        else{
            alleHjul.driveFunction(0);      //  STOP MOTOR
        }
        if (rangeMeasure2 < 50 && rangeMeasure1 > 100){
            alleHjul.driveFunction(3);      // VENSTRE
        }
        else if (rangeMeasure1 < 50 && rangeMeasure2 > 100){
            alleHjul.driveFunction(4);      // HØJRE
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

//  Robot task
//  Control the robot arm from the Joysticks
void RobotTask(void *parameter){
    while(myData.interruptBtn == 3){
        if (myData.x1 <= 50){structPWM.servoClockwise(2);}
        if (myData.x1 >= 200){structPWM.servoCounterClockwise(2);}
        if (myData.y1 <= 50){structPWM.servoClockwise(3);}
        if (myData.y1 >= 200){structPWM.servoCounterClockwise(3);}
        if (myData.x2 <= 50){structPWM.servoClockwise(0);}
        if (myData.x2 >= 200){structPWM.servoCounterClockwise(0);}
        if (myData.y2 <= 50){structPWM.servoClockwise(1);}
        if (myData.y2 >= 200){structPWM.servoCounterClockwise(1);}
        vTaskDelay(400 / portTICK_PERIOD_MS);
    }
}

//  Default setup
void setup(){
    Serial.begin(115200);
    Serial.println("Starting FreeRTOS Rover");
    delay(500);
    
    Serial.println("PWM Setup\n");
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ); // Set PWM frequency to 50 Hz
    delay(500);

    Serial.println("Pins Setup\n");
    alleHjul.setupPins();
    /*
    display.setup();
    batteri.SetupBattery();
    */
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    /*pinMode(xShut_Pin, OUTPUT);
    
    Wire.begin();
    digitalWrite(xShut_Pin, LOW);
    delay(100);
    digitalWrite(xShut_Pin, HIGH);
    delay(200);
    ToFSensor1.begin();
    ToFSensor1.setAddress(0x2A);
    delay(100);
    pinMode(xShut_Pin, INPUT);
    delay(500);
    */

    Serial.println("Creating Tasks\n");

    // Create tasks
    Serial.println("Task 1\n");
    xTaskCreatePinnedToCore(AutoTask, "AutoTask", 10000, NULL, 7, &AutoTaskHandle, 0);
    vTaskSuspend(AutoTaskHandle);
    Serial.println("Task 2\n");
    xTaskCreatePinnedToCore(ManualTask, "ManualTask", 10000, NULL, 6, &ManualTaskHandle, 0);
    vTaskSuspend(ManualTaskHandle);
    Serial.println("Task 3\n");
    xTaskCreatePinnedToCore(RobotTask, "RobotTask", 10000, NULL, 5, &RobotTaskHandle, 0);
    vTaskSuspend(RobotTaskHandle);
    xTaskCreatePinnedToCore(btnTask, "ButtonTask", 4096, NULL, 1, &ButtonTaskHandle, 1);
    delay(500);

    Serial.println("WiFi Setup\n");
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    esp_now_init();
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    delay(500);
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    Serial.println("Setup done!\n\n");
}

//  Default loop
void loop(){
    Serial.println(myData.interruptBtn);
    delay(50);
    //  display.OLEDWrite(batteryMeasure.readBatteryLevel());
}