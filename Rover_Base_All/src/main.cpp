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
                            EN                      23      -- X-SHUT TOF1
         BATTERY PIN --     VP                      22      -- SCK
                            VN                      TX
                            34                      RX
           ULS2 ECHO --     35                      21      -- SDA
           ULS1 ECHO --     32                      GND
        ULS1 TRIGGER --     33                      19      -- MOTOR A1
        ULS2 TRIGGER --     25                      18      -- MOTOR A2
                            26                      5       
                            27                      17      -- MOTOR B2
                            14                      16      -- MOTOR B1
                            12                      4
                            GND                     0
                            13                      2       
                            D2                      15
                            D3                      D1
                            CMD        ____         D0
                            5V_________|  |_________CLK
        
*/

//  PIN configurations
#define trigPin 33
#define echoPin 32
#define xShut_Pin1 25
#define xShut_Pin2 5
#define batteryLevelPin 36

#define motorA1 18
#define motorA2 19
#define motorB1 16
#define motorB2 17

#define DRIVE_FORWARD 3
#define DRIVE_BACKWARD 4
#define DRIVE_LEFT 2
#define DRIVE_RIGHT 1
#define DRIVE_STOP 0

//  Class constructers
Adafruit_PWMServoDriver pwm;
U8G2_SH1106_128X32_VISIONOX_F_HW_I2C Display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Adafruit_VL53L0X ToFSensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X ToFSensor2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

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

//  Used variables
int btnSettings = 0,  BTN_Mode = 0, pulseleng;
long BTN_ChangeMode_Time = 0, BTN_ChangeMode_Last = 0;

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
        vTaskDelay(70 / portTICK_PERIOD_MS);
        for (size_t i = 0; i < 4; i++)
        {
            digitalWrite(motorPins[i], driveModes[0][i]);
        }
        vTaskDelay(30 / portTICK_PERIOD_MS);
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

struct RangeSensors{
    int rangeMeasure1 = 0, rangeMeasure2 = 0;
    long duration = 0, distance = 0, range1MM = 0, range2MM = 0;

    void setID() {
        // all reset
        digitalWrite(xShut_Pin1, LOW);    
        digitalWrite(xShut_Pin2, LOW);
        delay(10);

        // activating LOX1 and resetting LOX2
        digitalWrite(xShut_Pin1, HIGH);
        digitalWrite(xShut_Pin2, LOW);

        // initing LOX1
        while(!ToFSensor1.begin(0x30)){}
        delay(50);

        // activating LOX2
        digitalWrite(xShut_Pin2, HIGH);
        delay(10);

        //initing LOX2
        while (!ToFSensor2.begin(0x31)){}
        
    }
    void readULSDistances(){
        //  Send pulse
        digitalWrite(trigPin, HIGH);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        digitalWrite(trigPin, LOW);

        // Read echo
        duration = pulseInLong(echoPin, HIGH);
        distance = (duration/2.0) / 29.1;
    }
    void readToFDistances(){
        // Read ToFs
        ToFSensor1.rangingTest(&measure1, false);
        ToFSensor2.rangingTest(&measure2, false);

        range1MM = measure1.RangeMilliMeter;
        range2MM = measure2.RangeMilliMeter;
    }
};
RangeSensors range;

struct PWMBoard{
    void servoClockwise(int chan, int speed, int data){
        pwm.setPWM(chan, 0, SERVO_CLOSING_DEADZONE - data);
        vTaskDelay(speed / portTICK_PERIOD_MS);
        servoStop(chan);
    }
    void servoCounterClockwise(int chan, int speed, int data){
        pwm.setPWM(chan, 0, SERVO_OPENING_DEADZONE - data);
        vTaskDelay(speed / portTICK_PERIOD_MS);
        servoStop(chan);
    }
    void servoStop(int chan){
        pwm.setPWM(chan, 0, SERVOSTOP);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    };
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
            vTaskSuspend(AutoTaskHandle);
            vTaskSuspend(RobotTaskHandle);
            vTaskResume(ManualTaskHandle);
        }
        while(myData.interruptBtn == 2){
            vTaskSuspend(ManualTaskHandle);
            vTaskSuspend(RobotTaskHandle);
            vTaskResume(AutoTaskHandle);
        }
        while(myData.interruptBtn == 3){
            vTaskSuspend(AutoTaskHandle);
            vTaskSuspend(ManualTaskHandle);
            vTaskResume(RobotTaskHandle);
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

//  Manual task
//  Controls the rover from the Joysticks
void ManualTask(void *parameter){
    // Task is activated, and the rover is now controlled manually
    // Manual motor control goes here!!!
    Serial.println("Manual Task");

    while(myData.interruptBtn == 1){
        if (myData.x2 <= -75){alleHjul.driveFunction(DRIVE_FORWARD);}        //  FREMAD
        else if (myData.x2 >= 75){alleHjul.driveFunction(DRIVE_BACKWARD);}   //  BAGUD
        else if (myData.y2 <= -75){alleHjul.driveFunction(DRIVE_LEFT);}     //  HØJRE
        else if (myData.y2 >= 75){alleHjul.driveFunction(DRIVE_RIGHT);}       //  VENSTRE
        else{alleHjul.driveFunction(DRIVE_STOP);}                            //  STOP
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//  Autopilot task
//  Self driving mode
void AutoTask(void *parameter){
    while(myData.interruptBtn == 2){

        range.readULSDistances();
        range.readToFDistances();

        if (range.distance > 100)
        {
            if (range.range1MM < 175)
            {
                alleHjul.driveFunction(DRIVE_RIGHT);
                alleHjul.driveFunction(DRIVE_FORWARD);
            }
            else if (range.range2MM < 175)
            {
                alleHjul.driveFunction(DRIVE_LEFT);
                alleHjul.driveFunction(DRIVE_FORWARD);
            }
            else {
                alleHjul.driveFunction(DRIVE_FORWARD);
            }
        }
        else if (range.distance < 100){
            alleHjul.driveFunction(DRIVE_BACKWARD);      //  BAGUD
            alleHjul.driveFunction(DRIVE_BACKWARD);      //  BAGUD
            alleHjul.driveFunction(DRIVE_STOP);
        }
        
    }
}

//  Robot task
//  Control the robot arm from the Joysticks
void RobotTask(void *parameter){
    while(myData.interruptBtn == 3){
        if (myData.x1 <= -75){structPWM.servoClockwise(0, 40, myData.x1);}
        if (myData.x1 >= 75){structPWM.servoCounterClockwise(0, 40, myData.x1);}
        if (myData.y1 <= -75){structPWM.servoClockwise(1, 50, myData.y1);}
        if (myData.y1 >= 75){structPWM.servoCounterClockwise(1, 50, myData.y1);}
        if (myData.x2 <= -75){structPWM.servoClockwise(2, 50, myData.x2);}
        if (myData.x2 >= 75){structPWM.servoCounterClockwise(2, 50, myData.x2);}
        if (myData.y2 <= -75){structPWM.servoClockwise(3, 50, myData.y2);}
        if (myData.y2 >= 75){structPWM.servoCounterClockwise(3, 50, myData.y2);}
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

//  Default setup
void setup(){
    Serial.begin(115200);
    // wait until serial port opens for native USB devices
    while (! Serial) { delay(1); }

    Wire.begin();

    alleHjul.setupPins();
    
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    pinMode(xShut_Pin1, OUTPUT);
    pinMode(xShut_Pin2, OUTPUT);

    range.setID();

    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ); // Set PWM frequency to 50 Hz
    delay(50);

    // Create tasks
    xTaskCreatePinnedToCore(AutoTask, "AutoTask", 10000, NULL, 7, &AutoTaskHandle, 0);
    vTaskSuspend(AutoTaskHandle);
    xTaskCreatePinnedToCore(ManualTask, "ManualTask", 10000, NULL, 6, &ManualTaskHandle, 0);
    vTaskSuspend(ManualTaskHandle);
    xTaskCreatePinnedToCore(RobotTask, "RobotTask", 10000, NULL, 5, &RobotTaskHandle, 0);
    vTaskSuspend(RobotTaskHandle);
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