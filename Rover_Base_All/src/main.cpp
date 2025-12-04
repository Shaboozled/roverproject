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

#define trigPin 32
#define echoPin 35
#define xShut_Pin 2
#define batteryLevelPin 36

#define motorA1 25
#define motorA2 26
#define motorB1 27
#define motorB2 14
#define motorC1 5
#define motorC2 18
#define motorD1 16
#define motorD2 17

Adafruit_PWMServoDriver pwm;
U8G2_SH1106_128X32_VISIONOX_F_HW_I2C Display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Adafruit_VL53L0X ToFSensor1;
Adafruit_VL53L0X ToFSensor2;
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

// Define the servo parameters
#define SERVO_OPENING_DEADZONE 297    // Deadzone start (312)
#define SERVOSTOP 350                 // Complete deadzone
#define SERVO_CLOSING_DEADZONE 425    // Deadzone start (410)
#define SERVO_FREQ 50                 // Analog servos run at ~50 Hz

//  Define different tasks handlers
TaskHandle_t ButtonTaskHandle = NULL;
TaskHandle_t ManualTaskHandle = NULL;
TaskHandle_t AutoTaskHandle = NULL;
TaskHandle_t RobotTaskHandle = NULL;

//  Used variables
int btnSettings = 0,  BTN_ChangeMode = 0, pulseleng;
long BTN_ChangeMode_Time = 0, BTN_ChangeMode_Last = 0;
char oledInput[11];

//  Struct creations with creators
struct Hbro{    
    // Pin setup for motors
    // Noter til motorer er outdated, opdatér gerne hvis det lyster
    // Alle hjul bliver sat op sammen, på samme måde ved højre/venstre
    
    void setupPins() 
    {
        pinMode(motorA1, OUTPUT);
        pinMode(motorA2, OUTPUT);
        pinMode(motorB1, OUTPUT);
        pinMode(motorB2, OUTPUT);
        pinMode(motorC1, OUTPUT);
        pinMode(motorC2, OUTPUT);
        pinMode(motorD1, OUTPUT);
        pinMode(motorD2, OUTPUT);
    }
    void fremad()
    {
        digitalWrite(motorA1, 1); // A1 = A1A på Hbroen, og er den der får motor 1 til at køre fremad.
        digitalWrite(motorA2, 0);
        digitalWrite(motorB1, 1); // B1 = B1A på Hbroen, og er den der får motor 2 til at køre fremad.
        digitalWrite(motorB2, 0);

        digitalWrite(motorC1, 1); 
        digitalWrite(motorC2, 0);
        digitalWrite(motorD1, 1); 
        digitalWrite(motorD2, 0);
    }
    void bagud()
    {
        digitalWrite(motorA1, 0);
        digitalWrite(motorA2, 1); // A2 = A1B på Hbroen, og er den der får motor 1 til at køre bagud.
        digitalWrite(motorB1, 0);
        digitalWrite(motorB2, 1); // B2 = B1B på Hbroen, og er den der får motor 2 til at køre bagud.
        
        digitalWrite(motorC1, 0);
        digitalWrite(motorC2, 1); 
        digitalWrite(motorD1, 0);
        digitalWrite(motorD2, 1); 
    }
    void venstre()
    {
        digitalWrite(motorA1, 0);
        digitalWrite(motorA2, 1); // Motor 1, den venstre motor, kører bagud, for at dreje mod venstre.
        digitalWrite(motorB1, 1); // Motor 2, den højre motor, kører fremad, for at dreje mod venstre.
        digitalWrite(motorB2, 0);
        
        digitalWrite(motorC1, 0);
        digitalWrite(motorC2, 1); 
        digitalWrite(motorD1, 1); 
        digitalWrite(motorD2, 0);
    }
    void hojre()
    {
        digitalWrite(motorA1, 1); // Motor 1, venstre motor, kører fremad, for at dreje mod højre.
        digitalWrite(motorA2, 0);
        digitalWrite(motorB1, 0);
        digitalWrite(motorB2, 1); // Motor 2, højre motor, kører bagud, for at dreje mod højre.
        
        digitalWrite(motorC1, 1);
        digitalWrite(motorC2, 0);
        digitalWrite(motorD1, 0);
        digitalWrite(motorD2, 1);
    }
    void stopmotor()
    {
        digitalWrite(motorA1, 0);
        digitalWrite(motorA2, 0);
        digitalWrite(motorB1, 0);
        digitalWrite(motorB2, 0);

        digitalWrite(motorC1, 0);
        digitalWrite(motorC2, 0);
        digitalWrite(motorD1, 0);
        digitalWrite(motorD2, 0);
    }
};
Hbro alleHjul;

struct OLED
{
    void setup()
    {
        Display1.begin();
    }
    void OLEDWrite(char* Text)
    {
        Display1.clearBuffer();                 // clear the internal memory
        Display1.setFont(u8g2_font_4x6_mf);     // choose a suitable font
        Display1.drawStr(2,5, Text);            // write something to the internal memory
        Display1.sendBuffer();                  // transfer internal memory to the display
        delay(1000);
    }
};
OLED display;

struct Battery
{
  void SetupBattery()
  {
      pinMode(batteryLevelPin, INPUT);
  }
  char* readBatteryLevel()
  {
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
        pulseleng++;
        delay(250);
        pwm.setPWM(chan, 0, SERVOSTOP);
    }
    void servoCounterClockwise(int chan){
        pwm.setPWM(chan, 0, SERVO_OPENING_DEADZONE);
        pulseleng--;
        delay(250);
        pwm.setPWM(chan, 0, SERVOSTOP);
    }
};
PWMBoard structPWM;

void btnTask(void *paramter)
{
    for(;;)
    {
        BTN_ChangeMode = myData.interruptBtn;
        Serial.println(BTN_ChangeMode);

        // Activation of Manual mode
        if (BTN_ChangeMode == 1)
        {
            Serial.println("Manual mode activated");

            //Suspension of all other tasks than "Manual"
            vTaskSuspend(AutoTaskHandle);
            vTaskSuspend(RobotTaskHandle);

            // Activation of "Manual"
            vTaskResume(ManualTaskHandle);
        }

        // Activation of Drive mode
        if (BTN_ChangeMode == 2)
        {
            Serial.println("Auto mode activated");

            // Suspension of other tasks
            vTaskSuspend(ManualTaskHandle);
            vTaskSuspend(RobotTaskHandle);

            // Activation of all tasks needed for DriveTask
            vTaskResume(AutoTaskHandle);
        }

        // Activation of Robot Arm
        if (BTN_ChangeMode == 3)
        {
            Serial.println("Robot Arm activated");

            //Suspension of all other tasks than "RobotTask"
            vTaskSuspend(AutoTaskHandle);
            vTaskSuspend(ManualTaskHandle);

            vTaskResume(RobotTaskHandle);
        }

        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
}

void ManualTask(void *parameter)
{
    while(true);
    {
        // Task is activated, and the rover is now controlled manually
        // Manual motor control goes here!!!

        


    }
}

void AutoTask(void *parameter)
{
    long distance = 0;
    long rangeMeasure1 = 0;
    long rangeMeasure2 = 0;
    for (;;)
    {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        // Read echo
        long duration = pulseInLong(echoPin, HIGH);
        long distance = (duration/2.0) / 29.1;

        // Read ToFs
        int rangeMeasure1 = ToFSensor1.rangingTest(&measure1, false);
        int rangeMeasure2 = ToFSensor2.rangingTest(&measure2, false);

        vTaskDelay(200 / portTICK_PERIOD_MS);

        if (distance > 25)
        {
            Serial.println("No obstacles, moving forward");
            alleHjul.fremad();
        }
        else if (distance > 0 && distance <= 25)
        {
            Serial.println("Too close, reversing!");
            alleHjul.bagud();
            vTaskDelay(400 / portTICK_PERIOD_MS);
            alleHjul.stopmotor();
        }
        else
        {
            alleHjul.stopmotor();
        }
        if (rangeMeasure1 < 50 && rangeMeasure2 > 100)
        {
            alleHjul.hojre();
        }
        else if (rangeMeasure2 < 50 && rangeMeasure1 > 100)
        {
            alleHjul.venstre();
        }
    }
}

void RobotTask(void *parameter){
    for (;;)
    {
        if (myData.x1 <= 50){structPWM.servoClockwise(2);}
        if (myData.x1 >= 200){structPWM.servoCounterClockwise(2);}
        if (myData.y1 <= 50){structPWM.servoClockwise(3);}
        if (myData.y1 >= 200){structPWM.servoCounterClockwise(3);}
        if (myData.x2 <= 50){structPWM.servoClockwise(0);}
        if (myData.x2 >= 200){structPWM.servoCounterClockwise(0);}
        if (myData.y2 <= 50){structPWM.servoClockwise(1);}
        if (myData.y2 >= 200){structPWM.servoCounterClockwise(1);}
    }
    vTaskDelay(400 / portTICK_PERIOD_MS);
}

void setup() 
{
    Serial.begin(9600);
    Serial.println("Starting FreeRTOS Rover");
    delay(500);
    
    Serial.println("PWM Setup\n");
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ); // Set PWM frequency to 50 Hz
    delay(500);

    Serial.println("Pins Setup\n");
    alleHjul.setupPins();
    //frontSensor.setupULS();
    //tofSensor.SetupTOF();
    display.setup();
    batteri.SetupBattery();

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(xShut_Pin, OUTPUT);
    
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

    
    Serial.println("WiFi Setup\n");
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    esp_now_init();
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    delay(500);

    Serial.println("Creating Tasks\n");
    // Create tasks
    xTaskCreatePinnedToCore(AutoTask, "DriveTask", 4096, NULL, 1, &AutoTaskHandle, 0);
    vTaskSuspend(AutoTaskHandle);
    xTaskCreatePinnedToCore(ManualTask, "ManualTask", 4096, NULL, 1, &ManualTaskHandle, 0);
    vTaskSuspend(ManualTaskHandle);
    xTaskCreatePinnedToCore(RobotTask, "RobotTask", 4096, NULL, 1, &RobotTaskHandle, 0);
    vTaskSuspend(RobotTaskHandle);
    xTaskCreatePinnedToCore(btnTask, "ButtonTask", 4096, NULL, 1, &ButtonTaskHandle, 1);
    delay(500);

    Serial.println("Setup done!\n\n");
}

void loop()
{
    //display.OLEDWrite(batteryMeasure.readBatteryLevel());
}