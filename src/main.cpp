#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "Adafruit_VL53L0X.h"
#include <cstddef>
#include "freertos/portmacro.h"

#define motorA1 25
#define motorA2 26
#define motorB1 27
#define motorB2 14
#define motorC1 5
#define motorC2 18
#define motorD1 16
#define motorD2 17
#define trigPin 33
#define echoPin 32
#define batteryLevelPin 36
#define SDAPin 21
#define SCKPin 22
#define BTN_manual 5//definér pin senere. Denne knap er til at aktivere ManualTask
#define BTN_auto 4//definér pin senere. Denne knap er til at aktivere DriveTask
#define BTN_robot 3 //definér pin senere. Denne knap er til at aktivere RobotarmTask
#define XSHUT1 1 // definér pin til XSHUT1 (Brug ikke denne pin)
#define XSHUT2 2 // definér pin til XSHUT2 (TIL TOF2 - Brug ikke denne pin)

Adafruit_VL53L0X ToF1 = Adafruit_VL53L0X();
Adafruit_VL53L0X ToF2 = Adafruit_VL53L0X();
U8G2_SH1106_128X32_VISIONOX_F_HW_I2C Display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

struct Hbro 
{    
    // Pin setup for motors
    // Noter til motorer er outdated, opdatér gerne hvis det lyster
    
    void setupPins() 
    {
        pinMode(motorA1, OUTPUT);
        pinMode(motorA2, OUTPUT);
        pinMode(motorB1, OUTPUT);
        pinMode(motorB2, OUTPUT);
    }
    void fremad()
    {
        digitalWrite(motorA1, 1); // A1 = A1A på Hbroen, og er den der får motor 1 til at køre fremad.
        digitalWrite(motorA2, 0);
        digitalWrite(motorB1, 1); // B1 = B1A på Hbroen, og er den der får motor 2 til at køre fremad.
        digitalWrite(motorB2, 0);
    }
    void bagud()
    {
        digitalWrite(motorA1, 0);
        digitalWrite(motorA2, 1); // A2 = A1B på Hbroen, og er den der får motor 1 til at køre bagud.
        digitalWrite(motorB1, 0);
        digitalWrite(motorB2, 1); // B2 = B1B på Hbroen, og er den der får motor 2 til at køre bagud.
    }
    void venstre()
    {
        digitalWrite(motorA1, 0);
        digitalWrite(motorA2, 1); // Motor 1, den venstre motor, kører bagud, for at dreje mod venstre.
        digitalWrite(motorB1, 1); // Motor 2, den højre motor, kører fremad, for at dreje mod venstre.
        digitalWrite(motorB2, 0);
    }
    void hojre()
    {
        digitalWrite(motorA1, 1); // Motor 1, venstre motor, kører fremad, for at dreje mod højre.
        digitalWrite(motorA2, 0);
        digitalWrite(motorB1, 0);
        digitalWrite(motorB2, 1); // Motor 2, højre motor, kører bagud, for at dreje mod højre.
    }
    void stopmotor()
    {
        digitalWrite(motorA1, 0);
        digitalWrite(motorA2, 0);
        digitalWrite(motorB1, 0);
        digitalWrite(motorB2, 0);
    }

};

struct Hbro2
{    
    // Pin setup for motors

    void setupPins() 
    {
        pinMode(motorC1, OUTPUT);
        pinMode(motorC2, OUTPUT);
        pinMode(motorD1, OUTPUT);
        pinMode(motorD2, OUTPUT);
    }
    void fremad()
    {
        digitalWrite(motorC1, 1); 
        digitalWrite(motorC2, 0);
        digitalWrite(motorD1, 1); 
        digitalWrite(motorD2, 0);
    }
    void bagud()
    {
        digitalWrite(motorC1, 0);
        digitalWrite(motorC2, 1); 
        digitalWrite(motorD1, 0);
        digitalWrite(motorD2, 1); 
    }
    void venstre()
    {
        digitalWrite(motorC1, 0);
        digitalWrite(motorC2, 1); 
        digitalWrite(motorD1, 1); 
        digitalWrite(motorD2, 0);
    }
    void hojre()
    {
        digitalWrite(motorC1, 1);
        digitalWrite(motorC2, 0);
        digitalWrite(motorD1, 0);
        digitalWrite(motorD2, 1);
    }
    void stopmotor()
    {
        digitalWrite(motorC1, 0);
        digitalWrite(motorC2, 0);
        digitalWrite(motorD1, 0);
        digitalWrite(motorD2, 0);
    }
};

struct Ultralydssensor
{
    long duration, cm;

    void setupULS()
    {
        
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    long distance()
    {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(5);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
 
        // Convert the time into a distance
        cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343

        delay (50);

        return cm;

    }
};

struct OLED
{
    void setup()
    {
        Display1.begin();
    }
    void OLEDWrite()
    {
        Display1.clearBuffer();         // clear the internal memory
        Display1.setFont(u8g2_font_t0_11_tf); // choose a suitable font
        Display1.drawStr(0,10,"Ardustore.dk!"); // write something to the internal memory
        Display1.sendBuffer();          // transfer internal memory to the display
        delay(1000);
    }
};

struct Battery{
    void pinSetup()
    {
        pinMode(batteryLevelPin, INPUT);
    }
    float readBatteryLevel()
    {
        int analogInput = analogRead(batteryLevelPin);
        float rawVolts = analogInput * 3.3/4096;

        return rawVolts;
    }
};

struct TOFs
{

void setupToFSensors() {

pinMode (XSHUT1,OUTPUT);
pinMode(XSHUT2, OUTPUT);

//Sluk begge ToF sensorer først
digitalWrite(XSHUT1, LOW); 
digitalWrite(XSHUT2, LOW);
delay(100);

//Tænd sensor 1
digitalWrite(XSHUT1, HIGH);
delay(200);
ToF1.begin();
ToF1.setAddress(0x40);
delay(100);

//Tænd sensor 2
digitalWrite(XSHUT2, HIGH);
delay(200);
ToF2.begin();
ToF2.setAddress(0x41);
}



};

Hbro forhjul;
Hbro2 baghjul;
Ultralydssensor frontSensor;
TOFs ToFSensors;



TaskHandle_t UltraTaskHandle = NULL;
TaskHandle_t ManualTaskHandle = NULL;
TaskHandle_t DriveTaskHandle = NULL;
TaskHandle_t RobotTaskHandle = NULL;
TaskHandle_t ToFTask1 = NULL;
TaskHandle_t ToFTask2 = NULL;

QueueSetHandle_t Distances = NULL;
QueueSetHandle_t ToFQueue1 = NULL;
QueueSetHandle_t ToFQueue2 = NULL;

void ButtonTask(void *parameter)
{
    bool prevManual = false;
    bool prevDrive = false;
    bool prevRobotarm = false;

    while(true);
    {
        // Ved ikke om dette virker, så andre må gerne vurdere om der skal laves ændringer.
        bool manualNow = digitalRead(BTN_manual) == LOW;
        bool autoNow = digitalRead(BTN_auto) == LOW;
        bool robotNow = digitalRead(BTN_robot) == LOW;

        // Activation of Manual mode
        if (manualNow && !prevManual)
        {
            Serial.println("Manual mode activated");

            //Suspension of all other tasks than "Manual"
            vTaskSuspend(DriveTaskHandle);
            vTaskSuspend(UltraTaskHandle);
            vTaskSuspend(RobotTaskHandle);
            // vTaskSuspend(); Insert suspension of ToF tasks!!!

            // Activation of "Manual"
            vTaskResume(ManualTaskHandle);
        }

        // Activation of Drive mode
        if (autoNow && !prevDrive)
        {
            Serial.println("Auto mode activated");

            // Suspension of other tasks
            vTaskSuspend(ManualTaskHandle);
            vTaskSuspend(RobotTaskHandle);

            // Activation of all tasks needed for DriveTask
            vTaskResume(DriveTaskHandle);
            vTaskResume(UltraTaskHandle);
            // Insert ToF tasks here!!!
            // Insert ToF tasks here!!!
            // Insert ToF tasks here!!!
            // Insert ToF tasks here!!!
        }

        // Activation of Robot Arm
        if (robotNow && !prevRobotarm)
        {
            Serial.println("Robot Arm activated");

            //Suspension of all other tasks than "RobotTask"
            vTaskSuspend(DriveTaskHandle);
            vTaskSuspend(UltraTaskHandle);
            vTaskSuspend(ManualTaskHandle);
            // vTaskSuspend(); Insert suspension of ToF tasks!!!

            vTaskResume(RobotTaskHandle);
        }

        prevManual = manualNow;
        prevDrive = autoNow;

        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
}

void UltraTask(void *parameter)
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
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
    
    // Send to queue
    xQueueSend(Distances, &distance, portMAX_DELAY);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    vTaskDelay(200 / portTICK_PERIOD_MS);
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

void DriveTask(void *parameter)
{
    long distance = 0;
    long ToF1Distance;     
    long ToF2Distance;     
    for (;;)
    {
        if (xQueueReceive(Distances, &distance, portMAX_DELAY) == pdTRUE)
        if (xQueueReceive(ToFQueue1, &ToF1Distance, 10 / portTICK_PERIOD_MS) ==pdTRUE){}
        if (xQueueReceive(ToFQueue2, &ToF2Distance, 10 / portTICK_PERIOD_MS) ==pdTRUE){}




        if (distance > 25)
        {
            Serial.println("No obstacles, moving forward");
            forhjul.fremad();
            baghjul.fremad();
        }
        else if (distance > 0 && distance <= 25)
        {
            Serial.println("Too close, reversing!");
            forhjul.bagud();
            baghjul.bagud();
            vTaskDelay(400 / portTICK_PERIOD_MS);
            forhjul.stopmotor();
            baghjul.stopmotor();
        }
        else
        {
            forhjul.stopmotor();
            baghjul.stopmotor();
        }
    }
}

void ToFSensor1 (void*parameter) //FX VENSTRE ToF
{

    while (true)
    {

VL53L0X_RangingMeasurementData_t measure;

//Serial.print ("Reading measurement..");
ToF1.rangingTest(&measure, false);

if (measure.RangeStatus != 4) {
    //Serial.print("Distance: ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range");
  }
  long ToF1Distance = measure.RangeMilliMeter;
  
  xQueueSend(ToFQueue1, &ToF1Distance, portMAX_DELAY);
 
  Serial.print("Sensor 1 distance:   ");
  //Serial.println(distance);
  vTaskDelay(1000 / portTICK_PERIOD_MS); //Dette delay skal ændres, dette er kun for tests!

    }

}

void ToFSensor2 (void*parameter)
{
 while (true)
    {

VL53L0X_RangingMeasurementData_t measure2;

//Serial.print ("Reading measurement..");
ToF2.rangingTest(&measure2, false);

if (measure2.RangeStatus != 4) {
    //Serial.print("Distance: ");
    Serial.print (" ");
    Serial.print(measure2.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range");
  }
  long ToF2Distance = measure2.RangeMilliMeter;
  
  xQueueSend(ToFQueue2, &ToF2Distance, portMAX_DELAY);
  Serial.print("Sensor 2 distance:   ");
  
  vTaskDelay(1000 / portTICK_PERIOD_MS); // dette delay skal ændres, kun for tests!

    }

}


void setup() 
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting FreeRTOS Rover");

    forhjul.setupPins();   
    baghjul.setupPins();
    frontSensor.setupULS();
    ToFSensors.setupToFSensors();

    //Queue Sizes
    int ToF1QueueSize = 5;
    int ToF2QueueSize = 5;

    // Create queue for distances
    Distances = xQueueCreate(10, sizeof(long));
    ToFQueue1 = xQueueCreate(ToF1QueueSize, sizeof(long));
    ToFQueue2 = xQueueCreate(ToF2QueueSize, sizeof(long));

    // Create tasks
    xTaskCreatePinnedToCore(UltraTask, "UltraTask", 4096, NULL, 1, &UltraTaskHandle, 0);
    xTaskCreatePinnedToCore(DriveTask, "DriveTask", 4096, NULL, 1, &DriveTaskHandle, 1);
    xTaskCreatePinnedToCore(ToFSensor1, "TimeOfFlight", 4096, NULL, 1, &ToFTask1, 1);
    xTaskCreatePinnedToCore(ToFSensor2, "TimeOfFlight2", 4096, NULL, 1, &ToFTask2, 1);
}

void loop()
{
  
}
