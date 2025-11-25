#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

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
        Serial.begin (9600);
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
        U8G2_SH1106_128X32_VISIONOX_F_HW_I2C Display1(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

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

Hbro forhjul;
Hbro2 baghjul;
Ultralydssensor frontSensor;

TaskHandle_t UltraTaskHandle = NULL;
TaskHandle_t DriveTaskHandle = NULL;

QueueSetHandle_t Distances = NULL;

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

void DriveTask(void *parameter)
{
    long distance = 0;
    for (;;)
    {
        if (xQueueReceive(Distances, &distance, portMAX_DELAY) == pdTRUE)

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

void setup() 
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting FreeRTOS Rover");

    forhjul.setupPins();   
    baghjul.setupPins();
    frontSensor.setupULS();

    // Create queue for distances
    Distances = xQueueCreate(10, sizeof(long));

    // Create tasks
    xTaskCreatePinnedToCore(UltraTask, "UltraTask", 4096, NULL, 1, &UltraTaskHandle, 0);
    xTaskCreatePinnedToCore(DriveTask, "DriveTask", 4096, NULL, 1, &DriveTaskHandle, 1);
}

void loop()
{
  
}
