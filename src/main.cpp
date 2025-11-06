#include <Arduino.h>
#define motorA1 17
#define motorA2 18
#define motorB1 15
#define motorB2 16
#define motorC1 21
#define motorC2 22
#define motorD1 4
#define motorD2 5
#define trigPin 25
#define echoPin 26


struct Hbro 
{    
    // Pin setup for motors

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

Hbro forhjul;
Hbro2 baghjul;
Ultralydssensor frontSensor;

void setup() 
{
  forhjul.setupPins();
  baghjul.setupPins();
  frontSensor.setupULS();
}

void loop()
{
    int stringout = frontSensor.distance();  // get distance reading

    Serial.print("Distance: ");
    Serial.print(stringout);
    Serial.println(" cm");
  
  if (frontSensor.cm > 0 && frontSensor.cm < 20) // defining thresholds for wheel reversal
  {
    Serial.println("Too close! Reversing...");
    forhjul.stopmotor();
    baghjul.stopmotor();
    delay (250);
    forhjul.bagud();
    baghjul.bagud();
    delay (1000);
    forhjul.stopmotor();
    baghjul.stopmotor();
    delay (250);
  } 
    else // if not within threshold move forward
    {
        Serial.println("Moving forward...");
        forhjul.fremad();
        baghjul.fremad();
    }

  delay(500);
}