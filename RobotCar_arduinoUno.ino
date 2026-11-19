#include "MecanumCar_v2.h"
#include <Servo.h>

// Ultrasonic pins
const int EchoPin = 13;
const int TrigPin = 12;

// Servo
Servo servoSensor;

// Car object
mecanumCar mecanumCar(3, 2);

// Distance thresholds
const float OBSTACLE_DISTANCE = 20.0;
const float CLEAR_DISTANCE = 40.0;


// =====================================================
// Ultrasonic distance
// =====================================================

float getDistance(const int trigger, const int echo)
{
    digitalWrite(trigger, LOW);
    delayMicroseconds(2);

    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigger, LOW);

    long duration = pulseIn(echo, HIGH, 30000);

    if (duration == 0)
    {
        return 999;
    }

    return duration * 0.0343f / 2.0f;
}


// =====================================================
// Scan left and right
// Returns true if left is better
// Returns false if right is better
// =====================================================

bool detectSide()
{
    mecanumCar.Stop();
    delay(300);

    // Look left
    servoSensor.write(180);
    delay(500);

    float leftDistance = getDistance(TrigPin, EchoPin);

    // Look right
    servoSensor.write(0);
    delay(500);

    float rightDistance = getDistance(TrigPin, EchoPin);

    // Return to centre
    servoSensor.write(90);
    delay(300);

    Serial.print("Left: ");
    Serial.println(leftDistance);

    Serial.print("Right: ");
    Serial.println(rightDistance);

    if (leftDistance > rightDistance)
    {
        Serial.println("Choosing LEFT");
        return true;
    }
    else
    {
        Serial.println("Choosing RIGHT");
        return false;
    }
}


// =====================================================
// Obstacle avoidance
// =====================================================

void avoidObstacle()
{
    bool goLeft = detectSide();

    if (goLeft)
    {
        mecanumCar.Turn_Left();

        while (true)
        {
            float frontDistance =
                getDistance(TrigPin, EchoPin);

            Serial.print("Turning Left. Front = ");
            Serial.println(frontDistance);

            if (frontDistance > CLEAR_DISTANCE)
            {
                break;
            }

            delay(50);
        }
    }
    else
    {
        mecanumCar.Turn_Right();

        while (true)
        {
            float frontDistance =
                getDistance(TrigPin, EchoPin);

            Serial.print("Turning Right. Front = ");
            Serial.println(frontDistance);

            if (frontDistance > CLEAR_DISTANCE)
            {
                break;
            }

            delay(50);
        }
    }

    mecanumCar.Stop();
    delay(300);

    Serial.println("Path Clear");
}


// =====================================================
// Setup
// =====================================================

void setup()
{
    Serial.begin(9600);

    pinMode(EchoPin, INPUT);
    pinMode(TrigPin, OUTPUT);

    servoSensor.attach(9);
    servoSensor.write(90);

    mecanumCar.Init();

    // Slow speed
    speed_Upper_L = 50;
    speed_Lower_L = 50;
    speed_Upper_R = 50;
    speed_Lower_R = 50;

    delay(500);
}


// =====================================================
// Main loop
// =====================================================

void loop()
{
    float frontDistance =
        getDistance(TrigPin, EchoPin);

    Serial.print("Front: ");
    Serial.println(frontDistance);

    if (frontDistance < OBSTACLE_DISTANCE)
    {
        Serial.println("Obstacle detected");

        avoidObstacle();
    }
    else
    {
        mecanumCar.Advance();
    }

    delay(50);
}