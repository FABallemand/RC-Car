/*#define ABS(X)                \
    {                         \
        (X) < 0 ? -(X) : (X); \
    }*/
#define ABS(X) (X) < 0 ? -(X) : (X)

//== Motors ===================================================================
// Common
#define MAX_SPEED 256
int speed = 0;

// Motor Right
int enA = 5;
int in1 = 4;
int in2 = 3;

// Motor Left
int enB = -1;
int in3 = -1;
int in4 = -1;

void accelerate()
{
    // Turn on motors and set motors forward
    if (speed == 0)
    {
        // Motor Rigth
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        // Motor Left
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }

    // speed = speed < MAX_SPEED ? ++speed : MAX_SPEED;

    if (ABS(speed) < MAX_SPEED)
    {
        ++speed;
        analogWrite(enA, ABS(speed));
        analogWrite(enB, ABS(speed));
    }
}

void decelerate()
{
    // Turn on motors and set motors backward
    if (speed == 0)
    {
        // Motor Rigth
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);

        // Motor Left
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }

    if (ABS(speed) < MAX_SPEED)
    {
        --speed;
        analogWrite(enA, ABS(speed));
        analogWrite(enB, ABS(speed));
    }
}

//== Servo Motor ==============================================================
#include <Servo.h>
#define MIN_TURN 0
#define MAX_TURN 180
#define SERVO 9
Servo direction_servo;
int direction = 0;

void turnRight()
{
    if (direction < MAX_TURN)
    {
        direction_servo.write(++direction);
    }
}

void turnLeft()
{
    if (direction > MIN_TURN)
    {
        direction_servo.write(--direction);
    }
}

//== Bluetooth ================================================================


//=============================================================================
//== SETUP ====================================================================
//=============================================================================
void setup()
{
    // Motor Right
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    // Motor Left
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // Direction Servo
    direction_servo.attach(SERVO);
}

//=============================================================================
//== LOOP =====================================================================
//=============================================================================
void loop()
{
}
