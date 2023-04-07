#define ABS(X) (X) < 0 ? -(X) : (X)

//== Motors ===================================================================
// Common
#define MIN_SPEED 80
#define MAX_SPEED 255

#define IDLE 0
#define FORWARD 1
#define BACKWARD 2

#define ACCELERATION 2
#define DECELERATION 4

int state = IDLE;
int speed = 0;

// Motor Right
int enA = 5;
int in1 = 4;
int in2 = 3;

// Motor Left
/*
  int enB = 8;
  int in3 = 7;
  int in4 = 6;
*/
void accelerate()
{
  if(state == FORWARD)
  {
    speed = (speed + ACCELERATION <= MAX_SPEED) ? speed + ACCELERATION : MAX_SPEED;
  }
  else
  {
    if (speed == 0)
    {
      state = FORWARD;
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      speed = MIN_SPEED;
    }
    else
    {
      speed = (speed + ACCELERATION <= -MIN_SPEED) ? speed + ACCELERATION : 0;
    }
  }
  analogWrite(enA, ABS(speed));
}

void decelerate()
{
  if(state == BACKWARD)
  {
    speed = (speed - DECELERATION >= -MAX_SPEED) ? speed - DECELERATION : -MAX_SPEED;
  }
  else
  {
    if (speed == 0)
    {
      state = BACKWARD;
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      speed = -MIN_SPEED;
    }
    else
    {
      speed = (speed - DECELERATION >= MIN_SPEED) ? speed - DECELERATION : 0;
    }
  }
  analogWrite(enA, ABS(speed));
}


//== Servo Motor ==============================================================
#include <Servo.h>

#define MIN_TURN 0
#define MAX_TURN 180
#define SERVO 9

Servo direction_servo;

#define TURNING_RATE 15
int direction = 0;

void turnLeft()
{
  direction = (direction - TURNING_RATE >= MIN_TURN) ? direction - TURNING_RATE : MIN_TURN;
  direction_servo.write(direction);
}

void turnRight()
{
  direction = (direction + TURNING_RATE <= MAX_TURN) ? direction + TURNING_RATE : MAX_TURN;
  direction_servo.write(direction);
}

//== Bluetooth ================================================================
#include <SoftwareSerial.h>

byte received;
SoftwareSerial BTHC08Serial(10, 11); // RX | TX

//=============================================================================
//== SETUP ====================================================================
//=============================================================================
void setup()
{
  // Serial
  Serial.begin(9600); // For testing purpose only

  // Motor Right
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Motor Left
  /*
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
  */

  // Direction Servo
  direction_servo.attach(SERVO);

  // Bluetooth
  BTHC08Serial.begin(9600);

  Serial.println("Ready to drive!");
}

//=============================================================================
//== LOOP =====================================================================
//=============================================================================
void loop()
{
  if (BTHC08Serial.available())
  {
    received = BTHC08Serial.read();
    switch (received)
    {
      case 0:
        //Serial.println("forward");
        accelerate();
        break;
      case 1:
        //Serial.println("backward");
        decelerate();
        break;
      case 2:
        //Serial.println("left");
        turnLeft();
        break;
      case 3:
        //Serial.println("right");
        turnRight();
        break;
    }
  }
  Serial.println(state);
  Serial.println(speed);
}
