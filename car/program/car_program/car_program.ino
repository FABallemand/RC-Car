#define ABS(X) (X) < 0 ? -(X) : (X)

//== Motors ===================================================================
// Motor Right
#define EN_A 9
#define IN_1 8
#define IN_2 7

// Motor Left
#define EN_B 4
#define IN_3 6
#define IN_4 5

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

void accelerate()
{
  if (state == FORWARD)
  {
    speed = (speed + ACCELERATION <= MAX_SPEED) ? speed + ACCELERATION : MAX_SPEED;
  }
  else
  {
    if (speed == 0)
    {
      state = FORWARD;

      digitalWrite(IN_1, HIGH);
      digitalWrite(IN_2, LOW);

      digitalWrite(IN_3, HIGH);
      digitalWrite(IN_4, LOW);

      speed = MIN_SPEED;
    }
    else
    {
      speed = (speed + ACCELERATION <= -MIN_SPEED) ? speed + ACCELERATION : 0;
    }
  }
  analogWrite(EN_A, ABS(speed));
  analogWrite(EN_B, ABS(speed));
}

void decelerate()
{
  if (state == BACKWARD)
  {
    speed = (speed - DECELERATION >= -MAX_SPEED) ? speed - DECELERATION : -MAX_SPEED;
  }
  else
  {
    if (speed == 0)
    {
      state = BACKWARD;

      digitalWrite(IN_1, LOW);
      digitalWrite(IN_2, HIGH);

      digitalWrite(IN_3, LOW);
      digitalWrite(IN_4, HIGH);

      speed = -MIN_SPEED;
    }
    else
    {
      speed = (speed - DECELERATION >= MIN_SPEED) ? speed - DECELERATION : 0;
    }
  }
  analogWrite(EN_A, ABS(speed));
  analogWrite(EN_B, ABS(speed));
}

//== Servo Motor ==============================================================
#include <Servo.h>

#define SERVO 10

#define MIN_TURN 0
#define MAX_TURN 180
#define TURNING_RATE 15

int direction = 0;

Servo direction_servo;

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

#define BLUETOOTH_RX 11
#define BLUETOOTH_TX 12

byte received;

SoftwareSerial BTHC08Serial(BLUETOOTH_RX, BLUETOOTH_TX);

//== Sonar ====================================================================
#include <NewPing.h>

#define TRIGGER_PIN 3
#define ECHO_PIN 3

#define MIN_DISTANCE 2
#define MAX_DISTANCE 400
#define ITERATIONS 5 // Default = 5

float distance, duration, sound_speed; // Speed of sound in cm/ms

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

//== Temperature & Humidity Sensor ============================================
#include <Adafruit_AHT10.h>

Adafruit_AHT10 aht;

//== Accelerometer=============================================================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

/*====================================================
  NOTE: In addition to connection 5v, GND, SDA, and
  SCL, this sketch depends on the MPU-6050's INT pin
  being connected to the Arduino's external interrupt
  #0 pin.
  On the Arduino Nano, this is digital I/O pin 2.
  ====================================================*/

MPU6050 mpu(0x68); // Default adresses: AD0 low = 0x68, AD0 high = 0x69
#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// Orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// Interruption detection routine
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

// Output
// #define OUTPUT_READABLE_QUATERNION // quaternion components
// #define OUTPUT_READABLE_EULER // Euler angles (in degree) (http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL // Yaw/Pitch/Roll angles (in degree, also requires gravity vector calculations) (http://en.wikipedia.org/wiki/Gimbal_lock)
// #define OUTPUT_READABLE_REALACCEL // Acceleration components with gravity removed (not compensadted for orientation)
// #define OUTPUT_READABLE_WORLDACCEL // Acceleration components with gravity removed and adjusted for the world frame of reference (yaw is relative to initial orientation, since no magnetometer is present in this case)

//=============================================================================
//== SETUP ====================================================================
//=============================================================================
void setup()
{
  // Serial
  Serial.begin(9600); // For testing purpose only

  // Motor Right
  pinMode(EN_A, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);

  // Motor Left
  pinMode(EN_B, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  // Direction Servo
  direction_servo.attach(SERVO);

  // Bluetooth
  BTHC08Serial.begin(9600);

  // Temperature & Humidity Sensor
  if (!aht.begin())
  {
    Serial.println("Could not find AHT10? Check wiring");
    while (1)
      delay(10);
  }

// Accelerometer
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  if (!mpu.testConnection())
  {
    Serial.println("ERROR: IMU"); // PROBLEME D4ADRESSES
  }

  // DMP (Digital Motion Processor) from IMU =======
  // Load DMP
  devStatus = mpu.dmpInitialize();
  // Configure with your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // Working test (0 => OK)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6); // ???
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // Turn on the DMP
    mpu.setDMPEnabled(true);
    // Enable Arduino interrupt detection
    /*Serial.print("Enabling interrupt detection (Arduino external interrupt ");
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(")");*/
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // Set DMP Ready flag
    dmpReady = true;
    // Get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("ERROR: DMP (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  Serial.println("Ready to drive!");
}

//=============================================================================
//== LOOP =====================================================================
//=============================================================================
void loop()
{
  // Bluetooth & Motors
  if (BTHC08Serial.available())
  {
    received = BTHC08Serial.read();
    switch (received)
    {
    case 0:
      // Serial.println("forward");
      accelerate();
      break;
    case 1:
      // Serial.println("backward");
      decelerate();
      break;
    case 2:
      // Serial.println("left");
      turnLeft();
      break;
    case 3:
      // Serial.println("right");
      turnRight();
      break;
    }
  }
  Serial.print("State = ");
  Serial.print(state);
  Serial.print(" | Speed = ");
  Serial.print(speed);

  // Temperature & Humidity Sensor
  sensors_event_t temperature, humidity;
  aht.getEvent(&humidity, &temperature);

  // Sonar
  sound_speed = (331.4 + (0.606 * temperature.temperature) + (0.0124 * humidity.relative_humidity)) / 10000;
  duration = sonar.ping_median(ITERATIONS);
  distance = (duration / 2) * sound_speed;

  Serial.print(" | Temperature = ");
  Serial.print(temperature.temperature);
  Serial.print(" | Humidity = ");
  Serial.print(humidity.relative_humidity);
  Serial.print(" | Distance = ");
  if (distance > MAX_DISTANCE || distance < MIN_DISTANCE)
  {
    Serial.print("Out of range!!\n");
  }
  else
  {
    Serial.print(distance);
    Serial.print(" cm");
  }

  // Accelerometer
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
#ifdef OUTPUT_READABLE_QUATERNION
    // Display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // Display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // Display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print(" | ypr = ");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print(" ");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print(" ");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // Display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // Display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

    // Blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
