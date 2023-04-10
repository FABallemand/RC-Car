/*
  // Sonar
  #include <NewPing.h>

  #define TRIGGER_PIN 2
  #define ECHO_PIN 2
  #define MIN_DISTANCE 2
  #define MAX_DISTANCE 400
  #define ITERATIONS 5 // Default = 5

  NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

  float distance, duration;

  void setup()
  {
    Serial.begin(9600);
  }

  void loop()
  {
    // distance = sonar.ping_cm();
    duration = sonar.ping_median(ITERATIONS);
    distance = (duration / 2) * 0.0343;

    Serial.print("Distance = ");
    if (distance > MAX_DISTANCE || distance < MIN_DISTANCE)
    {
        Serial.print("Out of range!!\n");
    }
    else
    {
        Serial.print(distance);
        Serial.print(" cm\n");
    }

    delay(500);
  }
*/
//=============================================================================

// Sonar
#include <NewPing.h>

#define TRIGGER_PIN 2
#define ECHO_PIN 2
#define MIN_DISTANCE 2
#define MAX_DISTANCE 400
#define ITERATIONS 5 // Default = 5

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

float distance, duration;

// Temperature/Humidity Sensor
#include <Adafruit_AHT10.h>

Adafruit_AHT10 aht;

float sound_speed; // Speed of sound in cm/ms

void setup()
{
  Serial.begin(9600);

  // Temperature-Humidity Sensor
  if (! aht.begin()) {
    Serial.println("Could not find AHT10? Check wiring");
    while (1) delay(10);
  }
}

void loop()
{
  // Temperature-Humidity Sensor
  sensors_event_t temperature, humidity;
  aht.getEvent(&humidity, &temperature);

  sound_speed = (331.4 + (0.606 * temperature.temperature) + (0.0124 * humidity.relative_humidity)) / 10000;

  duration = sonar.ping_median(ITERATIONS);

  distance = (duration / 2) * sound_speed;

  Serial.print("Temperature = ");
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
    Serial.print(" cm\n");
  }

  delay(500);
}
