// Test motors with Arduino //

// Motor A
int enA = 5;
int in1 = 4;
int in2 = 3;

void setup()
{
  // Serial
  Serial.begin(9600);
  
  // Set motor control pins to output
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

/**
   \brief Rotate for 2 seconds

*/
void demo1()
{ 
  // Turn on motor and set rotation one way
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 200);

  delay(5000);

  // Turn off motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  
  delay(2000);

  // Turn on motor and set rotation the other way
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 200);

  delay(5000);

  // Turn off motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
}

/**
   \brief Accelerate and decelerate

*/
void demo2()
{
  // Turn on motor and set rotation
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

  // Accelerate from 0 to max speed
  int i = 0;
  for (; i < 256; i++)
  {
    analogWrite(enA, i);
    delay(100);
  }
  for (; i >= 0; i--)
  {
    analogWrite(enA, i);
    delay(100);
  }

  delay(2000);

  // Turn off motor
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
}

void loop()
{
  demo1();

  delay(1000);

  demo2();
}
