/*#define LED 13
  bool state = LOW;
  char received;

  void changeState()
  {
  state = !state;
  digitalWrite(LED, state);
  }

  void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  }

  void loop() {
  received = Serial.read();
  if (received == 'a')
  {
    changeState();
    Serial.write("Received");
  };
  delay(500);
  }*/

//====================================================================

/*#include <SoftwareSerial.h>

  SoftwareSerial BTHC08Serial(10, 11); // RX | TX

  void setup() {
  Serial.begin(9600);
  BTHC08Serial.begin(9600);

  Serial.println("Enter AT commands:");
  }

  void loop() {
  // Keep reading from HC-08 and send to Arduino serial monitor
  if(BTHC08Serial.available())
    Serial.write(BTHC08Serial.read());

  // Keep reading from Arduino serial monitor and sen to HC-08
  if(Serial.available())
    BTHC08Serial.write(Serial.read());
  }*/

//====================================================================

#include <SoftwareSerial.h>

// char received;
byte received;

SoftwareSerial BTHC08Serial(10, 11); // RX | TX

void setup()
{
  Serial.begin(9600);
  BTHC08Serial.begin(9600);
  Serial.println("Ready to receive data!");
}

void loop()
{
  // Keep reading from HC-08 and send to Arduino serial monitor
  if (BTHC08Serial.available())
  {
    received = BTHC08Serial.read();
    Serial.println(received);
    switch (received)
    {
    case 0:
      Serial.println("forward");
      break;
    case 1:
      Serial.println("backward");
      break;
    case 2:
      Serial.println("left");
      break;
    case 3:
      Serial.println("right");
      break;
    }
  }
}

// void loop()
// {
//   // Keep reading from HC-08 and send to Arduino serial monitor
//   if (BTHC08Serial.available())
//   {
//     received = BTHC08Serial.read();
//     if (received == 'f')
//     {
//       Serial.println("forward");
//     }
//     else
//     {
//       if (received == 'b')
//       {
//         Serial.println("backward");
//       }
//       else
//       {
//         if (received == 'l')
//         {
//           Serial.println("left");
//         }
//         else
//         {
//           if (received == 'r')
//           {
//             Serial.println("right");
//           }
//           else
//           {
//             Serial.println("received something weird...");
//           }
//         }
//       }
//     }
//   }
// }
