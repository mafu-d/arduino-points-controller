#include <Adafruit_PWMServoDriver.h>
#include <Easing.h>

// Identify this control unit
const int id = 1;

// Initialise the servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

class Servo
{
public:
  Servo(
      int pin, )
  {
    servoPin = pin;
    currentPosition = 350; // Default middle
    targetPosition = 350;
  }

  void set(
      int position,
      int type,
      int dur)
  {
    startTime = now() / 1000;
    duration = dur;
    targetPosition = position;
    easingType = type;
  }

private:
  int servoPin;
  int currentPosition;
  int targetPosition;
  int easingType;
  int duration;
  int startTime;
}

Servo *servos[16];

String decodeHead(msg)
{
  return msg.substring(0, 3);
}

int decodeBoardId(msg)
{
  return msg.substring(4, 5).toInt();
}

int decodeServoId(msg)
{
  return msg.substring(6, 8).toInt();
}

int decodePosition(msg)
{
  return msg.substring(9, 12).toInt();
}

int decodeEasing(msg)
{
  return msg.substring(13, 14).toInt();
}

int decodeSpeed(msg)
{
  return msg.substring(15, 19).toInt();
}

// Do this once
void setup()
{
  // Set up the servo driver board
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  // Initialise the serial monitor
  Serial.begin(9600);
}

// Keep doing this
void loop()
{
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Check the head
    if (decodeHead(command) != "SVO")
    {
      return;
    }
  }
}