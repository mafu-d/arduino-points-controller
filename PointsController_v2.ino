#include <Adafruit_PWMServoDriver.h>
#include <ezButton.h>
#include <Arduino.h>
#include <Easing.h>

// Initialise the servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/**
 * Servo class
 */
class Servo
{
public:
  // Constructor
  Servo(
      int id,
      String name,
      int servoPin,
      int buttonPin,
      int min,
      int max,
      int duration,
      bool isSignal)
      : id(id),
        servoPin(servoPin),
        servoMin(min),
        servoMax(max),
        duration(duration),
        current(servoMin),
        isSignal(isSignal)
  {
    // Reset servo position
    pwm.setPWM(servoPin, 0, current);
    // Initialise button
    button = new ezButton(buttonPin);
    button->setDebounceTime(350);
    // Set up easing functions
    easePosition = 0;
    easeSignal.duration(duration);
    easeSignal.scale(servoMax - servoMin);
    easePoint.duration(duration);
    easePoint.scale(servoMax - servoMin);
    isThrown = false;
    // Output to serial that this servo is ready
    // Serial.println((String)id + ": " + name + " ready: servo " + (String)servoPin + ", button " + (String)buttonPin);
  }

  // Update the servo's positions
  void update()
  {
    // Listen for a button press
    button->loop();

    if ((!button->getState() && (int)current == servoMin) || (button->getState() && (int)current == servoMax))
    {
      Serial.println(id);
    }

    int oldPosition = current;

    // If button is pressed and servo isn't at max position, move it forward a bit
    if (!button->getState() && (int)current < servoMax)
    {
      current = servoMin + getDelta();
      Serial.println("Setting " + (String)id + ": " + (int)current);
    }

    // If button is not pressed and servo isn't at min position, move it back a bit
    if (button->getState() && (int)current > servoMin)
    {
      current = servoMax - getDelta();
      Serial.println("Setting " + (String)id + ": " + (int)current);
    }

    // Set the servo position
    if (oldPosition != current)
    {
      pwm.setPWM(servoPin, 0, current);
    }

    // Output for serial monitor
    // Serial.print(current);
  }

  void updateBySerial()
  {
    int oldPosition = current;

    // If button is pressed and servo isn't at max position, move it forward a bit
    if (isThrown && (int)current < servoMax)
    {
      if (current == servoMin)
      {
        Serial.println("Throwing " + (String)id + "...");
      }
      current = servoMin + getDelta();
      if (oldPosition != current && current == servoMax)
      {
        Serial.println((String)id + " is thrown.");
      }
    }

    // If button is not pressed and servo isn't at min position, move it back a bit
    if (!isThrown && (int)current > servoMin)
    {
      if (current == servoMax)
      {
        Serial.println("Resetting " + (String)id + "...");
      }
      current = servoMax - getDelta();
      if (oldPosition != current && current == servoMin)
      {
        Serial.println((String)id + " is reset.");
      }
    }

    // Set the servo position
    if (oldPosition != current)
    {
      pwm.setPWM(servoPin, 0, current);
    }

    // Output for serial monitor
    // Serial.print(current);
  }

  int getId()
  {
    return id;
  }

  bool getIsThrown()
  {
    return isThrown;
  }

  void setIsThrown(bool state)
  {
    isThrown = state;
  }

  void getStatus()
  {
    Serial.print(" ID:");
    Serial.print(id);
    Serial.print(" Name:");
    Serial.print(name);
    Serial.print(" Current:");
    Serial.print(current);
    Serial.println();
  }

private:
  int id;                                 // The unique ID of the servo
  String name;                            // Display name of what the servo operates
  int servoPin;                           // Which output on the servo controller board
  int servoMin;                           // Minimum pulse length (150-600)
  int servoMax;                           // Maximum pulse length (150-600)
  int duration;                           // How long animation lasts
  ezButton *button;                       // Button
  bool isThrown;                          // Servo state for USB command
  float current;                          // Current servo position
  int easePosition;                       // Position in easing animation
  EasingFunc<Ease::BounceOut> easeSignal; // Easing function for signals
  EasingFunc<Ease::CubicInOut> easePoint; // Easing function for points
  bool isSignal;                          // Whether to animate as a signal or a point

  // Calculate how much to move the servo
  int getDelta()
  {
    // Reset animation position
    if (easePosition - 1 == duration)
    {
      easePosition = 0;
    }
    // Generate the new delta based on the easing function
    int delta = easePoint.get(easePosition);
    if (isSignal)
    {
      delta = easeSignal.get(easePosition);
    }
    // Increase the animation position
    easePosition++;

    return delta;
  }
};

// Set up an array of 9 uninstatiated Servo objects
Servo *servos[10];

// Whether to use serial control
bool useSerialControl;

// Do this once
void setup()
{
  // Set up the servo driver board
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  // Initialise the serial monitor
  Serial.begin(9600);
  Serial.println("Starting up...");
  // For my 9g servos the full 180 degree range is 100 to 500. YMMV.
  servos[0] = new Servo(100, "Point B: Yard Entrance", 0, 2, 370, 450, 50, false);
  servos[1] = new Servo(0, "Point C: Platform 2 / Loop", 1, 3, 310, 490, 75, false);
  servos[2] = new Servo(1, "Signal 1: Up Platform 1 Start", 2, 4, 250, 340, 75, true);
  servos[3] = new Servo(2, "Signal 2: Up Platform 2 Start", 3, 5, 270, 360, 75, true);
  servos[4] = new Servo(6, "Point 6: Platform 1 / Goods Siding", 5, 6, 330, 450, 75, false);
  servos[5] = new Servo(9, "Point 9: Platform 2 / Loop", 4, 7, 350, 480, 75, false);
  servos[6] = new Servo(14, "Point 14A: Main / Sidings", 6, 8, 280, 430, 75, false);
  servos[7] = new Servo(14, "Point 14B: Main / Sidings", 7, 8, 350, 520, 75, false);
  servos[8] = new Servo(16, "Signal 16: Down Platform 2 Inner Home", 8, 10, 295, 360, 75, true);
  servos[9] = new Servo(17, "Signal 17: Down Platform 1 Inner Home", 9, 11, 260, 380, 75, true);

  useSerialControl = true;
}

// Keep doing this
void loop()
{
  // Listen for command
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    Serial.println("Received message");
    command.trim();
    if (command.startsWith("SET_"))
    {
      int firstUnderscore = command.indexOf("_");
      int lastUnderscore = command.lastIndexOf("_");
      String id = command.substring(firstUnderscore + 1, lastUnderscore);
      String state = command.substring(lastUnderscore + 1);
      for (auto servo : servos)
      {
        if (servo->getId() == id.toInt())
        {
          Serial.println(servo->getId());
          servo->setIsThrown(state == "ON");
        }
      }
    }
    if (!useSerialControl && command.equals("SERIAL_CONTROL"))
    {
      useSerialControl = true;
    }
    // if (useSerialControl && command.equals("BUTTON_CONTROL")) {
    //   useSerialControl = false;
    // }
    if (command.equals("STATUS"))
    {
      for (auto servo : servos)
      {
        // Serial.println(servo->getId() + servo->getIsThrown());
        servo->getStatus();
      }
    }
  }

  // Update each servo
  for (auto servo : servos)
  {
    if (useSerialControl)
    {
      servo->updateBySerial();
    }
    else
    {
      servo->update();
    }
    // Separate serial outputs
    // Serial.print(" ");
  }
  // Serial.println();
  delay(15);
}
