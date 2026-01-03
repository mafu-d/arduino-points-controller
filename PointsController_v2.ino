#include <Adafruit_PWMServoDriver.h>
#include <ezButton.h>
#include <Arduino.h>
#include <Easing.h>

const int layoutId = 0; // Frontington
// const int layoutId = 1; // Fiddle yard
// const int layoutId = 2; // Tutherside

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

// Set up an array of 16 uninstatiated Servo objects
// It's 16 because the PCA9685 has 16 channels
Servo *servos[16];
int servoCount = 0;

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
  switch (layoutId)
  {
  case 0: // Frontington
    servos[servoCount++] = new Servo(0, "Point 0: Platform 2 / Loop", 1, 3, 310, 490, 75, false);
    servos[servoCount++] = new Servo(1, "Signal 1: Up Platform 1 Start", 2, 4, 250, 340, 75, true);
    servos[servoCount++] = new Servo(2, "Signal 2: Up Platform 2 Start", 3, 5, 270, 360, 75, true);
    servos[servoCount++] = new Servo(6, "Point 6: Platform 1 / Goods Siding", 5, 6, 330, 450, 75, false);
    servos[servoCount++] = new Servo(9, "Point 9: Platform 2 / Loop", 4, 7, 350, 480, 75, false);
    servos[servoCount++] = new Servo(14, "Point 14A: Main / Sidings", 6, 8, 280, 430, 75, false);
    servos[servoCount++] = new Servo(14, "Point 14B: Main / Sidings", 7, 8, 350, 520, 75, false);
    servos[servoCount++] = new Servo(16, "Signal 16: Down Platform 2 Inner Home", 8, 10, 295, 360, 75, true);
    servos[servoCount++] = new Servo(17, "Signal 17: Down Platform 1 Inner Home", 9, 11, 260, 380, 75, true);
    break;

  case 1: // Fiddle yard
    servos[servoCount++] = new Servo(1, "Point 1: Yard Tutherside end", 1, 3, 100, 500, 75, false);
    servos[servoCount++] = new Servo(2, "Point 2: Yard Frontington end", 2, 4, 100, 500, 75, false);
    servos[servoCount++] = new Servo(3, "Point 3: Yard / Main line", 3, 5, 100, 500, 75, false);
    servos[servoCount++] = new Servo(4, "Point 4: Main line / loop", 4, 6, 100, 500, 75, false);
    break;

  case 2: // Tutherside
    servos[servoCount++] = new Servo(1, "Signal 1: Up Platform 1 Start", 1, 3, 100, 500, 75, true);
    servos[servoCount++] = new Servo(2, "Signal 2: Up Platform 2 Start", 2, 4, 100, 500, 75, true);
    servos[servoCount++] = new Servo(3, "Signal 3: Up Frontington Advance Start", 3, 5, 100, 500, 75, true);
    servos[servoCount++] = new Servo(4, "Signal 4: Up Garth Bridge Advance Start", 4, 6, 100, 500, 75, true);
    servos[servoCount++] = new Servo(7, "Point 7A: Platform 2 headshunt", 5, 7, 100, 500, 75, false);
    servos[servoCount++] = new Servo(7, "Point 7B: Loop headshunt", 6, 8, 100, 500, 75, false);
    servos[servoCount++] = new Servo(10, "Point 10: Loop / Goods siding", 7, 9, 100, 500, 75, false);
    servos[servoCount++] = new Servo(13, "Point 13A: Loop / Headshunt", 8, 10, 100, 500, 75, false);
    servos[servoCount++] = new Servo(13, "Point 13B: Platform 2 / Loop", 9, 11, 100, 500, 75, false);
    servos[servoCount++] = new Servo(15, "Point 15: Platform 2 / Platform 1", 10, 12, 100, 500, 75, false);
    servos[servoCount++] = new Servo(17, "Point 17: Garth Bridge / Frontington", 11, 13, 100, 500, 75, false);
    servos[servoCount++] = new Servo(22, "Signal 22: Down Platform 1 Inner Home", 12, 14, 100, 500, 75, true);
    servos[servoCount++] = new Servo(23, "Signal 23: Down Platform 2 Inner Home", 13, 15, 100, 500, 75, true);
    break;

  default:
    break;
  }

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
      for (int i = 0; i < servoCount; i++)
      {
        if (servos[i]->getId() == id.toInt())
        {
          Serial.println(servos[i]->getId());
          servos[i]->setIsThrown(state == "ON");
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
      for (int i = 0; i < servoCount; i++)
      {
        // Serial.println(servo->getId() + servo->getIsThrown());
        servos[i]->getStatus();
      }
    }
  }

  // Update each servo
  for (int i = 0; i < servoCount; i++)
  {
    if (useSerialControl)
    {
      servos[i]->updateBySerial();
    }
    else
    {
      servos[i]->update();
    }
    // Separate serial outputs
    // Serial.print(" ");
  }
  // Serial.println();
  delay(15);
}
