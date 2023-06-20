#include <Adafruit_PWMServoDriver.h>
#include <ezButton.h>
#include <Arduino.h>
#include <Easing.h>

// Initialise the servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/**
* Servo class
*/
class Servo {
public:
  // Constructor
  Servo(
    String name,
    int servoPin,
    int buttonPin,
    int min,
    int max,
    int duration,
    bool isSignal)
    : name(name),
      servoPin(servoPin),
      servoMin(min),
      servoMax(max),
      duration(duration),
      current(servoMin),
      isSignal(isSignal) {
    // Reset servo position
    pwm.setPWM(servoPin, 0, current);
    // Initialise button
    button = new ezButton(buttonPin);
    button->setDebounceTime(150);
    // Set up easing functions
    easePosition = 0;
    easeSignal.duration(duration);
    easeSignal.scale(servoMax - servoMin);
    easePoint.duration(duration);
    easePoint.scale(servoMax - servoMin);
    // Output to serial that this servo is ready
    Serial.println(name + " ready: servo " + servoPin + ", button " + buttonPin);
  }

  // Update the servo's positions
  void update() {
    // Listen for a button press
    button->loop();

    int oldPosition = current;

    // If button is pressed and servo isn't at max position, move it forward a bit
    if (!button->getState() && (int)current < servoMax) {
      current = servoMin + getDelta();
    }

    // If button is not pressed and servo isn't at min position, move it back a bit
    if (button->getState() && (int)current > servoMin) {
      current = servoMax - getDelta();
    }

    // Set the servo position
    if (oldPosition != current) {
      pwm.setPWM(servoPin, 0, current);
    }

    // Output for serial monitor
    Serial.print(current);
  }

private:
  String name;                             // Display name of what the servo operates
  int servoPin;                            // Which output on the servo controller board
  int servoMin;                            // Minimum pulse length (150-600)
  int servoMax;                            // Maximum pulse length (150-600)
  int duration;                            // How long animation lasts
  ezButton *button;                        // Button
  float current;                           // Current servo position
  int easePosition;                        // Position in easing animation
  EasingFunc<Ease::BounceOut> easeSignal;  // Easing function for signals
  EasingFunc<Ease::CubicInOut> easePoint;  // Easing function for points
  bool isSignal;                           // Whether to animate as a signal or a point

  // Calculate how much to move the servo
  int getDelta() {
    // Reset animation position
    if (easePosition - 1 == duration) {
      easePosition = 0;
    }
    // Generate the new delta based on the easing function
    int delta = easePoint.get(easePosition);
    if (isSignal) {
      delta = easeSignal.get(easePosition);
    }
    // Increase the animation position
    easePosition++;

    return delta;
  }
};

// Set up an array of 9 uninstatiated Servo objects
Servo *servos[10];

// Do this once
void setup() {
  // Set up the servo driver board
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  // Initialise the serial monitor
  Serial.begin(38400);
  // For my 9g servos the full 180 degree range is 100 to 500. YMMV.
  servos[0] = new Servo("Point B: Yard Entrance", 0, 2, 300, 500, 100, false);
  servos[1] = new Servo("Point C: Platform 2 / Loop", 1, 3, 315, 490, 150, false);
  servos[2] = new Servo("Signal 1: Up Platform 1 Start", 2, 4, 230, 350, 100, true);
  servos[3] = new Servo("Signal 2: Up Platform 2 Start", 3, 5, 270, 360, 100, true);
  servos[4] = new Servo("Point 6: Platform 1 / Goods Siding", 4, 6, 300, 500, 100, false);
  servos[5] = new Servo("Point 9: Platform 2 / Loop", 5, 7, 300, 500, 100, false);
  servos[6] = new Servo("Point 14: Main / Sidings", 6, 8, 300, 500, 100, false);
  servos[7] = new Servo("Point 14: Main / Sidings", 7, 8, 300, 500, 100, false);
  servos[8] = new Servo("Signal 16: Down Platform 2 Inner Home", 8, 10, 300, 500, 100, true);
  servos[9] = new Servo("Signal 17: Down Platform 1 Inner Home", 9, 11, 300, 380, 100, true);

  // Deep breath before we begin...
  delay(1000);
}

// Keep doing this
void loop() {
  // Update each servo
  for (auto servo : servos) {
    servo->update();
    // Separate serial outputs
    Serial.print(" ");
  }
  Serial.println();
}
