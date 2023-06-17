#include <Adafruit_PWMServoDriver.h>
#include <ezButton.h>
#include <Arduino.h>

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
    float delta)
    : name(name),
      servoPin(servoPin),
      servoMin(min),
      servoMax(max),
      delta(delta),
      current(servoMin) {
    pwm.setPWM(servoPin, 0, current);
    button = new ezButton(buttonPin);
    button->setDebounceTime(50);
    Serial.println(name + " ready: servo " + servoPin + ", button " + buttonPin);
  }

  // Update the servo's positions
  void update() {
    // Listen for a button press
    button->loop();

    // If button is pressed and servo isn't at max position, move it forward a bit
    if (!button->getState() && current < servoMax) {
      current = current + delta;
      pwm.setPWM(servoPin, 0, current);
      if (!moving) {
        moving = true;
        Serial.println(name + " activating");
      }
    }

    // If button is not pressed and servo isn't at min position, move it back a bit
    if (button->getState() && current > servoMin) {
      current = current - delta;
      pwm.setPWM(servoPin, 0, current);
      if (!moving) {
        moving = true;
        Serial.println(name + " deactivating " + current);
      }
    }

    // If we've reached min or max, stop
    if (moving && (current >= servoMax || current <= servoMin)) {
      moving = false;
      Serial.println(name + " finished moving, now at " + current);
    }
  }

private:
  String name;          // Display name of what the servo operates
  int servoPin;         // Which output on the servo controller board
  int servoMin;         // Minimum pulse length (150-600)
  int servoMax;         // Maximum pulse length (150-600)
  float delta;          // How much current changes on each iteration
  ezButton *button;     // Button
  float current;        // Current position
  bool moving = false;  // Whether the servo is currently moving
};

// Set up an array of 9 uninstatiated Servo objects
Servo *servos[9];

// Do this once
void setup() {
  // Set up the servo driver board
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  // Initialise the serial monitor
  Serial.begin(9600);
  // For my 9g servos the full 180 degree range is 100 to 500. YMMV.
  servos[0] = new Servo("Point B: Yard Entrance", 0, 2, 300, 500, 1);
  servos[1] = new Servo("Point C: Platform 2 / Loop", 1, 3, 315, 490, 1.5);
  servos[2] = new Servo("Signal 1: Up Platform 1 Start", 2, 4, 230, 350, 1);
  servos[3] = new Servo("Signal 2: Up Platform 2 Start", 3, 5, 300, 380, 1);
  servos[4] = new Servo("Point 6: Platform 1 / Goods Siding", 4, 6, 300, 500, 1);
  servos[5] = new Servo("Point 9: Platform 2 / Loop", 5, 7, 300, 500, 1);
  servos[6] = new Servo("Point 14: Main / Sidings", 6, 8, 300, 500, 1);
  servos[7] = new Servo("Signal 16: Down Platform 2 Inner Home", 7, 9, 300, 500, 1);
  servos[8] = new Servo("Signal 17: Down Platform 1 Inner Home", 8, 10, 300, 500, 1);

  // Deep breath before we begin...
  delay(2000);
}

// Keep doing this
void loop() {
  // Update each servo
  for (auto servo : servos) {
    servo->update();
  }

  // Slow everything down a bit
  delay(10);
}
