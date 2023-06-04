#include <Adafruit_PWMServoDriver.h>
#include <ezButton.h>
#include <Arduino.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

class Servo {
public:
  Servo(
    String name,
    int servoPin,
    int buttonPin,
    int min,
    int max,
    float start,
    float delta)
    : name(name),
      servoPin(servoPin),
      servoMin(min),
      servoMax(max),
      delta(delta),
      current(start) {
    pwm.setPWM(servoPin, 0, (int)current);
    button = new ezButton(buttonPin);
    button->setDebounceTime(200);
    Serial.println(name + " ready");
  }

  void update() {
    button->loop();
    if (button->isPressed() && current < servoMax) {
      current += delta;
      pwm.setPWM(servoPin, 0, (int)current);
      if (!moving) {
        moving = true;
        Serial.println(name + " activating");
      }
    }
    if (button->isReleased() && current > servoMin) {
      current -= delta;
      pwm.setPWM(servoPin, 0, (int)current);
      if (!moving) {
        moving = true;
        Serial.println(name + " deactivating");
      }
    }
    if (moving && (current >= servoMax || current <= servoMin)) {
      moving = false;
      Serial.println(name + " finished moving");
    }
  }

private:
  String name;
  int servoPin;      // Which output on the servo controller board
  int servoMin;      // Minimum pulse length (150-600)
  int servoMax;      // Maximum pulse length (150-600)
  float delta;       // How much current changes on each iteration
  ezButton *button;  // Button
  float current;     // Current position
  bool moving = false;
};

Servo *servos[4];

void setup() {
  pwm.begin();
  Serial.begin(9600);
  servos[0] = new Servo("Servo 1", 0, 2, 300, 400, 300, 1);
  servos[1] = new Servo("Servo 2", 1, 3, 300, 400, 400, 2);
  servos[2] = new Servo("Servo 3", 2, 4, 300, 400, 250, 0.5);
  servos[3] = new Servo("Servo 4", 3, 5, 300, 400, 300, 0.1);
  delay(1000);  // Deep breath before we begin...
}

void loop() {
  for (int i = 0; i < 4; i++) {
    servos[i]->update();
  }
}
