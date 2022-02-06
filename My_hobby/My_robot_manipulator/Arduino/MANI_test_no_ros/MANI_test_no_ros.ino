#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 150
#define SERVOMAX 600
#define CENTER 375

int Servo_Pin = 5;


void setup() {
  // put your setup code here, to run once:
  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop() {
  // put your main code here, to run repeatedly:
  int angle = 180;
  angle = map(angle,0,180, SERVOMIN, SERVOMAX);
  pwm.setPWM(Servo_Pin,0,angle);
  delay(1000);
}
