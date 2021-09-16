#include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>
#include <PCA9685.h>            //PCA9685用ヘッダーファイル（秋月電子通商作成）

//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定（アドレスジャンパ未接続時）

#define SERVOMIN 150
#define SERVOMAX 600
#define CENTER 375

void setup() {
  // put your setup code here, to run once:
  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop() {
  // put your main code here, to run repeatedly:
  int angle = 90;
  angle = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0, 0, angle);
}
