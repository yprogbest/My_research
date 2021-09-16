#include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>
#include <PCA9685.h>            //PCA9685用ヘッダーファイル（秋月電子通商作成）

//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定（アドレスジャンパ未接続時）
//PCA9685 pwm1 = PCA9685(0x41);


#define SERVOMIN 150
#define SERVOMAX 600
#define CENTER 375


byte val = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(60);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){
    val = Serial.read();
  }

  if(val == 'a'){
    int angle = 180;
    angle = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(0, 0, angle);
  }else if(val == '0'){
    int angle = 0;
    angle = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(0, 0, angle);
  }
  
}
