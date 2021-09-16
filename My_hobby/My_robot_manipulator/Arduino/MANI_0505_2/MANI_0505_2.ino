#include <Wire.h>
//#include <PCA9685.h> 
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
// #include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 600
#define CENTER 375


void messageCb(const std_msgs::UInt16MultiArray& msg){
  double angle0 = msg.data[0];
  double angle1 = msg.data[1];
  double angle2 = msg.data[2];
  double angle3 = msg.data[3];
  double angle4 = msg.data[4];
  double angle5 = msg.data[5];



  angle0 = map(angle0, 0, 180, SERVOMIN, SERVOMAX);
  angle0 = angle0*(5.0/6.0); //180 ==> 150
  pwm.setPWM(0, 0, angle0);

  angle1 = map(angle1, 0, 180, SERVOMIN, SERVOMAX);
  angle1 = angle1*(5.0/6.0); //180 ==> 150
  pwm.setPWM(1, 0, angle1);

  angle2 = map(angle2, 0, 180, SERVOMIN, SERVOMAX);
  angle2 = angle2*(5.0/6.0); //180 ==> 150
  pwm.setPWM(2, 0, angle2);

  angle3 = map(angle3, 0, 180, SERVOMIN, SERVOMAX);
  angle3 = angle3*(5.0/6.0); //180 ==> 150
  pwm.setPWM(3, 0, angle3);

  angle4 = map(angle4, 0, 180, SERVOMIN, SERVOMAX);
  angle4 = angle4*(5.0/6.0); //180 ==> 150
  pwm.setPWM(4, 0, angle4);

  angle5 = map(angle5, 0, 180, SERVOMIN, SERVOMAX);
  angle5 = angle5*(5.0/6.0); //180 ==> 150
  pwm.setPWM(5, 0, angle5);
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("pwm", messageCb);

void  setup(){
  pwm.begin();
  pwm.setPWMFreq(50);
  nh.initNode();
  nh.subscribe(sub);
  Wire.setClock(400000);
}


void loop(){
  nh.spinOnce();
}
