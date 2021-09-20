#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define LeftMotorDirPin2 8
#define LeftMotorDirPin1 7
#define speedPinL 6
#define RightMotorDirPin2 11
#define RightMotorDirPin1 12
#define speedPinR 9

//default speed 
const int default_vel = 255;


void go_Forward(const size_t speed)
{
  digitalWrite(RightMotorDirPin1,HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(RightMotorDirPin1,HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  analogWrite(speedPinL, speed);
  analogWrite(speedPinR, speed);
}

void go_Back(const size_t speed)  //Reverse
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR,speed);
}

void go_Left(const size_t speed)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR, speed);
}

void go_Forward_Left(const size_t speed)
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR,speed);
}

void go_Back_Left(const size_t speed)
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR,speed);
}

void go_Right(const size_t speed)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR,speed);
}

void go_Forward_Right(const size_t speed)
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR,speed);
}

void go_Back_Right(const size_t speed)
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
  analogWrite(speedPinR,speed);
}

void Stop() {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
}


void cmd_vel_cb(const geometry_msgs::Twist &msg)
{
  const float x = msg.linear.x;
  const float z_rotation = msg.angular.z;

  if(x < 0.0 && z_rotation == 0.0)
  {
    // Go forward
    go_Forward(default_vel);
  }
  else if(x > 0.0 && z_rotation == 0.0)
  {
    // Go back
    go_Back(default_vel);
  }
  else if(z_rotation < 0.0)
  {
    if(x == 0.0)
    {
      //Turn left
      go_Left(default_vel);
    }
    else if(x < 0.0)
    {
      // Go left forward
      go_Forward_Left(default_vel);
    }
    else
    {
      // Go right back
      go_Back_Right(default_vel);
    }
  }
  else if(z_rotation > 0.0)
  {
    if(x == 0.0)
    {
      // Turn right
      go_Right(default_vel);
    }
    else if(x < 0.0)
    {
      // Go right forward
      go_Forward_Right(default_vel);
    }
    else
    {
      // Go left back
      go_Back_Left(default_vel);
    }
  }
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_cb);

void setup() {
  // put your setup code here, to run once:
  pinMode(RightMotorDirPin1, OUTPUT); 
	pinMode(RightMotorDirPin2, OUTPUT); 
	pinMode(speedPinL, OUTPUT);  
	pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
