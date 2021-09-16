// define USE_USBCON before including ros.h to use atmega32u4
//#define USE_USBCON

#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define DETECT_LOST_CONNECTION_MS 3000
#define MOTOR_L_B 8
#define MOTOR_L_F 7
#define MOTOR_L_PWM 6
#define MOTOR_R_B 11
#define MOTOR_R_F 12
#define MOTOR_R_PWM 9
//#define MOTOR_STBY 10
#define MOTOR_MAX_VALUE 1500

float motor_value_rate = 0.1;

unsigned long lastCommunicatedAt = 0;
unsigned long detectLostAt = 0;
String currentMotorL = "";
String currentMotorR = "";

void updateLastCommunicatedAt() {
  lastCommunicatedAt = millis();
  detectLostAt = lastCommunicatedAt + (unsigned long) DETECT_LOST_CONNECTION_MS;
}

void setMotorSpeed(String value, int b_pin, int f_pin, int pwm_pin) {
  if (value == "b" || value == "brake") {
    digitalWrite(f_pin, HIGH);
    digitalWrite(b_pin, HIGH);
    analogWrite(pwm_pin, 0);
    return;
  }
  int intValue = value.toInt();
  if (intValue == 0) {
    digitalWrite(f_pin, LOW);
    digitalWrite(b_pin, LOW);
  } else if (intValue > 0) {
    digitalWrite(f_pin, HIGH);
    digitalWrite(b_pin, LOW);
  } else {
    digitalWrite(f_pin, LOW);
    digitalWrite(b_pin, HIGH);
  }
  analogWrite(pwm_pin, abs(intValue));
}

void setMotorsSpeed(String left, String right) {
  //digitalWrite(MOTOR_STBY, HIGH);
  setMotorSpeed(left, MOTOR_L_B, MOTOR_L_F, MOTOR_L_PWM);
  setMotorSpeed(right, MOTOR_R_B, MOTOR_R_F, MOTOR_R_PWM);
}

void sleepMotors() {
  setMotorsSpeed("0", "0");
  //digitalWrite(MOTOR_STBY, LOW);
}

void messageCb(const geometry_msgs::Twist& twist) {
  updateLastCommunicatedAt();
  const float linear_x = twist.linear.x;
  const float angle_z = twist.angular.z;
  float speed = MOTOR_MAX_VALUE * motor_value_rate;
  String plusSpeedStr = String(speed);
  String minusSpeedStr = String(-speed);
  String newMotorL = "0";
  String newMotorR = "0";
  if (linear_x > 0.0 && angle_z == 0.0) {
    // Go forward
    newMotorL = plusSpeedStr;
    newMotorR = plusSpeedStr;
  } else if (linear_x < 0.0 && angle_z == 0.0) {
    // Go back
    newMotorL = minusSpeedStr;
    newMotorR = minusSpeedStr;
  } else if (angle_z > 0.0) {
    if (linear_x == 0.0) {
      // Turn left
      newMotorL = minusSpeedStr;
      newMotorR = plusSpeedStr;
    } else if (linear_x > 0.0) {
      // Go left forward
      newMotorR = plusSpeedStr;
    } else {
      // Go right back
      newMotorL = minusSpeedStr;
    }
  } else if (angle_z < 0.0) {
    if (linear_x == 0.0) {
      // Turn right
      newMotorL = plusSpeedStr;
      newMotorR = minusSpeedStr;
    } else if (linear_x > 0.0) {
      // Go right forward
      newMotorL = plusSpeedStr;
    } else {
      // Go left back
      newMotorR = minusSpeedStr;
    }
  }
  if (currentMotorL == newMotorL && currentMotorR == newMotorR) {
    return;
  }
  if ((newMotorL == "0" && newMotorR == "0") || (newMotorL == "0.0" && newMotorR == "0.0")) {
    sleepMotors();
  } else {
    setMotorsSpeed(newMotorL, newMotorR);
  }
  currentMotorL = newMotorL;
  currentMotorR = newMotorR;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

void setup() {
  pinMode(MOTOR_L_B, OUTPUT);
  pinMode(MOTOR_L_F, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_R_B, OUTPUT);
  pinMode(MOTOR_R_F, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  //pinMode(MOTOR_STBY, OUTPUT);
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  if (detectLostAt < millis() && currentMotorL != "" && currentMotorR != "") {
    currentMotorL = "";
    currentMotorR = "";
    sleepMotors();
  }
  delay(1);
}
