
#if (ARDUINO &gt;= 100)
#include 
#else
#include 
#endif

#include 
#include 
// Pin variables for motors.
const int right_pwm_pin = 5;
const int right_dir_pin = A0;
const int left_pwm_pin = 6;
const int left_dir_pin = A1;
const bool left_fwd = true;
const bool right_fwd = false;

// Default_speed.
const int default_vel = 201;

ros::NodeHandle  nh;

void MoveFwd(const size_t speed) {
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  analogWrite(right_pwm_pin, speed);
  analogWrite(left_pwm_pin, speed);
}

void MoveStop() {
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  analogWrite(right_pwm_pin, 0);
  analogWrite(left_pwm_pin, 0);
}

void cmd_vel_cb(const geometry_msgs::Twist &amp; msg) {
  // Read the message. Act accordingly.
  // We only care about the linear x, and the rotational z.
  const float x = msg.linear.x;
  const float z_rotation = msg.angular.z;

  // Decide on the morot state we need, according to command.
  if (x &gt; 0 &amp;&amp; z_rotation == 0) {
    MoveFwd(default_vel);
  }
  else {
    MoveStop();
  }
}
ros::Subscriber sub("cmd_vel", cmd_vel_cb);
void setup() {
  pinMode(right_pwm_pin, OUTPUT);    // sets the digital pin 13 as output
  pinMode(right_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  // Set initial values for directions. Set both to forward.
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
