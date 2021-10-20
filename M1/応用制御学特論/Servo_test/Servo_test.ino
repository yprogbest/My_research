#include <Servo.h>

Servo penguin;

//https://miraluna.hatenablog.com/entry/penguin
void setup() {
  // put your setup code here, to run once:
  penguin.attach(3);
}

void loop() {
  // put your main code here, to run repeatedly:
  penguin.write(0);//0度の位置まで回転
  delay(400);
  penguin.write(120);//120度まで
}
