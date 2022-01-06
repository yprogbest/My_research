#include <Servo.h>

Servo penguin;

//https://miraluna.hatenablog.com/entry/penguin
void setup() {
  // put your setup code here, to run once:
  penguin.attach(3);
}

void loop() {

  
 // put your main code here, to run repeatedly:
  //penguin.write(0);//0度の位置まで回転
  //delay(1000);
  penguin.write(55);//120度まで
  delay(1000);
  penguin.write(145);
  delay(1000);
  //penguin.write(90);//120度まで
  //delay(1000);
  //penguin.write(0);//0度の位置まで回転
  
  
  while(1){

  }
}
