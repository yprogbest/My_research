


//右
#define PIN_RIGHT_IN1 7
#define PIN_RIGHT_IN2 8
#define PIN_RIGHT_VREF 9

//左
#define PIN_LEFT_IN1 6
#define PIN_LEFT_IN2 5
#define PIN_LEFT_VREF 10

//サーボモータ
int penguin=3;


//DCモータのスピード
int Speed = 127;


//前進
void foward(int speed)
{
  digitalWrite(PIN_RIGHT_IN1,HIGH);
  digitalWrite(PIN_RIGHT_IN2,LOW);
  digitalWrite(PIN_LEFT_IN1,HIGH);
  digitalWrite(PIN_LEFT_IN2,LOW);
  analogWrite(PIN_RIGHT_VREF,speed); 
  analogWrite(PIN_LEFT_VREF,speed);
}


//右
void right(int speed)
{
  digitalWrite(PIN_RIGHT_IN1,LOW);
  digitalWrite(PIN_RIGHT_IN2,HIGH);
  digitalWrite(PIN_LEFT_IN1,HIGH);
  digitalWrite(PIN_LEFT_IN2,LOW);
  analogWrite(PIN_RIGHT_VREF,speed); 
  analogWrite(PIN_LEFT_VREF,speed);
}

//左
void left(int speed)
{
  digitalWrite(PIN_RIGHT_IN1,HIGH);
  digitalWrite(PIN_RIGHT_IN2,LOW);
  digitalWrite(PIN_LEFT_IN1,LOW);
  digitalWrite(PIN_LEFT_IN2,HIGH);
  analogWrite(PIN_RIGHT_VREF,speed); 
  analogWrite(PIN_LEFT_VREF,speed);
}

//後ろ
void back(int speed)
{
  digitalWrite(PIN_RIGHT_IN1,LOW);
  digitalWrite(PIN_RIGHT_IN2,HIGH);
  digitalWrite(PIN_LEFT_IN1,LOW);
  digitalWrite(PIN_LEFT_IN2,HIGH);
  analogWrite(PIN_RIGHT_VREF,speed); 
  analogWrite(PIN_LEFT_VREF,speed);
}


//停止
int stop()
{
  digitalWrite(PIN_RIGHT_IN1,LOW);
  digitalWrite(PIN_RIGHT_IN2,LOW);
  digitalWrite(PIN_LEFT_IN1,LOW);
  digitalWrite(PIN_LEFT_IN2,LOW);
}



//サーボモータ
void penDash(int x)
{//xの値は0~180。
  int kyori = (x*10.25)+450;//角度からパルス幅への変換式
  digitalWrite(penguin,HIGH);
  delayMicroseconds(kyori);
  digitalWrite(penguin,LOW);
  delay(5);//速度　5~30くらいが良好。
}





void setup() {
  // put your setup code here, to run once:
  pinMode(penguin,OUTPUT);

  pinMode(PIN_RIGHT_IN1,OUTPUT); 
  pinMode(PIN_RIGHT_IN2,OUTPUT); 
  pinMode(PIN_LEFT_IN1,OUTPUT); 
  pinMode(PIN_LEFT_IN2,OUTPUT); 
}


void loop() {

  foward(Speed);

  penDash(20); //0
  delay(2000);
  penDash(95); //90
  delay(2000);
  penDash(190); //180
  delay(2000);
  penDash(95); //90
  delay(2000);
  penDash(20); //0
  delay(2000);


  stop();

  while(1)
  {

  }

}
