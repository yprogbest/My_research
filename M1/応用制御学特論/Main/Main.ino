#include <SoftwareSerial.h>

//DCモータ
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

//LiDAR
SoftwareSerial Serial1(12,11);

int dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature; 
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package



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



//LiDAR
void LiDAR()
{
   if(Serial1.available())
  {
    if(Serial1.read()==HEADER)
    {
      uart[0]=HEADER;

      if(Serial1.read()==HEADER)
      {
        uart[1]=HEADER;

        for(i=2;i<9;i++)
        {
          uart[i]=Serial1.read();
        }

        check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];

        if(uart[8]==(check&0xff))
        {
          dist=uart[2]+uart[3]*256;
          
          strength=uart[4]+uart[5]*256;
          
          temprature=uart[6]+uart[7]*256;
          temprature=temprature/8-256;


          //シリアルモニタに表示

          // Serial.print("dist = ");
          // Serial.print(dist);
          // Serial.print("\t");
          // Serial.print("strength = ");
          // Serial.print(strength);
          // Serial.print("\t Chip Temprature = ");
          // Serial.print(temprature);
          // Serial.println(" celcius degree"); //output chip temperature of LiDAR


          //シリアルブロックに表示

          //Serial.print("dist = ");
          Serial.print(dist); //output measure distance value of LiDAR
          Serial.print(" ");
          //Serial.print("strength = ");
          Serial.print(strength); //output signal strength value
          //Serial.print("\t Chip Temprature = ");
          Serial.print(" ");
          Serial.print(temprature);
          Serial.println();
          //Serial.println(" celcius degree");
          
        }
      }
    }
  }
}




void setup() {
  // put your setup code here, to run once:
  pinMode(penguin,OUTPUT);

  pinMode(PIN_RIGHT_IN1,OUTPUT); 
  pinMode(PIN_RIGHT_IN2,OUTPUT); 
  pinMode(PIN_LEFT_IN1,OUTPUT); 
  pinMode(PIN_LEFT_IN2,OUTPUT); 


  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200);

}


void loop() {

  penDash(20); //0
  LiDAR();
  delay(2000);
  penDash(95); //90
  LiDAR();
  delay(2000);
  penDash(190); //180
  LiDAR();
  delay(2000);
  penDash(95); //90
  LiDAR();
  delay(2000);
  penDash(20); //0
  LiDAR();
  delay(2000);


  while(1)
  {
    
  }
}
