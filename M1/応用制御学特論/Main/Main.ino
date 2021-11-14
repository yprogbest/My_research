#include <SoftwareSerial.h>


//PID制御(https://kurobekoblog.com/pid)
#define target 30.0 //壁との距離 30cm
#define Kp 15.0 //比例ゲイン
#define Ki 50.0 //積分ゲイン
#define Kd 0.3 //微分ゲイン


//DCモータ
//右
#define PIN_RIGHT_IN1 7
#define PIN_RIGHT_IN2 8
#define PIN_RIGHT_VREF 9

//左
#define PIN_LEFT_IN1 6
#define PIN_LEFT_IN2 5
#define PIN_LEFT_VREF 10

//DCモータのスピード
int left_Speed;
int right_Speed;


//サーボモータ
int penguin=3;
String servo_direction; //サーボモータの向き


//LiDAR
SoftwareSerial Serial1(12,11);

int dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature; 
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package


int flont_distance = 300; //正面との距離
int side_distance = 150; //左右との距離



//PID制御
int duty = 0;
float dt, preTime;
float x;
float P, I, D, preP;


unsigned int count=0;



//前進
void foward(int left_speed, int right_speed)
{
  digitalWrite(PIN_RIGHT_IN1,HIGH);
  digitalWrite(PIN_RIGHT_IN2,LOW);
  digitalWrite(PIN_LEFT_IN1,HIGH);
  digitalWrite(PIN_LEFT_IN2,LOW);
  analogWrite(PIN_LEFT_VREF,left_speed);
  analogWrite(PIN_RIGHT_VREF,right_speed); 
}


//右
void right(int left_speed, int right_speed)
{
  digitalWrite(PIN_RIGHT_IN1,LOW);
  digitalWrite(PIN_RIGHT_IN2,HIGH);
  digitalWrite(PIN_LEFT_IN1,HIGH);
  digitalWrite(PIN_LEFT_IN2,LOW);
  analogWrite(PIN_LEFT_VREF,left_speed);
  analogWrite(PIN_RIGHT_VREF,right_speed); 
}

//左
void left(int left_speed, int right_speed)
{
  digitalWrite(PIN_RIGHT_IN1,HIGH);
  digitalWrite(PIN_RIGHT_IN2,LOW);
  digitalWrite(PIN_LEFT_IN1,LOW);
  digitalWrite(PIN_LEFT_IN2,HIGH);
  analogWrite(PIN_LEFT_VREF,left_speed);
  analogWrite(PIN_RIGHT_VREF,right_speed); 
}

//後ろ
void back(int left_speed, int right_speed)
{
  digitalWrite(PIN_RIGHT_IN1,LOW);
  digitalWrite(PIN_RIGHT_IN2,HIGH);
  digitalWrite(PIN_LEFT_IN1,LOW);
  digitalWrite(PIN_LEFT_IN2,HIGH);
  analogWrite(PIN_LEFT_VREF,left_speed);
  analogWrite(PIN_RIGHT_VREF,right_speed); 
}


//停止
int stop_(int left_speed, int right_speed)
{
  digitalWrite(PIN_RIGHT_IN1,LOW);
  digitalWrite(PIN_RIGHT_IN2,LOW);
  digitalWrite(PIN_LEFT_IN1,LOW);
  digitalWrite(PIN_LEFT_IN2,LOW);
  analogWrite(PIN_LEFT_VREF,left_speed);
  analogWrite(PIN_RIGHT_VREF,right_speed); 
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
        }
      }
    }
  }

  if(dist>800) dist=800;
  if(dist<0) dist=0;
}


void PID()
{
  x = (float)dist;
  
  dt = (micros() - preTime) / 1000000.0;
  preTime = micros();
  P  = target - x;
  I += P * dt;
  D  = (P - preP) / dt;
  preP = P;

  duty += Kp * P + Ki * I + Kd * D;

  if(duty > 800) duty=800;
  if(duty < 0) duty=0;
  
}




void setup() {
  // put your setup code here, to run once:
  //サーボモータ
  pinMode(penguin,OUTPUT);
  //タイヤ
  pinMode(PIN_RIGHT_IN1,OUTPUT); 
  pinMode(PIN_RIGHT_IN2,OUTPUT); 
  pinMode(PIN_LEFT_IN1,OUTPUT); 
  pinMode(PIN_LEFT_IN2,OUTPUT); 
  //LiDAR
  Serial.begin(9600);
  Serial1.begin(115200);

}


void loop() {

  

  if(count==0)
  {
    penDash(20); //0°
    servo_direction = "right";
    delay(5000);
  }

  LiDAR();
  
  if(servo_direction == "right") //もし，サーボが右を向いていたら，
  {
    if(dist > target) //30cm以上なら
    {
      foward(255, 230); //右に傾ける
    }
    else
    {
      foward(230, 255); //左に傾ける
    }

    //階段がある箇所での処理
    if(dist > side_distance) //もし，1.5mよりも距離が大きくなったら．
    {
      stop_(0,0);
      
      penDash(95); //正面を向く
      delay(2000);
      LiDAR();

      if(dist > flont_distance) //もし，正面に障害物が無いなら
      {
        penDash(190); //左を向く
        servo_direction = "left";
        delay(2000);
      }

    }
  }




  if(servo_direction == "left") //もし，サーボが左を向いていたら，
  {
    if(dist > target) //30cm以上なら
    {
      foward(230, 255); //左に傾ける
    }
    else
    {
      foward(255, 230); //右に傾ける
    }

    //階段がある箇所での処理
    if(dist > side_distance) //もし，1.5mよりも距離が大きくなったら．
    {
      stop_(0,0);
      
      penDash(95); //正面を向く
      delay(2000);
      LiDAR();
      if(dist > flont_distance) //もし，正面に障害物が無いなら
      {
        penDash(20); //右を向く
        servo_direction = "right";
        delay(2000);
      }
    }
  }



  count++;


  Serial.print("target = ");
  Serial.print(target);
  Serial.print("\t");
  Serial.print("dist = ");
  Serial.print(dist);
  Serial.print("\n");



  // penDash(20); //0°
  // delay(2000);
  // LiDAR();
  
  // penDash(95); //90°
  // delay(2000);
  // LiDAR();
  
  // penDash(190); //180°
  // delay(2000);
  // LiDAR();
  
  // penDash(95); //90°
  // delay(2000);
  // LiDAR();
  
  // penDash(20); //0°
  // delay(2000);
  // LiDAR();

  // stop();

  // while(1)
  // {
    
  // }

  
}
