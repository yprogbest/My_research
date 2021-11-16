#include <SoftwareSerial.h>

//LiDAR
SoftwareSerial Serial1(12,11);

float dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
int temprature; 
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package


//PID制御(https://kurobekoblog.com/pid)
#define target 40.0 //壁との距離 30cm
#define Kp 5.0 //比例ゲイン
#define Ki 0.0 //積分ゲイン
#define Kd 0.0 //微分ゲイン

int U;
int duty = 0;
float dt, preTime;
float x;
float P, I, D, preP;




//ローパスフィルタ
float LPF=0;
float lastLPF=0;
float k = 0.1;





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
          dist=float(uart[2]+uart[3]*256);
          
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


void raw_pass_filter() //https://garchiving.com/lpf-by-program/
{
  LPF = (1 - k) * lastLPF + k * (float)dist;
  lastLPF = LPF;
}


void PID(float Distance)
{
  x = Distance;
  
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
  //LiDAR
  Serial.begin(9600);
  Serial1.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  LiDAR();
  raw_pass_filter();
  
  PID(LPF);

  Serial.print("target = ");
  Serial.print(target);
  Serial.print("\t");
  Serial.print("dist = ");
  Serial.print(dist);
  Serial.print("\t");
  Serial.print("duty = ");
  Serial.print(duty);
  Serial.print("\n");
}
