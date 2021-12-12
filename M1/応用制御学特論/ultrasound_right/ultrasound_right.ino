#include <SoftwareSerial.h>


#define TRIG 5
#define ECHO A0

#define temperature 22

double duration = 0;
double distance = 0;
double speed_of_sound = 331.5 + 0.6 * temperature;


//ローパスフィルタ（LiDAR）
float LPF_LiDAR;

//ローパスフィルタ（超音波センサ）
float LPF_ultra_left;
float LPF_ultra_right;



//LiDAR
SoftwareSerial Serial1(12,11);

float dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature; 
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package




void raw_pass_filter(float Distance, float k, float &lpf) //https://garchiving.com/lpf-by-program/
{
  float lastLPF=0.0;

  lpf = (1 - k) * lastLPF + k * Distance;
  lastLPF = lpf;

}



void ultrasound()
{
  digitalWrite( TRIG, HIGH );
  delayMicroseconds( 10 ); 
  digitalWrite( TRIG, LOW );
  duration = pulseIn( ECHO, HIGH ); // 往復にかかった時間が返却される[マイクロ秒]

  if (duration > 0) {
    duration = duration / 2; // 往路にかかった時間
    distance = duration * speed_of_sound * 100 / 1000000;
  }
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
          dist = float(uart[2]+uart[3]*256);
          
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







void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  pinMode(ECHO, INPUT );
  pinMode(TRIG, OUTPUT );
}

void loop() {
  
    LiDAR();
    raw_pass_filter(dist, 0.1, LPF_LiDAR);
    
    ultrasound();
    raw_pass_filter(distance, 0.1, LPF_ultra_right);


    Serial.print("LPF_LiDAR = ");
    Serial.print(LPF_LiDAR);
    Serial.print("\t");
    Serial.print("LPF_ultra_right:");
    Serial.print(LPF_ultra_right);
    Serial.print("\n");

}
