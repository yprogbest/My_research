#include <SoftwareSerial.h>
SoftwareSerial Serial1(2,3);

int dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature; 
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
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

          // Serial.print("dist = ");
          // Serial.print(dist);
          // Serial.print("\t");
          // Serial.print("strength = ");
          // Serial.print(strength);
          // Serial.print("\t Chip Temprature = ");
          // Serial.print(temprature);
          // Serial.println(" celcius degree"); //output chip temperature of LiDAR


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
