int penguin=3;

void setup() 
{
  pinMode(penguin,OUTPUT);
}

void loop() 
{
  for(int i=0;i<=180;i++)
  {//0°から180°へ回転。
    penDash(i);
  }
  for(int j=180;j>0;j--)
  {//180°から0°へ回転。
    penDash(j);
  }


    while(1)
  {
    
  }
}


void penDash(int x)
{//xの値は0~180。
  int kyori = (x*10.25)+450;//角度からパルス幅への変換式
  digitalWrite(penguin,HIGH);
  delayMicroseconds(kyori);
  digitalWrite(penguin,LOW);
  delay(5);//速度　5~30くらいが良好。
}