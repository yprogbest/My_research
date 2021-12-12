int penguin=3;

void setup() 
{
  pinMode(penguin,OUTPUT);
}

void loop() 
{
    penDash(20); //0
    delay(2000);
    //penDash(95); //90
    //delay(2000);
    //penDash(190); //180
    //delay(2000);
    //penDash(95); //90
    //delay(2000);
    //penDash(20); //0
    //delay(2000);
    
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
