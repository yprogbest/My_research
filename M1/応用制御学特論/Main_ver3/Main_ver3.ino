//参考記事
//P制御(https://monozukuri-c.com/mbase-pcontrol/)


#include <SoftwareSerial.h>

// https://garchiving.com/pwm-control-with-arduino/
// https://qiita.com/thorikawa/items/a6377d2d4b4535dd9004
#include <Servo.h>

Servo penguin;


//曲がり角の処理
#define wall_distant  250 // cm

// int corner_count = 0;
int flag_right=0;
unsigned long int frame_count = 0;

int left_servo_count = 0;
int right_servo_count = 0;


//サーボモータの傾き
#define servo_right_curve  35 //45°付近
#define servo_left_curve  147 //135°付近


//PID制御(https://kurobekoblog.com/pid)

#define PID_taget 0.0 //PIDの目標値

#define target 100.0 //壁との距離 100cm

#define Kp 2.5 //比例ゲイン
#define Ki 0.0 //積分ゲイン
#define Kd 0.0 //微分ゲイン


//DCモータ
//右
#define PIN_RIGHT_IN1 7
#define PIN_RIGHT_IN2 8
#define PIN_RIGHT_VREF 5

//左
#define PIN_LEFT_IN1 4
#define PIN_LEFT_IN2 2
#define PIN_LEFT_VREF 6

//DCモータのスピード
int left_Speed;
int right_Speed;

//最大と最小のスピードを指定
// #define High_Speed 255
// #define Low_Speed 220
#define High_Speed 195
#define Low_Speed 150


//サーボモータ
// #define penguin 9
String servo_direction; //サーボモータの向き
unsigned long currentTime, loopTimer;


//LiDAR
SoftwareSerial Serial1(12, 11);
float dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature;
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER = 0x59; //frame header of data package



//右（Right）超音波センサ
// #define TRIG_right 5
// #define ECHO_right A0


//左（Left）超音波センサ
// #define TRIG_left 6
// #define ECHO_left 13


//気温（超音波センサの計測で使用）
// #define ultra_temp 14.6


// //超音波センサ
// double duration_left = 0;
// double dist_ultra_left = 0;
// double speed_of_sound_left = 331.5 + 0.6 * ultra_temp;


// double duration_right = 0;
// double dist_ultra_right = 0;
// double speed_of_sound_right = 331.5 + 0.6 * ultra_temp;



// //ローパスフィルタ（LiDAR）
// float LPF_LiDAR;

// //ローパスフィルタ（超音波センサ）
// float LPF_ultra_left;
// float LPF_ultra_right;




// //PID制御
// int duty = 0;
// float dt, preTime;
// float x;
// float P, I, D, preP;



//前進
void foward(int left_speed, int right_speed)
{
  digitalWrite(PIN_RIGHT_IN1, HIGH);
  digitalWrite(PIN_RIGHT_IN2, LOW);
  digitalWrite(PIN_LEFT_IN1, HIGH);
  digitalWrite(PIN_LEFT_IN2, LOW);
  analogWrite(PIN_LEFT_VREF, left_speed);
  analogWrite(PIN_RIGHT_VREF, right_speed);
}


//右
void right(int left_speed, int right_speed)
{
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, HIGH);
  digitalWrite(PIN_LEFT_IN1, HIGH);
  digitalWrite(PIN_LEFT_IN2, LOW);
  analogWrite(PIN_LEFT_VREF, left_speed);
  analogWrite(PIN_RIGHT_VREF, right_speed);
}

//左
void left(int left_speed, int right_speed)
{
  digitalWrite(PIN_RIGHT_IN1, HIGH);
  digitalWrite(PIN_RIGHT_IN2, LOW);
  digitalWrite(PIN_LEFT_IN1, LOW);
  digitalWrite(PIN_LEFT_IN2, HIGH);
  analogWrite(PIN_LEFT_VREF, left_speed);
  analogWrite(PIN_RIGHT_VREF, right_speed);
}

//後ろ
void back(int left_speed, int right_speed)
{
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, HIGH);
  digitalWrite(PIN_LEFT_IN1, LOW);
  digitalWrite(PIN_LEFT_IN2, HIGH);
  analogWrite(PIN_LEFT_VREF, left_speed);
  analogWrite(PIN_RIGHT_VREF, right_speed);
}


//停止
int stop_(int left_speed, int right_speed)
{
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, LOW);
  digitalWrite(PIN_LEFT_IN1, LOW);
  digitalWrite(PIN_LEFT_IN2, LOW);
  analogWrite(PIN_LEFT_VREF, left_speed);
  analogWrite(PIN_RIGHT_VREF, right_speed);
}



//サーボモータ
// void penDash(int x)
// { //xの値は0~180。
//   int kyori = (x * 10.25) + 450; //角度からパルス幅への変換式
//   digitalWrite(penguin, HIGH);
//   delayMicroseconds(kyori);
//   digitalWrite(penguin, LOW);
//   delay(5);//速度　5~30くらいが良好。
// }




//LiDAR
void LiDAR()
{
  if (Serial1.available())
  {
    if (Serial1.read() == HEADER)
    {
      uart[0] = HEADER;

      if (Serial1.read() == HEADER)
      {
        uart[1] = HEADER;

        for (i = 2; i < 9; i++)
        {
          uart[i] = Serial1.read();
        }

        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];

        if (uart[8] == (check & 0xff))
        {
          dist = float(uart[2] + uart[3] * 256);

          //strength = uart[4] + uart[5] * 256;

          //temprature = uart[6] + uart[7] * 256;
          //temprature = temprature / 8 - 256;
        }
      }
    }
  }

  if (dist > 800) dist = 800;
  if (dist < 0) dist = 0;
}



//超音波センサ
// void ultrasound_left()
// {

//   digitalWrite( TRIG_left, HIGH );
//   delayMicroseconds( 10 );
//   digitalWrite( TRIG_left, LOW );
//   duration_left = pulseIn( ECHO_left, HIGH ); // 往復にかかった時間が返却される[マイクロ秒]

//   if (duration_left > 0) {
//     duration_left = duration_left / 2; // 往路にかかった時間
//     dist_ultra_left = duration_left * speed_of_sound_left * 100 / 1000000;

//     dist_ultra_left = dist_ultra_left + 4.0;
//   }

  
//   // if (dist_ultra_left > 300)
//   // {
//   //   dist_ultra_left = 300;
//   // }

// }


// void ultrasound_right()
// {
//   digitalWrite( TRIG_right, HIGH );
//   delayMicroseconds( 10 );
//   digitalWrite( TRIG_right, LOW );
//   duration_right = pulseIn( ECHO_right, HIGH ); // 往復にかかった時間が返却される[マイクロ秒]

//   if (duration_right > 0) {
//     duration_right = duration_right / 2; // 往路にかかった時間
//     dist_ultra_right = duration_right * speed_of_sound_right * 100 / 1000000;

//     dist_ultra_right = dist_ultra_right + 4.0;
//   }

//   // if (dist_ultra_right > 300)
//   // {
//   //   dist_ultra_right = 300;
//   // }

// }




// void raw_pass_filter(float distance, float k, float &lpf) //https://garchiving.com/lpf-by-program/
// {
//   float lastLPF;

//   lpf = (1 - k) * lastLPF + k * distance;
//   lastLPF = lpf;
// }



// void PID(float Distance)
// {
//   x = Distance;

//   dt = (micros() - preTime) / 1000000.0;
//   preTime = micros();
//   P  = target - x;
//   I += P * dt;
//   D  = (P - preP) / dt;
//   preP = P;

//   duty += Kp * P + Ki * I + Kd * D;

//   //if(duty<-10) duty = -10;
//   //if(duty>10) duty = 10;

// }




void setup() {
  // put your setup code here, to run once:

  //サーボモータ
  // pinMode(penguin, OUTPUT);
  
  penguin.attach(9);
  
  //タイヤ
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);
  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);

  //LiDAR
  Serial.begin(9600);
  Serial1.begin(115200);

  //超音波センサ
  // pinMode(ECHO_right, INPUT );
  // pinMode(TRIG_right, OUTPUT );
  // pinMode(ECHO_left, INPUT );
  // pinMode(TRIG_left, OUTPUT );


  // penDash(servo_left_curve); //135°
  penguin.write(servo_left_curve);

  servo_direction = "left";
  delay(5000);
  
  LiDAR();
  
}




void loop() {


  int left_tire_R, right_tire_R; //右側の壁に沿って走行
  int left_tire_L, right_tire_L; //左の壁に沿って走行


  left_tire_R = int((float(High_Speed - Low_Speed) / 200.0)*dist + Low_Speed);
  right_tire_R = int((float(Low_Speed - High_Speed) / 200.0)*dist + High_Speed);

  left_tire_L = int((float(Low_Speed - High_Speed) / 200.0)*dist + High_Speed);
  right_tire_L = int((float(High_Speed - Low_Speed) / 200.0)*dist + Low_Speed);



  if(servo_direction == "left") //もし，サーボが左を向いていたら，
  {
    
    if(left_servo_count == 0)
    {
      penguin.write(servo_left_curve);
      //penDash(servo_left_curve); //135°  
    }


    left_servo_count++;

    if(dist >= wall_distant)
    {

      foward(Low_Speed, High_Speed+20);
      delay(30);


      // corner_count++;

      if(flag_right == -1)
      {
        flag_right = -1;

      }
      else
      {
        flag_right=1;
      }


      left_servo_count = 0;

      frame_count = 0;
    }


    // if(corner_count >= 50 && corner_count <= 100)
    // {
    //   servo_direction = "right";
    // }


    if(flag_right == 1)
    {

      if(dist<wall_distant)
      {
        frame_count++;

        if(frame_count == 80000) //100回，壁との距離が3m以内なら，
        {
          stop_(0,0);
          penguin.write(servo_right_curve);
          delay(1000);
          servo_direction = "right";

          //penDash(servo_right_curve); //45°
          
        }
      }

    }

    LiDAR();
    foward(left_tire_L, right_tire_L); //正面に進む 

  }



  if (servo_direction == "right") //もし，サーボが右を向いていたら，
  {
   
    LiDAR();
    foward(left_tire_R, right_tire_R); //正面に進む

    if(dist >= wall_distant) //壁との距離が遠くなったら（曲がり角）
    {

      servo_direction = "left";

      right_servo_count = 0;

      flag_right = -1;
    }


    right_servo_count++;

  }



  // Serial.println(servo_direction);



  //Serial.print("target = ");
  //Serial.print(target);
  //Serial.print("\t");
  //Serial.print("PID_taget = ");
  //Serial.print(PID_taget);
  //Serial.print("\t");
  // Serial.print("dist = ");
  // Serial.print(dist);
  // Serial.print("\n");
  // Serial.print("LPF_LiDAR = ");
  // Serial.print(LPF_LiDAR);
  // Serial.print("\n");
  // Serial.print("duty = ");
  // Serial.print(duty);
  // Serial.print("\t");

  // Serial.print("dist_ultra_right = ");
  // Serial.print(dist_ultra_right);
  // Serial.print("\n");
  // Serial.print("LPF_ultra_right = ");
  // Serial.print(LPF_ultra_right);
  // Serial.print("\n");

  //Serial.print("LPF_LiDAR-LPF_ultra_right");
  //Serial.println(LPF_LiDAR-LPF_ultra_right);



  // Serial.print("left_tire_R = ");
  // Serial.print(left_tire_R);
  // Serial.print("\t");


  // Serial.print("right_tire_R = ");
  // Serial.print(right_tire_R);
  // Serial.print("\n");


  // Serial.print("flag_right = ");
  // Serial.print(flag_right);
  // Serial.print("\t");


  // Serial.print("frame_count = ");
  // Serial.print(frame_count);
  // Serial.print("\n");


}
