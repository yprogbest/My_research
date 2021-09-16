#include <Wire.h>

int SLAVE_ADDRESS = 0x04;   //I2Cのアドレス『0x04』

/*setupは起動後ときに最小に呼び出される関数でここで初期化の処理を行います*/
void setup() {
   //シリアル通信の初期化しシリアルモニタへ文字列を出力できるようにする　9600はボーレート(通信速度)
   Serial.begin(9600);

  //I2C接続を開始する 
  Wire.begin(SLAVE_ADDRESS);

  //I2Cで受信したときに呼び出す関数を登録する
  Wire.onReceive(ReceiveMassage); 

  //I2Cでリクエスト受信したときに呼び出す関数を登録する 
  Wire.onRequest(RequestMassage);
}

/*setupの後、終了するまで繰り返し呼び出される関数です*/
void loop() {
}

/*setupの後、終了するまで繰り返し呼び出される関数です*/
void ReceiveMassage(int n){
  char cmd = Wire.read();     //文字を読む
  Serial.println(cmd);       //シリアルポートにcmdを出力し表示する
}

//リクエスト要求を受けたときに「A」を送信する。
void RequestMassage(){
  Wire.write("A");            //Aを送信
}
