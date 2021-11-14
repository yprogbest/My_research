#define Kp      2
#define Ki      120
#define Kd      1
#define target  2.5

bool LED;
float duty = 0;
float dt, preTime;
float vol;
float P, I, D, U, preP;

void setup() {
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  for (int i = 0; i < 1000; i++) {
    vol += analogRead(0);
  }
  vol = 5.0 * (vol / 1000) / 1023;

  PID();
  analogWrite(3, duty);

  //Serial.print(dt , 3); Serial.print(",");
  Serial.print(duty);
  Serial.print("\t");
  //Serial.print(P, 3); Serial.print(",");
  //Serial.print(I, 3); Serial.print(",");
  //Serial.print(D, 3); Serial.print(",");
  Serial.print(vol);
  Serial.print("\n");
}

inline void PID() {
  dt = (micros() - preTime) / 1000000.0;
  preTime = micros();
  P  = target - vol;
  I += P * dt;
  D  = (P - preP) / dt;
  preP = P;

  duty += Kp * P + Ki * I + Kd * D;
}
