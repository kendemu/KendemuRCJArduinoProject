// Yonrin Arduino Robot Program 2011-02-24
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要

#include "myrobot0.h"

//デジタルコンパスモジュールのアドレス設定
int compassAddress = 0x42 >> 1; //=0x21

// PWM用ピンの設定
void setupPin()
{
 // IN1、IN2、PWMの各ピンのモードを出力にセット
  //pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(in5Pin, OUTPUT);
  pinMode(in6Pin, OUTPUT);
  pinMode(in7Pin, OUTPUT);
  pinMode(in8Pin, OUTPUT);
  pinMode(in9Pin, OUTPUT);
}

void setupCompass()
{

#ifdef HMC6352
  //Continuous Modeに設定する
  Wire.beginTransmission(compassAddress);
  //RAM書き込み用コマンド
  Wire.write('G');
  //書き込み先指定
  Wire.write(0x74);
  //モード設定
  Wire.write(0x72);
  //通信終了
  Wire.endTransmission();
  //処理時間
  delayMicroseconds(70);
#endif
}

// モータ左前Left Forwardを回転させる
void motorLF(double val) {
  int value = 2.55 * val;
  if (value >=  255) value =  255;
  if (value <= -255) value = -255;

  if(value > 10){ //順方向
    analogWrite(in2Pin, value);
    analogWrite(in3Pin, 0);
  }
  else if(value < -10){ //逆方向
    analogWrite(in2Pin, 0);
    analogWrite(in3Pin, abs(value));

  }
  else{ //ブレーキを使用しないで停止
    digitalWrite(in2Pin, LOW);
    digitalWrite(in3Pin, LOW);
  }
}

// モータ左後Left Backwardを回転させる
void motorLB(double val) {
  int value = 2.55 * val;
  if (value >=  255) value =  255;
  if (value <= -255) value = -255;

  if(value > 10){ //順方向
    analogWrite(in4Pin, value);
    analogWrite(in5Pin, 0);

  }
  else if(value < -10){ //逆方向
    analogWrite(in4Pin, 0);
    analogWrite(in5Pin, abs(value));
  }
  else{ //ブレーキを使用しないで停止
    digitalWrite(in4Pin, LOW);
    digitalWrite(in5Pin, LOW);
  }
}

// モータ右Right Forwardを回転させる
void motorRF(double val) {
  int value = 2.55 * val;
  if (value >=  255) value =  255;
  if (value <= -255) value = -255;

  if(value > 10){ //順方向
    analogWrite(in6Pin, value);
    analogWrite(in7Pin, 0);

  }
  else if(value < -10){ //逆方向
    analogWrite(in6Pin, 0);
    analogWrite(in7Pin, abs(value));

  }
  else{ //ブレーキを使用しないで停止
    digitalWrite(in6Pin, LOW);
    digitalWrite(in7Pin, LOW);
  }
}

// モータ右後Right Backwardを回転させる
void motorRB(double val) {
  int value = 2.55 * val;
  if (value >=  255) value =  255;
  if (value <= -255) value = -255;

  if(value > 10){ //順方向
    analogWrite(in8Pin, value);
    analogWrite(in9Pin, 0);

  }
  else if(value < -10){ //逆方向
    analogWrite(in8Pin, 0);
    analogWrite(in9Pin, abs(value));

  }
  else{ //ブレーキを使用しないで停止
    digitalWrite(in8Pin, LOW);
    digitalWrite(in9Pin, LOW);
  }
}

// モータの制御
// rf: right forward, rb: right back, lb: left back, lf: left forward
void motor(int rf, int rb, int lb, int lf)
{
   motorRF(rf);
   motorRB(rb);
   motorLB(lb);
   motorLF(lf);
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

// 前進後進
void move(int power )
{
  // forward
  motorLF(-power);
  motorLB(-power);
  motorRF( power);
  motorRB( power);
}

// 前進
void moveForward(int power )
{
  // forward
  int offset = 5;
  motorLF(-power);
  motorLB(-power);
  motorRF( power+offset);
  motorRB( power+offset);
}

// 後進
void moveBack(int power)
{
  // forward
  motorLF(power);
  motorLB(power);
  motorRF(-power);
  motorRB(-power);
}

// 右回転
void turnRight(int power )
{
  motorLF(-power);
  motorLB(-power);
  motorRF(-power);
  motorRB(-power);
}

// 左回転
void turnLeft(int power)
{
  turnRight(-power);
}


//------------------------------------------------------------------------------
//ダイセンの多機能電子コンパスから角度を読み込みます。
// dno:0(dir回転), 1(pitch前後), 2(roll左右)
unsigned int getDir(unsigned char dno)
{
#ifdef HMC6352
  int reading = 999;

  //デバイスに２バイト分のデータを要求する
  Wire.requestFrom(compassAddress, 2);
  //要求したデータが２バイト分来たら
  if(Wire.available()>1){
    //１バイト分のデータの読み込み 
    reading = Wire.read();
    //読み込んだデータを８ビット左シフトしておく
    reading = reading << 8;
    //次の１バイト分のデータを読み込み
    //一つ目のデータと合成（２バイト）
    reading += Wire.read();
    //２バイト分のデータを１０で割る
    reading /= 10; 
  } 
  if (reading < 0)  return 999;
  else if (reading > 360) return 999;
  else return reading;
#else
  typedef union { //受信データ用共用体
    unsigned int W;
    struct {
      unsigned char L;
      unsigned char H;
    };
  } 
  U_UINT;
  U_UINT data; // 受信データ
  int adrs = 0x50>>1; //スレーブアドレス
  int reg; //レジスターアドレス

  reg = 0x20 + dno * 2;
  //data.W = 999; // initialize

  //通信開始
  Wire.beginTransmission(adrs);

  //register
  Wire.write(reg);

  //通信終了
  Wire.endTransmission();
  Wire.requestFrom(adrs, 2);

  if(Wire.available()>1){
    //１バイト分のデータの読み込み 
    data.H = Wire.read();
    //次の１バイト分のデータを読み込み
    data.L = Wire.read();
  } 
  if (data.W > 359) data.W = 999;
  return data.W;
#endif
}



unsigned int getPing(int pos)
{
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin[pos], OUTPUT);
  digitalWrite(pingPin[pos], LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin[pos], HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin[pos], LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin[pos], INPUT);
  duration = pulseIn(pingPin[pos], HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);

  // error
  if (cm > 400)  cm = 999;
  if (cm <=   0) cm = 999;

  return cm;

  //slcd.setCursor(0,0);
  //slcd.print("Ping:");
  // slcd.print(cm,DEC);
  // slcd.print("cm");
}

unsigned int getCompass()
{
  return getDir(0);
}

unsigned int getBall(int pos)
{
  int value = 0;  // ball sensor value

  // read the value from the sensor:
  value = analogRead(ballSensorPin[pos]); 
  return value;
}

void ballSensorCheck()
{
  int ball[BALL_SENSOR_NUM] = {
    999, 999, 999, 999, 999, 999, 999, 999                };

  // measure the ball sensor
  for (int i = 0; i < BALL_SENSOR_NUM; i++)
  {
    ball[i] = getBall(i);
  }

  if (USE_LCD == 1)
  {
    slcd.setCursor(0,0);  
    slcd.print("B");
    for (int j = 0; j < BALL_SENSOR_NUM; j++)
    { 
      if (j == 4) {   
        slcd.setCursor(0,1); 
        slcd.print("B");
      }
      slcd.print(ball[j], DEC);
      slcd.print(" ");
    }    
  }
}

void pingCheck()
{
  int dist[PING_NUM] = {
    999, 999, 999, 999                };

  // measure the ultra sonic sensor   
  for (int i = 0; i < PING_NUM; i++)
  {
    dist[i] = getPing(i);  
  }

  if (USE_LCD == 1)
  {
    slcd.setCursor(0,0);
    slcd.print("Pg:"); 
    for (int j = 0; j < 2; j++)
    {
      slcd.print(dist[j], DEC);
      slcd.print(" ");
    }
    slcd.setCursor(0,1);
    for (int j = 2; j < PING_NUM; j++)
    {
      slcd.print(dist[j], DEC);
      slcd.print(" ");
    }
  }
  delay(20);
}

void compassCheck()
{
  int dir;

  // get Diretion
  dir = getDir(0);

  if (USE_LCD==1)
  {
    slcd.setCursor(0,0);  
    slcd.print(" Dir:");
    slcd.print(dir,DEC);
    slcd.print(" ");
  }
}

void kick()
{
  myservo.attach(14);
  myservo.write(135);
  delay(20);
  //myservo.write(85);
  //delay(20);
  myservo.write(160);
  delay(100);
  myservo.write(135);
  delay(300);
  myservo.detach();

  /* for (int i = 0; i < 10; i++) {
   myservo.write(70);
   delay(40);
   }
   myservo.write(85);
   delay(10);
   for (int i = 0; i < 10; i++) {
   myservo.write(100);
   delay(40);
   }  
   */
}

void dirCheck()
{
  int dir;
  dir = getDir(0);
  
  slcd.setCursor(0,0);
  slcd.print("Dir: ");
  if (dir < 10) slcd.print("  ");
  else if (dir < 100) slcd.print(" ");
  slcd.print(dir, DEC);  
}


