// A My Robot Library 2012-02-28
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要

#include "MyRobot0.h"
#include <Wire.h>

Robot::Robot()
{
  //デジタルコンパスモジュールのアドレス設定
  compassAddress = 0x42 >> 1; //=0x21

  // モータドライバのIN1、IN2に接続したピンの番号
  // const int in1Pin = 1;
  in2Pin = 2;
  in3Pin = 3;
  in4Pin = 4;
  in5Pin = 5;
  in6Pin = 6;
  in7Pin = 7;
  in8Pin = 8;
  in9Pin = 9;

  // this constant won't change.  It's the pin number
  // of the sensor's output:
  pingPin[0] = 10;
  pingPin[1] = 13;
  pingPin[2] = 30;
  pingPin[3] = 36;

  ballSensorPin[0] = 1;
  ballSensorPin[1] = 2;
  ballSensorPin[2] = 3;
  ballSensorPin[3] = 4;
  ballSensorPin[4] = 5;
  ballSensorPin[5] = 8;
  ballSensorPin[6] = 10;
  ballSensorPin[7] = 13;

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

  setupCompass();
}

// モータ左前Left Forwardを回転させる
void Robot::motorLF(double val) {
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
void Robot::motorLB(double val) {
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
void Robot::motorRF(double val) {
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
void Robot::motorRB(double val) {
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

long Robot::microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


void Robot::move(int power )
{
  // forward
  motorLF(-power);
  motorLB(-power);
  motorRF( power);
  motorRB( power);
}

void Robot::moveForward(int power )
{
  // forward
  int offset = 5;
  motorLF(-power);
  motorLB(-power);
  motorRF( power+offset);
  motorRB( power+offset);
}

void Robot::moveBack(int power)
{
  // forward
  motorLF(power);
  motorLB(power);
  motorRF(-power);
  motorRB(-power);
}

void Robot::turnRight(int power )
{
  motorLF(-power);
  motorLB(-power);
  motorRF(-power);
  motorRB(-power);
}

void Robot::turnLeft(int power)
{
  turnRight(-power);
}

void Robot::setupCompass()
{
  Wire.begin(); // I2C setup
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




//------------------------------------------------------------------------------
//ダイセンの多機能電子コンパスから角度を読み込みます。
// dno:0(dir回転), 1(pitch前後), 2(roll左右)
unsigned int Robot::getDir(unsigned char dno)
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

unsigned int Robot::getCompass()
{
  return getDir(0);
}

unsigned int Robot::getPing(int pos)
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

unsigned int Robot::getBall(int pos)
{
  int value = 0;  // ball sensor value

  // read the value from the sensor:
  value = analogRead(ballSensorPin[pos]); 
  return value;
}



