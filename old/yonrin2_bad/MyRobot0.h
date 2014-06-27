// A My Robot Library 2012-02-28
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要

#ifndef MYROBOT1_H
#define MYROBOT1_H

#include "Arduino.h"

#define PING_NUM   4  // 超音波センサの数
#define BALL_SENSOR_NUM 8 // ボールセンサの数
#define HMC6352 // 地磁気センサHMC6352を使う場合。使わない場合はコメントアウト


class Robot
{
public:
  Robot();    
  void motorLF(double val);
  void motorLB(double val);
  void motorRF(double val);
  void motorRB(double val);
  long microsecondsToCentimeters(long microseconds);
  void move(int power);
  void moveForward(int power);
  void moveBack(int power);
  void turnRight(int power);
  void turnLeft(int power);

  unsigned int getDir(unsigned char dno);
  unsigned int getCompass();
  unsigned int getPing(int pos);
  unsigned int getBall(int pos);

  void setupCompass();
  void ballSensorCheck();
  void pingCheck();
  void compassCheck();
private:

  //デジタルコンパスモジュールのアドレス設定
  int compassAddress;

  // モータドライバのIN1、IN2に接続したピンの番号
  // const int in1Pin = 1;
  int in2Pin, in3Pin, in4Pin, in5Pin, in6Pin, in7Pin, in8Pin, in9Pin;

  // this constant won't change.  It's the pin number
  // of the sensor's output:
  int pingPin[PING_NUM];
  int ballSensorPin[BALL_SENSOR_NUM];  
};
#endif

