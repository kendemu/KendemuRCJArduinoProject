// Yonrin Arduino Robot Program 2011-02-24
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要
#ifndef MYROBOT2_H
#define MYROBOT2_H

#include <SerialLCD.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>

#define HMC6352 // 地磁気センサHMC6352を使う場合。使わない場合はコメントアウト
#define ENEMY_GOAL_DIR 267  // 敵ゴールの方位
#define PING_NUM         4  // 超音波センサの数
#define BALL_SENSOR_NUM  8  // ボールセンサの数

#define FWD        0  // 前
#define FWD_RIGHT  1  // 右前
#define RIGHT      2  // 右
#define BACK_RIGHT 3  // 右後
#define BACK       4  // 後
#define BACK_LEFT  5  // 左後
#define LEFT       6  // 左
#define FWD_LEFT   7  // 左前


class MyRobot2
{
public:
  MyRobot2(void);
  MyRobot2(int no);
  void motorLF(double val);
  void motorLB(double val);
  void motorRF(double val);
  void motorRB(double val);

  void move(int power);
  void moveForward(int power);
  void moveBack(int power);
  void turnRight(int power);
  void turnLeft(int power);
  // void kick();

  unsigned int getCompass();
  unsigned int getPing(int pos);
  unsigned int getBall(int pos);

  void ballSensorCheck();
  void pingCheck();
  void compassCheck();


private:
  // this constant won't change.  It's the pin number
  // of the sensor's output:
  int pingPin[PING_NUM]; // digital pin number for Pings
  int ballSensorPin[BALL_SENSOR_NUM];    // analog pin for ball sensors

  int compassAddress; 
  int hmc6352; // if hmc6352 is 1 then use hmc6352, otherwise use daisen
  // initialize the library
  //SerialLCD slcd(11,12);//this is a must, assign soft serial pins

  // モータドライバのIN1、IN2に接続したピンの番号
  int in1Pin;
  int in2Pin;
  int in3Pin;
  int in4Pin;
  int in5Pin;
  int in6Pin;
  int in7Pin;
  int in8Pin;
  int in9Pin;

  long microsecondsToCentimeters(long microseconds);
  void setupCompass();
  unsigned int getDir(unsigned char dno);
};
#endif



