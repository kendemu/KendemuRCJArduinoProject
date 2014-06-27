// Yonrin Arduino Robot Program 2011-02-24
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要
#ifndef MYROBOT_H
#define MYROBOT_H

#include <SerialLCD.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>

#define HMC6352 // 地磁気センサHMC6352を使う場合。使わない場合はコメントアウト
#define ENEMY_GOAL_DIR 267   // 敵ゴールの方位
#define BALL_THRESH    10    // ボール発見の閾値
#define SHOOT_THRESH   900   // シュートをするボールの閾値

#define PING_NUM   4  // 超音波センサの数
#define BALL_SENSOR_NUM 8 // ボールセンサの数
#define FWD        0  // 前
#define FWD_RIGHT  1  // 右前
#define RIGHT      2  // 右
#define BACK_RIGHT 3  // 右後
#define BACK       4  // 後
#define BACK_LEFT  5  // 左後
#define LEFT       6  // 左
#define FWD_LEFT   7  // 左前


typedef enum {
  FIND_BALL_STATE, // ボールの方位を検出する状態
  GO_HOME_STATE, // 自陣に戻る状態
  APPROACH_STATE, // ボールに向かう状態
  // TURN_AROUND_STATE, // ボールに回り込む状態
  SHOOT_STATE, // シュートの状態
  ERROR // エラー
} 
state;




// this constant won't change.  It's the pin number
// of the sensor's output:
const int pingPin[PING_NUM] = {
  10, 13, 30, 36}; // digital pin number for Pings
const int ballSensorPin[BALL_SENSOR_NUM] = {
  1, 2, 3, 4, 5, 8, 10, 13};    // analog pin for ball sensors

extern SerialLCD slcd;
extern Servo myservo;
extern int USE_LCD;  // LCDが使えるとき１に設定される
// initialize the library
//SerialLCD slcd(11,12);//this is a must, assign soft serial pins

// モータドライバのIN1、IN2に接続したピンの番号
const int in1Pin = 1;
const int in2Pin = 2;
const int in3Pin = 3;
const int in4Pin = 4;
const int in5Pin = 5;
const int in6Pin = 6;
const int in7Pin = 7;
const int in8Pin = 8;
const int in9Pin = 9;

void motorLF(double val);
void motorLB(double val);
void motorRF(double val);
void motorRB(double val);
void motor(int rf, int rb, int lb, int lf);
long microsecondsToCentimeters(long microseconds);
void move(int power);
void moveForward(int power);
void moveBack(int power);
void turnRight(int power);
void turnLeft(int power);
void kick();

unsigned int getDir(unsigned char dno);
unsigned int getCompass();
unsigned int getPing(int pos);
unsigned int getBall(int pos);

void setupCompass();
void setupPin();
void ballSensorCheck();
void pingCheck();
void compassCheck();

#endif


