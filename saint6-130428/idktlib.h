/*********************************************************************/
// The iDKT Robot Library 2013-04-28
// Arduino Mega 2560を使ったロボット用ライブラリ
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要
/*********************************************************************/
// コーディング規則
// グローバル変数: 頭にg_を付ける．小文字．区切りはアンダーバー 
//     例 g_state
// 定数: 大文字区切りはアンダーバー 
//     例 BALL_THRESH
// 関数: 動詞名詞の順番．区切りは１文字だけ大文字にする
//     例 getBall()
/*********************************************************************/
#ifndef MYROBOT_H
#define MYROBOT_H

#include <SerialLCD.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>

//#define HMC6352 // 地磁気センサHMC6352を使う場合。使わない場合はコメントアウト
#define ENEMY_GOAL_DIR 267   // 敵ゴールの方位
#define BALL_THRESH    10    // ボール発見の閾値
#define SHOOT_THRESH   900   // シュートをするボールの閾値
#define MAX_POWER      90    // 最大パワー（モータを守るため．最大値１００）

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
  FIND_BALL_STATE,      // ボールの方位を検出する状態
  GO_HOME_STATE,        // 自陣に戻る状態
  APPROACH_STATE,       // ボールに向かう状態
  // TURN_AROUND_STATE, // ボールに回り込む状態
  SHOOT_STATE,          // シュートの状態
  ERROR                 // エラー
} 
state;

//デジタルコンパスモジュールのアドレス設定
static const int COMPASS_ADDRESS = 0x42 >> 1; //=0x21

// ピンの設定．コネクタの場所を変更するときはここを変更．
static const int PING_PIN[PING_NUM] = {
  3, 5, 7, 9}; // digital pin number for Pings
static const int BALL_SENSOR_PIN[BALL_SENSOR_NUM] = {
  1, 3, 5, 7, 9, 11, 13, 15};    // analog pin for ball sensors
static const int SERVO_PIN = 42;

static state g_state = FIND_BALL_STATE; // 初期状態はボールを捜すモード
static int g_use_lcd = 0;  // LCDが使えるとき１に設定される
static SerialLCD slcd(11,12); // LCDのピンをデジタル11, 12番に割り当て
static Servo myservo;         // サーボモータ

// モータドライバのIN1、IN2に接続したピンの番号
static const int PIN_1 = 1;
static const int PIN_2 = 2;
static const int PIN_3 = 3;
static const int PIN_4 = 4;
static const int PIN_5 = 5;
static const int PIN_6 = 6;
static const int PIN_7 = 7;
static const int PIN_8 = 8;
static const int PIN_9 = 9;

void checkBallSensor();
void checkLCD(unsigned int step);
void checkCompass();
void checkPing();

unsigned int getBall(int pos);
unsigned int getCompass();
unsigned int getDir(unsigned char dno);
unsigned int getPing(int pos);

void kick();
long microsecondsToCentimeters(long microseconds);
void motor(int rf, int rb, int lb, int lf);
void motorLF(double val);
void motorLB(double val);
void motorRF(double val);
void motorRB(double val);

void move(int power);
void moveForward(int power);
void moveBack(int power);
void moveLF(int power);
void moveRF(int power);
void moveLB(int power);
void moveRB(int power);
void moveRight(int power);
void moveLeft(int power);


void setupCompass();
void setupLCD();
void setupPin();
void Stop();

void turnRight(int power);
void turnLeft(int power);
void turn(int power);

void welcomeMessage();

///////new functions////////
void checkBallSensor3();
void goHome();
void moveXYW(int vx, int vy, int w); // vx:横方向の速度[cm/s], vy:縦方向の速度[cm/s]，w は各速度[deg/s]
void moveAngle(int power, int power_angle, int angle);
void findBall();
void noBall();
void getBallDir();
void Forward();
void turn();
void Keeper();
void PID();
#endif


