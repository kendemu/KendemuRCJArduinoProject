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
#include <MsTimer2.h>

#define HMC6352 // 地磁気センサHMC6352を使う場合。使わない場合はコメントアウト
#define ENEMY_GOAL_DIR 267   // 敵ゴールの方位
#define BALL_THRESH    10    // ボール発見の閾値
#define SHOOT_THRESH   900   // シュートをするボールの閾値
#define MAX_POWER      90    // 最大パワー（モータを守るため．最大値１００）

#define PING_NUM   4  // 超音波センサの数
#define BALL_SENSOR_NUM 8 // ボールセンサの数
#define LINE_NUM 4//line numbers

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
static const int BOTTON_PIN = 2;// ピンの設定．コネクタの場所を変更するときはここを変更．
static const int PING_PIN[PING_NUM] = {
  4, 5, 6, 7}; // digital pin number for Pings
//  3, 5, 8, 10}; // digital pin number for Pings
static const int BALL_SENSOR_PIN[BALL_SENSOR_NUM] = {
  0, 1, 2, 3, 4, 5, 6, 7};    // analog pin for ball sensors
static const int LINE_PIN[LINE_NUM] ={
 8, 9 , 10 , 11};
static const int SERVO_PIN = 42;

static state g_state = FIND_BALL_STATE; // 初期状態はボールを捜すモード
static int g_use_lcd = 0;  // LCDが使えるとき１に設定される
static SerialLCD slcd(12,13); // LCDのピンをデジタル12, 13番に割り当て
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


//void kick();
long microsecondsToCentimeters(long microseconds);
/* void motor(int rf, int rb, int lb, int lf);
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
void moveLeft(int power); */


void setupCompass();
void setupLCD();
//void setupPin();


void turnRight(int power);
void turnLeft(int power);

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
void motcom4ch(int power, int power_angle, int angle, int kick);
void moveAngle4ch(int power, int power_angle, int angle);
void motor4ch(int power_rb,int power_lb,int power_lf,int power_rf);
int getMaxball();
void movevec105(int vx,int vy,int spin, int kick);
void main_prog();
unsigned int getLine(int pos);
int checkLine();
int getxpos();
void checkDist();
boolean getBotton();
int saint9(int court);
double fuzzyball(double ball, double bmax, double bmin);
void setvec(double p_limit);
void wrp(int power);
void wrp(int power, int k_mode ,int k_power);
void kick();
void setTime();
void getData();
void saint10(int tac);
void setPing(int i);
#endif


