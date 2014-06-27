#include <SerialLCD.h>

/// An Arduino Robot Sample Program  2012-02-29
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要"
// inoファイルからファイルをインクルードしないと他のファイルmyrobot0.hからはインクルードできない
// ので必要なヘッダファイルを全てインクルードしている
#include "Arduino.h"
#include <SerialLCD.h>        // シリアルLCD用
#include <SoftwareSerial.h>   // シリアルLCD用
#include <Servo.h>            // サーボモータ用
#include <Wire.h>             // I2C用(地磁気センサ)
#include "idktlib.h"


// 初期設定
void setup() { 
  setupLCD();     // LCDの設定
  setupPin();     // PWM用ピンの設定 
  setupCompass(); // コンパスの設定 
  Serial.begin(115200);
  Serial1.begin(115200);
}

/* void getBallDir()
 { 
 int no , no2; 
 int t_max = 0 , t_max2 = 0;
 
 for (int i = 0 ; i < 8; i++)
 {
 g_ball[i] = getBall(i);
 if (g_ball[i] > t_max && g_ball[i] >g_debugBall[i] )
 {
 t_max = g_ball[i] - g_debugBall[i];
 no = i;
 }
 else if(g_ball[i] > t_max2 && g_ball[i] > g_debugBall[i])
 {
 t_max2 = g_ball[i] - g_debugBall[i];
 no2 = i;
 }
 }
 
 double value;
 value = t_max2 / t_max * 225;
 if (t_max <= 0) value = 9999;
 if(no2 < no) no = no2; 
 
 if (no < 0) no = 0;
 
 double value2   = no * 45;
 
 g_BallDir = value + value2;
 if (g_use_lcd==1)
 {
 slcd.setCursor(0,0);    
 slcd.print("BallDir:");
 slcd.print(g_BallDir, DEC);
 
 }
 }
 */

// ボールを見つけたら１を返す
int checkBall()
{
  for (int i=0; i < BALL_SENSOR_NUM; i++)
  {
    int tmp = getBall(i);
    if ((tmp != 999) && (tmp > BALL_THRESH)) return 1;
  }
  return 0;
}

void checkMotor4ch()
{
  int speed =40;
  motor4ch(speed,0, 0, 0);
  delay(5000);
  motor4ch(0, speed, 0, 0);
  delay(5000);
  motor4ch(0, 0, speed, 0);
  delay(5000);
  motor4ch(0,0, 0, speed);
  delay(5000);
  motor4ch(0, 0, 0, 0);  
}

void checkMotor()
{
  int speed =40;
  motor(speed,0, 0, 0); // LR +
  delay(5000);
  motor(0, speed, 0, 0); // LF - 
  delay(5000);
  motor(0, 0, speed, 0); // RR -
  delay(5000);
  motor(0,0, 0, speed); // RF +
  delay(5000);
  motor(0, 0, 0, 0);  
}

void moveForward4ch(int speed)
{
  motor4ch(speed,-speed, -speed, speed); // LR
}

void moveBackward4ch(int speed)
{
  motor4ch(-speed, speed, speed, -speed); // LR
}

void moveLeft4ch(int speed)
{
  motor4ch(speed, speed, speed, speed); // LR
}

void moveRight4ch(int speed)
{
  motor4ch(-speed, -speed, -speed, -speed); // LR
}

void check2Motor4ch()
{
  int speed =20;
  int period = 2000;
  
  moveForward4ch(speed);
  delay(period);
  moveRight4ch(speed);
  delay(period);
  moveBackward4ch(speed);
  delay(period);
  moveLeft4ch(speed);
  delay(period);
  motor4ch(0, 0, 0, 0);  
}

void check3Motor4ch()
{
  int speed =60;
  int period = 500;
  
  moveForward4ch(speed);
  delay(period);
  motor4ch(0,0,0,0);
  delay(period);
  //moveRight4ch(speed);
  //delay(period);
  moveBackward4ch(speed);
  delay(period);
  //moveLeft4ch(speed);
  //delay(period);
  motor4ch(0, 0, 0, 0); 
  delay(period); 
}



//checkBallSensor3();

void calibrationHMC6352()
{
 //キャリブレーション開始コマンド送信
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write('C');
  Wire.endTransmission();

 if (g_use_lcd==1)
  {
    slcd.setCursor(0,0);
    slcd.print(" Cal Begin.");
  }
  //30秒ほど待つ（6秒〜3分まで）
  //この間に数回モジュールを回転させる
  delay(30000);

  //キャリブレーション終了コマンド送信
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write('E');
  Wire.endTransmission();
  if (g_use_lcd==1)
  {
    slcd.setCursor(0,0);
    slcd.print(" Cal END.");
  }
  exit(1);
}


void loop()
{
  static int step = 0;
  int speed = 40;

  /* if(getBall(0) > 900) {
    moveForward(100);
  }
  else {
    motor(25, -25, -25, 25); 
  } */
  //calibrationHMC6352();
  //checkBallSensor();
  check3Motor4ch();
  //checkPing();
  //if (getPing(1) < 15) moveAngle4ch(60,0,0); 
  //fight();
  //motor4ch(speed,0, 0, 0);
  //checkLCD(step);
  //checkCompass();
  //turn(20);
  //moveForward(100);
  //checkMotor();
  //move(speed,0,0,0,0,0);
  //moveAngle4ch(0, 0, 0);
  /*moveAngle4ch(30, 0, 0);
   delay(3000);
   moveAngle4ch(30, 0, 90);
   delay(3000);
   moveAngle4ch(30, 0, 180);
   delay(3000);
   moveAngle4ch(30, 0, 270);
   delay(3000);
   moveAngle4ch(0, 0, 270); 
   */
  //Forward();

  //moveAngle4ch(0, 0, 0);
  //while (1) {
  //motor4ch(0,0, 0, 0);
  //
  //exit(1);
  step++;
}



//(power , rotation power ,angle)
//checkBallSensor();

//checkBallSensor3();
//checkCompass();

/* moveForward(30);
 delay(1000);
 moveRight(30);
 delay(1000);
 moveBack(30);
 delay(1000);
 moveLeft(30);
 delay(1000); */
//motorLF(20);  // OK
// motorLB(20);

//motorRB(20); 

//motorRB(20);






















