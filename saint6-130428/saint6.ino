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




//checkBallSensor3();

void loop()
{
  static int step = 0;
  int speed = 0;
  
  move4(speed, 0, 0, 0);
  checkLCD(step);
  //move(speed,0,0,0,0,0);
  /* if (step++ > 1) speed = 0;
  for (speed = -100; speed <= 100; speed++)
  {
    move4(speed, 0, 0, 0); // right back 
    delay(10);
  }

  for (speed = -100; speed <= 100; speed++)
  {
    move4(0, speed, 0, 0); // left back 
    delay(10);
  }

  for (speed = -100; speed <= 100; speed++)
  {
    move4(0, 0, speed, 0); // left forward
    delay(10);
  }

  for (speed = -100; speed <= 100; speed++)
  { 
    move4(0, 0, 0, speed); // right forward
    delay(10);
  } */

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




















