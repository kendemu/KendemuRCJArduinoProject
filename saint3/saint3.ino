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
#include "dktlib3.h"



// 初期設定
void setup() { 
  setupLCD();     // LCDの設定
  setupPin();     // PWM用ピンの設定 
  setupCompass(); // コンパスの設定 
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






void loop()
{
  static int step = 0;

  checkLCD(step);
  
  /* int accel = 0;
  int accel_old;
  
  if (step > 0) 
  {
     if ((accel - accl_old) > 
  } */
  
  
  moveAngle(60,30, 270); // 並進パワー，回転パワー，進む方向
  //delay(2000);
  //moveXYW(0, 30, 30);  // vx, vy, dw
  //delay(1000);
  //moveXYW( 0, -30, 0);  // vx, vy, dw
  //delay(1000);
  //moveXYW(  0, 30, 0);  // vx, vy, dw
  //delay(1000); 
  
  //moveXYW(0, 0, 30);
  
  //checkBallSensor3();
  //checkBallSensor3();
  //getBallDir();
  // checkCompass();
  
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

  step++;
}
















