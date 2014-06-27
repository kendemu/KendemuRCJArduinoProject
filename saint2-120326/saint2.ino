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
#include "dktlib2.h"

int g_loop;
int g_diff;
int g_integral = 0;


// 初期設定
void setup() { 
  setupLCD();     // LCDの設定
  setupPin();     // PWM用ピンの設定 
  setupCompass(); // コンパスの設定 
}

void getBallDir()
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
  g_loop++;

  checkLCD(step);
  //checkBallSensor3();
  //checkBallSensor3();
  getBallDir();
  //checkCompass();


  int last_diff = g_diff;
  static int tar = getCompass();
  g_diff = tar - getCompass();  

  if (g_diff > 180) g_diff -= 360; 

  else if (g_diff < -180) g_diff += 360;

  double pro = g_diff * 1 / 6 ;
  double differential = (last_diff - g_diff) * 1/2;

  if(last_diff + 2 > g_diff && last_diff - 2 < g_diff)  g_integral++;
  if(g_diff < 0) g_integral *= -1;

  if(g_integral < -15) g_integral = -15;

  else if(g_integral > 15) g_integral = 15;

  double PID = pro + g_integral;


  if (g_diff > 40 || g_diff < -40 )  turn(PID);

  else 
  {
    g_integral = 0;
    getBallDir();
    checkPing();
    if(g_ball[0] > g_debugBall[0])
    {
    moveForward(70);
    kick();
    }
    else if(g_ball[1] > g_debugBall[1]) moveLeft(70);
    else if(g_ball[2] > g_debugBall[2]) moveBack(70);
    else if(g_ball[3] > g_debugBall[3]) moveRB(70);
    else if(g_ball[4] > g_debugBall[4]) moveLeft(70);
    else if(g_ball[5] > g_debugBall[5]) moveLB(70);
    else if(g_ball[6] > g_debugBall[6]) moveBack(70);
    else if(g_ball[7] > g_debugBall[7]) moveRight(70);
    else gohome();
  }



  
  // LCDが使えるかチェック 
  //checkCompass();

  //checkBallSensor3();
  //checkPing();




  /*if (g_ball[0] > 5)
   {
   }
   else if (g_ball[1] > 5)
   {
   
   }
   else if (g_ball[2] > 5)
   {
   
   }
   else if (g_ball[3] > 5)
   {
   
   }
   else if (g_ball[4] > 5)
   {
   
   }
   else if (g_ball[5] > 5)
   {
   
   }
   else if(g_ball[6] > 5)
   {
   
   }
   else if (g_ball[7] > 5)
   {
   
   }
   else if (Back_Ping > 70)
   {
   moveBack(45);
   }
   else if(Back_Ping < 50)
   {
   moveForward(45);
   }
   else if(Left_Ping > 40 && Right_Ping > 40)
   {
   Stop();
   }
   else if(Right_Ping > Left_Ping)
   {
   moveRight(20);
   }
   else
   {
   moveLeft(20);
   }
   */
  step++;
}
















