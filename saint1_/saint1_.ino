//#include <WProgram.h>
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
  pinMode(BOTTON_PIN,INPUT);
  //setupPin();     // PWM用ピンの設定 
  setupCompass(); // コンパスの設定 
  //Serial.begin(9600);
  //Serial1.begin(115200);
  Serial2.begin(115200);
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

void checkMotor()
{
  int speed =90;
  motor4ch(speed,0, 0, 0);
  delay(5000);
  motor4ch(0,0,0,0);
  delay(100);
  motor4ch(-speed,0,0,0);
  delay(1000);
  motor4ch(0,0,0,0);
    delay(100);
  motor4ch(0, speed, 0, 0);
  delay(5000);
  motor4ch(0,0,0,0);
    delay(100);
  motor4ch(0,-speed,0,0);
  delay(1000);
  motor4ch(0,0,0,0);
    delay(100);
  motor4ch(0, 0, speed, 0);
  delay(5000);
  motor4ch(0,0,0,0);
    delay(100);
  motor4ch(0,0,-speed,0);
  delay(1000);
  motor4ch(0,0,0,0);
    delay(100);
  motor4ch(0,0, 0, speed);
  delay(5000);
  motor4ch(0,0,0,0);
    delay(100);
  motor4ch(0,0,0,-speed);
  delay(1000);
  motor4ch(0, 0, 0, 0);
  delay(100);  
}

void checkMotor4()
{
  int speed =80;
  motor4ch(0,0,0,0);
  delay(1);
  motor4ch(0,0, 0, speed);
  delay(5000);
  motor4ch(0,0, 0,-speed);
  delay(5000);
 //20 motor4ch(0, 0, 0, 0);  
//  motor4ch(0,0, 0,-speed);
//  delay(1000);
  
}
//checkBallSensor3();

void loop()
{
  static int step = 0;
  //PID();
  //checkLine();
  checkLCD(step);
  //motcom4ch(30,0,0,0);
  boolean run = getBotton();
  if(run == true)  while(true){
    checkLCD(step);
    //PID();
    motcom4ch(0,pid_control(),0,0);
    //checkPing();
    //saint9(0);
    step++;
  }
  step++;
  //motor4ch(0,0,0,0);
  //checkBallSensor();
  //moveAngle4ch(40, 0 , 0);
  //motor4ch(20,20,20,20);
  //motor4ch(0,0,0,20); // 右前、
  //checkMotor();
  //delay(100);
  //checkPing();
  //if (getPing(1) < 15) moveAngle4ch(60,0,0); 
  //fight();
  //motor4ch(speed,0, 0, 0);
  //checkLCD(step);
  //checkCompass();
  //turn(20);
  //Forward();
  //checkMotor();
  //motor4ch(0,0,0,20);
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





















