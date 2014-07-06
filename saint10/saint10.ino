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
#include <MsTimer2.h>
// 初期設定
void setup() { 
  setupLCD();     // LCDの設定
  pinMode(BOTTON_PIN,INPUT);
  //setupPin();     // PWM用ピンの設定 
  setupCompass(); // コンパスの設定 
  //Serial.begin(9600);
  //Serial1.begin(115200);
  MsTimer2::set(1,setTime);
  MsTimer2::start();
  Serial2.begin(115200);
}

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
  while(true){
  boolean run = getBotton();
    if(run == true)  while(true){
      checkLCD(step);
      //checkPing();
      //movevec105(0,40,0,0);
      saint10(1);
      step++;
    }
    step++;
  }
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





















