// Yonrin Arduino Robot Program 2011-02-24
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要"
// inoファイルからファイルをインクルードしないと他のファイルmyrobot0.hからはインクルードできない
#include <SerialLCD.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>

#include "MyRobot2.h"
// コンパスはHMC632とダイセンの電子コンパス対応．

SerialLCD slcd(11,12);
Servo myservo;
MyRobot2 robot(1); // HMC632を使用する場合は1, ダイセンを使用する場合は0

int USE_LCD = 0;  // LCDが使えるとき１に設定される

void ballSensorCheck()
{
  int ball[BALL_SENSOR_NUM] = {
    999, 999, 999, 999, 999, 999, 999, 999                  };

  // measure the ball sensor
  for (int i = 0; i < BALL_SENSOR_NUM; i++)
  {
    ball[i] = robot.getBall(i);
  }

  if (USE_LCD == 1)
  {
    slcd.setCursor(0,0);  
    slcd.print("B");
    for (int j = 0; j < BALL_SENSOR_NUM; j++)
    { 
      if (j == 4) {   
        slcd.setCursor(0,1); 
        slcd.print("B");
      }
      slcd.print(ball[j], DEC);
      slcd.print(" ");
    }    
  }
}

void pingCheck()
{
  int dist[PING_NUM] = {
    999, 999, 999, 999                  };

  // measure the ultra sonic sensor   
  for (int i = 0; i < PING_NUM; i++)
  {
    dist[i] = robot.getPing(i);  
  }

  if (USE_LCD == 1)
  {
    slcd.setCursor(0,0);
    slcd.print("Pg:"); 
    for (int j = 0; j < PING_NUM; j++)
    {
      slcd.print(dist[j], DEC);
      slcd.print(" ");
    }
  }
  delay(20);
}

void compassCheck()
{
  int dir;

  // get Diretion
  dir = robot.getCompass();

  if (USE_LCD==1)
  {
    slcd.setCursor(0,0);  
    slcd.print(" Dir:");
    slcd.print(dir,DEC);
    slcd.print(" ");
  }
}

void kick()
{
  myservo.attach(14);
  myservo.write(135);
  delay(20);
  //myservo.write(85);
  //delay(20);
  myservo.write(160);
  delay(100);
  myservo.write(135);
  delay(300);
  myservo.detach();

  /* for (int i = 0; i < 10; i++) {
   myservo.write(70);
   delay(40);
   }
   myservo.write(85);
   delay(10);
   for (int i = 0; i < 10; i++) {
   myservo.write(100);
   delay(40);
   }  
   */
}

void dirCheck()
{
  int dir;
  dir = robot.getCompass();

  slcd.setCursor(0,0);
  slcd.print("Dir: ");
  if (dir < 10) slcd.print("  ");
  else if (dir < 100) slcd.print(" ");
  slcd.print(dir, DEC);  
}

// その場回転してボールを捜し，発見したら前進してシュートするサンプルプログラム
void action1(int power)
{
  static int ccw = 1;
  int ball = robot.getBall(FWD);
  int ball_right  = robot.getBall(FWD_RIGHT);
  int ball_left   = robot.getBall(FWD_LEFT);

  if (USE_LCD == 1)
  {
    slcd.setCursor(0,0);  
    slcd.print("Ball:");
    slcd.print(ball, DEC);
    slcd.print(" ");  
    slcd.print(ball_right, DEC);
    slcd.print(" ");
    slcd.print(ball_left, DEC);
  }

  if (ball > 10) {
    robot.move(power);
    if (ball > 900) {
      delay(90);
      kick();
    }
    if (ccw == 1) ccw = 0;
    else ccw = 1;
  }
  else {
    if (ccw == 1) {
      robot.turnRight(30);
    }
    else       {
      robot.turnRight(-30);
    }
  }  
}

int returnGoal()
{
  float p_gain = 0.5;   // 比例ゲイン
  float i_gain = 0.05; // 積分ゲイン 0.001

  int tmp, tmp_sum;
  int mv; // 操作量
  static int diff = 0; // 偏差
  static int step = 0;
  static float i_sum = 0.0;
  //int diff_old; // １時刻前の偏差
  int error = 10;

  while (1)
  {   
    tmp = robot.getCompass();
    step++;
    tmp_sum = (int) i_sum; 

    //  ＬＣＤが使えるか1000ループに１度チェック
    if (step % 100 == 0) 
    {
      if (slcd.beginAndCheck()) {
        USE_LCD = 1;        
      }
      else {
        slcd.noPower();
        USE_LCD = 0;
      }
    }

    if (USE_LCD == 1)
    {
      slcd.setCursor(0,0);
      slcd.print(step,DEC);  
      slcd.print(" Dir:");
      slcd.print(tmp, DEC);
      slcd.setCursor(0,1);
      slcd.print("ISUM:");

      if (i_sum > 0) { 
        slcd.print(tmp_sum,DEC);
        slcd.print("  ");
      }
      else {     
        tmp_sum = ~tmp_sum + 1; // 負の場合は補数表現なので反転して１を足す
        slcd.print("-");
        slcd.print(tmp_sum,DEC);
        slcd.print("  ");      
      }  
    }


    diff = ENEMY_GOAL_DIR - tmp;
    if (diff < -180) diff += 360;
    if (diff > 180)  diff -= 360;

    i_sum += i_gain * (float) diff;  

    if (i_sum >  100) i_sum =  100;
    if (i_sum < -100) i_sum = -100;

    mv = p_gain * diff + (int) i_sum; // 操作量 manupilative value

    if (mv >=  100) mv =  100;
    if (mv <= -100) mv = -100;

    if ((-error <= diff ) && (diff < error)) {
      i_sum = 0;
      break;
    }
    else  {
      robot.turnRight(mv);
    }
    delay(20);  
  }
  robot.move(-50);
  delay(50);

  if (robot.getPing(2) < 20) {
    robot.move(0);
    delay(500);
  }
}

void setup() {
  
  robot.move(50);
  // set up
  //slcd.begin();  // serial LCD setup
  if (slcd.beginAndCheck()) 
  {
    USE_LCD = 1;    
  }
  else {
    USE_LCD = 0;
  }
 
  Wire.begin(); // I2C setup

  // サーボモーターを14ピン
  //myservo.attach(14);
  //myservo.write(100);
  //delay(40);

  slcd.setCursor(0,0);  
  slcd.blink();
  slcd.print("Yonrin start");
  delay(4000);
  slcd.clear();   
}

void loop() {
  //action1(100);
  // pingCheck();
  //dirCheck();

  robot.move(50);
  //pingCheck();


  /* int power;
   static int degree = 0;
   
   power = sin(degree*0.03)* 50;
   robot.move(power);
   degree++;
   delay(20); */
  //moveForward(100);



  //motorLF(100);
  //motorLB(100);
  //motorRF(100);
  //motorRB(100);

  // action1();
  // returnGoal();




  //ballSensorCheck();
  //motorLF( sin(degree*0.03)*255 );
  //motorLB( sin(degree*0.03)*255 );
  //motorRF( sin(degree*0.03)*255 );
  // motorRB( sin(degree*0.03)*255 );
  //degree++;

  //moveForward(100);
  //static double tmp = -255;
  //motorLF(tmp);
  //motorLB(tmp);
  //motorRF(tmp);
  //motorRB(tmp);
  //tmp++;

  //if (tmp >= 255) tmp = -255;
  //delay(100);
  // 前進
  // moveForward();

  /* static int val = 0;
   myservo.write(val++);
   if (val >= 255) val = 0;
   slcd.setCursor(0,0);  
   slcd.print("servo:");
   slcd.print(val, DEC);
   slcd.print(" ");
   delay(1000);*/



  /* for (int i = 0; i < 10; i++)
   {
   myservo.write(70);
   delay(2);
   }
   delay(50);
   for (int i=0; i < 10; i++)
   {
   myservo.write(100);
   delay(2);
   }
   delay(100); */


  //myservo.write(0);
  //delay(10);
  //myservo.write(0);
  //delay(200);

  /* static int val = 0;
   slcd.setCursor(0,0);
   slcd.print("servo:");
   slcd.print(val, DEC);
   
   for (int i = 0; i < 10; i++) {
   myservo.writeMicroseconds(val);
   delay(2);
   }
   delay(50);
   //myservo.writeMicroseconds(1000);
   val++;
   if (val >= 1000) val = 0; */

  /* for (int i=80 ; i <= 122; i+= 1)
   {
   myservo.write(i);
   delay(20);
   //slcd.print(myservo.read(),DEC);
   //delay(500);
   //slcd.clear();
   } */
  /* for (int i = 50; i >= 0; i--)
   {
   myservo.write(i);
   delay(10);
   } */

  //myservo.write(0);
  //myservo.write(0);
  //delay(100);
  //for (int i = 0; i < 10; i++)
  //{
  // myservo.write(1);
  // delay(1);
  //} 
  //myservo.write(68);
  //delay(2000); //delay(500);
  //
  //delay(199);
}









