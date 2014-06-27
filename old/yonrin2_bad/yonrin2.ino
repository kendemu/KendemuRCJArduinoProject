// Yonrin Arduino Robot Program 2011-02-24
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要

#include <SerialLCD.h>
#include <SoftwareSerial.h>

#include <Servo.h>
#include <Wire.h>

#include "MyRobot0.h" // A my robot libraryが必要

// initialize 
SerialLCD slcd(11,12);//this is a must, assign soft serial pins
Servo myservo;
Robot robot;

#define ENEMY_GOAL_DIR 267   // 敵ゴールの方位


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
  SHOOT_STATE // シュートの状態
} 
state;

state robot_state = FIND_BALL_STATE;

int USE_LCD = 0;  // LCDが使えるとき１に設定される

void ballSensorCheck()
{
  int ball[BALL_SENSOR_NUM] = {
    999, 999, 999, 999, 999, 999, 999, 999                      };

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
    999, 999, 999, 999                      };

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

// その場回転してボールを捜し，発見したら前進してシュートするサンプルプログラム
void action1(int power)
{
  static int ccw = 1;
  int ball        = robot.getBall(FWD);
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

int getBallDir()
{
  int max = 999, max_no = 999;
  int val;

  for (int i=0; i < BALL_SENSOR_NUM; i++)
  {
    int tmp = robot.getBall(i);
    if (tmp == 999) continue;
    if ((10 < tmp) && (tmp < max))  
    {
      max    = tmp;
      max_no = i;
    }
  }


  if (max == 999) 
  {
    val =  999;
  }
  else {
    val = robot.getCompass() + max_no * 45; 
    val %= 360;    
  }

  return val;  
}

void checkBallDir()
{
  int ball_dir = getBallDir();  
  int diff = ENEMY_GOAL_DIR - ball_dir;

  if (diff < -180) diff += 360;
  if (diff >  180) diff -= 360;
  diff = fabs(diff);

  showInfo();

  if (ball_dir != 999) {
    if (diff < 80)
    {
      robot_state = APPROACH_STATE;
      return; 
    }
    else {
      // robot_state = TURN_AROUND_STATE;
      robot_state = GO_HOME_STATE;
      return;
    }
  }  
}


void goHome()
{
  float p_gain = 0.5;   // 比例ゲイン
  float i_gain = 0.05; // 積分ゲイン 0.001


  int tmp, tmp_sum;
  int mv; // 操作量
  static int diff = 0; // 偏差
  static int step = 0;
  static float i_sum = 0.0;
  int diff_old; // １時刻前の偏差
  int error = 10;

  showInfo();
  while (1)
  {  


    checkBallDir(); // ボールを見つけていたら次の状態に遷移   
    tmp = robot.getCompass();
    step++;
    tmp_sum = (int) i_sum; 

    //  ＬＣＤが使えるか100ループに１度チェック
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


    /* if (USE_LCD == 1)
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
     } */

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

  while (1)
  {
    checkBallDir(); // ボールを見つけていたら次の状態に遷移

    robot.moveBack(50);
    delay(50);

    if (robot.getPing(2) < 40) {
      robot_state = FIND_BALL_STATE;
      return;
    }
  }
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

void showInfo()
{
  int tmp;

  if (USE_LCD == 1)
  {
    slcd.setCursor(0,0);  
    slcd.print("STATE:");
    slcd.print(robot_state, DEC);
    slcd.print(" ");  
    slcd.print("Dir:");
    tmp = robot.getCompass();
    if (tmp < 10) slcd.print("  ");
    else if (tmp < 100) slcd.print(" ");
    slcd.print(tmp, DEC);      

    slcd.setCursor(0,1);    
    slcd.print("Ball:");
    tmp = getBallDir();
    if (tmp < 10) slcd.print("  ");
    else if (tmp < 100) slcd.print(" ");
    slcd.print(tmp, DEC);
  }
}



void findBall()
{
  int diff = ENEMY_GOAL_DIR - getBallDir();

  if (diff < -180) diff += 360;
  if (diff >  180) diff -= 360;

  showInfo();   

  robot.move(0);
  if (fabs(diff) < 70) robot_state = APPROACH_STATE;
  else                 robot_state = GO_HOME_STATE;

}

void approach()
{
  robot.move(50);  
}

/* void turnAround()
 {
 
 
 } */

void shoot()
{
  showInfo();
  if (robot.getBall(FWD) > 900) {
    delay(90);
    kick();
  }
  checkBallDir(); // ボールを見つけていたら次の状態に遷移
}

void setup() {
  // set up
  // slcd.begin();  // serial LCD setup
  if (slcd.beginAndCheck()) 
  {
    USE_LCD = 1;    
  }
  else {
    USE_LCD = 0;
  }

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
  
  slcd.setCursor(0,0);
  slcd.print("Ping:");
  //  robot.move(30);
  /* switch (robot_state)
  {
  case FIND_BALL_STATE:
    findBall();
    break;
  case GO_HOME_STATE:  
    goHome();
    break;
  case APPROACH_STATE:  
    approach();
    break;
    //case TURN_AROUND_STATE: 
    //  turnAround();
    //  break;
  case SHOOT_STATE:  
    shoot();
    break;
  } */
  delay(10);

  // action1(100);
  //pingCheck();
  // dirCheck();
  // ballSensorCheck();
}










