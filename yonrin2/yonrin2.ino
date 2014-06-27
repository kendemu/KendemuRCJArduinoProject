// Yonrin Arduino Robot Program 2011-02-29
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要"
// inoファイルからファイルをインクルードしないと他のファイルmyrobot0.hからはインクルードできない
// ので必要なヘッダファイルを全てインクルードしている
#include "Arduino.h"
#include <SerialLCD.h>        // シリアルLCD用
#include <SoftwareSerial.h>   // シリアルLCD用
#include <Servo.h>            // サーボモータ用
#include <Wire.h>             // I2C用(地磁気センサ)
#include "myrobot0.h"


// 初期設定
void setup() {
  // slcd.begin();  // serial LCD setup
  // LCDが使えるかどうかチェック．使える状態ならg_use_lcdを1に設定
  if (slcd.beginAndCheck()) 
  {
    g_use_lcd = 1;  
    welcomeMessage();  // システム起動メッセージ
  }
  else {
    g_use_lcd = 0;
  }
 
  Wire.begin();   // I2C setup 
  setupPin();     // PWM用ピンの設定
  setupCompass(); // コンパスの設定
}


// その場回転してボールを捜し，発見したら前進してシュートするサンプルプログラム
void action1(int power)
{
  static int cw = 1; //  時計周り(clockwise)なら１
  int ball = getBall(FWD);
  int ball_right  = getBall(FWD_RIGHT);
  int ball_left   = getBall(FWD_LEFT);

  if (g_use_lcd == 1)
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
    moveForward(power);
    if (ball > 900) {
      delay(90);
      kick();
    }
    if (cw == 1) cw = 0;
    else cw = 1;
  }
  else {
    if (cw == 1) {
      turnRight(30);
    }
    else       {
      turnRight(-30);
    }
  }  
}

int getBallDir()
{
  int max = 999, max_no = 999;
  int val;

  for (int i=0; i < BALL_SENSOR_NUM; i++)
  {
    int tmp = getBall(i);
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
    val = getCompass() + max_no * 45; 
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

  if (ball_dir != 999) 
  {
    if (diff < 80)
    {
      g_state = APPROACH_STATE;
      return; 
    }
    else {
      // g_state = TURN_AROUND_STATE;
      g_state = GO_HOME_STATE;
      return;
    }
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

  // ロボットを敵ゴールの方向へ向ける
  while (1)
  {  
    showInfo();
    
    if (checkBall()) {
      g_state = FIND_BALL_STATE; // ボールを見つけていたら次の状態に遷移  
      return; 
    } 

    tmp = getCompass();
    tmp_sum = (int) i_sum; 

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

    //  誤差がプラスマイナスerror以内だったらループを抜ける
    if ((-error <= diff ) && (diff < error)) {
      i_sum = 0;
      break;
    }
    else  {
      turnRight(mv);
    }
    delay(20);  
  }

  // 自陣に戻る
  while (1)
  {
    showInfo();
    
    if (checkBall()) {
      g_state = FIND_BALL_STATE; // ボールを見つけていたら次の状態に遷移  
      return; 
    } 

    move(-50);
    delay(50);

    // 壁から４０ｃｍの位置まで戻る
    if (getPing(3) < 40) {
      move(0);
      delay(1000);
      g_state = FIND_BALL_STATE;
      return;
    }
  }
}

// 情報表示
void showInfo()
{
  int tmp;

  if (g_use_lcd == 1)
  {
    slcd.setCursor(0,0);  
    slcd.print("STATE:");
    slcd.print(g_state, DEC);
    slcd.print(" ");  
    slcd.print("Dir:");
    tmp = getCompass();
    if (tmp < 10) slcd.print("  ");
    else if (tmp < 100) slcd.print(" ");
    slcd.print(tmp, DEC);      

    slcd.setCursor(0,1);    
    slcd.print("Ball:");
    tmp = getBallDir();
    if (tmp < 10) slcd.print("  ");
    else if (tmp < 100) slcd.print(" ");
    slcd.print(tmp, DEC);
    
    slcd.print(" PING:");
    slcd.print(getPing(2), DEC);
  }
}

// ボール探索. オム二ロボットなので停止してボールセンサをチェック．
void findBall()
{
  showInfo();
  
  int diff = ENEMY_GOAL_DIR - getBallDir();

  if (diff < -180) diff += 360;
  if (diff >  180) diff -= 360;

  move(0);
  if (abs(diff) < 70)
  {
    g_state = APPROACH_STATE; // ボールが敵ゴールの方向
  }
  else               
  {
    g_state = GO_HOME_STATE;  // ボールが味方ゴールの方向
  }
}

// ボールに接近．今は前進だけの実装
void approach()
{
  showInfo();
  
  if (getBall(FWD_RIGHT) > getBall(FWD_LEFT) + 100)  {
     motor(0, 50, 50, 0);
  }
  else if (getBall(FWD_LEFT) > getBall(FWD_RIGHT) + 100)  {
     motor(50, 0, 0, 50);
  }
  else {
    move(50);
  }
  
  
  //delay(50);
  
  if (getBall(FWD) > SHOOT_THRESH) 
  {
    g_state = SHOOT_STATE;    // キック
  }
  else if (checkBall()==0) {
      g_state = GO_HOME_STATE; // ボールを見失ったら自陣に戻る
  } 
}

/* void turnAround()
 {
 
 
 } */

// シュート
void shoot()
{
  showInfo();
  
  delay(90); // ボールが十分近づくまで待つ
  kick();    // キック

  g_state = APPROACH_STATE;
}


void loop()
{
  static int step = 0;
 
  //checkLCD(step);  // LCDが使えるかチェック 
  
  //action1(50);
  moveForward(50);
  //delay(1000);
  //moveBack(50);
  //del

  /* switch (g_state)
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
  default:
    g_state = ERROR; // エラー状態
    break;
  } */
  step++;
  //delay(500);

}















