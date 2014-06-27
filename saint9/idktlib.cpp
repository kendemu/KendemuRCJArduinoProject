/*********************************************************************/
// The iDKT Robot Library 2013-04-28
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要
// ダイセン4chモータドライバに対応

#include "idktlib.h"

#define PI 3.14159265
#define DEG2RAD(deg)  ((float) (deg * PI/180.0))
#define RAD2DEG(rad)  ((float) (rad * 180.0/PI))

int g_debugBall[8] = {
  150 , 10 , 10 ,10 , 10, 10 , 10 ,10};
int g_debugLine[4] = {
  150, 150, 150, 150};
int g_BallDir = 0;
double g_Vx , g_Vy;
int g_ball[8];
int g_line[4];
int g_dist[4];
int g_no = 999;
int g_pid;
int g_diff;
int g_loop;
int g_kick = 0;
int g_max;
float g_integral;
//
// ボールセンサのチェック
void checkBallSensor()
{
  int ball[BALL_SENSOR_NUM] = {
    999, 999, 999, 999, 999, 999, 999, 999                                          };

  // measure the ball sensor
  for (int i = 0; i < BALL_SENSOR_NUM; i++)
  {
    ball[i] = getBall(i);
  }

  if (g_use_lcd == 1)
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

// 地磁気センサのチェック
void checkCompass()
{
  int dir;

  // get Diretion
  dir = getCompass();

  if (g_use_lcd==1)
  {
    slcd.setCursor(0,0);
    slcd.print(" Dir:");
    slcd.print(dir,DEC);
    slcd.print("    ");
  }
}

//  LCDのチェック．LCDが使えるか100ループに１度チェック
void checkLCD(unsigned int step)
{
  if (step % 100 == 0)
  {
    if (slcd.beginAndCheck()) {
      g_use_lcd = 1;
      // welcomeMessage();  // システム起動メッセージ
    }
    else {
      slcd.noPower();
      g_use_lcd = 0;
    }
  }
}

// 超音波センサのチェック．PingあるいはSeedo studioの超音波センサしか使えない．
void checkPing()
{
  int dist[PING_NUM] = {
    999, 999, 999, 999                                          };

  // measure the ultra sonic sensor
  for (int i = 0; i < PING_NUM; i++)
  {
    g_dist[i] = getPing(i);
  }
  if (g_use_lcd == 1)
   {
   slcd.setCursor(0,0);
   slcd.print("Pg:");
   for (int j = 0; j < 2; j++)
   {
   slcd.print(g_dist[j], DEC);
   slcd.print(" ");
   }
   slcd.setCursor(0,1);
   for (int j = 2; j < PING_NUM; j++)
   {
   slcd.print(g_dist[j], DEC);
   slcd.print(" ");
   }
   }   
}
void checkDist(){
  PID(); 
  for (int i = 0; i < PING_NUM; i++)
  {
    g_dist[i] = getPing(i) * abs(cos(g_diff));
  }
}
//ダイセンの多機能電子コンパスから角度を読み込み
// dno:0(dir回転), 1(pitch前後), 2(roll左右)
unsigned int getDir(unsigned char dno)
{
#ifdef HMC6352
  int reading = 999;

  //デバイスに２バイト分のデータを要求する
  Wire.requestFrom(COMPASS_ADDRESS, 2);
  //要求したデータが２バイト分来たら
  if(Wire.available()>1){
    //１バイト分のデータの読み込み
    reading = Wire.read();
    //読み込んだデータを８ビット左シフトしておく
    reading = reading << 8;
    //次の１バイト分のデータを読み込み
    //一つ目のデータと合成（２バイト）
    reading += Wire.read();
    //２バイト分のデータを１０で割る
    reading /= 10;
  }
  if (reading < 0)  return 999;
  else if (reading > 360) return 999;
  else return reading;
#else
  typedef union { //受信データ用共用体
    unsigned int W;
    struct {
      unsigned char L;
      unsigned char H;
    };
  }
  U_UINT;
  U_UINT data; // 受信データ
  int adrs = 0x50>>1; //スレーブアドレス
  int reg; //レジスターアドレス

  reg = 0x20 + dno * 2;
  //data.W = 999; // initialize

  //通信開始
  Wire.beginTransmission(adrs);

  //register
  Wire.write(reg);

  //通信終了
  Wire.endTransmission();
  Wire.requestFrom(adrs, 2);

  if(Wire.available()>1){
    //１バイト分のデータの読み込み
    data.H = Wire.read();
    //次の１バイト分のデータを読み込み
    data.L = Wire.read();
  }
  if (data.W > 359) data.W = 999;
  return data.W;
#endif
}

// 赤外線センサ（ボールセンサ）からデータ取得
unsigned int getBall(int pos)
{
  int value = 999;  // ball sensor value

  // read the value from the sensor:
  value = analogRead(BALL_SENSOR_PIN[pos]);
  return value;
}
unsigned int getLine(int pos)
{
  int value = 999;  // ball sensor value
  // read the value from the sensor:
  value = analogRead(LINE_PIN[pos]);
  return value;
}
int checkLine(){
  int state = 0;
  int w_kosu = 0;
  int b_kosu = 0;
  int num[4] = { 0 , 0 , 0 , 0 };
  for(int i = 0 ; i < 4; i++ ) {
    g_line[i] = getLine(i);
    if(g_line[i] - g_debugLine[i] > 400){
      w_kosu++;
      num[i] = 1;
    } 
    else if(g_line[i] - g_debugLine[i] > 0){
      b_kosu++;
      num[i] = 2;
    }
  }
  if(w_kosu > 0){// limit is 4 >= w_kosu >= 0
    if(w_kosu == 1){
      if(num[0] == 1)  state = 1; // ラインが左側
      else if(num[3] == 1) state = 2; //ラインが右側
      else state = 3; //ロボットがラインの間
    }
    if(w_kosu == 2){
           if(num[0] == 1 && num[1] == 1) state = 4;//ラインが左二つの上
      else if(num[2] == 1 && num[3] == 1) state = 5;//ラインが右二つの上
      else if(num[1] == 1 && num[2] == 1) state = 6;//ラインが後ろ二つの上
      else if(num[0] == 1 && num[3] == 1) state = 7;//ラインが前二つの上
      else if(num[0] == 1 && num[2] == 1) state = 8;//ラインが左上、右下
      else if(num[2] == 1 && num[3] == 1) state = 9;//ラインが左下, 右上
    }
    else state = 10;//角にロボットがいるとき
  }
  else if(b_kosu > 0){
    if(b_kosu == 1){
      if(num[0] == 2)  state = 11; // ラインが左側
      else if(num[3] == 2) state = 12; //ラインが右側
      else state = 13; //ロボットがラインの間
    }
    if(b_kosu == 2){
           if(num[0] == 2 && num[1] == 2) state = 14;//ラインが左二つの上
      else if(num[2] == 2 && num[3] == 2) state = 15;//ラインが右二つの上
      else if(num[1] == 2 && num[2] == 2) state = 16;//ラインが後ろ二つの上
      else if(num[0] == 2 && num[3] == 2) state = 17;//ラインが前二つの上
      else if(num[0] == 2 && num[2] == 2) state = 18;//ラインが左上、右下
      else if(num[2] == 2 && num[3] == 2) state = 19;//ラインが左下, 右上
    }
    else state = 20;
  } 
  /* if (g_use_lcd == 1)
  {
    slcd.setCursor(0,0);
    slcd.print("L");
    for (int j = 0; j < LINE_NUM; j++)
    {
      if (j == 4) {
        slcd.setCursor(0,1);
        slcd.print("L");
      }
      slcd.print(g_line[j], DEC);
      slcd.print(" ");
    }
  }*/
  return state;
}
// 地磁気センサからデータ取得
unsigned int getCompass()
{
  return getDir(0);
}

// 超音波センサからデータ取得．PingあるいはSeedo studioの超音波センサしか使えない．
unsigned int getPing(int pos)
{
  // establish variables for duration of the ping,
  // and the dist；nce result in inches and centimeters:
  long duration, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(PING_PIN[pos], OUTPUT);
  digitalWrite(PING_PIN[pos], LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN[pos], HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN[pos], LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(PING_PIN[pos], INPUT);
  duration = pulseIn(PING_PIN[pos], HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);

  // error
  if (cm > 400)  cm = 999;
  if (cm <=   0) cm = 999;

  return cm;
}

// サーボモータを使ったキック．



// モータの制御
// rf: right forward, rb: right back, lb: left back, lf: left forward
/* void motor(int rf, int rb, int lb, int lf)
 {
 motorRF(rf);
 motorRB(rb);
 motorLB(lb);
 motorLF(lf);
 } */


// モータ左前Left Forwardを回転させる

// モータ左後Left Backwardを回転させる
/* void motorRB(double val) {
 int value = - 2.55 * val;
 if (value >=  255) value =  255;
 if (value <= -255) value = -255;
 
 if(value > 10){ //順方向
 analogWrite(PIN_5, value);
 analogWrite(PIN_4, 0);
 
 }
 else if(value < -10){ //逆方向
 analogWrite(PIN_5, 0);
 analogWrite(PIN_4, abs(value));
 }
 else{ //ブレーキを使用しないで停止
 digitalWrite(PIN_5,  LOW);
 digitalWrite(PIN_4, LOW);
 }
 } */

// モータ右Right Forwardを回転させる
/* void motorRF(double val) {
 int value = 2.55 * val;
 if (value >=  255) value =  255;
 if (value <= -255) value = -255;
 
 if(value > 10){ //順方向
 analogWrite(PIN_7, value);
 analogWrite(PIN_6, 0);
 
 }
 else if(value < -10){ //逆方向
 analogWrite(PIN_7, 0);
 analogWrite(PIN_6, abs(value));
 
 }
 else{ //ブレーキを使用しないで停止
 digitalWrite(PIN_7, LOW);
 digitalWrite(PIN_6, LOW);
 }
 } */

// モータ右後Right Backwardを回転させる
/* void motorLF(double val) {
 int value = 2.55 * val;
 if (value >=  255) value =  255;
 if (value <= -255) value = -255;
 
 if(value > 10){ //順方向
 analogWrite(PIN_9, value);
 analogWrite(PIN_8, 0);
 
 }
 else if(value < -10){ //逆方向
 analogWrite(PIN_9, 0);
 analogWrite(PIN_8, abs(value));
 
 }
 else{ //ブレーキを使用しないで停止
 digitalWrite(PIN_9, LOW);
 digitalWrite(PIN_8, LOW);
 }
 } */

// マイクロ秒(10^-6)をセンチメートルに変換．超音波センサ用
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}



// 地磁気センサの設定
void setupCompass()
{
  Wire.begin();   // I2C setup 
#ifdef HMC6352
  //Continuous Modeに設定する
  Wire.beginTransmission(COMPASS_ADDRESS);
  //RAM書き込み用コマンド
  Wire.write('G');
  //書き込み先指定
  Wire.write(0x74);
  //モード設定
  Wire.write(0x72);
  //通信終了
  Wire.endTransmission();
  //処理時間
  delayMicroseconds(70);
#endif
}

//  LCDのチェック．LCDが使えるか100ループに１度チェック
void setupLCD()
{
  if (slcd.beginAndCheck()) {
    g_use_lcd = 1;
    welcomeMessage();  // システム起動メッセージ
  }
  else {
    slcd.noPower();
    g_use_lcd = 0;
  }
}


// PWM用ピンの設定
/*void setupPin()
{
  // PWMの各ピンのモードを出力にセット
  pinMode(PIN_2, OUTPUT);
  pinMode(PIN_3, OUTPUT);
  pinMode(PIN_4, OUTPUT);
  pinMode(PIN_5, OUTPUT);
  pinMode(PIN_6, OUTPUT);
  pinMode(PIN_7, OUTPUT);
  pinMode(PIN_8, OUTPUT);
  pinMode(PIN_9, OUTPUT);
}*/

// システム起動メッセージ
void welcomeMessage()
{
  slcd.setCursor(0,0);
  slcd.blink();
  slcd.print("Welcome to D.K.T. !");
  slcd.setCursor(0,1);
  slcd.print("System Start");
  delay(1000);
  slcd.clear();
}

//////////////////////////
// ボールセンサのチェック
void checkBallSensor2()
{  

  static int ball[8] = { 
    0,0,0,0,0,0,0,0                    };
  // measure the ball sensor
  for (int i = 0; i < BALL_SENSOR_NUM; i++)
  {
    if (ball[i] < getBall(i))
    {
      ball[i] = getBall(i);
    }
  }
  if (g_use_lcd == 1)
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
///ボールしきい値デバッグ///
void checkBallSensor3()
{
  static int ball[BALL_SENSOR_NUM] = {
    0, 0, 0, 0, 0, 0, 0, 0                                          };

  // measure the ball sensor
  for (int i = 0; i < BALL_SENSOR_NUM; i++)
  {
    if(getBall(i) > ball[i])
    {
      ball[i] = getBall(i);
    }
  }

  if (g_use_lcd == 1)
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

int getMaxball()
{
  int t_max = -1;
  g_no = 8;  
  for(int i = 0; i < BALL_SENSOR_NUM; i++) {
    g_ball[i] =-1;
  }

  for(int i = 0; i < BALL_SENSOR_NUM; i++)
  {
    g_ball[i] = getBall(i);
    if(g_ball[i] - g_debugBall[i] >= t_max)
    {
      t_max = g_ball[i];
      g_no = i;
      //return t_max = g_ball[i];
    }
  }
  g_max = t_max;
  return g_no;
}
void goHome()
{ 
  g_dist[1] = getPing(1);
  g_dist[2] = getPing(2);
  g_dist[3] = getPing(3);
  if(g_dist[2] > 70)
  {
    moveAngle4ch( 60, 0 , 180 );
  }
  else if(g_dist[1] >= 40 && g_dist[3] >= 40)
  {
    motor4ch(0,0,0,0);
  }
  else if(g_dist[1] > g_dist[3])
  {
    moveAngle4ch( 20 , 0 , 270 );
  }
  else
  {
    moveAngle4ch( 20 , 0 , 90);
  }
}
void getBallDir()
{ 
  int t_max = -1 , t_max2 = -1 , no = -1 , no2 = -1;
  for(int i ; i < 8 ; i++)
  {
    if(getBall(i) - g_debugBall[i] > t_max)
    {  
      t_max = getBall(i);
      no = i;
    }
    else if(getBall(i) - g_debugBall[i] > t_max2)
    {
      t_max2 = getBall(i);
      no2 = i;
    }
  }
  //int value = t_max2 / t_max * 22.5;
  //if(no > no2) value *= -1;
  //int value2 = no * 45;
  //g_BallDir = value + value2;
  slcd.setCursor(0,1);
  slcd.print("no");
  slcd.print(no, DEC);
  slcd.print(" ");
}
void main_prog(){
  //checkDist();
    getMaxball();
    int line = checkLine();
    if(line > 0 && 5 > line) motcom4ch(0,0,0,0);
    else if(line > 4 && 9 > line) {
      motcom4ch(30,0,270,70);
      delay(300);
    }
    //if(line == 1)  motcom4ch(0,0,0,0);
    else{
      if(g_no == 0)  motcom4ch(30,0,90,70);
      else if(g_no >= 1 && g_no <= 4) motor4ch(-40, 40, 40,70);
      else if(g_no >= 5 && g_no <= 7) motor4ch(40,-40,-40,70);
      else motor4ch(0,0,0,70);
    }
}
void PID()
{ 
  int last_diff = g_diff;
  static int target = getCompass();
  g_diff = target - getCompass();
  if(g_diff >= 180) g_diff -= 360;
  else if(g_diff < - 180) g_diff += 360;
  float pro = g_diff / 5.4;
  g_pid = pro + 7;
}
boolean getBotton(){
  if(digitalRead(BOTTON_PIN) == HIGH)  return  true;
  else {
    slcd.setCursor(0,1);
    slcd.print("PUSH to Start");
    return false;
  }
}


//////////////////////////////////////////////////
///////////// New functions //////////////////////
/////////////////////////////////////////////////

// ダイセン4chモータドライバ用のサブ関数
String motor4ChSub(int no, int power)
{
  int val = 0;

  String StringX = 0;
  String StringY = 0;
  String StringZ = 0;
  String String0 = 0;

  StringX = String(no);
  val = power; //*100/255;
  if(val >= -100 && val < 0)
  {
    StringY = String('R');
    val = abs(val);
    StringZ = String(val);
  }
  else if(val >= 0 && val <= 100)
  {
    StringY = String('F');
    StringZ = String(val);
  }
  else
  {
    StringY = String('F');
    StringZ = String(0);
  }

  if(val < 10)
  {
    String0 = String("00");
    StringZ = String0 + StringZ;
  }
  else if(val < 100)
  {
    String0 = String('0');
    StringZ = String0 + StringZ;
  }
  else
  {
  }
  return StringX + StringY + StringZ;
}

// rb: 1right back, lb: 2left back, lf: 3left forward, rf: 4 right forward
void motor4ch(int power_rb,int power_lb,int power_lf,int power_rf)
{
  String StringA = 0;
  String StringB = 0;
  String StringC = 0;
  String StringD = 0;

  StringA = motor4ChSub(1,-power_rb);
  StringB = motor4ChSub(2,-power_lb);
  StringC = motor4ChSub(3,power_lf);
  StringD = motor4ChSub(4,power_rf);
 
  //Serial.println(StringA+StringB+StringC+StringD); 
  //Serial1.println(StringA+StringB+StringC+StringD); 
  Serial2.println(StringA+StringB+StringC+StringD); 
}


// vx [cm/s],  vy[cm/s],  w[deg/s]
// 4輪オムニ。モータがx軸に対して45度傾いて設置。
// 通常のオムニホイール
/*void moveXYW45(int vx, int vy, int w)
 {
 float k = 0.7071; // sqrt(2)/2, d is hankei
 float v_rf, v_rb, v_lb, v_lf;
 
 v_rf =  vx - vy + w * 0.22 ; // 1.414 * 9 * 3.14/180.0
 v_rb = -vx - vy + w * 0.22 ; 
 v_lb = -vx + vy + w * 0.22 ; 
 v_lf =  vx + vy + w * 0.22; 
 //モーターパワーリミッター//
 if (v_rf >  MAX_POWER) v_rf =  MAX_POWER;
 if (v_rb >  MAX_POWER) v_rb =  MAX_POWER;
 if (v_lb >  MAX_POWER) v_lb =  MAX_POWER;
 if (v_lf >  MAX_POWER) v_lf =  MAX_POWER;
 if (v_rf < -MAX_POWER) v_rf = -MAX_POWER;
 if (v_rb < -MAX_POWER) v_rb = -MAX_POWER;
 if (v_lb < -MAX_POWER) v_lb = -MAX_POWER;
 if (v_lf < -MAX_POWER) v_lf = -MAX_POWER;
 
 motor(v_rf, v_rb, v_lb, v_lf);  
 }*/

// 4輪オムニ。モータがx軸に対して30度傾いて設置。
// 前後進の高速型オムニ Japan Open 2013用
// vx [cm/s],  vy[cm/s],  w[deg/s]
void moveXYW30(int vx, int vy, int w)
{
  // sin(30deg) = 0.5, cos(30deg) =  0.866
  float s30 = 0.5, c30 = 0.866;
  float k = 0.7071; // sqrt(2)/2, d is hankei
  float v_rf, v_rb, v_lb, v_lf;

  // ４輪オムニの逆運動学
  // vi = - sin(theta_i) * vx + cos(theta_i) * vy + Rw
  // vx：x方向の速度, vy：y方向の速度, Rw:回転速度
  // v_rf =  s30 * vx - c30 * vy + w * 0.22 ; // 1.414 * 9 * 3.14/180.0
  //v_rb = -(-s30 * vx - c30 * vy + w * 0.22) ; 
  //v_lb = -(-s30 * vx + c30 * vy + w * 0.22) ; 
  //v_lf =  s30 * vx + c30 * vy + w * 0.22; 

  v_rf = - s30 * vx + c30 * vy + w * 0.22 ; // 1.414 * 9 * 3.14/180.0
  v_rb =   s30 * vx + c30 * vy + w * 0.22 ; 
  v_lb = - s30 * vx + c30 * vy + w * 0.22; 
  v_lf =   s30 * vx + c30 * vy + w * 0.22; 
  //モーターパワーリミッター//
  if (v_rf >  MAX_POWER) v_rf =  MAX_POWER;
  if (v_rb >  MAX_POWER) v_rb =  MAX_POWER;
  if (v_lb >  MAX_POWER) v_lb =  MAX_POWER;
  if (v_lf >  MAX_POWER) v_lf =  MAX_POWER;
  if (v_rf < -MAX_POWER) v_rf = -MAX_POWER;
  if (v_rb < -MAX_POWER) v_rb = -MAX_POWER;
  if (v_lb < -MAX_POWER) v_lb = -MAX_POWER;
  if (v_lf < -MAX_POWER) v_lf = -MAX_POWER;

  motor4ch(v_rb, v_lb, v_lf, v_rf);  
}


// 指定したロボットからみた角度へ進む．正面０度，時計回り．単位は度
// ダイセン4chモータドライバ用
/*void moveAngle4ch(int power, int power_angle, int angle)
{
  float theta = angle * 3.14159 / 180;
  float vx = power * sin(theta);
  float vy = power * cos(theta);

  moveXYW30(vx, vy, power_angle);
}*/
int getxpos(){
  int line_state = checkLine();
  int x_judge ;
  int posx;
  g_dist[1] *= cos(DEG2RAD(abs(g_diff)));
  g_dist[3] *= cos(DEG2RAD(abs(g_diff)));
  if(15 >= 160 - g_dist[1] - g_dist[3] && 160 - g_dist[1] - g_dist[3] >= -15 )  x_judge = 0;//正確な位置産出
  else x_judge = 1;//正確な位置計測不可
  if(x_judge == 0){
    posx = -160 + g_dist[1] + g_dist[3];
    if(-40 > posx ){
      if(line_state == 1 ) posx = -40;
      else if(line_state == 2 || line_state == 3) posx = -50;
      else if(line_state == 4) posx = -60;
    }
    else if(posx > 40 ){
      if(line_state == 1)  posx = 40;
      else if(line_state == 2 || line_state == 3) posx = 50;
      else if(line_state == 4) posx = 60;
    }
  }
  else posx = 999;
  return posx;  
}
void movevec105(int vx , int vy , int spin ,int kick){ 
 float m_r, m_b,m_l;//clockwise
 m_r =   vx * sin(DEG2RAD(15))  - vy * cos(DEG2RAD(15))  - 9.6 * DEG2RAD(spin) * 21;
 m_b =  -vx * sin(DEG2RAD(270)) + vy * cos(DEG2RAD(270)) +  9.6 * DEG2RAD(spin) * 21;
 m_l =  -vx * sin(DEG2RAD(165)) + vy * cos(DEG2RAD(165)) + 9.6 * DEG2RAD(spin) * 21 ;
 motor4ch(m_r, m_b, m_l, kick);
}

void motcom4ch(int power, int spin, int angle, int kick){ //spin = rad/sec
 if(power > 80) power = 80;//motor limit
 else if(power < -80 ) power = -80;//motor limit
 float theta = DEG2RAD(angle);
 float vx = power * cos(theta);
 float vy = power * sin(theta);
 movevec105(vx, vy, spin, kick);
}
void moveAngle4ch(int power, int spin, int angle){
 if(power > 80) power = 80;//motor limit
 else if(power < -80 ) power = -80;//motor limit
 float theta = angle * 3.14159 / 180;
 float vx = power * cos(theta);
 float vy = power * sin(theta);
 movevec105(vx, vy, spin , 0);
}
int saint9(int court){
  int b_dir = getMaxball();
  int power = 50;
  double teisu;
  if(court == 0) teisu = 0.09;
  else teisu = 1.3;
  double spin1 = DEG2RAD(g_diff) * teisu;
  double spin = spin * 1.5;
    if(g_diff > 80 || g_diff < -80){ motor4ch(-spin1 * 9.6 * 21,spin1 * 9.6* 21,spin1* 9.6*21,0);
    }
    else {switch(b_dir){
      case 0:
        if(g_max > 800){
          if(10 > g_dist[0] ){
            if( 30 > g_dist[1] ) {
              motor4ch(60,-60,-60,0);
              delay(18);
            }
            else if(30 > g_dist[3]) {
              motor4ch(-60,60,60,0);
              delay(18);
            }
            else motcom4ch(power,spin,90,0);
          }
          else if(g_dist[0] > 20){
            switch(g_kick){
              case 0 :
                motcom4ch(power - 10,spin,90,-70);
                delay(20);
                g_kick = 1;
                break;
              case 1:
                motcom4ch(power - 10,spin,90,70);
                delay(20);
                g_kick = 2;
                break;
              case 2:
                motcom4ch(power,spin,90,0);
                delay(20);
                g_kick = 0;
                break;
            }
          }
          else motcom4ch(power,spin,90,0);
        }
        else motcom4ch(power,spin,90,0);
        break;
      case 1:
        if(g_max > 800){
          if( 12 > g_dist[1] )  motcom4ch(power,spin,90,0);
          else motcom4ch(power,spin,180,0);
        }
        else motcom4ch(power,spin,135,0);
        break;
    case 2:
      if(g_max > 800){
        if(30 >= g_dist[2]) motcom4ch(power,spin,200,0);
        else motcom4ch(power,spin,270,0);
      }
      else motcom4ch(power,spin,180,0);
      break;
    case 3:
      if(g_max > 800)  motcom4ch(power,spin,315,0);
      else motcom4ch(power,spin,225,0);
      break;
    case 4:
      if(g_max > 800){  
        if(g_dist[1] >= g_dist[3])  motcom4ch(power,spin,180,0); 
        else motcom4ch(power,spin,0,0);
      }
      else motcom4ch(power,spin,270,0);
      break;
    case 5:
      if(g_max > 800)  motcom4ch(power,spin,225,0);
      else motcom4ch(power,spin,315,0);
      break;
    case 6:
      if(g_max > 800)  {
        if(30 >= g_dist[2]) motcom4ch(power,spin,340,0);
        motcom4ch(power,spin,270,0);
      }
      else  motcom4ch(power,spin,0,0);
      break;
    case 7:
      if(g_max > 800){
        if(12 > g_dist[3] ) motcom4ch(power,spin,90,0); 
        else motcom4ch(power,spin,0,0);
      }
      else motcom4ch(power,spin,45,0);
      break;
    default:
      if(g_dist[2] > 30 )  motcom4ch(power-10,spin , 270,0); 
      else if(30 > g_dist[1] )  motcom4ch(power-10,spin,0,0);
      else if(30 > g_dist[3] )  motcom4ch(power-10,spin,180,0);
      else motcom4ch(0,spin,0,0);
      break;
    }
    }
  return 0;
}    
void pid_control(){
  double ang_vel = 40 * PI;
  double accel = 2.963;
  double max_vel  = 6.28;
  
}





