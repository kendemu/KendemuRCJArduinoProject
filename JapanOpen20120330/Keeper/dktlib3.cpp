/*********************************************************************/
// The DKT Robot Library Ver1   2012-02-29
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要

#include "dktlib3.h"

int g_debugBall[8] = {
  55 , 32 , 108 ,73 , 69, 60 , 52 , 134};
int g_BallDir = 0;
double g_Vx , g_Vy;
int g_ball[8];
int g_dist[4];
int g_no = 999;
int g_pid;
int g_diff;
int g_loop;





void turn(int power)
{
  motorLF(power);
  motorLB(power);
  motorRB(power);
  motorRF(power);
}
//
// ボールセンサのチェック
void checkBallSensor()
{
  int ball[BALL_SENSOR_NUM] = {
    999, 999, 999, 999, 999, 999, 999, 999                                    };

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
    999, 999, 999, 999                                    };

  // measure the ultra sonic sensor
  for (int i = 0; i < PING_NUM; i++)
  {
    g_dist[i] = getPing(i);
  }

  /*if (g_use_lcd == 1)
   {
   slcd.setCursor(0,0);
   slcd.print("Pg:");
   for (int j = 0; j < 2; j++)
   {
   slcd.print(dist[j], DEC);
   slcd.print(" ");
   }
   slcd.setCursor(0,1);
   for (int j = 2; j < PING_NUM; j++)
   {
   slcd.print(dist[j], DEC);
   slcd.print(" ");
   }
   }*/
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

// 地磁気センサからデータ取得
unsigned int getCompass()
{
  return getDir(0);
}

// 超音波センサからデータ取得．PingあるいはSeedo studioの超音波センサしか使えない．
unsigned int getPing(int pos)
{
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
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
void kick()
{
  myservo.attach(SERVO_PIN);
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


// モータの制御
// rf: right forward, rb: right back, lb: left back, lf: left forward
void motor(int rf, int rb, int lb, int lf)
{
  motorRF(rf);
  motorRB(rb);
  motorLB(lb);
  motorLF(lf);
}


// モータ左前Left Forwardを回転させる
void motorLB(double val) {
  int value = 2.55 * val;
  if (value >=  255) value =  255;
  if (value <= -255) value = -255;

  if(value > 10){ //順方向
    analogWrite(PIN_3, value);
    analogWrite(PIN_2, 0);
  }
  else if(value < -10){ //逆方向
    analogWrite(PIN_3, 0);
    analogWrite(PIN_2, abs(value));

  }
  else{ //ブレーキを使用しないで停止
    digitalWrite(PIN_3, LOW);
    digitalWrite(PIN_2, LOW);
  }
}

// モータ左後Left Backwardを回転させる
void motorRB(double val) {
  int value = 2.55 * val;
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
}

// モータ右Right Forwardを回転させる
void motorRF(double val) {
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
}

// モータ右後Right Backwardを回転させる
void motorLF(double val) {
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
}

// マイクロ秒(10^-6)をセンチメートルに変換．超音波センサ用
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void Stop()
{
  motorLF(0);
  motorLB(0);
  motorRF(0);
  motorRB(0);
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
void setupPin()
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
}

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
    0,0,0,0,0,0,0,0              };
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
    0, 0, 0, 0, 0, 0, 0, 0                                    };

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

void getMaxball()
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
    }
  }
}

void Keeper()
{
  static int g_loop = 0;
  getMaxball();
  PID();
  if (g_diff > 40 || g_diff < -40) turn(g_pid);
  else
  {
    if(g_no == 0 && g_ball[0] > 300) 
    {
      kick();
      moveAngle( 50 , 0 , 0);
    } 
    else if(g_no == 1) moveAngle( 50 , 0 , 270 );

    else if(g_no == 2) moveAngle( 50 , 0 , 270 );

    else if(g_no == 3 && g_ball[3] > 200) moveAngle( 50 , 0 , 135 );

    else if(g_no == 4 && g_ball[4] > 200) moveAngle( 0 , 0 , 0 );

    else if(g_no == 5 && g_ball[5] > 200) moveAngle( 50 , 0 , 225 );

    else if(g_no == 6) moveAngle( 50 , 0 , 90 );

    else if(g_no == 7) moveAngle( 50 , 0 , 90 );

    else goHome();
    
  }
  g_loop++;
}

void goHome()
{ 
  static int g_loop = 0;
  g_loop++;
  g_dist[1] = getPing(1);
  g_dist[2] = getPing(2);
  g_dist[3] = getPing(3);
  if ( g_loop == 1 ) 
   {
     Stop();
     delay(500);
     return;
   }
   
  if(g_dist[2] > 15)
  {
    moveAngle( 80 , 0 , 180 );
  }
  else if(g_dist[1] >= 40 && g_dist[3] >= 40)
  {
    Stop();
  }
  else if(g_dist[1] > g_dist[3])
  {
    moveAngle( 20 , 0 , 270 );
  }
  else
  {
    moveAngle( 20 , 0 , 90);
  }
}

void Forward()
{
  static int g_loop = 0;
  getMaxball();
  PID();

  if (g_diff > 45 || g_diff < -45) turn(g_pid); 
  else
  { 
    if(g_no == 0) 
    {
      if(g_ball[0] > 300)
      {  
        g_dist[0] = getPing(0);
        g_dist[1] = getPing(1);
        g_dist[3] = getPing(3);
        if (g_dist[0] < 12)
        {
          if(g_dist[1] < 40)
          {
            turn(80);
            delay(125);
          }
          else if (g_dist[3] < 40)
          { 
            turn(-80);
            delay(125);
          } 
        }
        else if(g_dist[1] >= 35 && g_dist[3] >= 35)
        {
          kick();
          moveAngle( 50 , 0 , 0 );
        }     
      }
      else moveAngle( 50 , 0 , 0 );
    } 
    else if(g_no == 1)
    { 
      g_dist[1] = getPing(1);
      if (g_dist[1] < 10) moveAngle (60 , 0 , 0);
      else if(g_ball[1] > 300)  moveAngle( 60 , 0 , 270 );
      else moveAngle( 60 , 0 , 315);
    } 
    else if(g_no == 2)
    {
      if(g_ball[2] > 300) moveAngle( 60 , 0 , 180);

      else moveAngle( 60 , 0 , 270);
    }
    else if(g_no == 3 ) moveAngle( 60 , 0 , 135 );



    else if(g_no == 4 ) 
    {
      g_dist[1] = getPing(1);
      g_dist[3] = getPing(3);
      if (g_dist[1] > g_dist[3]) moveAngle( 60 , 0 , 270 );

      else moveAngle(60 , 0 , 90 );
    }
    else if(g_no == 5)
    { 
      if (g_ball[5] > 300) moveAngle( 60 , 0 , 225 );

      else moveAngle( 60 , 0 , 135);
    }
    else if(g_no == 6) moveAngle( 60 , 0 , 180 );


    else if(g_no == 7) 
    {
      g_dist[3] = getPing(3);
      if (g_dist[3] < 10) moveAngle (60 , 0 , 0);
      else if (g_ball[7] > 300) moveAngle( 60 , 0 , 90 );
      else moveAngle ( 60 , 0 , 45);
    }
    else goHome();
  }
  g_loop++;
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
  int value = t_max2 / t_max * 22.5;
  if(no > no2) value *= -1;
  int value2 = no * 45;
  g_BallDir = value + value2;
  slcd.setCursor(0,1);
  slcd.print("BallDir");
  slcd.print(g_BallDir, DEC);
  slcd.print(" ");
}
void PID()
{ 
  int last_diff = g_diff;
  static int target = getCompass();
  g_diff = target - getCompass();
  if(g_diff > 180) g_diff -= 360;
  else if(g_diff < - 180) g_diff += 360;
  float pro = g_diff * 1 / 4.8;
  float integral = 0;
  if (last_diff + 2 >= g_diff && last_diff - 2 <= g_diff) integral++;
  if (g_diff < 0) integral *= -1;
  g_pid = pro + integral;
}
//////////////////////////////////////////////////
///////////// New functions //////////////////////
/////////////////////////////////////////////////

// vx [cm/s],  vy[cm/s],  w[deg/s]
void moveXYW(int vx, int vy, int w)
{
  float k = 0.7071; // sqrt(2)/2, d is hankei
  float v_rf, v_rb, v_lb, v_lf;

  v_rf =  vx - vy + w * 0.22 ; // 1.414 * 9 * 3.14/180.0
  v_rb = -vx - vy + w * 0.22 ; 
  v_lb = -vx + vy + w * 0.22 ; 
  v_lf =  vx + vy + w * 0.22 ; 
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
}

// 指定したロボットからみた角度へ進む．正面０度，時計回り．単位は度
void moveAngle(int power, int power_angle, int angle)
{
  float theta = angle * 3.14159 / 180;
  float pi4 = 3.14159/4.0;
  float vx = power * sin(theta);
  float vy = power * cos(theta);

  moveXYW(vx, vy, power_angle);

}







