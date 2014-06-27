/*********************************************************************/
// The DKT Robot Library Ver1   2012-02-29
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要

#include "dktlib3.h"

int g_debugBall[8] = {
  250 , 164 , 201 ,203 , 110, 215 , 181 , 145};
double g_BallDir;
double g_Vx , g_Vy;
int g_ball[8];
int g_dist[4];



//
// ボールセンサのチェック
void checkBallSensor()
{
  int ball[BALL_SENSOR_NUM] = {
    999, 999, 999, 999, 999, 999, 999, 999                          };

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
  int dir, accel;

  // get Diretion
  dir = getCompass();
  accel = getAccel();

  if (g_use_lcd==1)
  {
    slcd.setCursor(0,0);
    slcd.print(" Dir:");
    slcd.print(dir,DEC);
    slcd.print("    ");
    slcd.setCursor(0,1);
    slcd.print("a:");
    slcd.print(accel,DEC);    
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
    999, 999, 999, 999                          };

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
  if (dno == 3) return data.W;
  else if (data.W > 359) {
    data.W = 999;
    return data.W;
  }
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

// ダイセン多機能コンパスから加速度を取得
unsigned int getAccel()
{
  return getDir(3);
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



// 前進
void moveForward(int power )
{
  // forward
  motorLF( power);
  motorLB( power);
  motorRF(-power);
  motorRB(-power);
}

// 後進
void moveBack(int power)
{
  // forward
  motorLF(-power);
  motorLB(-power);
  motorRF( power);
  motorRB( power);
}
void moveLF(int power)
{
  motorLF(0);
  motorLB( power);
  motorRF(-power);
  motorRB(0);
}
void moveRF(int power)
{
  motorLF( power);
  motorLB(0);
  motorRF(0);
  motorRB(-power);
}
void moveRB(int power)
{
  motorLF(0);
  motorLB(-power);
  motorRF( power);
  motorRB(0);
}
void moveLB(int power)
{
  motorLF(-power);
  motorLB(0);
  motorRF(0);
  motorRB( power);
}
void moveLeft(int power)
{
  motorLF(-power);
  motorLB( power);
  motorRF(-power);
  motorRB( power);
}
void moveRight(int power)
{
  motorLF( power);
  motorLB(-power);
  motorRF( power);
  motorRB(-power);
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

// 右回転
void turnRight(int power )
{
  motorLF( power);
  motorLB( power);
  motorRF( power);
  motorRB( power);
}

// 左回転
void turnLeft(int power)
{
  motorLF(-power);
  motorLB(-power);
  motorRF(-power);
  motorRB(-power);
}
void turn(int power)
{
  motorLF(power);
  motorLB(power);
  motorRF(power);
  motorRB(power);
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
    0,0,0,0,0,0,0,0    };
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

///ボールの方位///



void checkBallSensor3()
{
  static int ball[BALL_SENSOR_NUM] = {
    0, 0, 0, 0, 0, 0, 0, 0                          };

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

void gohome()
{
  if(g_dist[2] > 10)
  {
    moveBack(60);
  }
  else if(g_dist[1] >= 40 && g_dist[3] >= 40)
  {
    Stop();
  }
  else if(g_dist[1] > g_dist[3])
  {
    moveLeft(20);
  }
  else
  {
    moveRight(20);
  }
}


void loop2()
{
  static int step = 0;
  static int g_loop = 0;
  int g_diff;
  static int g_integral = 0; 
  
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

//////////////////////////////////////////////////
///////////// New functions //////////////////////
/////////////////////////////////////////////////

// vx [cm/s],  vy[cm/s],  w[deg/s]
void moveXYW(int vx, int vy, int w)
{
  float k = 0.7071; // sqrt(2)/2, d is hankei
  float v_rf, v_rb, v_lb, v_lf;
  
  v_rf =  k * vx - k * vy + w * 8.0 * 3.14 / 180.0; // 0.157
  v_rb = -k * vx - k * vy + w * 7.5 * 3.14 / 180.0; 
  v_lb = -k * vx + k * vy + w * 7.5 * 3.14 / 180.0; 
  v_lf =  k * vx + k * vy + w * 8.0 * 3.14 / 180.0; 
  
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


