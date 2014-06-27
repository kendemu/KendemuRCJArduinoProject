/*********************************************************************/
// The DKT Robot Library Ver0.1   2012-02-29
// slcd.beginAndCheck()はGrove Serial LCD v1.0bのbegin()を改造し，LCDが使えるときは１を返し，
// 使えないときは0を返すように変更 SerialLCDemuraライブラリが必要

#include "myrobot0.h"


// ボールセンサのチェック
void checkBallSensor()
{
  int ball[BALL_SENSOR_NUM] = {
    999, 999, 999, 999, 999, 999, 999, 999                    };

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
  dir = getDir(0);

  if (g_use_lcd==1)
  {
    slcd.setCursor(0,0);  
    slcd.print(" Dir:");
    slcd.print(dir,DEC);
    slcd.print(" ");
  }
}

//  LCDのチェック．LCDが使えるか100ループに１度チェック
void checkLCD(unsigned int step)
{
  if (step % 100 == 0) 
  {
    if (slcd.beginAndCheck()) {
      g_use_lcd = 1;        
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
    999, 999, 999, 999                    };

  // measure the ultra sonic sensor   
  for (int i = 0; i < PING_NUM; i++)
  {
    dist[i] = getPing(i);  
  }

  if (g_use_lcd == 1)
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
  }
  delay(20);
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
  int value = 0;  // ball sensor value

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
void motorLF(double val) {
  int value = 2.55 * val;
  if (value >=  255) value =  255;
  if (value <= -255) value = -255;

  if(value > 10){ //順方向
    analogWrite(PIN_2, value);
    analogWrite(PIN_3, 0);
  }
  else if(value < -10){ //逆方向
    analogWrite(PIN_2, 0);
    analogWrite(PIN_3, abs(value));

  }
  else{ //ブレーキを使用しないで停止
    digitalWrite(PIN_2, LOW);
    digitalWrite(PIN_3, LOW);
  }
}

// モータ左後Left Backwardを回転させる
void motorLB(double val) {
  int value = 2.55 * val;
  if (value >=  255) value =  255;
  if (value <= -255) value = -255;

  if(value > 10){ //順方向
    analogWrite(PIN_4, value);
    analogWrite(PIN_5, 0);

  }
  else if(value < -10){ //逆方向
    analogWrite(PIN_4, 0);
    analogWrite(PIN_5, abs(value));
  }
  else{ //ブレーキを使用しないで停止
    digitalWrite(PIN_4,  LOW);
    digitalWrite(PIN_5, LOW);
  }
}

// モータ右Right Forwardを回転させる
void motorRF(double val) {
  int value = 2.55 * val;
  if (value >=  255) value =  255;
  if (value <= -255) value = -255;

  if(value > 10){ //順方向
    analogWrite(PIN_6, value);
    analogWrite(PIN_7, 0);

  }
  else if(value < -10){ //逆方向
    analogWrite(PIN_6, 0);
    analogWrite(PIN_7, abs(value));

  }
  else{ //ブレーキを使用しないで停止
    digitalWrite(PIN_6, LOW);
    digitalWrite(PIN_7, LOW);
  }
}

// モータ右後Right Backwardを回転させる
void motorRB(double val) {
  int value = 2.55 * val;
  if (value >=  255) value =  255;
  if (value <= -255) value = -255;

  if(value > 10){ //順方向
    analogWrite(PIN_8, value);
    analogWrite(PIN_9, 0);

  }
  else if(value < -10){ //逆方向
    analogWrite(PIN_8, 0);
    analogWrite(PIN_9, abs(value));

  }
  else{ //ブレーキを使用しないで停止
    digitalWrite(PIN_8, LOW);
    digitalWrite(PIN_9, LOW);
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

// 前進後進
void move(int power )
{
  // forward
  motorLF(-power);
  motorLB(-power);
  motorRF( power);
  motorRB( power);
}

// 前進
void moveForward(int power )
{
  // forward
  int offset = 5;
  motorLF(-power);
  motorLB(-power);
  motorRF( power+offset);
  motorRB( power+offset);
}

// 後進
void moveBack(int power)
{
  // forward
  motorLF(power);
  motorLB(power);
  motorRF(-power);
  motorRB(-power);
}

// 地磁気センサの設定
void setupCompass()
{
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
  motorLF(-power);
  motorLB(-power);
  motorRF(-power);
  motorRB(-power);
}

// 左回転
void turnLeft(int power)
{
  turnRight(-power);
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
