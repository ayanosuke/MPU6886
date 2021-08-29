//--------------------------------------------------------------------------------
// M5Stack Gray + MPU6886 6軸IMU Logger スケッチ
// Copyright(C)'2021 Ayanosuke(Maison de DCC)
// [MPU6886_Logger.ino]
//
// 3軸重力加速系、3軸ジャイロスコープ値をTextで表示
// ピッチ（赤線）、ロール（白線）、ヨー（黄線）をトレンドグラフ表示
// M5Stack 画面サイズ 320x240
//
// [参考]【M5Stack】第4回 M5Stack Fire内蔵のIMU（MPU6886）を使って姿勢角算出
// スケッチ ライブラリを使用のベースをお借りしています。
// https://shizenkarasuzon.hatenablog.com/entry/2020/05/21/013307
//
// [参考]３次元ベクトルの回転「ロール・ピッチ・ヨー」
// https://watako-lab.com/2019/01/23/roll_pitch_yaw/
//
// http://maison-dcc.sblo.jp/ http://dcc.client.jp/ http://ayabu.blog.shinobi.jp/
// https://twitter.com/masashi_214
//
// DCC電子工作連合のメンバーです
// https://desktopstation.net/tmi/ https://desktopstation.net/bb/index.php
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
//
// 2021/08/29 初版作成
//--------------------------------------------------------------------------------

#define M5STACK_MPU6886 
//#define M5STACK_MPU9250 
//#define M5STACK_MPU6050
//#define M5STACK_200Q

#include <M5Stack.h>        // このヘッダファイルを #define M5STACK_MPU6886 より前に置いたらコンパイル通らなくなります  

#define XHMax 60            // 赤線用の垂直位置
#define YHMax 120           // 白線用の垂直位置
#define ZHMax 180           // 黄線用の垂直位置

#define RAD_TO_DEG 57.324

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float pitch, roll, yaw;
float my_pitch, my_roll, my_yaw;
float Temp;

int Xdot[320];
int Ydot[320];
int Zdot[320];
float RngTbl[5] ={ 1.0 , 2.5 , 5.0 , 0.25 , 0.5 };
float RngTblAcc[5] ={ 1.0 , 2.5 , 5.0 , 10.0 , 15.0 };
int Rng = 0;
unsigned long PreviosTime = 0;

void setup(){
  M5.begin();
  M5.Power.begin();
  M5.Speaker.write(0); // スピーカーをオフする これ入れないとカタカタ言う
  M5.IMU.Init();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(1);

  PreviosTime = millis();
}

void loop() {

  int x = 0;
  static char sel = '\0';

  if( (millis() - PreviosTime ) >= 100 ){  // 10:10msec 前回から10ms経過したかチェック
    PreviosTime = millis();
    sel = btnchk();
  }

  // 3軸ジャイロスコープ 
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("gyro=(%5.1f, %5.1f, %5.1f)", gyroX, gyroY, gyroZ);

  // 3軸重力加速度計
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  M5.Lcd.setCursor(0, 50);
  M5.Lcd.printf("acc=(%5.1f, %5.1f, %5.1f)", accX, accY, accZ);
  
  M5.IMU.getAhrsData(&pitch,&roll,&yaw);
  M5.Lcd.setCursor(0, 100);
  M5.Lcd.printf("PRY=(%5.1f, %5.1f, %5.1f)", pitch, roll, yaw);

  // 姿勢情報
  //reference = https://myenigma.hatenablog.com/entry/2015/11/09/183738
  float my_roll = atan(accY / accZ) * RAD_TO_DEG;
  float my_pitch = atan(-accX / sqrtf(accY*accY + accZ*accZ)) * RAD_TO_DEG;
  M5.Lcd.setCursor(0, 150);
  M5.Lcd.printf("pitch = %5.1f, roll = %5.1f", my_pitch, my_roll);

  // 温度情報
  M5.IMU.getTempData(&Temp);
  M5.Lcd.setCursor(0, 200);
  M5.Lcd.printf("Temperature=%.2f C", Temp);

  switch(sel){  // ボタンの状態でグラフ表示する内容を切り替える
    case 'A': // 3軸ジャイロスコープロギング
              Xdot[319] = gyroX * RngTbl[Rng];
              Ydot[319] = gyroY * RngTbl[Rng];
              Zdot[319] = gyroZ * RngTbl[Rng];
              M5.Lcd.setCursor(0, 220);
              M5.Lcd.printf("[3 axis gyroscope            ] [x%4.2f]",RngTbl[Rng]);           
              break;
    case 'B': // 3軸重力加速度計ロギング
              Xdot[319] = accX * RngTblAcc[Rng];
              Ydot[319] = accY * RngTblAcc[Rng];
              Zdot[319] = accZ * RngTblAcc[Rng];
              M5.Lcd.setCursor(0, 220);
              M5.Lcd.printf("[3 Axis Gravity Accelerometer] [x%4.2f]",RngTblAcc[Rng]);          
              break;              
    case 'C': // 姿勢情報ロギング
              Xdot[319] = pitch * RngTbl[Rng];
              Ydot[319] = roll * RngTbl[Rng];
              Zdot[319] = yaw * RngTbl[Rng];
              M5.Lcd.setCursor(0, 220);
              M5.Lcd.printf("[Posture information         ] [x%4.2f]",RngTbl[Rng]);         
              break;  
    default:
              Xdot[319] = gyroX * RngTbl[Rng];
              Ydot[319] = gyroY * RngTbl[Rng];
              Zdot[319] = gyroZ * RngTbl[Rng];
              M5.Lcd.setCursor(0, 220);
              M5.Lcd.printf("[3 axis gyroscope            ] [x%4.2f]",RngTbl[Rng]);           
              break;      
  }

  for( x = 0 ; x <=318 ; x++ ){ // 前回のグラフを消去
    M5.Lcd.drawLine( x, XHMax - Xdot[x], x+1, XHMax - Xdot[x+1], BLACK);
    M5.Lcd.drawLine( x, YHMax - Ydot[x], x+1, YHMax - Ydot[x+1], BLACK);
    M5.Lcd.drawLine( x, ZHMax - Zdot[x], x+1, ZHMax - Zdot[x+1], BLACK);
    Xdot[x] = Xdot[x+1];  // 表示内容をシフト処理
    Ydot[x] = Ydot[x+1];
    Zdot[x] = Zdot[x+1];
  }
  for( x = 0 ; x <=317 ; x++ ){ // 今回のグラフを表示
    M5.Lcd.drawLine( x, XHMax - Xdot[x], x+1, XHMax - Xdot[x+1], RED);
    M5.Lcd.drawLine( x, YHMax - Ydot[x], x+1, YHMax - Ydot[x+1], WHITE);
    M5.Lcd.drawLine( x, ZHMax - Zdot[x], x+1, ZHMax - Zdot[x+1], YELLOW);
  }
}

//--------------------------------------------------------------------------------
// ABCボタンを押されたかチェック
// isPressed() 現在ボタンが押されていれば1、そうでなければ0を返す
// [参考][M5Stack】第3回 LCD下部のボタンの状態を取得する
// https://shizenkarasuzon.hatenablog.com/entry/2020/05/21/012707
//--------------------------------------------------------------------------------
char btnchk(void)
{
  static char PreBtn = 'A';
  char NowBtn = '\0';
  int x;
  
  M5.update();              // update button state これを入れないとボタン情報読めない
  
  if (M5.BtnA.isPressed()) {
    NowBtn = 'A';
  }
  if (M5.BtnB.isPressed()) {
    NowBtn = 'B';
  }
  if (M5.BtnC.isPressed()) {
    NowBtn = 'C';
  }

  if(NowBtn != '\0'){       // 何か押された？    
    if(PreBtn == NowBtn){   // 同じボタンが押されたらレンジを変更
      Rng++;
      if(Rng >= 5){
        Rng = 0; 
      }
    }
    if(PreBtn != NowBtn){   // 前回と違うボタン?
#if 0
      for( x = 0 ; x <=318 ; x++ ){ // 初回グラフ初期値設定
        Xdot[x] = 0;
        Ydot[x] = 0;
        Zdot[x] = 0;
      }
      M5.Lcd.fillScreen(BLACK);
#endif
      PreBtn = NowBtn;
      Rng = 0;
    }
  }
  return PreBtn;
}
