/*
   STUDUINO
   (C) 2014 Artec Corporation

   本プログラムをArduino IDEで実行する場合、Studuinoライブラリを
   インストールしなければなりません。

   Studuinoライブラリのインストール方法は、下記のホームページを
   参考にしてください。
   http://www.artec-kk.co.jp/studuino

   下記に主要な関数を説明します
   --------------------------------------
   ■ artecRobotSetup関数：接続パーツ等の初期化関数です
   ■ artecRobotMain関数 ：「制御スタート」ブロックに接続したプログラムです
   ■ ARSR_*関数         ：「関数ブロック」で作成したプログラムです
   --------------------------------------
*/
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MMA8653.h>
#include <MPU6050.h>
//#include <MsTimer2.h>
#include "Studuino.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// **********************************************************************
// 宣言
// **********************************************************************
// リストの要素
struct cell_t {
  struct cell_t* next;
  float data;
};
typedef cell_t cell;
// **********************************************************************
// 定義
// **********************************************************************
//#define DCMPWR(power)  ((byte)(min(max(0, ((float)(power) * 2.55)),255)))
//
#define BMIN    (0)     // Blockプログラミング環境側のセンサーの最小値
#define BMAX    (100)   // Blockプログラミング環境側のセンサーの最大値
//#define ANAMIN  (0)     // Studuino基板のアナログセンサーの最小値
//#define ANAMAX  (1023)  // Studuino基板のアナログセンサーの最大値
#define ACCMIN  (-128)  // Studuino基板の加速度センサーの最小値
#define ACCMAX  (127)   // Studuino基板の加速度センサーの最大値
//#define PUSHSWITCH(port)       (board.GetPushSwitchValue(port))
//#define TOUCH_SENSOR(port)     (board.GetTouchSensorValue(port))
//#define LIGHT_SENSOR(port)     (map(board.GetLightSensorValue(port), ANAMIN, ANAMAX, BMIN, BMAX))
//#define SOUND_SENSOR(port)     (map(board.GetSoundSensorValue(port), ANAMIN, ANAMAX, BMIN, BMAX))
//#define IRPHOTOREFLECTOR(port) (map(board.GetIRPhotoreflectorValue(port), ANAMIN, ANAMAX, BMIN, BMAX))
#define ULTRASONIC_SENSOR()    (GetUltrasonicSensorValue())
#define DWEIGHT  (1.818)

//
//const byte SQRT  =  (0);   // √n
//const byte ABS   =  (1);   // |n|
//const byte SIN   =  (2);   // sin(n)
//const byte COS   =  (3);   // cos(n)
//const byte TAN   =  (4);   // tan(n)
//const byte LN    =  (5);   // loge
//const byte LOG   =  (6);   // log10
//const byte POWE  =  (7);   // e^
//const byte POW10 =  (8);   // 10^
//
//const word BTONE[] = {
//  BZR_C3,  BZR_CS3, BZR_D3,  BZR_DS3, BZR_E3,  BZR_F3,  BZR_FS3, BZR_G3,  BZR_GS3, BZR_A3,  BZR_AS3, BZR_B3,
//  BZR_C4,  BZR_CS4, BZR_D4,  BZR_DS4, BZR_E4,  BZR_F4,  BZR_FS4, BZR_G4,  BZR_GS4, BZR_A4,  BZR_AS4, BZR_B4,
//  BZR_C5,  BZR_CS5, BZR_D5,  BZR_DS5, BZR_E5,  BZR_F5,  BZR_FS5, BZR_G5,  BZR_GS5, BZR_A5,  BZR_AS5, BZR_B5,
//  BZR_C6,  BZR_CS6, BZR_D6,  BZR_DS6, BZR_E6,  BZR_F6,  BZR_FS6, BZR_G6,  BZR_GS6, BZR_A6,  BZR_AS6, BZR_B6,
//  BZR_C7,  BZR_CS7, BZR_D7,  BZR_DS7, BZR_E7,  BZR_F7,  BZR_FS7, BZR_G7,  BZR_GS7, BZR_A7,  BZR_AS7, BZR_B7,
//  BZR_C8,
//};
//#define TONENUM ((sizeof(BTONE)/sizeof(word))-1)
//#define BHZ(num)  (BTONE[(byte)(min(max(0, (num-48)),TONENUM))])

// for Servomotor calibration
const byte SVCD2  = (4);
const byte SVCD4  = (5);
const byte SVCD7  = (6);
const byte SVCD8  = (7);
const byte SVCD9  = (0);
const byte SVCD10 = (1);
const byte SVCD11 = (2);
const byte SVCD12 = (3);
// Min/Max of servomotor's degree
#define SVRANGE(deg)  (min(max(0, (deg)), 180))
#define SYNCSVRANGE(dly)  (min(max(0, (dly)), 20))

//定数
#define GENE_LEN 8
#define KOTAISUU 10
#define KAISUU 40
#define T_MIN 600
#define T_MAX 2000
#define POINT_MIN 0
#define POINT_MAX 45
#define A_MIN 70
#define A_MAX 100
#define A_INIT 90
#define B_MIN 40
#define B_MAX 90
#define B_INIT 90
#define C_MIN 65
#define C_MAX 115
#define C_INIT 90
#define MAX_CHILD 100
#define HENI_TIMES 10
#define BASIC_INTERVAL 200

// **********************************************************************
// プロトタイプ宣言
// **********************************************************************
// リスト
int listDelete(struct cell_t* p, int pos);              // リストの要素を削除
int listAdd(struct cell_t* p, float data);              // リストに要素を追加
int listInsert(struct cell_t *p, int pos, float data);  // リストに要素を挿入
int listReplace(struct cell_t *p, int pos, float data); // リストの要素を置き換える
int listLength(struct cell_t *p);                       // リストの長さを取得
float listItem(struct cell_t *p, int pos);              // リストの要素を取得
bool listIsContain(struct cell_t *p, float data);       // リストにデータが含まれるかどうかの確認
// 丸め処理
int scratchRound(float arg);
//// 数学処理
float math(byte opeID, float arg);              // 算術処理
//// タイマー処理関数
void resetTimer();
float getTimer();

// ロボットセットアップ処理
void artecRobotSetup();
// ロボットメイン処理
void artecRobotMain();

// **********************************************************************
// グローバル変数
// **********************************************************************
Studuino board; // Studuino基板オブジェクト
//int StartTime;  // For timer
bool IRRemoteUsed = false;  // 赤外線受信センサ使用中フラグ
bool BeepOn = false;
bool DCMotorOn = false;

//変数宣言
//int T, t1, t2, T_first, T_second; //周期
byte i,j;
int first; //1位個体
int second; //2位個体
int child[2]; //子個体
int gene; //実行中個体
int group[KOTAISUU];
int top;
byte worst;
byte bad;
double point,  point_first, point_second, point_T, point2;
int64_t point_temp;
int count;
byte flag = 0;
byte flag_mtop = 0;
double seiseki_child[2];
double seiseki_group[KOTAISUU];
double seiseki_reverse[KOTAISUU];
double seiseki_top;
byte pos;
byte child_number; //何番目の個体かを表す変数
int children[MAX_CHILD]; //個体記録用変数
//byte degreee[2][2][2][3];
byte range;
byte count_heni;
double maximum, maximum_reverse;
double roulette;
//double middle;

// **********************************************************************
// プログラム
// **********************************************************************
// ---------------------------------------
// Servomotor calibration data
// ---------------------------------------
char CalibrationData[] = { 0, 0, 0, 0, 0, 0, 0, 0,  };
// ---------------------------------------
// prototype declaration
// ---------------------------------------
void artecRobotMain();
void ARSR_foot();
// ---------------------------------------
// Global variable
// ---------------------------------------
float ARVAL_irValue;
float ARVAL_soundValue;
float ARVAL_speed;
byte port[8];
// ---------------------------------------
// Artec robot setup routine
// ---------------------------------------
void artecRobotSetup() {
  board.InitSensorPort(PORT_A0, PORT_A1, PIDULTRASONICSENSOR);
  board.InitSensorPort(PORT_A2, PIDLED);
  //board.InitI2CPort(PIDGYROSCOPE);
  board.InitDCMotorPort(PORT_M1);
  board.InitDCMotorPort(PORT_M2);
  board.InitServomotorPort(PORT_D10);
  board.InitServomotorPort(PORT_D11);
  board.InitServomotorPort(PORT_D12);
  board.InitI2CPort(PIDGYROSCOPE);
  board.SetServomotorCalibration(CalibrationData);
}
// ---------------------------------------
// Artec robot mainroutine
// ---------------------------------------

//関数宣言
void play(int);
void foot(void);
void prepare(void);
void find_top(void);
int choose_parents(void);
void crossing(void);
int child_overlap(int);
int heni(int);
byte choose_worst(void);
void comparison(void);

void artecRobotMain() {
  Serial.begin(115200);
  ARVAL_speed = 1000;
  children[0] = 0;
  children[1] = pow(2, GENE_LEN) * 3 -1;
  child_number = 0;
  port[0] = PORT_D10; // 足の付け根
  port[1] = PORT_D11; // 膝
  port[2] = PORT_D12; // かかと
//  range = SYNCSVRANGE(scratchRound(20-20))+3;
  for(i = 0; i < 100; i++){
    delay(300);
    int gyro_value = board.GetGyroscopeValue(GX_AXIS);
    Serial.println(gyro_value);
  }
  range = 0;

  // 初期個体を出鱈目に生成する
  for (i=0; i<KOTAISUU; i++) group[i] = (random(pow(2, GENE_LEN)) << GENE_LEN * 2) + (random(pow(2, GENE_LEN)) << GENE_LEN) + random(pow(2, GENE_LEN));

  //group実行
  for (i=0; i<KOTAISUU; i++) {
    child_number++;
    children[child_number] = group[i];
    play(group[i]);
    seiseki_group[i] = point;
    if (seiseki_group[i] < 1) seiseki_group[i] = 1;//comment out by Ken Takaki
  }
  find_top();

  while (child_number < MAX_CHILD) {
    Serial.print("child_number = ");
    Serial.print(child_number); 
    first = choose_parents();
    second = choose_parents();
    while (first == second) second = choose_parents();
    crossing(); // 交叉

   for (i=0; i<2; i++) {
     while (child_overlap(child[i])) {
       count_heni++;
       if (count_heni < HENI_TIMES) {
         child[i] = heni(child[i]);
       } else child[i]++;
     }

     //child実行  
     child_number++;
     children[child_number] = child[i];
     play(child[i]);
     seiseki_child[i] = point;// - middle;
     if (seiseki_child[i] < 1) seiseki_child[i] = 1;
   }
    comparison();
    worst = choose_worst();
    bad = choose_worst();
    while (worst == bad) bad = choose_worst();
    group[worst] = child[0];
    group[bad] = child[1];
    seiseki_group[worst] = seiseki_child[0];
    seiseki_group[bad] = seiseki_child[1];
  }

   board.LED(PORT_A2, ON);
   while(1) play(top);
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------

void play(int gene_fixed) {
  prepare();
  point = -50000;
  gene = gene_fixed;
  point_temp = 0;
  for(j=0; j<GENE_LEN; j++){
    foot();
    int gyro_value = board.GetGyroscopeValue(GX_AXIS);
    gyro_value = abs(gyro_value);
    point_temp += gyro_value;

  }
  for(j = 0; j < 10; j++){
    delay(300);
    int gyro_value = board.GetGyroscopeValue(GX_AXIS);
    gyro_value = abs(gyro_value);
    point_temp += gyro_value;
    Serial.println(gyro_value);
  }
  if(point_temp < 0.1) point_temp = 0.1;
  //point_temp = 1000.0/point_temp;
  point_temp = -point_temp;
  
//  Serial.println(point_temp);
  if (point < point_temp){
    point = point_temp;
  }

}

void foot(void) {
  byte next_servo[3];	
  next_servo[0] = gene & 0x01 ? SVRANGE(A_MAX) : SVRANGE(A_MIN);
  next_servo[1] = (gene>> GENE_LEN ) & 0x01 ? SVRANGE(B_MAX) : SVRANGE(B_MIN);
  next_servo[2] = (gene>> GENE_LEN * 2) & 0x01 ? SVRANGE(C_MAX) : SVRANGE(C_MIN);
  
  board.SyncServomotors(port, next_servo, 3, range);
  gene = gene >> 1; //1ビット右にシフト
  flag = 1;
}

void prepare(void) {
  board.Servomotor(PORT_D10, SVRANGE(A_INIT));
  board.Servomotor(PORT_D11, SVRANGE(B_INIT));
  board.Servomotor(PORT_D12, SVRANGE(C_INIT));
  count = 0;
  count_heni = 0;
  point_temp = 0;
  delay(3000);
}

void find_top(void) {
  top = group[0];
  seiseki_top = seiseki_group[0];
  for (i=1; i<KOTAISUU; i++) {
    if (seiseki_top < seiseki_group[i]) {
      top = group[i];
      seiseki_top = seiseki_group[i];
    }
  }
}

int choose_parents(void) {
  maximum = 0;
  for (i=0; i<KOTAISUU; i++) maximum += seiseki_group[i] * 10;
  roulette = (double) random(maximum);
  for(i=0; i<KOTAISUU; i++){
    roulette -= seiseki_group[i] * 10;
    if (roulette < 0) return group[i];
  }
  return 0;
}

void crossing(void) {
  int front, behind;
  pos = random(GENE_LEN - 1);
//  switch (pos) {
//    case 0:
//      front = 0x8080; //10000000
//      break;
//    case 1:
//      front = 0xc0c0; //11000000
//      break;
//    case 2:
//      front = 0xe0e0; //11100000
//      break;
//    case 3:
//      front = 0xf0f0; //11110000
//      break;
//    case 4:
//      front = 0xf8f8; //11111000
//      break;
//    case 5:
//      front = 0xfcfc; //11111100
//      break;
//    case 6:
//      front = 0xfefe; //11111110
//      break;
//    default:
//      Serial.println("crossing error");
//      break;
//  }
  front = pow(2, GENE_LEN) - pow(2, pos);
  behind = ~ front;

  child[0] = front & first + behind & second;
  child[1] = front & second + behind & first;
}

int child_overlap(int child) {
  int i;
  for (i=0; i<=child_number; i++) {
    if (children[i] == child) return 1;
  }
  return 0;
}

int heni(int child) {
  return child ^ (0x0101 << random(GENE_LEN));
}

void comparison(void) {
  if (seiseki_child[0] > seiseki_top) {
    if (seiseki_child[1] > seiseki_top) {
      if (seiseki_child[0] > seiseki_child[1]) {
        top = child[0];
        seiseki_top = seiseki_child[0];
      } else {
        top = child[1];
        seiseki_top = seiseki_child[1];
      }
    } else {
      top = child[0];
      seiseki_top = seiseki_child[0];
    }
  } else {
    if (seiseki_child[1] > seiseki_top) {
      top = child[1];
      seiseki_top = seiseki_child[1];
    }
  }
}

byte choose_worst(void) {
  for (i=0; i<KOTAISUU; i++){
    seiseki_reverse[i] = 100000/seiseki_group[i];//maximum - seiseki_group[i] * 10;
  }
  maximum_reverse = 0;
  for (i=0; i<KOTAISUU; i++) maximum_reverse += seiseki_reverse[i];
  roulette = (double) random(maximum_reverse);
  for(i=0; i<KOTAISUU; i++){
    roulette -= seiseki_reverse[i];
    if (roulette < 0){
      return i;
    }
  }
  Serial.println("error:choose_worst");
  return 0;
}


// --------------------------------------------
// 概要    : セットアップ処理
// --------------------------------------------

void setup() {
  randomSeed(analogRead(0));
//  resetTimer();
  artecRobotSetup();
  artecRobotMain();
}

void loop() {}

// --------------------------------------------
// 概要    : リストからの削除処理
// 引数    : struct _cell_t *p  リストのポインタ
//         : int pos            リストから削除する位置
// 戻り値  : 成功：0, エラー：-1
// --------------------------------------------
//int listDelete(struct cell_t* p, int pos)
//{
//  // 削除位置が0以下の場合、エラーを返す(何もしない)
//  if (pos <= 0) { return (-1); }
//  // 削除位置がリストの長さよりも大きい場合、エラーを返す(何もしない)
//  int l = listLength(p);        // リスト長を取得
//  if (l < pos) { return (-1); }
//
//  cell_t *target, *before;      // 削除する要素とその前の要素
//  target = p->next;             // 先頭の次の要素を設定
//  before = NULL;
//  if (target == NULL) return (-1);  // 既に削除する要素がない場合、エラーを返す
//  // 削除対象となる要素に移動する
//  before = p;
//  for (int i = 0;i < pos-1;i++) {
//    if (target->next == NULL) return (-1);  // 削除対象となる要素がない場合、エラーを返す
//    before = target;        // 削除対象となる要素の一つ前の要素を退避
//    target = target->next;  // 削除対象となる要素の更新
//  }
//  // 削除対象となる要素が存在する場合
//  before->next = target->next;  // 対象の一つ前の要素に対象の次の要素を設定
//  delete target;  // 対象を削除
//  return(0);
//}
//
//// --------------------------------------------
//// 概要    : リストへの追加処理
//// 引数    : struct _cell_t *p  リストのポインタ
////         : int    data        追加データ
//// 戻り値  : 成功：0, エラー：-1
//// --------------------------------------------
//int listAdd(struct cell_t* p, float data)
//{
//  cell_t *elm, *last;
//  // リスト要素の確保
//  elm = new cell_t;
//  // 要素の確保に失敗した場合
//  if(elm == NULL) {
//    // エラーを返す
//    return(-1);
//  }
//  // lastにリストの終端を設定
//  last = p;
//  for (;;) {
//    if (last->next == NULL) break;
//    last = last->next;
//  }
//  // リストの終端に追加する要素の設定
//  elm->data = data;
//  elm->next = NULL;
//  last->next = elm;
//  return(0);
//}
//
//// --------------------------------------------
//// 概要    : リスト長の取得
//// 引数    : struct _cell_t *p  リストのポインタ
//// 戻り値  : リスト長
//// --------------------------------------------
//int listLength(struct cell_t* p)
//{
//  struct cell_t *last;
//  // リストの終端に移動
//  last = p;
//  int length = 0;
//  for (;;) {
//    if (last->next == NULL) break;
//    last = last->next;
//    length++;
//  }
//  // リストの終端に追加する要素の設定
//  return(length);
//}
//
//// --------------------------------------------
//// 概要    : リスト要素の取得
//// 引数    : struct _cell_t *p  リストのポインタ
////         : int    pos         リスト要素の取得位置
//// 戻り値  : リスト要素、要素が存在しない場合は、0を返す
//// --------------------------------------------
//float listItem(struct cell_t *p, int pos)
//{
//  // 取得位置が0以下の場合、0を返す
//  if (pos <= 0) { return (0); }
//  // 取得位置がリストの長さよりも大きい場合、0を返す
//  int l = listLength(p);    // リスト長を取得
//  if (l < pos) { return (0); }
//
//  struct cell_t *target;    // 取得する要素
//  target = p;               // 先頭の要素を設定
//  // 取得対象となる要素に移動する
//  for (int i = 0;i < pos;i++) {
//    target = target->next;  // 取得対象となる要素の更新
//  }
//  return target->data;
//}
//
//// --------------------------------------------
//// 概要    : リストへの挿入処理
//// 引数    : struct _cell_t *p   リストのポインタ
////         : int pos             挿入する位置
////         : float data          挿入するデータ
//// 戻り値  : 成功：0, エラー：-1
//// --------------------------------------------
//int listInsert(struct cell_t *p, int pos, float data)
//{
//  // 挿入位置が0以下の場合、エラーを返す(何もしない)
//  if (pos <= 0) { return (-1); }
//  // 挿入位置がリストの長さ+1よりも大きい場合、エラーを返す(何もしない)
//  int l = listLength(p);  // リスト長を取得
//  if (l+1 < pos) { return (-1); }
//  // 挿入位置がリストの終端の場合
//  if (l+1 == pos) {
//    // リストの終端に追加する
//    listAdd(p, data);
//    return (0);
//  }
//
//  struct cell_t *item, *target, *before;  // 挿入する要素、挿入する位置の要素とその前の要素
//  // リスト要素の確保
//  item = new cell_t;
//  // 要素の確保に失敗した場合、エラーを返す(何もしない)
//  if(item == NULL) { return(-1); }
//
//  target = p;
//  // 挿入対象となる要素に移動する
//  for (int i = 0;i < pos;i++) {
//    before = target;        // 挿入対象となる要素の一つ前の要素を退避
//    target = target->next;  // 挿入対象となる要素の更新
//  }
//  // 挿入対象となる要素が存在する場合
//  item->data = data;    // 要素のデータの設定
//  item->next = target;  // 次の要素を設定
//  before->next = item;  // 1つ前の要素に対象の次の要素を設定
//  return(0);
//}
//
//// --------------------------------------------
//// 概要    : リストの要素の置換処理
//// 引数    : struct _cell_t *p  リストのポインタ
////         : int pos            置換する位置
////         : float data         置換するデータ
//// 戻り値  : 成功：0, エラー：-1
//// --------------------------------------------
//int listReplace(struct cell_t *p, int pos, float data)
//{
//  // 置換位置が0以下の場合、エラーを返す(何もしない)
//  if (pos <= 0) { return (-1); }
//  // 置換位置がリストの長さよりも大きい場合、エラーを返す(何もしない)
//  int l = listLength(p);  // リスト長を取得
//  if (l < pos) { return (-1); }
//
//  struct cell_t *target;  // 置換する要素
//
//  target = p;
//  // 置換対象となる要素に移動する
//  for (int i = 0;i < pos;i++) {
//    target = target->next;  // 置換対象となる要素の更新
//  }
//  // 置換対象となる要素が存在する場合
//  target->data = data;      // 要素のデータの設定
//  return(0);
//}
//
//// --------------------------------------------
//// 概要    : リストの要素に指定データが存在するか？
//// 引数    : struct _cell_t *p  リストのポインタ
////         : float data         検索するデータ
//// 戻り値  : 存在する：true, 存在しない：false
//// --------------------------------------------
//bool listIsContain(struct cell_t *p, float data)
//{
//  struct cell_t *elm = p;
//  // リストの全要素に対してdataを検索する
//  for (;;) {
//    // リスト終端に到達したらbreak
//    if (elm->next == NULL) break;
//    // リストの次の要素を取得
//    elm = elm->next;
//    // リストにdataが存在する場合、trueを返す
//    if (elm->data == data) return true;
//  }
//  // リストにdataが存在しない場合、falseを返す
//  return false;
//}
//
//// --------------------------------------------
//// 概要    : 丸め処理
////         : float  arg    引数
//// 戻り値  : 演算結果
// --------------------------------------------
int scratchRound(float arg)
{
  return round(arg);
}
//
//// --------------------------------------------
//// 概要    : 算術演算処理
//// 引数    : byte   opeID  操作ID
////         : float  arg    引数
//// 戻り値  : 演算結果
//// --------------------------------------------
//float math(byte opeID, float arg)              // 算術処理
//{
//  float result;
//  switch (opeID) {
//    case SQRT:
//      result = sqrt(arg);
//    break;
//    case ABS:     // |n|
//      result = abs(arg);
//    break;
//    case SIN:     // sin(n)
//    {
//      float rad = arg * PI / 180.0;
//      result = sin(rad);
//    }
//    break;
//    case COS:     // cos(n)
//    {
//      float rad = arg * PI / 180.0;
//      result = cos(rad);
//    }
//    break;
//    case TAN:     // tan(n)
//    {
//      float rad = arg * PI / 180.0;
//      result = tan(rad);
//    }
//    break;
///*
//    case ASIN:    // arcsin(n)
//    case ACOS:    // arccos(n)
//    case ATAN:    // arctan(n)
//    break;
//*/
//    case LN:      // loge
//      result = log(arg);
//    break;
//    case LOG:     // log10
//      result = log10(arg);
//    break;
//    case POWE:    // e^
//      result = exp(arg);
//    break;
//    case POW10:   // 10^
//      result = pow(10, arg);
//    break;
//    default:
//      result = 0;
//    break;
//
//  }
//  return result;
//}

// --------------------------------------------
// 概要    : タイマー値の取得
// 戻り値  : タイマーの値(sec)
// --------------------------------------------
//float getTimer()
//{
//  return ((millis() - StartTime) / 1000.0);
//}

// --------------------------------------------
// 概要    : タイマー値のリセット
// --------------------------------------------
//void resetTimer()
//{
//  StartTime = millis();
//}

// ---------------------------------------------------------------------
// 概要    : 超音波センサーの値を取得
// ---------------------------------------------------------------------
float GetUltrasonicSensorValue() {
  float Distance = board.GetUltrasonicSensorValue(PORT_A0, PORT_A1);
  // ビープ処理中 or DCモーター回転中ではない場合
  if (IRRemoteUsed) {
    if (!(BeepOn | DCMotorOn)) {
      Distance = Distance * DWEIGHT;
    }
  }
  Distance = Distance / 58.0;
  return min(Distance, 400);
}

