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

#define ACCMIN  (-128)  // Studuino基板の加速度センサーの最小値
#define ACCMAX  (127)   // Studuino基板の加速度センサーの最大値

#define ULTRASONIC_SENSOR()    (GetUltrasonicSensorValue())
#define DWEIGHT  (1.818)


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
#define MAX_CHILD 60
#define HENI_TIMES 10
#define BASIC_INTERVAL 200

// **********************************************************************
// プロトタイプ宣言
// **********************************************************************
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
byte i,j;
unsigned long first; //1位個体
unsigned long second; //2位個体
unsigned long child[2]; //子個体
unsigned long gene; //実行中個体
unsigned long group[KOTAISUU];
unsigned long top;
byte worst;
byte bad;
double point,  point_first, point_second, point_T, point2;
int point_temp;
int count;
byte flag = 0;
byte flag_mtop = 0;
double seiseki_child[2];
double seiseki_group[KOTAISUU];
double seiseki_reverse[KOTAISUU];
double seiseki_top;
byte pos;
byte child_number; //何番目の個体かを表す変数
unsigned long children[MAX_CHILD]; //個体記録用変数

byte range;
byte count_heni;
double maximum, maximum_reverse;
double roulette;
int generation = 0;

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
void play(unsigned long);
void foot(void);
void prepare(void);
void find_top(void);
unsigned long choose_parents(void);
void crossing(void);
int child_overlap(unsigned long);
unsigned long heni(unsigned long);
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
  range = 0;

  // 初期個体を出鱈目に生成する
  for (i=0; i<KOTAISUU; i++) {
    group[i] = random(pow(2, GENE_LEN)) + (random(pow(2, GENE_LEN)) << GENE_LEN) + (random(pow(2, GENE_LEN)) << GENE_LEN * 2);
    Serial.println(group[i], BIN);
  }

  //group実行
  for (i=0; i<KOTAISUU; i++) {
    child_number++;
    children[child_number] = group[i];
    play(group[i]);
    seiseki_group[i] = point;
    if (seiseki_group[i] < 1) seiseki_group[i] = 1; //comment out by Ken Takaki
  }
  find_top();

  while (child_number < MAX_CHILD) {
    Serial.print("generation: ");
    Serial.print(generation);
    Serial.print("\n");
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
       child[i] = 0b00000000111111111111111111111111 & child[i];
     }

     //child実行  
     child_number++;
     children[child_number] = child[i];
     play(child[i]);
     seiseki_child[i] = point; // - middle;
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

    generation++;
  }

   board.LED(PORT_A2, ON);
   while(1) play(top);
}

// ---------------------------------------
// Artec robot subroutine
// ---------------------------------------

void play(unsigned long gene_fixed) {
  prepare();
  point = 0;
  point_temp = 0;
  gene = gene_fixed;
  Serial.print("gene: ");
  Serial.print(gene_fixed, BIN);
  Serial.print("\n");

  for(j=0; j<GENE_LEN; j++){
    foot();
    for (int i = 0; i < 5; ++i) {
      point_temp = ULTRASONIC_SENSOR();
      if (point_temp > 30) continue;
      if (point < point_temp){
        point = point_temp;
      }
      delay(1);
    }
  }
  for(j = 0; j < 10; j++){
    point_temp = ULTRASONIC_SENSOR();
    if (point_temp > 30) continue;
    if (point < point_temp){
      point = point_temp;
    }
    delay(80);
  }
  
  if (point < point_temp){
    point = point_temp;
  }
  Serial.print("gene: ");
  Serial.print(gene, BIN);
  Serial.print("\n");
  Serial.print("point: ");
  Serial.print(point);
  Serial.print("\n");
  Serial.print("\n");
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
  delay(5000);
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
  Serial.print("top: ");
  Serial.print(top, BIN);
  Serial.print("\n");
  Serial.print("seiseki top: ");
  Serial.print(seiseki_top);
  Serial.print("\n");
}

unsigned long choose_parents(void) {
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
  unsigned long front, behind;
  pos = random(GENE_LEN * 3 - 1);
  front = pow(2, GENE_LEN * 3) - pow(2, pos);
  behind = ~ front;
  behind = 0b00000000111111111111111111111111 & behind;
  child[0] = front & first | behind & second;
  child[1] = front & second | behind & first;
}

int child_overlap(unsigned long child) {
  int i;
  for (i=0; i<child_number; i++) {
    if (children[i] == child) return 1;
  }
  return 0;
}

unsigned long heni(unsigned long child) {
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
  artecRobotSetup();
  artecRobotMain();
}

void loop() {}

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

