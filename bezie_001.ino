//2020/02/06 TR 4輪オムニ 接地エンコーダ2個 lpms-me1 用いて動作確認済み

#include <Arduino.h>
#include <MsTimer2.h>
#include <math.h>
#include <stdio.h>
#include "PID.h"
#include "Sakura_modules.h"
#include "secondary_filter.h"
#include "encCounter.h"
#include "lpms_me1Peach.h"

S_UnderBody MEKANAMU_modu(&Serial0);

#define PI 3.1415926535
#define SERIAL_LPMSME1 Serial1

double INT_TIME = 0.010;
int MS_INT_TIME = INT_TIME * 1000;
double x[4] = {0.0, 2.0, -2.0, 1.0};
double y[4] = {0.0, 0.5, 1.0, 1.5};
double X_A = 0.0, X_B = 0.0, X_C = 0.0, X_D = 0.0, a_bejie = 0.0, b_bejie = 0.0, c_bejie = 0.0, d_bejie = 0.0, e_bejie = 0.0, f_bejie = 0.0;
double Y_A = 0.0, Y_B = 0.0, Y_C = 0.0, Y_D = 0.0;
double pre_t_be = 0.1;
double t_be = 0.0;
double TRnoXziku = 0.0;
double TRnoYziku = 0.0;
double epsilon = 0.0;
double bejie_point_x = 0.0;
double bejie_point_y = 0.0;
double gaiseki_angle = 0.0;
double dist = 0.0;
double tafrc_x = 0.0;
double tafrc_y = 0.0;
double refVtan = 0.0;
double refVper = 0.0;
double refKakudo = 0.0;
double refVrot = 0.0;
int LED_count = 0;
int print_count = 0;
double angle_rad = 0.0;
double refVxg = 0.0;
double refVyg = 0.0;
double refVzg = 0.0;
double refVx = 0.0;
double refVy = 0.0;
double refVz = 0.0;
bool flag10ms = false;
double TimerENC1 = 0.0;
double TimerENC2 = 0.0;
double LCL_ENC_mps_x = 0.0;
double LCL_ENC_mps_y = 0.0;
double GLO_ENC_mmps_x = 0.0;
double GLO_ENC_mmps_y = 0.0;
double ENC_DIS_x = 0.0;
double ENC_DIS_y = 0.0;
double pre_enc_one = 0.0;
double pre_enc_two = 0.0;
double d_bejie_dounyu = 0.0;
double e_bejie_dounyu = 0.0;
double f_bejie_dounyu = 0.0;

lpms_me1 lpms(&SERIAL_LPMSME1);

//PIDクラスのインスタンス化
PID distPID(2.5, 0.01, 0.0, INT_TIME);
PID kakudoPID(5.0, 0.01, 0.0, INT_TIME);
//2次遅れのインスタンス化
secondary_filter sokudo_filter(INT_TIME);
secondary_filter kakudo_filter(INT_TIME);

// 自己位置推定用のエンコーダ
encCounter enc1(1);
encCounter enc2(2);

int enc_one = 0, enc_two = 0;

double SUITYOKU_NAISEKI(double t)
{
  return a_bejie * pow(t, 5.0) + b_bejie * pow(t, 4.0) + c_bejie * pow(t, 3.0) + d_bejie_dounyu * pow(t, 2.0) + e_bejie_dounyu * t + f_bejie_dounyu;
}
double SUITYOKU_NAISEKI_dash(double t)
{
  return 5.0 * a_bejie * pow(t, 4.0) + 4.0 * b_bejie * pow(t, 3.0) + 3.0 * c_bejie * pow(t, 2.0) + 2.0 * d_bejie_dounyu * t + e_bejie_dounyu;
}

double BEJIE_X(double t)
{
  return X_A * pow(t, 3.0) + 3.0 * X_B * pow(t, 2.0) + 3.0 * X_C * t + X_D; //ベジエ曲線上の垂線の座標
}
double BEJIE_Y(double t)
{
  return Y_A * pow(t, 3.0) + 3.0 * Y_B * pow(t, 2.0) + 3.0 * Y_C * t + Y_D; //ベジエ曲線上の垂線の座標
}

double dash_BEJIE_X(double t)
{
  return 3.0 * X_A * pow(t, 2.0) + 6.0 * X_B * t + 3.0 * X_C; //ベジエ曲線上の垂線の座標
}
double dash_BEJIE_Y(double t)
{
  return 3.0 * Y_A * pow(t, 2.0) + 6.0 * Y_B * t + 3.0 * Y_C; //ベジエ曲線上の垂線の座標
}

// LEDをチカチカさせるための関数
void LEDblink(int times, int interval)
{
  for (int i = 0; i < times; i++)
  {
    delay(interval);
    analogWrite(PIN_LED_RED, HIGH);
    delay(interval);
    analogWrite(PIN_LED_RED, LOW);
  }
}

void timer_warikomi()
{
  flag10ms = true;

  print_count++;

  //エンコーダデータ取得
  enc_one = enc1.getCount(); //ピンはp1_0とp1_10　https://www.core.co.jp/product/m2m/gr-peach/pdf/history/gr-peach_specification_c.pdf
  enc_two = enc2.getCount(); //ピンはp1_1とp1_11

  // LPMS-ME1のから角度を取得
  angle_rad = (double)lpms.get_z_angle();

  //エンコーダの差
  TimerENC1 = (enc_one - pre_enc_one);
  TimerENC2 = (enc_two - pre_enc_two);

  //エンコーダ速度(m/s)変換
  TRnoYziku = (0.048 * PI * ((double)TimerENC1 / 800.0)) / INT_TIME;
  TRnoXziku = (0.048 * PI * ((double)TimerENC2 / 800.0)) / INT_TIME;

  //エンコーダローカル完成
  LCL_ENC_mps_x = TRnoXziku * INT_TIME;
  LCL_ENC_mps_y = TRnoYziku * INT_TIME;

  //エンコーダデータをローカルからグローバルへ
  ENC_DIS_x += (LCL_ENC_mps_x * cos(angle_rad)) + (-LCL_ENC_mps_y * sin(angle_rad));
  ENC_DIS_y += (LCL_ENC_mps_x * sin(angle_rad)) + (LCL_ENC_mps_y * cos(angle_rad));

  tafrc_x = x[0] - ENC_DIS_x;
  tafrc_y = y[0] - ENC_DIS_y;

  d_bejie_dounyu = d_bejie + (X_A * tafrc_x) + (Y_A * tafrc_y);
  e_bejie_dounyu = e_bejie + (2.0 * X_B * tafrc_x) + (2.0 * Y_B * tafrc_y);
  f_bejie_dounyu = f_bejie + (X_C * tafrc_x) + (Y_C * tafrc_y);

  int newton_count = 0;
  do //ニュートン法実装部分　
  {
    t_be = pre_t_be - SUITYOKU_NAISEKI(pre_t_be) / SUITYOKU_NAISEKI_dash(pre_t_be);
    epsilon = abs((t_be - pre_t_be) / pre_t_be);

    pre_t_be = t_be;
    newton_count++;

  } while (epsilon >= 1e-4 && newton_count <= 50);

  bejie_point_x = BEJIE_X(t_be); //ベジエ曲線の座標を求める
  bejie_point_y = BEJIE_Y(t_be);

  gaiseki_angle = atan2(dash_BEJIE_Y(t_be), dash_BEJIE_X(t_be));
  dist = (bejie_point_y - ENC_DIS_y) * cos(gaiseki_angle) - ((bejie_point_x - ENC_DIS_x) * sin(gaiseki_angle));

  refVtan = sokudo_filter.SecondOrderLag(0.5);
  refVper = distPID.getCmd(dist, 0.0, 0.5); //最高速度0.5
  refKakudo = kakudo_filter.SecondOrderLag(0.0);
  refVrot = kakudoPID.getCmd(refKakudo, angle_rad, 1.00);

  //グローバル座標系の指令速度
  refVxg = refVtan * cos(gaiseki_angle) - refVper * sin(gaiseki_angle);
  refVyg = refVtan * sin(gaiseki_angle) + refVper * cos(gaiseki_angle);
  refVzg = refVrot;

  //ローカル座標系の指令速度  単位は(M/s)?
  refVy = -(refVxg * cos(angle_rad) + refVyg * sin(angle_rad));
  refVx = -refVxg * sin(angle_rad) + refVyg * cos(angle_rad);
  refVz = refVzg;

  //エンコーダの前回値代入
  pre_enc_one = enc_one;
  pre_enc_two = enc_two;
}

void setup()
{
  sokudo_filter.setSecondOrderPara(5.0, 1.0, 0.0);
  kakudo_filter.setSecondOrderPara(10.0, 1.0, 0.0);
  distPID.PIDinit(0, 0);

  X_A = x[3] - 3.0 * x[2] + 3.0 * x[1] - x[0]; //ベジエ曲線
  X_B = x[2] - 2.0 * x[1] + x[0];
  X_C = x[1] - x[0];
  X_D = x[0];
  Y_A = y[3] - 3.0 * y[2] + 3.0 * y[1] - y[0]; //ベジエ曲線
  Y_B = y[2] - 2.0 * y[1] + y[0];
  Y_C = y[1] - y[0];
  Y_D = y[0];

  a_bejie = pow(X_A, 2.0) + pow(Y_A, 2.0); //垂線求めるために微分して現在位置をかけるやつ
  b_bejie = 5.0 * ((X_A * X_B) + (Y_A * Y_B));
  c_bejie = 2.0 * ((3.0 * pow(X_B, 2.0) + 2.0 * X_A * X_C) + (3.0 * pow(Y_B, 2.0) + 2.0 * Y_A * Y_C));
  d_bejie = (9.0 * X_B * X_C) + (9.0 * Y_B * Y_C);
  e_bejie = (3.0 * pow(X_C, 2.0)) + (3.0 * pow(Y_C, 2.0));
  f_bejie = 0.0;

  Serial.begin(115200);
  MEKANAMU_modu.begin(115200); //足回り-足回りモジュール通信準備

  delay(10);
  // // LPMS-ME1の初期化
  if (lpms.init() == 0)
  {
    digitalWrite(PIN_LED_RED, HIGH);
  }
  delay(750);

  enc1.init();
  enc2.init();

  MsTimer2::set((int)MS_INT_TIME, timer_warikomi); // 10ms period
  MsTimer2::start();

  pinMode(PIN_LED_RED, OUTPUT);
  LEDblink(5, 100);
}

void loop()
{

  if (flag10ms)
  {
    MEKANAMU_modu.moveXY((double)refVx, (double)refVy, (double)refVz); //足回りモジュールへ送信

    Serial.print(ENC_DIS_x, DEC);
    Serial.print(' ');
    Serial.print(ENC_DIS_y, DEC);
    Serial.print(' ');
    Serial.print(bejie_point_x, DEC);
    Serial.print(' ');
    Serial.println(bejie_point_y, DEC);

    flag10ms = false;
  }
}
