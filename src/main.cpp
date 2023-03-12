/*
@brief  伊藤研究室でのドローン制御の武者修行時のメインコード
@detail Atomfly2を角速度フィードバックのPID制御でフライト
@author nosaka
*/
#include <math.h>
#include <Arduino.h>
#include <M5Atom.h>
#include "rc.hpp"
#include "sensor.hpp"
#include "pid.hpp"

// M5AtomのLED カラーコード
#define WHITE 0xffffff
#define BLUE  0x0000ff
#define RED   0xff0000
#define GREEN 0x00ff00

// 不変定数
const uint32_t Freq = 100000; // 100k [Hz]
const uint8_t nBits = 8;      // PWMのデューティ比の分解能 [bit] (default: 2^8)
const uint8_t PIN_FL = 22;    // Front Left  GPIO -> 22
const uint8_t PIN_FR = 19;    // Front Right GPIO -> 19
const uint8_t PIN_RL = 23;    // Rear  Left  GPIO -> 23
const uint8_t PIN_RR = 33;    // Rear  Right GPIO -> 33
const uint8_t PWM_CH_FL = 0;  // channel for FL
const uint8_t PWM_CH_FR = 1;  // channel for FR
const uint8_t PWM_CH_RL = 2;  // channel for RL
const uint8_t PWM_CH_RR = 3;  // channel for RR

// 各軸周りの角速度に関するPID制御器のインスタンス
PID for_p; // X軸
PID for_q; // Y軸
PID for_r; // Z軸

// メイン処理で使用する変数群
const float UGAIN = 251.0f;     // デューティ比の最大量の任意制限
const float CTRL_LIMIT = 0.7f;  // リモコン指令値の任意制限
const uint32_t CYCLE = 4;       // 制御周期[ms]
const float MAX_VOLTAGE = 3.7f; // 最大電圧[V]
float vol = 0.0f;
int vol_flag = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;
float thrust_0 = 0.0f;
float thrust = 0.0f;
float t1 = 0.0f;
float t2 = 0.0f;
float t3 = 0.0f;
float t4 = 0.0f;

float target_wp = 0.0f; // リモコン指令角度からの，適当なスケーリング目標角速度
float target_wq = 0.0f; // リモコン指令角度からの，適当なスケーリング目標角速度
float target_wr = 0.0f; // リモコン指令角度からの，適当なスケーリング目標角速度
float err_wp = 0.0f; // 現在の誤差
float err_wq = 0.0f; // 現在の誤差
float err_wr = 0.0f; // 現在の誤差

void limitThrust0(float *thr_0);
void limitThrust(float *thr);
void limitRoll(float *roll);
void limitPitch(float *pitch);
void limitYaw(float *yaw);
float limitValue(float val);

void setup()
{
  // 各使用チャンネルと周波数，PWMのduty比の分解能の設定
  ledcSetup(PWM_CH_FL, Freq, nBits);
  ledcSetup(PWM_CH_FR, Freq, nBits);
  ledcSetup(PWM_CH_RL, Freq, nBits);
  ledcSetup(PWM_CH_RR, Freq, nBits);

  // 各PWM出力ピンとチャンネルの設定
  ledcAttachPin(PIN_FL, PWM_CH_FL);
  ledcAttachPin(PIN_FR, PWM_CH_FR);
  ledcAttachPin(PIN_RL, PWM_CH_RL);
  ledcAttachPin(PIN_RR, PWM_CH_RR);

  // M5AtomのLED等の初期化
  M5.begin(true, false, true);

  // M5リモコンとのwifi通信の初期化
  rc_init();

  // 角速度等センサの初期化
  sensor_init();

  // 各軸の角速度(p, q, r)のPID制御器の初期化
  for_p.set_parameter(0.87f, 3.0f, 0.016f, 0.125f, 0.004f);
  for_q.set_parameter(0.87f, 3.0f, 0.016f, 0.125f, 0.004f);
  for_r.set_parameter(5.0f, 3.0f, 0.00f, 0.125f, 0.004f);

  // シリアルモニタ用の初期化
  Serial.begin(115200);

  // 開始待機時間
  delay(1000);
}

void loop()
{
  // 角速度センサから，各軸の角速度 Wp, Wq, Wrを取得(更新)
  sensor_read();

  // M5リモコンや角速度センサから情報を取得
  vol = Voltage; // 電圧センサからの電圧を取得 ([V])
  roll  = Stick[AILERON];  // M5リモコンから，X軸周りの角度を取得 [-1 ~ +1]
  pitch = Stick[ELEVATOR]; // M5リモコンから，Y軸周りの角度を取得 [-2 ~ +2]
  yaw   = Stick[RUDDER];   // M5リモコンから，Z軸周りの角度を取得 [-1 ~ +1]
  thrust_0 = Stick[THROTTLE]; // M5リモコンから，スロットル値を取得 [0 ~ 1]
  thrust = 0.0f; // スロットルの基本値は0.0 (-> リモコンでのスロットルのみで推力を発生)

  // 電圧が下がった場合にLEDでお知らせ処理
  if (vol < 3.1f){
    M5.dis.drawpix(0, RED);
    vol_flag += 1;
  }
  if ((vol <= 3.05f) && (vol_flag >= 1000)){ 
    M5.dis.drawpix(0, RED);
    vol_flag = 0;
  }
  if (vol >= 3.1f){
    M5.dis.drawpix(0, BLUE);
  }

  // 本処理 (PID制御を掛けるか否か)
  if (thrust_0 < 0.4f){
    for_p.reset();
    for_q.reset();
    for_r.reset();
    t1 = 0.0f;
    t2 = 0.0f;
    t3 = 0.0f;
    t4 = 0.0f;
  }
  else{
    // リモコン指令角度・スロットル値の制限
    limitThrust0(&thrust_0);
    limitThrust(&thrust);
    limitRoll(&roll);
    limitPitch(&pitch);
    limitYaw(&yaw);
    // リモコン指令のスロットル値に最大電圧値を乗算
    thrust_0 = thrust_0*MAX_VOLTAGE;

    // 指令角度から適当にスケーリングし, これを目標角速度とみなす
    target_wp = (PI/2.0)*roll;  // 適当に(π/2)でスケーリング
    target_wq = (PI/2.0)*pitch; // 適当に(π/2)でスケーリング
    target_wr = (PI/2.0)*yaw;   // 適当に(π/2)でスケーリング

    // 目標値と取得値との誤差計算
    err_wp = target_wp - Wp;
    err_wq = target_wq - Wq;
    err_wr = target_wr - Wr;

    // 角速度制御(rate control) PID
    float u_p = for_p.update(err_wp);
    float u_q = for_q.update(err_wq);
    float u_r = for_r.update(err_wr);

    // ミキシング計算
    t1 = UGAIN*(thrust_0 + (thrust - u_p + u_q + u_r)/4.0f)/MAX_VOLTAGE;
    t2 = UGAIN*(thrust_0 + (thrust - u_p - u_q - u_r)/4.0f)/MAX_VOLTAGE;
    t3 = UGAIN*(thrust_0 + (thrust + u_p - u_q + u_r)/4.0f)/MAX_VOLTAGE;
    t4 = UGAIN*(thrust_0 + (thrust + u_p + u_q - u_r)/4.0f)/MAX_VOLTAGE;

    // デューティ比の制限
    t1 = limitValue(t1);
    t2 = limitValue(t2);
    t3 = limitValue(t3);
    t4 = limitValue(t4);
  }

  ledcWrite(PWM_CH_FR, (uint32_t)t1);
  ledcWrite(PWM_CH_FL, (uint32_t)t4);
  ledcWrite(PWM_CH_RR, (uint32_t)t2);
  ledcWrite(PWM_CH_RL, (uint32_t)t3);

  delay(CYCLE);
}

void limitThrust0(float *thr_0)
{
  if (*thr_0 > 0.95f){
    *thr_0 = 0.95f;
  }
  else if (*thr_0 < 0.0f){
    *thr_0 = 0.0f;
  }
}

void limitThrust(float *thr)
{
  if(*thr > CTRL_LIMIT){
    *thr = CTRL_LIMIT;
  }
  else if(*thr < 0.0f){
    *thr = 0.0f;
  }
}

void limitRoll(float *roll)
{
  if(*roll > CTRL_LIMIT){
    *roll = CTRL_LIMIT;
  }
  if(*roll < -CTRL_LIMIT){
    *roll = -CTRL_LIMIT;
  }

}

void limitPitch(float *pitch)
{
  if(*pitch >  CTRL_LIMIT){
    *pitch =  CTRL_LIMIT;
  }
  if(*pitch < -CTRL_LIMIT){
    *pitch = -CTRL_LIMIT;
  }

}

void limitYaw(float *yaw)
{
  if(*yaw > CTRL_LIMIT){
    *yaw = CTRL_LIMIT;
  }
  if(*yaw < -CTRL_LIMIT){
    *yaw = -CTRL_LIMIT;
  }

}

float limitValue(float val)
{
  float max_val = pow(2, nBits) - 1; // n[bit]時の最大値
  float min_val = 0; // n[bit]時の最小値

  if (val > max_val){
    val = max_val;
  }
  else if (val < min_val){
    val = min_val;
  }
  return val;
}
