/**********************************************************************/
/**
 * @file	ra4m1_advance_okako.ino
 *
 * @brief	2024 RA4M1ベースプログラム.
 *0
 * @author	月虹.
 *
 * Language	Arduino
 *
 * --------------------------------------------------------------------
 * @note
 *
 * --------------------------------------------------------------------
 * Copyright (c) 2024 Suzaki. All Rights Reserved.
 */
/**********************************************************************/

/**********************************************************************/
/*memo
  dipsw_get() 2(OFF)1(OFF):0  2(OFF)1(ON):1   2(ON)1(OFF):2  2(ON)1(ON):3
  モータ停止モード(dipsw_get() & 0x01)
  Angle 右:減る　左：増える
  PG:
*/
/*
 * インクルード
 */
#include <SPI.h>    // SPIライブラリ
#include <SD.h>     // SDカード処理ライブラリ
#include <EEPROM.h> // 内臓フラッシュ用ライブラリ

#include "FspTimer.h"    // 割り込み用ライブラリ
#include "mcr_gpt_lib.h" // MCR用RAライブラリ
#include "mcr_ad_lib.h"  // MCR用RAライブラリ

#include "lcd_lib.h"        // LCD用ライブラリ
#include "switch_lib.h"     // スイッチ用ライブラリ
#include "i2c_eeprom_lib.h" // I2C用ライブラリ
// #include "18curves_array.h"
// #include "curves_array.h"
#include "iE_value.h"

/**********************************************************************/
/*
 * マクロ/定数定義
 */

/*
 *	RRモータ PWM周期 : GPT2
 *	@note
 *		16ms/{1/(HOCO/16)} = 0.016 /{1/(48e6/16)} = 48000
 */
#define MOTOR_RR_PWM_CYCLE 2999

/*
 *	RLモータ PWM周期(FLと同じ) : GPT0
 *	@note
 *		16ms/{1/(HOCO/16)} = 0.016 /{1/(48e6/16)} = 48000
 */
#define MOTOR_RL_PWM_CYCLE 2999

/*
 *	FRモータ PWM周期 : GPT4
 *	@note
 *		16ms/{1/(HOCO/16)} = 0.016 /{1/(48e6/16)} = 48000
 */
#define MOTOR_FR_PWM_CYCLE 2999

/*
 *	FLモータ PWM周期(RLと同じ) : GPT0
 *	@note
 *		16ms/{1/(HOCO/16)} = 0.016 /{1/(48e6/16)} = 48000
 */
#define MOTOR_FL_PWM_CYCLE MOTOR_RL_PWM_CYCLE

/*
 *	ステアモータ PWM周期 : GPT7
 *	@note
 *		16ms/{1/(HOCO/16)} = 0.016 /{1/(48e6/16)} = 48000
 */
#define MOTOR_ST_PWM_CYCLE 2999

/*
 *	モータピン定義(personal_setting.hへ移動)
 */
#define MOTOR_RR_B 76        // CN8 17 P115
#define MOTOR_RR_A 77        // CN8 18 P114
#define MOTOR_RR_PWM GTIOC2A // CN8 19 P113

#define MOTOR_RL_A 48        // CN4  8 P412
#define MOTOR_RL_B 49        // CN4  7 P413
#define MOTOR_RL_PWM GTIOC0B // CN4  6 P414

#define MOTOR_FR_A 73        // CN8 14 P610
#define MOTOR_FR_B 74        // CN8 15 P609
#define MOTOR_FR_PWM GTIOC4B // CN8 16 P608

#define MOTOR_FL_A 51        // CN4  10 P410
#define MOTOR_FL_B 50        // CN4  9 P411
#define MOTOR_FL_PWM GTIOC0A // CN4  5 P415

#define MOTOR_ST_B 70        // CN8 11 P601
#define MOTOR_ST_A 71        // CN8 12 P602
#define MOTOR_ST_PWM GTIOC7A // CN8 13 P603

/*
 *	モータIOレジスタ 定義
 */
#define RR_B (R_PORT1->PODR_b.PODR15)
#define RR_A (R_PORT1->PODR_b.PODR14)

#define RL_A (R_PORT4->PODR_b.PODR12)
#define RL_B (R_PORT4->PODR_b.PODR13)

#define FR_B (R_PORT6->PODR_b.PODR10)
#define FR_A (R_PORT6->PODR_b.PODR9)

#define FL_A (R_PORT4->PODR_b.PODR10)
#define FL_B (R_PORT4->PODR_b.PODR11)

#define ST_B (R_PORT6->PODR_b.PODR1)
#define ST_A (R_PORT6->PODR_b.PODR2)

/*
 *	モータ初期化マクロ
 */
#define MOTOR_INIT_RR                                     \
  /* RRモータ方向 */                                      \
  pinMode(MOTOR_RR_A, OUTPUT);                            \
  pinMode(MOTOR_RR_B, OUTPUT);                            \
  /* PWMピン設定(P113:GTIOC2A) */                         \
  setGPTterminal(1, 13);                                  \
  MOTOR_RR_PWM = 0;                                       \
  /* GPT2 GTIOCAを使用 分周:16 周期:MOTOR_RR_PWM_CYCLE */ \
  startPWM_GPT2(GTIOCA, DIV16, MOTOR_RR_PWM_CYCLE);
#define MOTOR_INIT_RL                                     \
  /* RLモータ方向 */                                      \
  pinMode(MOTOR_RL_A, OUTPUT);                            \
  pinMode(MOTOR_RL_B, OUTPUT);                            \
  /* PWMピン設定(P414:GTIOC0B) */                         \
  setGPTterminal(4, 14);                                  \
  MOTOR_RL_PWM = 0;                                       \
  /* GPT0 GTIOCBを使用 分周:16 周期:MOTOR_RL_PWM_CYCLE */ \
  startPWM_GPT0(GTIOCB, DIV16, MOTOR_RL_PWM_CYCLE);
#define MOTOR_INIT_FR                                     \
  /* FRモータ方向 */                                      \
  pinMode(MOTOR_FR_A, OUTPUT);                            \
  pinMode(MOTOR_FR_B, OUTPUT);                            \
  /* PWMピン設定(P608:GTIOC4B) */                         \
  setGPTterminal(6, 8);                                   \
  /* PWM設定 */                                           \
  MOTOR_FR_PWM = 0;                                       \
  /* GPT4 GTIOCBを使用 分周:16 周期:MOTOR_FR_PWM_CYCLE */ \
  startPWM_GPT4(GTIOCB, DIV16, MOTOR_FR_PWM_CYCLE);
#define MOTOR_INIT_FL                                     \
  /* FLモータ方向 */                                      \
  pinMode(MOTOR_FL_A, OUTPUT);                            \
  pinMode(MOTOR_FL_B, OUTPUT);                            \
  /* PWMピン設定(P415:GTIOC0A) */                         \
  setGPTterminal(4, 15);                                  \
  /* PWM設定 */                                           \
  MOTOR_FL_PWM = 0;                                       \
  /* GPT0 GTIOCAを使用 分周:16 周期:MOTOR_FL_PWM_CYCLE */ \
  startPWM_GPT0(GTIOCA, DIV16, MOTOR_FL_PWM_CYCLE);
#define MOTOR_INIT_ST                                     \
  /* STモータ方向 */                                      \
  pinMode(MOTOR_ST_A, OUTPUT);                            \
  pinMode(MOTOR_ST_B, OUTPUT);                            \
  /* PWMピン設定(P603:GTIOC7A) */                         \
  setGPTterminal(6, 3);                                   \
  /* PWM設定 */                                           \
  MOTOR_ST_PWM = 0;                                       \
  /* GPT7 GTIOCAを使用 分周:16 周期:MOTOR_ST_PWM_CYCLE */ \
  startPWM_GPT7(GTIOCA, DIV16, MOTOR_ST_PWM_CYCLE);

/*
 *	RUNスイッチピン定義
 */
#define RUN_SWITCH 34

/*
 *	デジタルセンサピン定義(不使用)
 */
// #define SENS_D_LL 45  // CN4  4 P708
// #define SENS_D_CL 55  // CN4 14 P206
// #define SENS_D_CC 44  // CN4  3 P214
// #define SENS_D_CR 56  // CN4 15 P200
// #define SENS_D_RR 43  // CN4  2 P215

/*
 *	デジタルセンサ値取得マクロ(不使用)
 */
// #define SENS_LL !(R_PORT7->PIDR_b.PIDR8)
// #define SENS_CL !(R_PORT2->PIDR_b.PIDR6)
// #define SENS_C !(R_PORT2->PIDR_b.PIDR14)
// #define SENS_CR !(R_PORT2->PIDR_b.PIDR0)
// #define SENS_RR !(R_PORT2->PIDR_b.PIDR15)
// #define SENS_ALL (SENS_LL << 2 | SENS_C << 1 | SENS_RR)
// #define GATE SENS_RR

/*
 *	アナログセンサピン定義(@TODO ポートを確認の上設定)
 */
// #define SENS_A_UR 8  // CN8  2 D61　坂用右
#define SENS_A_RR 17 // CN8  9 D68　マーカー右
#define SENS_A_CR 10 // CN8  5 D64　トレース用右
#define SENS_A_CC 25 // CN8  4 D63　真ん中
#define SENS_A_CL 24 // CN8  3 D62　トレース用左
#define SENS_A_LL 16 // CN8  8 D67　マーカー左 23
// #define SENS_A_UL 23 // CN8  6 D65　坂用左
#define SENS_A_VR 18 // CN8  7 D66　ポテンショメータ
// #define SENS_A_RR 16    // CN8  9 D68
// #define SENS_A_LL 17    // CN8  8 D67
// #define SENS_A_VR 18    // CN8  7 D66
// #define SENS_A_SAKA 23  // CN8  6 D65
// #define SENS_A_0 24     // CN8  5 D64
// #define SENS_A_1 25     // CN8  4 D63
// #define SENS_A_2 10     // CN8  3 D62
// #define SENS_A_3 8      // CN8  2 D61

/*
 *	アナログ値取得マクロ
 *	@note
 *		R8C:10bit / RA:14bit
 *		R8仕様に合わせる為4bitシフトしているが、
 *		14bitを使う方が細かく動作が指定できるので要調整
 */
// #define ANA_SENS_UR (ad.getDataDual(SENS_A_UR))
#define ANA_SENS_RR (ad.getDataDual(SENS_A_RR))
#define ANA_SENS_CR (ad.getDataDual(SENS_A_CR))
#define ANA_SENS_CC (ad.getDataDual(SENS_A_CC))
#define ANA_SENS_CL (ad.getDataDual(SENS_A_CL))
#define ANA_SENS_LL (ad.getDataDual(SENS_A_LL))
// #define ANA_SENS_UL (ad.getDataDual(SENS_A_UL))

// #define ANA_SENS_UR (ad.getDataDual(SENS_A_UR) >> 4) // >> 4　↓
// #define ANA_SENS_RR (ad.getDataDual(SENS_A_RR) >> 4)
// #define ANA_SENS_CR (ad.getDataDual(SENS_A_CR) >> 4)
// #define ANA_SENS_CC (ad.getDataDual(SENS_A_CC) >> 4)
// #define ANA_SENS_LL (ad.getDataDual(SENS_A_CL) >> 4)
// #define ANA_SENS_CL (ad.getDataDual(SENS_A_LL) >> 4)
// #define ANA_SENS_UL (ad.getDataDual(SENS_A_UL) >> 4)

#define BAR_ANGLE (1023 - (ad.getDataDual(SENS_A_VR) >> 4))

// #define SLOPE_ANGLE -1
// #define ANA_SENS_L (ad.getDataDual(SENS_A_LL) >> 4)
// #define ANA_SENS_R (ad.getDataDual(SENS_A_RR) >> 4)
// #define SLOPE_ANGLE (ad.getDataDual(SENS_A_SAKA) >> 6)
// #define BAR_ANGLE (1023 - (ad.getDataDual(SENS_A_VR) >> 4))

/*
 *	ディップSWによるモード
 */
#define MODE_NORMAL 0
#define MODE_STOP_MOTOR 1

/*
 *	LED定義
 *	@TODO ピン番号知らない…
 */
#define INFRARED_LED (R_PORT2->PODR_b.PODR6) // MD トレース用赤外線LED
#define L_LED (R_PORT8->PODR_b.PODR9)        // MD 左LED
#define R_LED (R_PORT8->PODR_b.PODR8)        // MD 右LED
#define CPU_LED_2 (R_PORT2->PODR_b.PODR5)    // CPUボード LED2
#define CPU_LED_3 (R_PORT1->PODR_b.PODR11)   // CPUボード LED3

// #define THRESHOLD_H 1000 // アナログセンサ(0～1023)白判定しきい値  1000
// #define THRESHOLD_L 800  // アナログセンサ(0～1023)黒判定しきい値  800

#define ON 1
#define OFF 0 // 走行パラメータ

#define ACCEL 2
#define BRAKE 1

#define sLL 0
#define sCL 1
#define sCC 2
#define sCR 3
#define sRR 4

#define STREAT_JUDGE_ANGLE 40
#define CORNER_SPEED 65         // 未使用
#define CORNER_PWM 100          // コーナー脱出時の最高PWM
#define CORNER_FREEPWM_ANGLE 70 // コーナー　モータフリー（パーシャル）判定角度
#define CORNER_FREEPWM_SPEED 1  // コーナー　モータフリースピード
// #define SLOPE_UP_SPEED 60       // 坂の上り速度 60= 4.0m/s
// #define SLOPE_DOWN_SPEED 60     // 坂の下り速度 60= 4.0m/s
// #define SLOPE_DOWN_SPEED 60     // 坂の下り速度 60= 4.0m/s



// ========== 直線区間構造体 ==========
#define MAX_STRAIGHT_SECTIONS 50

typedef struct {
    uint32_t start_distance;    // 開始位置 [パルス]
    uint32_t end_distance;      // 終了位置 [パルス]
    float next_corner_speed;    // 次のコーナー速度 [m/s]
} StraightSection;

StraightSection straight_sections[MAX_STRAIGHT_SECTIONS];


#define THR_Sens 400    //白判定しきい値  400
#define THR_M_Sens 300  //白判定しきい値  300
#define thrSensBK 100   //黒判定しきい値  200 150 100 使ってない

#define ACCEL_ANGLE 20  //再生走行用直線判断アングル
#define BRAKE_DISTANCE 800  //再生走行用ブレーキング距離

#define STOP 0    //PWM＝0
#define TRACE 1   //ライン追従
#define ANGLE 2   //角度指定

#define CENTER 0
#define RIGHT 1
#define LEFT 2

// ========== マシン仕様定数 ==========
#define TIRE_DIAMETER   0.022f
#define CONTROL_PERIOD  0.01f
#define ENC_PULSE_REV   100.0f
#define PI              3.14159265f

// ========== 計算済み定数 ==========
#define M_PER_PULSE     0.000691f
#define CM_PER_PULSE    0.0691f
#define PULSE_TO_MS     0.0691f
#define MS_TO_PULSE_PER_10MS  14.468f

// ========== 変換マクロ ==========
#define DATA_TO_MS(x)   ((float)(x) / 10.0f)

/*
 *	レーン角度
 */
#define LANE_ANGLE_R 45 // 右レーンアングル 90
#define LANE_ANGLE_L 45 // 左レーンアングル 90
/*
 *	クランク角度
 */
#define CRANK_ANGLE_R 90 // 右クランクアングル 110
#define CRANK_ANGLE_L 90 // 左クランクアングル 110

#define CRANK_TOP_SPEED   2.2// 2.4m/s
#define CRANK_MIN_SPEED   2.0// 2.0m/s


#define PWM_MAX 40  //足回りモーターのマックスPWM（記憶走行）

/*
 *	ステアセンター値
 *	@note 必ず車体ごとに変更！！
 */
#define VR_CENTER 520 //497

// 1mの距離
#define METER 1447L

/*
 *	坂関連
 */
#define SLOPE_UP_START 420    // 上り開始判定   450

#define SLOPE_DOWN_START 600  // 下り開始判定   500

/*
 *	内臓フラッシュ関連
 */
#define FLASH_SIZE 0x2000
#define FLASH_HEADER 0xA5
typedef enum
{
  HEADER_ADDR = 0x00,       // ヘッダー
  TOTAL_DIST_ADDR = 0x01,   // 走行停止距離
  START_TIME_ADDR = 0x02,   // スタート待ち時間
  PROP_GAIN_ADDR = 0x03,    // トレース比例ゲイン
  DIFF_GAIN_ADDR = 0x04,    // トレース微分ゲイン
  TRG_SPEED_ADDR = 0x05,    // 直線速度
  CORNER_SPEED_ADDR = 0x06, // コーナー速度
  CRANK_SPEED_ADDR = 0x07,  // クランク進入速度
  LANE_SPEED_ADDR = 0x08,   // レーン進入速度
  SLOPE_SPEED_ADDR = 0x09,     // 坂感知閾値
  ACCEL_SPEED_ADDR = 0x0A,     // 直線加速速度
  // LANE_ANGLE_L_ADDR	=		0x09,	// 左レーンアングル
  // LANE_ANGLE_R_ADDR	=		0x0A,	// 右レーンアングル
  // SENS_L_THOLD1_ADDR	=		0x0B,	// アナログセンサ左閾値(上位)
  // SENS_L_THOLD2_ADDR	=		0x0C,	// アナログセンサ左閾値(下位)
  // SENS_R_THOLD1_ADDR	=		0x0D,	// アナログセンサ右閾値(上位)
  // SENS_R_THOLD2_ADDR	=		0x0E,	// アナログセンサ右閾値(下位)
  // DF_KP_OVER_ADDR=0x09 /* トレース大外れ比例制御係数   */
  // DF_KD_OVER_ADDR= 0x0A /* トレース大外れ微分制御係数   */
  // DF_TH_OVER_ADDR= 0x0B /* 大外れ閾値                   */

  MAX_NUM_ADDR
} FLASH_ADDR;

// #define MTPWM_START 70 //%

/*autoブレーキ関係*/
// #define F_Brake 100      //70
// #define R_Brake 100      //55
// #define Inside_ofset 65 //%　　85
