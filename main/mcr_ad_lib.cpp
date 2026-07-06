//**************************************************************************
// ﾌｧｲﾙ内容     A/D変換　シングルスキャンモード 制御ライブラリ
// バージョン   Ver.1.02
// Date         2024.08.22
// Copyright    ジャパンマイコンカーラリー実行委員会
// ライセンス   This software is released under the MIT License.
//              http://opensource.org/licenses/mit-license.php
//**************************************************************************

//======================================
// インクルード
//======================================
#include "mcr_ad_lib.h"


//======================================
// テーブル
//======================================

const int16_t AD_PORT_ChA0[15]{
	0b000000000000001 ,
	0b000000000000010 ,
	0b000000000000100 ,
	0b000000000001000 ,
	0b000000000010000 ,
	0b000000000100000 ,
	0b000000001000000 ,
	0b000000010000000 ,
	0b000000100000000 ,
	0b000001000000000 ,
	0b000010000000000 ,
	0b000100000000000 ,
	0b001000000000000 ,
	0b010000000000000 ,
	0b100000000000000
};

const int16_t AD_PORT_ChA1[10]{
	0b0000000001 ,
	0b0000000010 ,
	0b0000000100 ,
	0b0000001000 ,
	0b0000010000 ,
	0b0000100000 ,
	0b0001000000 ,
	0b0010000000 ,
	0b0100000000 ,
	0b1000000000
};

//**********************************************************************
// A/D変換　チャンネル（端子）選択
//**********************************************************************
void mcr_ad::useCh( uint8_t ch ) {
  switch( ch ) {
  case 0:
    R_PFS->PORT[0].PIN[0].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[0].PmnPFS_b.ASEL = 1;        // P000=AN000
    R_PFS->PORT[0].PIN[0].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 1:
    R_PFS->PORT[0].PIN[1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[1].PmnPFS_b.ASEL = 1;        // P001=AN001
    R_PFS->PORT[0].PIN[1].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 2:
    R_PFS->PORT[0].PIN[2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[2].PmnPFS_b.ASEL = 1;        // P002=AN002
    R_PFS->PORT[0].PIN[2].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 3:
    R_PFS->PORT[0].PIN[3].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[3].PmnPFS_b.ASEL = 1;        // P003=AN003
    R_PFS->PORT[0].PIN[3].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 4:
    R_PFS->PORT[0].PIN[4].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[4].PmnPFS_b.ASEL = 1;        // P004=AN004
    R_PFS->PORT[0].PIN[4].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 5:
    R_PFS->PORT[0].PIN[10].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[10].PmnPFS_b.ASEL = 1;        // P010=AN005
    R_PFS->PORT[0].PIN[10].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 6:
    R_PFS->PORT[0].PIN[11].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[11].PmnPFS_b.ASEL = 1;        // P011=AN006
    R_PFS->PORT[0].PIN[11].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 7:
    R_PFS->PORT[0].PIN[12].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[12].PmnPFS_b.ASEL = 1;        // P012=AN007
    R_PFS->PORT[0].PIN[12].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 8:
    R_PFS->PORT[0].PIN[13].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[13].PmnPFS_b.ASEL = 1;        // P013=AN008
    R_PFS->PORT[0].PIN[13].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 9:
    R_PFS->PORT[0].PIN[14].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[14].PmnPFS_b.ASEL = 1;        // P014=AN009
    R_PFS->PORT[0].PIN[14].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 10:
    R_PFS->PORT[0].PIN[15].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[15].PmnPFS_b.ASEL = 1;        // P015=AN010
    R_PFS->PORT[0].PIN[15].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 11:
    R_PFS->PORT[0].PIN[5].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[5].PmnPFS_b.ASEL = 1;        // P005=AN011
    R_PFS->PORT[0].PIN[5].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 12:
    R_PFS->PORT[0].PIN[6].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[6].PmnPFS_b.ASEL = 1;        // P006=AN012
    R_PFS->PORT[0].PIN[6].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 13:
    R_PFS->PORT[0].PIN[7].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[7].PmnPFS_b.ASEL = 1;        // P007=AN013
    R_PFS->PORT[0].PIN[7].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 14:
    R_PFS->PORT[0].PIN[8].PmnPFS_b.PMR = 0;
    R_PFS->PORT[0].PIN[8].PmnPFS_b.ASEL = 1;        // P008=AN014
    R_PFS->PORT[0].PIN[8].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 15:
    // AN015ありません
    break;

  case 16:
    R_PFS->PORT[5].PIN[0].PmnPFS_b.PMR = 0;
    R_PFS->PORT[5].PIN[0].PmnPFS_b.ASEL = 1;        // P500=AN016
    R_PFS->PORT[5].PIN[0].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 17:
    R_PFS->PORT[5].PIN[1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[5].PIN[1].PmnPFS_b.ASEL = 1;        // P501=AN017
    R_PFS->PORT[5].PIN[1].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 18:
    R_PFS->PORT[5].PIN[2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[5].PIN[2].PmnPFS_b.ASEL = 1;        // P502=AN018
    R_PFS->PORT[5].PIN[2].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 19:
    R_PFS->PORT[1].PIN[3].PmnPFS_b.PMR = 0;
    R_PFS->PORT[1].PIN[3].PmnPFS_b.ASEL = 1;        // P103=AN019
    R_PFS->PORT[1].PIN[3].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 20:
    R_PFS->PORT[1].PIN[2].PmnPFS_b.PMR = 0;
    R_PFS->PORT[1].PIN[2].PmnPFS_b.ASEL = 1;        // P102=AN020
    R_PFS->PORT[1].PIN[2].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 21:
    R_PFS->PORT[1].PIN[1].PmnPFS_b.PMR = 0;
    R_PFS->PORT[1].PIN[1].PmnPFS_b.ASEL = 1;        // P101=AN021
    R_PFS->PORT[1].PIN[1].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 22:
    R_PFS->PORT[1].PIN[0].PmnPFS_b.PMR = 0;
    R_PFS->PORT[1].PIN[0].PmnPFS_b.ASEL = 1;        // P100=AN022
    R_PFS->PORT[1].PIN[0].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 23:
    R_PFS->PORT[5].PIN[3].PmnPFS_b.PMR = 0;
    R_PFS->PORT[5].PIN[3].PmnPFS_b.ASEL = 1;        // P503=AN023
    R_PFS->PORT[5].PIN[3].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 24:
    R_PFS->PORT[5].PIN[4].PmnPFS_b.PMR = 0;
    R_PFS->PORT[5].PIN[4].PmnPFS_b.ASEL = 1;        // P504=AN024
    R_PFS->PORT[5].PIN[4].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;

  case 25:
    R_PFS->PORT[5].PIN[5].PmnPFS_b.PMR = 0;
    R_PFS->PORT[5].PIN[5].PmnPFS_b.ASEL = 1;        // P505=AN025
    R_PFS->PORT[5].PIN[5].PmnPFS_b.PMR = 1;         // 0:汎用入出力 1:周辺機能用
    break;
  }
}

//**********************************************************************
// シングルA/D変換　設定
//**********************************************************************
void mcr_ad::start( void ) {
  R_ADC0->ADCSR_b.ADCS = 0b00;  // シングルスキャンモード
  R_ADC0->ADCSR_b.ADHSC = 0b0;  // 高速スキャンモード
  R_ADC0->ADCER_b.ADPRC = 0b11;  // 14bit精度
  R_ADC0->ADCER_b.ACE = 0;  // 14bit精度
}

//**********************************************************************
// A/D変換　値取得
//**********************************************************************
int16_t mcr_ad::getData( uint8_t ch ) {
  int ret = -1;
  
  /* AD変換するチャンネル指定 */
 if( ch >= 16 ) {
   R_ADC0->ADANSA[1] = AD_PORT_ChA1[ch-16];
  }else{
   R_ADC0->ADANSA[0] = AD_PORT_ChA0[ch];
  }
  R_ADC0->ADCSR_b.ADST = 1;     // スタート

  while(R_ADC0->ADCSR_b.ADST == 1); //AD変換待機
  ret = R_ADC0->ADDR_b[ch].ADDR; //AD値取得

  /* チャンネル指定のリセット */
  R_ADC0->ADANSA[0] = 0;
  R_ADC0->ADANSA[1] = 0;

  return ret;
}

//**********************************************************************
// A/D変換二回　値取得
//**********************************************************************
int16_t mcr_ad::getDataDual( uint8_t ch ) {
  int ret = -1;
  
  /* AD変換するチャンネル指定 */
 if( ch >= 16 ) {
   R_ADC0->ADANSA[1] = AD_PORT_ChA1[ch-16];
  }else{
   R_ADC0->ADANSA[0] = AD_PORT_ChA0[ch];
  }
  R_ADC0->ADCSR_b.ADST = 1;     // スタート

  while(R_ADC0->ADCSR_b.ADST == 1); //AD変換待機

  R_ADC0->ADCSR_b.ADST = 1;     // 二回目スタート

  while(R_ADC0->ADCSR_b.ADST == 1); //AD変換待機

  ret = R_ADC0->ADDR_b[ch].ADDR; //AD値取得

  /* チャンネル指定のリセット */
  R_ADC0->ADANSA[0] = 0;
  R_ADC0->ADANSA[1] = 0;

  return ret;
}

//**********************************************************************
// A/D変換　複数チャンネル一括取得（2回スキャンし2回目の値を採用）
// getDataDualをチャンネル毎に呼ぶより、ソフトの起動・待機オーバーヘッドが
// 1/チャンネル数になる。変換順序はハードウェアがチャンネル番号昇順で実行。
//**********************************************************************
void mcr_ad::scanDual( const uint8_t *ch, int16_t *ret, uint8_t num ) {
  uint16_t a0 = 0, a1 = 0;

  /* AD変換するチャンネルをまとめて指定 */
  for( uint8_t i = 0; i < num; i++ ) {
    if( ch[i] >= 16 ) {
      a1 |= AD_PORT_ChA1[ch[i]-16];
    } else {
      a0 |= AD_PORT_ChA0[ch[i]];
    }
  }
  R_ADC0->ADANSA[0] = a0;
  R_ADC0->ADANSA[1] = a1;

  R_ADC0->ADCSR_b.ADST = 1;     // スタート
  while(R_ADC0->ADCSR_b.ADST == 1); //AD変換待機

  R_ADC0->ADCSR_b.ADST = 1;     // 二回目スタート
  while(R_ADC0->ADCSR_b.ADST == 1); //AD変換待機

  for( uint8_t i = 0; i < num; i++ ) {
    ret[i] = R_ADC0->ADDR_b[ch[i]].ADDR; //AD値取得
  }

  /* チャンネル指定のリセット */
  R_ADC0->ADANSA[0] = 0;
  R_ADC0->ADANSA[1] = 0;
}

/*
Ver.1.00 2023.09.04 作成
Ver.1.01 2024.02.17 AN023,AN024,AN025のAD変換が正しく動作しないバグを修正
Ver.1.02 2024.08.22 ADシングルモードへ 想毬ちゃん
Ver.1.03 2026.07.02 複数チャンネル一括スキャン(scanDual)追加
*/

