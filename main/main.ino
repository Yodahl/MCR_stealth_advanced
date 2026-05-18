#include "personal_setting.h"
/*
 * ログ関係
 * @note
 *	10ms間隔でログ記録
 *	100：1秒分 / 1000：5秒分
 */
#define LOG_BUFF_SIZE 400

/*
 *	データ保存関連
 */
volatile bool saveFlag = false; // ログ記録フラグ
volatile int16_t saveDataA[15][LOG_BUFF_SIZE];
volatile int32_t saveDistA[LOG_BUFF_SIZE];
// volatile int16_t saveDataB[16][LOG_BUFF_SIZE];
char filename[32];
volatile uint16_t logCt = 0;
volatile uint16_t logRd = 0;
volatile bool whichsave = false;
volatile bool isWriting = false;
volatile uint16_t log_pattern = 0;
volatile int8_t motor_buff_Fl, motor_buff_Fr, motor_buff_Rl, motor_buff_Rr, motor_buff_stare;
volatile uint16_t nomal_buff;

/**********************************************************************/
/*
 * 型定義ｈ
 */
/**********************************************************************/
/*
 * プロトタイプ宣言
 */
void timerCallback(timer_callback_args_t __attribute((unused)) * p_args);

void initSens(void);
void initMotor(void);

void Diff_Nomal(void);

unsigned char dipsw_get(void);
unsigned char pushsw_get(void);
void motor_r(int accele_l, int accele_r);
void motor2_r(int accele_l, int accele_r);
void motor_f(int accele_l, int accele_r);
void motor2_f(int accele_l, int accele_r);
// void motor_mode_r(int mode_l, int mode_r);  // 未使用
// void motor_mode_f(int mode_l, int mode_r);  // 未使用
void servoPwmOut(int pwm);
int check_crossline(void);
int check_rightline(void);
int check_leftline(void);

int Get_Distance_cm(void);

// int diff_fi(int pwm); // front_in内輪差を求める
// int diff_ri(int pwm); // diff_ri
// int diff_ro(int pwm); // diff_ri
// int diff(int pwm);

int getServoAngle(void);
int getAnalogSensor(void);
void servoControl(void);
void servoControl2(void);
void readDataFlashParameter(void);
void writeDataFlashParameter(void);
int lcdProcess(void);
int slopeCheck(void);

void mtTest(void); // モータテスト
// int angleStreatCheck(int i, int jide_angle); // ブレーキ時のノイズ対策

// ブレーキ用
short Dig_M(short angle);
short Ang(void);
// void PDtrace_Control(short Dig, char boost_trig, short SP);
void PDtrace_Control(short Dig, short target_speed_ms, char boost_trig = false);
void PDtrace_Control_S(short Dig, short target_speed_ms);

// void StartTime(void);

// ログ関係
void SD_file_open(void);
void SD_file_close(void);
void writeLog(void);
void LOG_rec(void);

// 再生走行関係
void Log_Analysis(void);
void Open_Rep(void);
int Check_StraightSection(int32_t current_dist_pulse, float current_spd_m_s);

int straight_section_count = 0; // 実際に読み込まれた直線区間数

/**********************************************************************/
/*
 * 変数定義
 */

/*
 *	割り込み関係
 *	@note
 *		ログをSDカードに書きたいためここをメインとする！！
 */
static FspTimer interruptTimer;

/*
 *	ログ関係
 */
ArduinoSPI SPI(MISO1, MOSI1, SCK1, FORCE_SPI1_MODE); // RMC-RA4M1のmicroSD用SPIを選択(SPIの各端子はpins_arduino.hで定義)
char str[256];                                       // SDカードに書き込む用文字列とりあえず256Byte用意　　
File microSD;                                        // microSDのファイルアクセス用

/*
 *	analogセンサ用
 */
static mcr_ad ad;

// LED ON時のセンサ値
volatile uint16_t anaSensUR_on;
volatile uint16_t anaSensRR_on;
volatile uint16_t anaSensCR_on;
volatile uint16_t anaSensCC_on;
volatile uint16_t anaSensCL_on;
volatile uint16_t anaSensLL_on;
volatile uint16_t anaSensUL_on;
// LED OFF時のセンサ値
volatile uint16_t anaSensUR_off;
volatile uint16_t anaSensRR_off;
volatile uint16_t anaSensCR_off;
volatile uint16_t anaSensCC_off;
volatile uint16_t anaSensCL_off;
volatile uint16_t anaSensLL_off;
volatile uint16_t anaSensUL_off;
// LED ON/OFFの差分
volatile int16_t anaSensUR_diff;
volatile int16_t anaSensRR_diff;
volatile int16_t anaSensCR_diff;
volatile int16_t anaSensCC_diff;
volatile int16_t anaSensCL_diff;
volatile int16_t anaSensLL_diff;
volatile int16_t anaSensUL_diff;

// 2値化用閾値
volatile uint16_t thrSensUR;
volatile uint16_t thrSensRR;
volatile uint16_t thrSensCR;
volatile uint16_t thrSensCC;
volatile uint16_t thrSensCL;
volatile uint16_t thrSensLL;
volatile uint16_t thrSensUL;
// volatile uint16_t THR_Sens = 800;
// volatile uint16_t thrSensBK; // 黒閾値

// 2値化
volatile uint8_t digiSensUR;
volatile uint8_t digiSensRR;
volatile uint8_t digiSensCR;
volatile uint8_t digiSensCC;
volatile uint8_t digiSensCL;
volatile uint8_t digiSensLL;
volatile uint8_t digiSensUL;

volatile uint16_t sensorMax; // センサの最大値
volatile uint16_t sensorMin; // センサの最小値

volatile uint16_t sensorMaxRR; // RRの最大値
volatile uint16_t sensorMinRR; // RRの最小値

volatile uint16_t sensorMaxLL; // LLの最大値
volatile uint16_t sensorMinLL; // LLの最小値

volatile uint16_t sensorMaxCR; // CRの最大値
volatile uint16_t sensorMinCR; // CRの最小値

volatile uint16_t sensorMaxCL; // CLの最小値
volatile uint16_t sensorMinCL; // CLの最小値

volatile uint16_t old_sensorMax;
volatile uint16_t old_sensorMin;

volatile uint16_t sensNormalized[5];

volatile bool LED_flag = false;

volatile int g_current_servo_angle = 0;
volatile int g_current_raw_adc = 0;

/*
 *	R8Cプログラムから引用
 */
// const char *C_DATE = __DATE__; /* コンパイルした日付           */
// const char *C_TIME = __TIME__; /* コンパイルした時間           */

volatile int16_t pattern = 0; // マイコンカー動作パターン
volatile int16_t old_pattern = 0;

/*
 *	タイマカウント
 *	@note
 *		割り込み内でインクリメント
 */
volatile uint64_t cnt;
volatile uint64_t cnt1;          // タイマ用
volatile uint64_t cnt2;          // タイマ用
volatile uint64_t cnt3;          // タイマ用
volatile uint64_t check_sen_cnt; // タイマ用
volatile uint64_t check_enc_cnt; // タイマ用
volatile uint64_t cnt_lcd;       // LCD処理で使用
volatile uint64_t safety_cnt;    //

/*
 *	走行モード・時間処理等
 */
volatile int8_t isBrakeOn;            // ブレーキ判定フラグ
volatile int8_t crankMode = 0;        // クランク判定   1:クランクモード 0:通常
volatile int8_t crankDirection = 'N'; // クランクの方向 R:右 L:左
volatile int8_t laneMode = 0;         // レーン判定
volatile int8_t laneDirection = 'N';  // レーンの方向 R:右 L:左
volatile int8_t slopeTotalCount = 0;  // 坂通過数

volatile long slopeFinTime = 0;  // 登坂後の安定待ち
volatile int laneClearTime = 0;  // レーン後のブレーキ防止
volatile int crankClearTime = 0; // クランク後のブレーキ防止
volatile int mtPower = 0;        // コーナ立ち上がり徐々に
volatile int temp;

volatile int8_t crank_count = 0;
volatile int8_t linellcount = 0;
volatile int8_t linerrcount = 0;

volatile int8_t anglecount = 0;
volatile bool No_slope = false;

volatile int8_t slopecount = 0;
volatile bool isSlope = false;

volatile int8_t Trace_position = CENTER;

// volatile int16_t trace_offset = 0;

volatile int8_t sensLLon = OFF;
volatile int8_t sensRRon = OFF;

volatile bool SLOPE_flag = false;

volatile bool START_flag = false;
volatile bool Cheat_flag = false;
volatile bool Run_end = false;

volatile int8_t Motor_Max_PWM = PWM_MAX;

volatile int8_t mode = 0;

/*
 *	エンコーダ関連
 */
volatile int iTimer10;              // 10msカウント用
volatile long lEncoderTotal;        // 積算値保存用
volatile int iEncoder;              // 10ms毎の最新値
volatile unsigned int uEncoderBuff; // 計算用　割り込み内で使用
volatile long lEncoderBuff;         // エンコーダの値取得(距離制御用)

/*
 *	サーボ関連
 */
volatile int16_t iSensorBefore;     // 前回のセンサ値保存
volatile int16_t iServoPwm;         // サーボＰＷＭ値
volatile int16_t iAngle0;           // 中心時のA/D値保存
volatile int16_t iAngle2;           // ステアリング角速度
volatile int16_t iAngleBuff;        // 計算用 割り込み内で使用
volatile int8_t iServo_flag = STOP; // STOP:PWM＝0　TRACE:ライン追従　ANGLE:角度指定

/*
 *	サーボ関連2
 */
volatile int16_t iSetAngle;
volatile int16_t iAngleBefore2;
volatile int16_t iServoPwm2;

// キャリブレーション関係
// volatile int16_t angle = 80; // 切り替えしサーボ角度
// volatile int16_t start_PWM = 4;
// volatile int16_t cycleCount = 0; // 首を振った回数
// volatile int16_t calib_pattern = 0;

// volatile float Judg_percent = 0.42;   // 閾値パーセント  0.4
// volatile float Judg_BK_percent = 0.2; // 閾値パーセント

// volatile int atten_percent = 0.9; // 閾値減衰パーセント

/*
 *	DataFlash関係
 */
uint8_t data_buff[16];

// トレース用
volatile short Angle_D;    //
volatile short Angle_D_GF; //
// volatile char boost_flag = 0;
//  PD用変数
volatile char PD_trig = 0;
// ストップフラグ
// volatile char stop_D = 0;

volatile bool which_slope = true;
int16_t Slope_thr[5];   // サカ検知用
int16_t Slope_thr_1[5]; // サカ検知用
volatile int8_t slope_thr_cnt = 0;
volatile int8_t slope_thr_cnt_1 = 0;
volatile int8_t slope_start_cnt = 0;

/*
 *	LCD関連
 */
volatile uint8_t lcd_pattern = 1;

/************************************************************************/
/**
 * セットアップ(初期化).
 */
void setup()
{
    /* タイマ割り込み初期化 */
    // AGT 1msごとの割り込み処理の設定 PCLKB=24MHz ∴TIMER_SOURCE_DIV_1(1分周)なら、1/(24e6*1) * 24000 = 1ms  設定は１小さい値である23999を設定する
    // interruptTimer.begin(TIMER_MODE_PERIODIC, AGT_TIMER, 1, 23999, 1, (timer_source_div_t)TIMER_SOURCE_DIV_1, timerCallback);

    // AGTタイマーの設定
    interruptTimer.begin(
        TIMER_MODE_PERIODIC,
        AGT_TIMER,
        1,
        4799, // 【変更】0.2ms (200us) に設定
        1,
        (timer_source_div_t)TIMER_SOURCE_DIV_1,
        timerCallback);

    IRQManager::getInstance().addPeripheral(IRQ_AGT, (void *)interruptTimer.get_cfg());
    interruptTimer.open();

    /* シリアル初期化 */
    Serial2.begin(9600);
    // Serial2.begin(38400);

    //  while(!Serial);

    /* プッシュスイッチ初期化 */
    pinMode(RUN_SWITCH, INPUT);

    /* dip-sw初期化 */
    // ディップスイッチ1
    pinMode(25, INPUT);
    // ディップスイッチ2
    pinMode(26, INPUT);

    /* CPUボードLED */
    // LED D2
    pinMode(23, OUTPUT);
    // LED D3
    pinMode(13, OUTPUT);
    // LED R
    pinMode(57, OUTPUT);
    // LED L
    pinMode(58, OUTPUT);
    // 赤外線LED
    pinMode(55, OUTPUT);

    /* モーター初期化 */
    initMotor();

    /* センサ初期化 */
    initSens();

    /* エンコーダ初期化 */
    // 1相エンコーダ GPT6 P401(D32端子)のGTETRGAを使用
    startGPT6_1SouEncoder(GTETRGA, 4, 1);

    /* スイッチ初期化 */
    SwitchInit();
    /* LCD初期化 */
    LcdInit();
    /* I2C初期化 */
    //	initI2CEeprom();
    // 走行パターン初期化
    pattern = 0;
    /* シリアル初期化 */
    Serial2.println("readyOK");
}

/************************************************************************/
/**
 * ループ(メイン処理).
 */

void loop()
{
    signed int i, ret;

    // DataFlashパラメータ読み込み
    readDataFlashParameter();

    // マイコンカーの状態初期化
    Serial2.print("start\n");
    motor_f(0, 0);
    motor_r(0, 0);
    servoPwmOut(0);

    // リセット動作確認
    for (i = 0; i < 5; i++)
    {
        CPU_LED_2 = ON;
        // L_LED = ON;
        // R_LED=ON;
        delay(100);

        CPU_LED_2 = OFF;
        // L_LED = OFF;
        // R_LED=OFF;
        delay(100);
    }

    while (1)
    {

        /* 途中で停止処理 */
        // プッシュSW操作判定
        // Serial2.print("lEncoderTotal=");
        // Serial2.print(lEncoderTotal);
        // Serial2.print("  pattern=");
        // Serial2.println(pattern);

        //			LcdPosition(0, 0);
        //			LcdPrintf("main_loop");
        //			LcdPosition(0, 1);
        //			LcdPrintf("pattern = %d ", pattern);
        //		I2CEepromProcess(); /* I2C EEP-ROM保存処理          */

        //		while(1) {
        //			printf("lane_count=%d  linerrcount=%d\n",lane_count,linerrcount);
        //			printf("check_leftline()=%d  check_rightline()=%d\n",check_leftline(),check_rightline());

        // }

        if (logCt != logRd && !Run_end)
        {
            writeLog(); // ログデータの書き込み
        }

        switch (pattern)
        {
        case 0:
            cnt1 = 0;
            Trace_position = CENTER;
            pattern = 1;
            break;

            /*
             * プッシュスイッチ押下待ち
             */
        case 1:
            servoPwmOut(0);
            // LCD表示、パラメータ設定処理
            lcdProcess();

            if (pushsw_get())
            {
                if (dipsw_get() & 0x02)
                {
                    Open_Rep();
                    Cheat_flag = true;
                    Motor_Max_PWM = 100;
                }
                SD_file_open();
                //   パラメータ保存
                writeDataFlashParameter();
                Serial2.print("writeDataFlashParameter");
                cnt1 = 0;
                iAngle0 = VR_CENTER; // センター値固定
                LED_flag = true;
                iServo_flag = STOP;
                // iAngle0 = getServoAngle(); // 0度の位置記憶
                pattern = 3; // 3

                // sensorMax = 800; // センサの最大値
                // sensorMin = 300; // センサの最小値

                // sensorMaxRR = 800; // RRの最大値
                // sensorMinRR = 300; // RRの最小値

                // sensorMaxLL = 800; // LLの最大値
                // sensorMinLL = 300; // LLの最小値

                // sensorMaxCR = 800; // CRの最大値
                // sensorMinCR = 300; // CRの最小値

                // sensorMaxCL = 800; // CLの最小値
                // sensorMinCL = 300; // CLの最小値

                break;
            }
            break;

        // 角度PDゲイン調整
        case 2:
            iSetAngle = 0;
            iServo_flag = ANGLE;

            break;

            // オートセット開始
        case 3:
            i = getServoAngle();
            if (sensLLon == OFF || sensRRon == OFF)
            {
                iServo_flag = TRACE;
            }
            else
            {
                iServo_flag = STOP;
            }
            if (pushsw_get() && sensRRon == OFF)
            {
                START_flag = true;
            }

            if (START_flag)
            {
                if (sensRRon == ON) // スタートバー発見
                {
                    pattern = 4;
                    cnt2 = 0;
                    break;
                }
                else
                {
                    // motor_f(15, 15);
                    // motor_r(10, 10);
                    PDtrace_Control_S(i, 0.9);
                }
            }

            break;

        case 4:
            i = getServoAngle();
            iSetAngle = 0;
            // servoPwmOut(0);
            iServo_flag = STOP;
            if (sensRRon == OFF)
            {
                motor_f(0, 0);
                motor_r(0, 0);
                if (cnt1 > 500)
                {
                    pattern = 5;
                    break;
                }
                break;
            }
            else
            {
                // motor_f(15, 15);
                // motor_r(10, 10);
                PDtrace_Control_S(i, 0.9);
                cnt1 = 0;
            }

            break;

        case 5:
            // servoPwmOut(iServoPwm / 2);
            // motor_f(0, 0);
            if (sensRRon == ON && iEncoder == 0)
            {
                CPU_LED_2 = OFF;
                cnt1 = 0;
                // saveIndex = 0;
                // saveFlag = true; //pattern :5 で行う　/* データ保存開始 */
                check_sen_cnt = 0;
                check_enc_cnt = 0;
                cnt1 = 0;
                pattern = 9;
                break;
            }
            // else
            // {
            //     if (cnt1 < 800)
            //     {
            //         CPU_LED_2 = ON;
            //         R_LED = ON;
            //         L_LED = OFF;
            //     }
            //     else
            //     {
            //         CPU_LED_2 = OFF;
            //         R_LED = OFF;
            //         L_LED = ON;
            //         if (cnt1 > 1600)
            //         {
            //             cnt1 = 0;
            //         }
            //     }
            // }
            break;

        /*
         * スタートSW待ち
         */
        case 8:
            iServo_flag = TRACE;
            // lcdProcess();
            if (pushsw_get() == ON && cnt2 > 300)
            {
                if (digiSensRR == ON || digiSensLL == ON || digiSensCC == OFF) // キャリブレーション失敗
                    pattern = 1;                                               // キャリブレーションやり直し
                else
                {
                    // iAngle0 = getServoAngle();
                    // iAngle0 = VR_CENTER; // センター値固定
                    CPU_LED_2 = OFF;
                    cnt1 = 0;
                    // saveIndex = 0;
                    // saveFlag = true; //pattern :5 で行う　/* データ保存開始               */
                    check_sen_cnt = 0;
                    check_enc_cnt = 0;
                    pattern = 9;
                    delay(100);
                    while (pushsw_get() == ON)
                    {
                        if (cnt1 < 100)
                        {
                            CPU_LED_2 = ON;
                            R_LED = ON;
                            L_LED = OFF;
                        }
                        else
                        {
                            CPU_LED_2 = OFF;
                            R_LED = OFF;
                            L_LED = ON;
                            if (cnt1 > 200)
                            {
                                cnt1 = 0;
                            }
                        }
                    }
                }
            }
            if (cnt1 < 300)
            {
                CPU_LED_2 = ON;
                R_LED = ON;
                L_LED = OFF;
            }
            else
            {
                CPU_LED_2 = OFF;
                R_LED = OFF;
                L_LED = ON;
                if (cnt1 > 600)
                {
                    cnt1 = 0;
                }
            }
            break;

        case 9:
            i = getServoAngle();
            motor_f(0, 0);
            motor_r(0, 0);
            iSetAngle = 0;
            iServo_flag = STOP;
            if (cnt2 < 100)
            {
                CPU_LED_2 = ON;
                R_LED = ON;
                L_LED = OFF;
            }
            else
            {
                CPU_LED_2 = OFF;
                R_LED = OFF;
                L_LED = ON;
                if (cnt2 > 200)
                {
                    cnt2 = 0;
                }
            }

            if (cnt1 > data_buff[START_TIME_ADDR] * 1000)
            {
                LcdPosition(0, 0);
                // LcdPrintf("case 11");
                // iAngle0 = getServoAngle(); /* 0度の位置記憶 */
                SLOPE_flag = false;
                // エンコーダ関係初期化
                lEncoderBuff = 0;
                lEncoderTotal = 0;
                pattern = 10;
                cnt1 = 0;
                saveFlag = true; // データ保存開始
                check_sen_cnt = 0;
                check_enc_cnt = 0;
            }
            break;

        case 10:
            i = getServoAngle();
            iSetAngle = 0;
            iServo_flag = STOP;
            // servoPwmOut(0);
            motor_f(100, 100);
            motor_r(100, 100);
            if (lEncoderTotal >= 150)
                pattern = 11;

            break;

        /*
         * 通常走行処理
         */
        case 11:
            /* 通常トレース */
            Trace_position = CENTER;
            iServo_flag = TRACE;
            i = getServoAngle(); // -120 ~ 120 の範囲
            iSetAngle = 0;

            // --- Angle_D_GFの計算（角度による速度補正値） ---
            if (Angle_D > 0)
            {
                Angle_D_GF = 0;
            }
            else
            {
                Angle_D_GF = Angle_D; // 負の値（減速方向）
            }

            // --- 目標速度の決定 ---
            float target_value; // PDtrace_Controlに渡す制御値

            // ========== 舵角による速度調整 ==========
            if (abs(i) > 110)
            {
                target_value = (data_buff[CORNER_SPEED_ADDR] * 80.0f / 100.0f) + Angle_D_GF; // 50
            }
            else if (abs(i) > 80)
            {
                target_value = (data_buff[CORNER_SPEED_ADDR] * 86.0f / 100.0f) + Angle_D_GF; // 65
            }
            else if (abs(i) > 68)
            {
                target_value = (data_buff[CORNER_SPEED_ADDR] * 94.0f / 100.0f) + Angle_D_GF; // 80
            }
            else if (abs(i) > 47)
            {
                target_value = (data_buff[CORNER_SPEED_ADDR] * 96.0f / 100.0f) + Angle_D_GF; // 88
            }
            else if (abs(i) > 20)
            {
                target_value = (data_buff[CORNER_SPEED_ADDR] * 98.0f / 100.0f) + Angle_D_GF;
            }
            else if (abs(i) > 10)
            {
                target_value = (data_buff[CORNER_SPEED_ADDR]) + Angle_D_GF;
            }
            // ========== 直線区間（舵角小さい） ==========
            else
            {
                if (Cheat_flag)
                {
                    if (mode == ACCEL)
                    {
                        target_value = 100; // 全力加速指示値
                    }
                    else if (mode == BRAKE)
                    {
                        // ブレーキ判定時：コーナー速度に落とす
                        target_value = data_buff[CORNER_SPEED_ADDR];
                    }
                    else // mode == OFF
                    {
                        // 直線区間外：通常走行速度
                        target_value = data_buff[TRG_SPEED_ADDR];
                    }
                }
                else
                {
                    // Cheat_flag無効時：通常走行速度
                    target_value = data_buff[TRG_SPEED_ADDR];
                }
            }

            // --- PDtrace_Control呼び出し ---
            PDtrace_Control(i, target_value, 0);

            // --- SLOPE_flag管理 ---
            if (lEncoderTotal - lEncoderBuff >= 2000 && !SLOPE_flag && slopeTotalCount != 0)
            {
                SLOPE_flag = true;
            }

            if (slope_start_cnt == 10 && slopeTotalCount == 0)
            {
                SLOPE_flag = true;
            }
            break;

        case 50:                 // 坂
            iServo_flag = TRACE; // ライントレース
            Trace_position = CENTER;
            i = getServoAngle();
            iSetAngle = 0;
            PDtrace_Control(i, data_buff[TRG_SPEED_ADDR]);

            if (abs(i) > 15)
            {
                anglecount++;
                if (anglecount > 50) // 45
                {
                    No_slope = true;
                    pattern = 11;
                    lEncoderBuff = lEncoderTotal;
                }
            }
            else
            {
                anglecount = 0;
                No_slope = false;
            }

            // if (No_slope)
            // {
            // }

            if (Get_Distance_cm() > 30)
            {
                pattern = 51;
                lEncoderBuff = lEncoderTotal;
                break;
            }
            break;

        case 51:
            iServo_flag = TRACE; // ライントレース
            Trace_position = CENTER;
            i = getServoAngle();
            iSetAngle = 0;
            // SLOPE_flag = false;
            PDtrace_Control(i, data_buff[SLOPE_SPEED_ADDR]);

            if ((slopeCheck() == -1 && Get_Distance_cm() > 200) || Get_Distance_cm() > 250)
            { // 下り検知
                pattern = 52;
                lEncoderBuff = lEncoderTotal;
            }
            break;

        case 52:
            iServo_flag = TRACE; // ライントレース
            Trace_position = CENTER;
            i = getServoAngle();
            iSetAngle = 0;
            SLOPE_flag = false;
            PDtrace_Control(i, data_buff[TRG_SPEED_ADDR]);
            if (Get_Distance_cm() > 180)
            {
                pattern = 11;
                lEncoderBuff = lEncoderTotal;
                break;
            }
            break;

            /*
             * クランク走行処理
             */
        case 101:
            /* クロスライン通過処理 */
            iSetAngle = 0;
            i = getServoAngle();
            Trace_position = CENTER;
            if (digiSensLL == ON && digiSensRR == ON) // 左右に振られる対処
                iServo_flag = STOP;
            else
                iServo_flag = TRACE;

            R_LED = ON;
            L_LED = ON;
            PDtrace_Control(i, data_buff[CRANK_SPEED_ADDR]);
            if (Get_Distance_cm() > 35) // 誤読み防止(350mm)
            {
                cnt1 = 0;
                lEncoderBuff = lEncoderTotal;
                pattern = 106;
                laneMode = 0; // レーンモードフラグクリア
                break;
            }

            break;

            // case 104: // クロスライン後の処理(1段目の減速処理)
            //     i = getServoAngle();
            //     iServo_flag = TRACE;
            //     PDtrace_Control(i, data_buff[CRANK_SPEED_ADDR]);

            //     if (lEncoderTotal - lEncoderBuff >= 200) // 200
            //     {
            //         // 200m
            //         lEncoderBuff = lEncoderTotal;
            //         pattern = 106;
            //         break;
            //     }
            //     break;

        case 106: // クランク処理 (2段目の減速処理)　ハーフライン検出
            i = getServoAngle();
            iServo_flag = TRACE;
            PDtrace_Control(i, data_buff[CRANK_SPEED_ADDR]);
            if (sensLLon == ON) // クランク方向　左
            {
                crankDirection = 'L'; // クランク方向記憶変数＝左クランク
                lEncoderBuff = lEncoderTotal;
                pattern = 108;
                break;
            }
            else if (sensRRon == ON) // クランク方向　右
            {
                crankDirection = 'R'; // クランク方向記憶変数＝左クランク
                lEncoderBuff = lEncoderTotal;
                pattern = 108;
                break;
            }

            if (lEncoderTotal - lEncoderBuff >= 1000) // 2000m
            {
                pattern = 11; // 通常に戻す
                break;
            }
            break;

        case 108: // クランク処理	 　ハーフライン検出後
            i = getServoAngle();
            if (crankDirection == 'L') // クランク方向　左
            {
                iSetAngle = CRANK_ANGLE_L / 3;
                iServo_flag = ANGLE;

                if (iEncoder * PULSE_TO_MS > CRANK_TOP_SPEED)
                {
                    motor_f(-100, 10); // 前
                    motor_r(-100, 10); // 後
                }
                else if (iEncoder * PULSE_TO_MS < CRANK_MIN_SPEED)
                {
                    motor_f(10, 100); // 前
                    motor_r(10, 100); // 後
                }
                else
                {
                    motor_f(10, 10); // 前
                    motor_r(10, 10); // 後
                }
            }

            else if (crankDirection == 'R') // クランク方向　右
            {
                iSetAngle = -CRANK_ANGLE_R / 3;
                iServo_flag = ANGLE;

                if (iEncoder * PULSE_TO_MS > CRANK_TOP_SPEED)
                {
                    motor_f(10, -100); // 前
                    motor_r(10, -100); // 後
                }
                else if (iEncoder * PULSE_TO_MS < CRANK_MIN_SPEED)
                {
                    motor_f(100, 10); // 前
                    motor_r(100, 10); // 後
                }
                else
                {
                    motor_f(10, 10); // 前
                    motor_r(10, 10); // 後
                }
            }

            if (Get_Distance_cm() >= 8)
            {
                pattern = 110;
                lEncoderBuff = lEncoderTotal;
                cnt1 = 0;
                CPU_LED_2 = ON;
                break;
            }
            break;

        case 110: // クランク処理	 　ハーフライン検出後
            if (crankDirection == 'L')
            {                              // クランク方向　左
                iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                iServo_flag = ANGLE;

                if (iEncoder * PULSE_TO_MS > CRANK_TOP_SPEED)
                {
                    motor_f(-100, 10); // 前
                    motor_r(-100, 10); // 後
                }
                else if (iEncoder * PULSE_TO_MS < CRANK_MIN_SPEED)
                {
                    motor_f(10, 100); // 前
                    motor_r(10, 100); // 後
                }
                else
                {
                    motor_f(10, 10); // 前
                    motor_r(10, 10); // 後
                }
            }
            else if (crankDirection == 'R')
            {                               // クランク方向　右
                iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                iServo_flag = ANGLE;

                if (iEncoder * PULSE_TO_MS > CRANK_TOP_SPEED)
                {
                    motor_f(10, -100); // 前
                    motor_r(10, -100); // 後
                }
                else if (iEncoder * PULSE_TO_MS < CRANK_MIN_SPEED)
                {
                    motor_f(100, 10); // 前
                    motor_r(100, 10); // 後
                }
                else
                {
                    motor_f(10, 10); // 前
                    motor_r(10, 10); // 後
                }
            }

            if (Get_Distance_cm() >= 12)
            {
                pattern = 118;
                cnt1 = 0; // 116:20ms待ち
                break;
            }
            break;

            // case 112:
            //     if (crankDirection == 'L')
            //     { // クランク方向　左

            //         iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
            //         iServo_flag = ANGLE;
            //         motor_f(30, 50);   // 前 （左,右）
            //         motor_r(-60, -25); // 後モータ（左,右）

            //         // 通常クランク処理
            //         if (digiSensCC == ON && digiSensRR == ON)
            //         {
            //             pattern = 116; // 114
            //         }
            //     }
            //     else if (crankDirection == 'R')
            //     { // クランク方向　右

            //         iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
            //         iServo_flag = ANGLE;
            //         motor_f(50, 30);   // 前 （左,右）
            //         motor_r(-25, -60); // 後モータ（左,右）

            //         // 通常クランク処理
            //         if (digiSensCC == ON && digiSensLL == ON)
            //         {
            //             pattern = 116; // 114
            //         }
            //         // コースアウト対応クランク処理(←対応しない)
            //     }
            //     break;

            // case 116: // クランク処理
            //     if (crankDirection == 'L')
            //     {                              // クランク方向　左
            //         iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
            //         iServo_flag = ANGLE;
            //         motor_f(30, 40); // 前 （左,右)(100,70) (30, 0)
            //         motor_r(-40, 0); // 後モータ(左,右)

            //         if (sensLLon == ON /*&& anaSensCL_diff < thrSensCL*/)
            //         {
            //             pattern = 118; // ﾌﾛﾝﾄ左ｾﾝｻ（ﾃﾞｼﾞﾀﾙまたはｱﾅﾛｸﾞ）反応時次の処理へ
            //             cnt1 = 0;
            //             break;
            //         }
            //         /* ステアリング角 0:中央 +:左 -:右 1.6=1度 */
            //         //					ST_A = 0;
            //         //					ST_B = 1;
            //         //					ST_PWM=10;
            //     }
            //     else if (crankDirection == 'R')
            //     {                               // クランク方向　右
            //         iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
            //         iServo_flag = ANGLE;
            //         motor_f(40, 30); // 前 （左,右）(70,100) (0, 30)
            //         motor_r(0, -40); // 後モータ（左,右)

            //         if (sensRRon == ON /*&& anaSensCR_diff < thrSensCR*/)
            //         {
            //             pattern = 118; // ﾌﾛﾝﾄ左ｾﾝｻ（ﾃﾞｼﾞﾀﾙまたはｱﾅﾛｸﾞ）反応時次の処理へ
            //             cnt1 = 0;
            //         }
            //     }
            //     break;

        case 118:
            if (crankDirection == 'L')
            {                              // クランク方向　左
                iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                iServo_flag = ANGLE;

                if (iEncoder * PULSE_TO_MS > CRANK_TOP_SPEED)
                {
                    motor_f(-100, 10); // 前
                    motor_r(-100, 10); // 後
                }
                else if (iEncoder * PULSE_TO_MS < CRANK_MIN_SPEED)
                {
                    motor_f(10, 100); // 前
                    motor_r(10, 100); // 後
                }
                else
                {
                    motor_f(10, 10); // 前
                    motor_r(10, 10); // 後
                }
            }
            else if (crankDirection == 'R')
            {                                 // クランク方向　右
                iSetAngle = -(CRANK_ANGLE_R); /* +で左 -で右に曲がります      */
                iServo_flag = ANGLE;

                if (iEncoder * PULSE_TO_MS > CRANK_TOP_SPEED)
                {
                    motor_f(10, -80); // 前
                    motor_r(10, -80); // 後
                }
                else if (iEncoder * PULSE_TO_MS < CRANK_MIN_SPEED)
                {
                    motor_f(100, 10); // 前
                    motor_r(100, 10); // 後
                }
                else
                {
                    motor_f(10, 10); // 前
                    motor_r(10, 10); // 後
                }
            }
            if (digiSensLL == OFF && digiSensCC == ON && digiSensRR == OFF)
            {
                lEncoderBuff = lEncoderTotal;
                pattern = 120;
                crankMode = 0; // クランクモードクリア
            }
            break;

        case 120:
            /* 少し時間が経つまで待つ */
            i = getServoAngle(); // ステアリング角度取得
            iServo_flag = TRACE;
            motor_r(100, 100);
            motor_f(100, 100);
            if (abs(i) < 8 && Get_Distance_cm() >= 20)
            {
                cnt1 = 0;
                pattern = 11;
                lEncoderBuff = lEncoderTotal;
                crankDirection = 0; // クランクモード（クランク方向）クリア
                laneMode = 0;       // レーンモードクリア
                laneDirection = 0;  // レーンモード（レーン方向）クリア
                crankClearTime = 50;
                break;
            }

            /* if(check_leftline()){
            crankDirection = 0;  // クランクモード（クランク方向）クリア
            laneMode = 0;  // レーンモードクリア
            laneDirection = 0;  // レーンモード（レーン方向）クリア
            crankClearTime=50;
            cnt1 = 0;
            laneMode = 1;
            pattern = 151;
            laneDirection = 'L';
            break;
          }
          if(check_rightline()){
            crankDirection = 0;  // クランクモード（クランク方向）クリア
            laneMode = 0;  // レーンモードクリア
            laneDirection = 0;  // レーンモード（レーン方向）クリア
            crankClearTime=50;
            cnt1 = 0;
            laneMode = 1;
            pattern = 151;
            laneDirection = 'R';
            break;
          }
          */
            break;

            /************************************************************************/
            /* レーンチェンジの処理*/
            /************************************************************************/
        case 151: // ハーフライン後の処理１（速度制御）

            iSetAngle = 0;
            i = getServoAngle();

            if (Get_Distance_cm() < 5)
            { // 左右に振られる対処
                iServo_flag = STOP;
                if (check_crossline())
                { /* クロスラインチェック         */
                    cnt1 = 0;
                    crankMode = 1;
                    lEncoderBuff = lEncoderTotal;
                    pattern = 101;
                    break;
                }
            }
            else
            {
                iServo_flag = TRACE;
                if (laneDirection == 'L') // レーン方向　左
                    Trace_position = RIGHT;
                else if (laneDirection == 'R') // レーン方向　右
                    Trace_position = LEFT;
                ;
            }

            PDtrace_Control(i, data_buff[LANE_SPEED_ADDR]);
            crankMode = 1; // ステアリング制御補正なし

            if (Get_Distance_cm() > 40) // 60
            {                           // 50mm
                lEncoderBuff = lEncoderTotal;
                pattern = 152;
                break;
            }
            break;

        case 152:                // クロスライン後の処理(白線トレース時)
            i = getServoAngle(); // ステアリング角度取得
            iServo_flag = TRACE;
            if (laneDirection == 'L')
            { // レーン方向　左
                Trace_position = RIGHT;
                if (digiSensCC == OFF && digiSensCR == OFF && digiSensRR == OFF)
                {
                    pattern = 154; // 全てのセンサ黒検出時次の処理へ
                    lEncoderBuff = lEncoderTotal;
                }
            }
            else if (laneDirection == 'R')
            { // レーン方向　右
                Trace_position = LEFT;
                if (digiSensLL == OFF && digiSensCL == OFF && digiSensCC == OFF)
                {
                    pattern = 154; // 全てのセンサ黒検出時次の処理へ
                    lEncoderBuff = lEncoderTotal;
                }
            }
            PDtrace_Control(i, data_buff[LANE_SPEED_ADDR]);

            // レーン誤検知用の通常復帰
            if (Get_Distance_cm() >= 150)
            {                 // 2000m
                pattern = 11; // 通常に戻す
                break;
            }

            /* クロスラインチェック         */
            // if (check_crossline())
            // {
            //     cnt1 = 0;
            //     crankMode = 1;
            //     pattern = 101;
            //     break;
            // }
            break;

        case 154:                // 白線トレース終了後処理	最外センサ　白反応待ち
            i = getServoAngle(); // ステアリング角度取得
            Trace_position = CENTER;

            if (laneDirection == 'L')
            {                             // レーン方向　左
                iSetAngle = LANE_ANGLE_L; // +で左 -で右に曲がります
                iServo_flag = ANGLE;      //
                motor_f(10, 10);          // 前 （左,右-70,25）
                motor_r(10, 10);          // 後（左,右-30,10)

                if (sensLLon == ON) // digiSensCL
                {
                    cnt1 = 0;
                    pattern = 160;
                }
            }
            else if (laneDirection == 'R')
            {                              // レーン方向　右
                iSetAngle = -LANE_ANGLE_R; // +で左 -で右に曲がります
                iServo_flag = ANGLE;       //
                motor_f(10, 10);           // 前 （左,右25,-70）
                motor_r(10, 10);           // 後（左,右10,-30)

                if (sensRRon == ON) // digiSensCR
                {
                    cnt1 = 0;
                    pattern = 160;
                }
            }
            break;

        case 160: // 10m秒待ち後の処理　最内センサ　白反応時待ち
            if (laneDirection == 'L')
            { // レーン方向　左

                iSetAngle = LANE_ANGLE_L; /* +で左 -で右に曲がります */
                iServo_flag = ANGLE;      // 2角度制御 3:割込制御無
                motor_f(10, 10);          // 前 （左,右）
                motor_r(10, 10);          // 後（左,右)

                if (digiSensCC == ON) // CL
                {
                    cnt1 = 0;
                    pattern = 164;
                }
            }
            else if (laneDirection == 'R')
            {                              // レーン方向　右
                iSetAngle = -LANE_ANGLE_R; /* +で左 -で右に曲がります */
                iServo_flag = ANGLE;       // 2角度制御 3:割込制御無
                motor_f(10, 10);           // 前 （左,右）
                motor_r(10, 10);           // 後（左,右)

                if (digiSensCC == ON) // CR
                {
                    cnt1 = 0;
                    pattern = 164;
                }
            }
            break;

        case 164: // 10ms待ち後の処理　最内センサ　黒反応時待ち
            if (laneDirection == 'L')
            {                             // レーン方向　左
                iSetAngle = LANE_ANGLE_L; /* +で左 -で右に曲がります */
                iServo_flag = ANGLE;      // 2角度制御 3:割込制御無
                motor_f(10, 10);          // 前 （左,右）
                motor_r(10, 10);          // 後（左,右)

                if (sensRRon == ON)
                {
                    pattern = 166;
                    cnt1 = 0;
                }
            }

            else if (laneDirection == 'R')
            {                              // レーン方向　右
                iSetAngle = -LANE_ANGLE_R; /* +で左 -で右に曲がります */
                iServo_flag = ANGLE;       // 2角度制御 3:割込制御無
                motor_f(10, 10);           // 前 （左,右）
                motor_r(10, 10);           // 後（左,右)

                if (sensLLon == ON)
                {
                    pattern = 166;
                    cnt1 = 0;
                }
            }
            break;

        case 166: // 最内センサ　黒反応後の処理（大カウンター）　最内センサ　白反応時待ち
            if (laneDirection == 'L')
            {                                      // レーン方向　左
                iSetAngle = -((LANE_ANGLE_L / 2)); // カウンター　
                iServo_flag = ANGLE;               // 2角度制御 3:割込制御無
                motor_f(10, 10);                   // 前 （左,右）
                motor_r(10, 10);                   // 後（左,右)
                if (digiSensCC == ON && cnt1 >= 10)
                {
                    pattern = 168;
                }
            }
            else if (laneDirection == 'R')
            {                                     // レーン方向　右　カウンター処理
                iSetAngle = ((LANE_ANGLE_R / 2)); // カウンター
                iServo_flag = ANGLE;              // 2角度制御 3:割込制御無
                motor_f(10, 10);                  // 前 （左,右）
                motor_r(10, 10);                  // 後（左,右)
                if (digiSensCC == ON && cnt1 >= 10)
                {
                    pattern = 168;
                }
                break;
            }
            break;

        case 168: // センターセンサ　白反応時待ち
            if (laneDirection == 'L')
            { // レーン方向　左
                if (sensLLon)
                {
                    iSetAngle = -((LANE_ANGLE_L / 4)); /* +で左 -で右に曲がります */
                    iServo_flag = ANGLE;               // 2角度制御 3:割込制御無
                    motor_f(10, 10);                   // 前 （左,右）
                    motor_r(10, 10);                   // 後（左,右
                }
            }

            else if (laneDirection == 'R')
            { // レーン方向　右　カウンター処理
                if (sensRRon)
                {
                    iSetAngle = ((LANE_ANGLE_R / 4)); /* +で左 -で右に曲がります */
                    iServo_flag = ANGLE;              // 2角度制御 3:割込制御無
                    motor_f(10, 10);                  // 前 （左,右）
                    motor_r(10, 10);                  // 後（左,右)
                }
            }

            if (digiSensCC == ON)
            // if (digiSensLL == OFF && digiSensCC == ON && digiSensRR == OFF)
            {
                pattern = 170; /*中央デジタルセンサ反応時次の処理へ*/
                Trace_position = CENTER;
                lEncoderBuff = lEncoderTotal;
                cnt1 = 0;
                break;
            }
            break;

        case 170:
            /* 少し時間が経つまで待つ */
            i = getServoAngle(); // ステアリング角度取得
            iServo_flag = TRACE;
            if (Get_Distance_cm() >= 10)
            {
                motor_r(10, 10);
                motor_f(10, 10);
            }
            else
            {
                motor_r(100, 100);
                motor_f(100, 100);
            }
            // if (abs(i) < 10 && cnt1 > 100)
            if (Get_Distance_cm() >= 20 && abs(anaSensCL_diff - anaSensCR_diff) < 80) // 100
            {
                cnt1 = 0;
                pattern = 11;
                crankMode = 0;      // クランクモードクリア
                crankDirection = 0; // クランクモード（クランク方向）クリア
                laneMode = 0;       // レーンモードクリア
                laneDirection = 0;  // レーンモード（レーン方向）クリア
                laneClearTime = 200;
                break;
            }
            break;

        case 220:
            i = getServoAngle();
            iServo_flag = TRACE;
            PDtrace_Control_S(i, 1.2);
            if (iEncoder <= 1)
            {
                lEncoderBuff = lEncoderTotal;
                pattern = 230;
                break;
            }
            break;

        case 230:
            i = getServoAngle();
            iServo_flag = TRACE;
            PDtrace_Control_S(i, 0.6);
            if (Get_Distance_cm() >= 100)
            {
                pattern = 231;
                break;
            }
            break;

        case 231:
            /* 停止処理 */
            iServo_flag = TRACE;
            motor_f(0, 0);
            motor_r(0, 0);
            pattern = 232;
            break;

        case 232:
            iServo_flag = TRACE;
            // PDtrace_Control(i, 0.2);
            if (iEncoder < 1)
            {
                // servoPwmOut(0);
                pattern = 233;
                cnt1 = 0;
                break;
            }
            break;

        case 233:
            iServo_flag = STOP;
            LED_flag = false;
            saveFlag = false; // ログデータ保存停止
            Run_end = true;
            if (cnt2 < 100)
            {
                CPU_LED_2 = ON;
                CPU_LED_3 = OFF;
            }
            else
            {
                CPU_LED_2 = OFF;
                CPU_LED_3 = ON;
                if (cnt2 > 200)
                {
                    cnt2 = 0;
                }
            }
            if (pushsw_get() && cnt1 > 500)
            {
                SD_file_close(); // SDカードのファイル閉じる（閉じないとファイルが保存されない）
                pattern = 234;
                cnt1 = 0;
            }
            break;

        case 234:
            iServo_flag = STOP;
            // ログ出力
            Serial2.print("\n");
            Serial2.print("Run Data Out\n");
            CPU_LED_2 = ON;
            CPU_LED_3 = ON;
            if (pushsw_get() && cnt1 > 500)
            {
                Log_Analysis();
                pattern = 235;
            }
            break;

        case 235:
            break;

        case 241:
            break;

        case 242:
            break;

        case 243:
            break;

        case 245:
            break;

        case 246:
            /* データ転送 */
            /* 終わりのチェック */
            break;

        case 247:
            break;

        default:
            break;
        }
    }
}

/**********************************************************************/
/*
 * 0.2msタイマ割り込み (5ステップ制御 / Duty 20%)
 */
void timerCallback(timer_callback_args_t __attribute((unused)) * p_args)
{
    static unsigned int timer_counter = 0; // 0.2msごとのカウンタ (0-4)
    signed long i;

    // 1ms周期 (0.2ms * 5 = 1.0ms)
    switch (++timer_counter)
    {
    /* ----------------------------------------------------------------
       case 1: [0.0ms - 0.2ms]
       カウンタ更新・消灯時(OFF)の値取得・LED点灯開始
    ---------------------------------------------------------------- */
    case 1:
        // --- カウンタ・タイマー更新 ---
        cnt1++;
        cnt2++; // LED制御用
        cnt3++;
        slopeFinTime++; // 坂誤検出防止タイマー

        // mtPower++; // コーナリング時PWMを徐々にアップ用

        if (laneClearTime > 0)
        {
            laneClearTime--;
        }

        if (crankClearTime > 0)
        {
            crankClearTime--;
        }

         // 【究極の対策】他センサとの混線を防ぐため、この安全なタイミングでADCを読む
         g_current_raw_adc = BAR_ANGLE;
         if (g_current_raw_adc < 1000)
         {
             // 取得した生データをフィルタに通し、グローバル変数に保存
             g_current_servo_angle = Dig_M(g_current_raw_adc) - iAngle0;
         }

         if (digiSensLL == ON)
        {
            linellcount++;
            if (linellcount > 8) // 45
            {
                sensLLon = ON;
                // CPU_LED_2 = ON;
            }
        }
        else
        {
            linellcount = 0;
            sensLLon = OFF;
        }

        if (digiSensRR == ON)
        {
            linerrcount++;
            if (linerrcount > 8)
            {
                sensRRon = ON;
                // CPU_LED_2 = ON;
            }
        }
        else
        {
            linerrcount = 0;
            sensRRon = OFF;
        }

        if (slopeCheck() == 1)
        {
            slopecount++;
            if (slopecount > 8) // 30
            {
                isSlope = true;
            }
        }
        else
        {
            slopecount = 0;
            isSlope = false;
        }

        // --- センサ値取得(OFF値: 外乱光) ---
        // anaSensUR_off = ANA_SENS_UR;
        anaSensRR_off = ANA_SENS_RR;
        anaSensCR_off = ANA_SENS_CR;
        anaSensCC_off = ANA_SENS_CC;
        anaSensCL_off = ANA_SENS_CL;
        anaSensLL_off = ANA_SENS_LL;
        // anaSensUL_off = ANA_SENS_UL;

        // --- LED点灯 (次の0.2ms間だけ光る) ---
        if (LED_flag)
            INFRARED_LED = ON;

        break;

    /* ----------------------------------------------------------------
       case 2: [0.2ms - 0.4ms]
       点灯時(ON)の値取得・LED消灯・差分計算・正規化・2値化
    ---------------------------------------------------------------- */
    case 2:
        // --- センサ値取得(ON値) ---
        // anaSensUR_on = ANA_SENS_UR;
        anaSensRR_on = ANA_SENS_RR;
        anaSensCR_on = ANA_SENS_CR;
        anaSensCC_on = ANA_SENS_CC;
        anaSensCL_on = ANA_SENS_CL;
        anaSensLL_on = ANA_SENS_LL;
        // anaSensUL_on = ANA_SENS_UL;

        // --- LED消灯 (Duty 20%確保のため即消灯) ---
        INFRARED_LED = OFF;

        // --- 差分計算 ---
        // anaSensUR_diff = anaSensUR_on - anaSensUR_off;
        anaSensRR_diff = anaSensRR_on - anaSensRR_off;
        anaSensCR_diff = anaSensCR_on - anaSensCR_off;
        anaSensCC_diff = anaSensCC_on - anaSensCC_off;
        anaSensCL_diff = anaSensCL_on - anaSensCL_off;
        anaSensLL_diff = anaSensLL_on - anaSensLL_off;
        // anaSensUL_diff = anaSensUL_on - anaSensUL_off;

        // --- 差分の正規化 ---
        // if (pattern == 11 || pattern == 8)
        Diff_Nomal();

        // --- 2値化 ---
        digiSensRR = ((sensNormalized[sRR] > THR_M_Sens) ? ON : OFF);
        digiSensCR = ((sensNormalized[sCR] > THR_Sens) ? ON : OFF);
        digiSensCC = ((sensNormalized[sCC] > THR_Sens) ? ON : OFF);
        digiSensCL = ((sensNormalized[sCL] > THR_Sens) ? ON : OFF);
        digiSensLL = ((sensNormalized[sLL] > THR_M_Sens) ? ON : OFF);
        break;

    /* ----------------------------------------------------------------
       case 3: [0.4ms - 0.6ms]
    ---------------------------------------------------------------- */
    case 3:
        if (iServo_flag == TRACE)
        {
            servoControl();
            servoPwmOut(iServoPwm);
        }
        else if (iServo_flag == ANGLE)
        {
            servoControl2();
            servoPwmOut(iServoPwm2);
        }
        else
        {
            servoPwmOut(0);
        }

        break;

   /* ----------------------------------------------------------------
       case 4: [0.6ms - 0.8ms]
    ---------------------------------------------------------------- */
    case 4:
        if (Cheat_flag)
        {
            mode = Check_StraightSection(lEncoderTotal);
        }

        if ((pattern == 11 || pattern == 50) && lEncoderTotal > 600)
        {
            if (isSlope && SLOPE_flag && !check_leftline() && !check_rightline() && !check_crossline())
            {
                // 坂走行処理へ	のぼるくん
                slopeTotalCount++;
                pattern = 50;
                lEncoderBuff = lEncoderTotal;
            }

            // クロスラインチェック
            if (check_crossline() && SLOPE_flag)//&& crankClearTime == 0 && laneClearTime == 0
            {
                cnt1 = 0;
                crankMode = 1;
                pattern = 101;
                lEncoderBuff = lEncoderTotal;
            }

            // 左ハーフラインチェック
            if (check_leftline() && SLOPE_flag)//&& crankClearTime == 0 && laneClearTime == 0
            {
                cnt1 = 0;
                laneMode = 1;
                pattern = 151;
                laneDirection = 'L';
                Trace_position = RIGHT;
                lEncoderBuff = lEncoderTotal;
            }
            // 右ハーフラインチェック
            if (check_rightline() && SLOPE_flag)//&& crankClearTime == 0 && laneClearTime == 0
            {
                cnt1 = 0;
                laneMode = 1;
                pattern = 151;
                laneDirection = 'R';
                Trace_position = LEFT;
                lEncoderBuff = lEncoderTotal;
            }
        }
        
        break;

    /* ----------------------------------------------------------------
       case 5: [0.8ms - 1.0ms]
       10ms周期処理・停止判定・リセット
    ---------------------------------------------------------------- */
    case 5:
        // 10ms周期処理(エンコーダ/ログ/舵角加速度)
        /* 10回中1回実行する処理 */
        switch (++iTimer10)
        {
        case 1:
            i = R_GPT6->GTCNT;
            iEncoder = (int16_t)(i - uEncoderBuff);
            lEncoderTotal += iEncoder;
            uEncoderBuff = i;
            PD_trig = 1;
            break;
        case 2:
            // i = BAR_ANGLE;
            i = g_current_raw_adc;
            iAngle2 = i - iAngleBuff;
            iAngleBuff = i;
            Angle_D = Ang();
            break;

        case 3:
            break;

        case 4:
            break;
        case 8:
            if (saveFlag)
            {
                LOG_rec(); // ログをRAMに保存
            }
            break;

        case 9:
            // 坂検出ロジック
            if (pattern == 11 && abs(getServoAngle()) < 8)
            {
                if (slope_start_cnt <= 10)
                {
                    slope_start_cnt++;
                }
                if (which_slope)
                {
                    Slope_thr[slope_thr_cnt] = anaSensCC_diff;
                    if (slope_thr_cnt >= 5)
                    {
                        slope_thr_cnt = 0;
                        which_slope = false;
                    }
                    else
                    {
                        slope_thr_cnt++;
                    }
                }
                else
                {
                    Slope_thr_1[slope_thr_cnt_1] = anaSensCC_diff;
                    if (slope_thr_cnt_1 >= 5)
                    {
                        slope_thr_cnt_1 = 0;
                        which_slope = true;
                    }
                    else
                    {
                        slope_thr_cnt_1++;
                    }
                }
            }
            break;
        case 10:
            iTimer10 = 0;
            break;
        }

        // --- 停止処理 (エンコーダ処理の後に記述) ---
        if (pattern >= 11 && pattern <= 230)
        {
            /* 距離による停止処理 */
            if (lEncoderTotal >= (long)METER * (long)data_buff[TOTAL_DIST_ADDR] && !(dipsw_get() & 0x01))
            {
                pattern = 220;
            }

            /* 脱輪時の停止処理（デジタルセンサ） */
            if ((digiSensLL == OFF && digiSensCL == OFF && digiSensCC == OFF && digiSensCR == OFF && digiSensRR == OFF) ||
                (digiSensLL == ON && digiSensCL == ON && digiSensCC == ON && digiSensCR == ON && digiSensRR == ON))
            {
                check_sen_cnt++;
                if (check_sen_cnt >= 600 && !(dipsw_get() & 0x01)) // 1000
                {
                    pattern = 231;
                }
            }
            else
            {
                check_sen_cnt = 0;
            }

            /* 脱輪時の停止処理（ロータリエンコーダ） */
            if (iEncoder <= 2 && !(dipsw_get() & 0x01))
            {
                check_enc_cnt++;
                if (check_enc_cnt >= 500) // 2000
                {
                    pattern = 231;
                }
            }
            else
            {
                check_enc_cnt = 0;
            }
            /* 途中で停止処理 */
            if (pushsw_get() == ON && lEncoderTotal > 1000)
            {
                pattern = 231;
                cnt1 = 0;
            }

            if (pattern == old_pattern && pattern > 100)
            {
                safety_cnt++;

                // 600カウント（6秒: 600 × 10ms）で異常停止
                if (safety_cnt >= 600 && !(dipsw_get() & 0x01))
                {
                    pattern = 231; // エラーパターンへ遷移
                }
            }
            else
            {
                // パターンが変わった、または通常走行（pattern <= 50）の場合
                safety_cnt = 0;
            }

            // 次回の比較のために現在のパターンを保存
            old_pattern = pattern; // ← 判定の「後」に更新するのが重要！
        }

        timer_counter = 0; // カウンタリセット (次はcase 1へ)
        break;

    default:
        timer_counter = 0; // 安全策：範囲外ならリセット
        break;
    }
}

/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み                         */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0～15                                              */
/************************************************************************/
unsigned char dipsw_get(void)
{
    //	return (!digitalRead(26) << 1 | !digitalRead(25) << 0);
    return ((!R_PORT3->PIDR_b.PIDR6) << 1) | ((!R_PORT3->PIDR_b.PIDR7) << 0);
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のプッシュスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get(void)
{
    //	return !digitalRead(RUN_SWITCH);
    return !(R_PORT0->PIDR_b.PIDR3);
}

/************************************************************************/
/* 後輪の速度制御                                                       */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_r(int accele_l, int accele_r)
{
    // モータ停止モード(dipsw_get() & 0x01)
    if (!(dipsw_get() & 0x01))
    {
        motor2_r(accele_l * -1, accele_r);
    }
    else if (pattern >= 11)
    {
        motor2_r(-10, 10);
    }
}

/************************************************************************/
/* 後輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_r(int accele_l, int accele_r)
{

    if (accele_l >= Motor_Max_PWM)
    {
        accele_l = Motor_Max_PWM;
    }
    if (accele_l <= -Motor_Max_PWM)
    {
        accele_l = -Motor_Max_PWM;
    }
    if (accele_r >= Motor_Max_PWM)
    {
        accele_r = Motor_Max_PWM;
    }
    if (accele_r <= -Motor_Max_PWM)
    {
        accele_r = -Motor_Max_PWM;
    }

    motor_buff_Rl = accele_l * -1;
    motor_buff_Rr = accele_r;
    // saveData[13][logCt] = accele_l;
    // saveData[12][logCt] = accele_r;

    // 左モータ制御
    if (accele_l > 0)
    {
        //	digitalWrite(MOTOR_RL_A, HIGH);
        RL_A = HIGH;
        //	digitalWrite(MOTOR_RL_B, LOW);
        RL_B = LOW;
        MOTOR_RL_PWM = (long)(MOTOR_RL_PWM_CYCLE + 1) * accele_l / 100;
    }
    else if (accele_l < 0)
    {
        //	digitalWrite(MOTOR_RL_A, LOW);
        RL_A = LOW;
        //	digitalWrite(MOTOR_RL_B, HIGH);
        RL_B = HIGH;
        MOTOR_RL_PWM = (long)(MOTOR_RL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }

    else
    {
        //	digitalWrite(MOTOR_RL_A, LOW);
        RL_A = LOW;
        //	digitalWrite(MOTOR_RL_B, HIGH);
        RL_B = LOW;
        MOTOR_RL_PWM = (long)(MOTOR_RL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }
    // MOTOR_RL_PWM = accele_l;

    // 右モータ制御
    if (accele_r > 0)
    {
        //	digitalWrite(MOTOR_RR_A, HIGH);
        RR_A = HIGH;
        //	digitalWrite(MOTOR_RR_B, LOW);
        RR_B = LOW;
        MOTOR_RR_PWM = (long)(MOTOR_RR_PWM_CYCLE + 1) * accele_r / 100;
    }
    else if (accele_r < 0)
    {
        //	digitalWrite(MOTOR_RR_A, LOW);
        RR_A = LOW;
        //	digitalWrite(MOTOR_RR_B, HIGH);
        RR_B = HIGH;
        MOTOR_RR_PWM = (long)(MOTOR_RR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }

    else
    {
        //	digitalWrite(MOTOR_RR_A, LOW);
        RR_A = LOW;
        //	digitalWrite(MOTOR_RR_B, HIGH);
        RR_B = LOW;
        MOTOR_RR_PWM = (long)(MOTOR_RR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }
    // MOTOR_RR_PWM = accele_r;
}

/************************************************************************/
/* 前輪の速度制御                                                       */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_f(int accele_l, int accele_r)
{
    // モータ停止モード
    if (!(dipsw_get() & 0x01))
    {
        motor2_f(accele_l * -1, accele_r);
    }
    else if (pattern >= 11)
    {
        motor2_f(-10, 10);
    }
}

/************************************************************************/
/* 前輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_f(int accele_l, int accele_r)
{
    if (accele_l >= Motor_Max_PWM)
    {
        accele_l = Motor_Max_PWM;
    }

    if (accele_l <= -Motor_Max_PWM)
    {
        accele_l = -Motor_Max_PWM;
    }

    if (accele_r >= Motor_Max_PWM)
    {
        accele_r = Motor_Max_PWM;
    }

    if (accele_r <= -Motor_Max_PWM)
    {
        accele_r = -Motor_Max_PWM;
    }

    motor_buff_Fl = accele_l * -1;
    motor_buff_Fr = accele_r;

    // saveData[11][logCt] = accele_l;
    // saveData[10][logCt] = accele_r;

    // 左モータ制御
    if (accele_l > 0)
    {
        //	digitalWrite(MOTOR_FL_A, HIGH);
        FL_A = HIGH;
        //	digitalWrite(MOTOR_FL_B, LOW);
        FL_B = LOW;
        MOTOR_FL_PWM = (long)(MOTOR_FL_PWM_CYCLE + 1) * accele_l / 100;
    }
    else if (accele_l < 0)
    {
        //	digitalWrite(MOTOR_FL_A, LOW);
        FL_A = LOW;
        //	digitalWrite(MOTOR_FL_B, HIGH);
        FL_B = HIGH;
        MOTOR_FL_PWM = (long)(MOTOR_FL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }
    else
    {
        //	digitalWrite(MOTOR_FL_A, LOW);
        FL_A = LOW;
        //	digitalWrite(MOTOR_FL_B, HIGH);
        FL_B = LOW;
        MOTOR_FL_PWM = (long)(MOTOR_FL_PWM_CYCLE + 1) * (-accele_l) / 100;
    }

    // 右モータ制御
    if (accele_r > 0)
    {
        //	digitalWrite(MOTOR_FR_A, HIGH);
        FR_A = HIGH;
        //	digitalWrite(MOTOR_FR_B, LOW);
        FR_B = LOW;
        MOTOR_FR_PWM = (long)(MOTOR_FR_PWM_CYCLE + 1) * accele_r / 100;
    }
    else if (accele_r < 0)
    {
        //	digitalWrite(MOTOR_FR_A, LOW);
        FR_A = LOW;
        //	digitalWrite(MOTOR_FR_B, HIGH);
        FR_B = HIGH;
        MOTOR_FR_PWM = (long)(MOTOR_FR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }
    else
    {
        //	digitalWrite(MOTOR_FR_A, LOW);
        FR_A = LOW;
        //	digitalWrite(MOTOR_FR_B, HIGH);
        FR_B = LOW;
        MOTOR_FR_PWM = (long)(MOTOR_FR_PWM_CYCLE + 1) * (-accele_r) / 100;
    }
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 サーボモータPWM：-100～100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void servoPwmOut(int pwm)
{
    motor_buff_stare = pwm;

    // saveData[9][logCt] = pwm;
    // モータ制御
    // 限界設定
    if (abs(getServoAngle()) < 130)
    {
        if (pwm >= 0)
        {
            //	digitalWrite(MOTOR_ST_A, HIGH);
            ST_A = HIGH;
            //	digitalWrite(MOTOR_ST_B, LOW);
            ST_B = LOW;
            MOTOR_ST_PWM = (long)(MOTOR_ST_PWM_CYCLE + 1) * pwm / 100;
        }
        else
        {
            //	digitalWrite(MOTOR_ST_A, LOW);
            ST_A = LOW;
            //	digitalWrite(MOTOR_ST_B, HIGH);
            ST_B = HIGH;
            MOTOR_ST_PWM = (long)(MOTOR_ST_PWM_CYCLE + 1) * (-pwm) / 100;
        }
    }
    else
    {
        MOTOR_ST_PWM = 0;
    }
}

/************************************************************************/
/* 差分の正規化                                                          */
/* 引数　 なし                                                          */
/* 戻り値 なし                                                          */
/************************************************************************/
void Diff_Nomal(void)
{
    int i;

    // 初期値設定（負の値が来ても更新されるように少し大きめにしておく、またはあり得ない最大値にする）
    sensorMax = 500; // 初期値を極端に小さく
    sensorMin = 200; // 初期値を極端に大きく

    // 【修正1】配列を int16_t (符号あり) に変更
    // ※ anaSensLL_diff なども int16_t である必要があります
    int16_t sensDiff[5] = {
        anaSensLL_diff, anaSensCL_diff,
        anaSensCC_diff, anaSensCR_diff, anaSensRR_diff};

    if (pattern < 152)
    {
        for (int u = 0; u < 5; u++)
        {
            int16_t sensorValue = sensDiff[u];

            if (sensorValue > sensorMax)
            {
                sensorMax = sensorValue;
                if (pattern < 152)
                    old_sensorMax = sensorValue;
            }

            // 修正案：最小値を0以下にしない
            if (sensorValue < sensorMin)
            {
                // ここでガードを入れる
                if (sensorValue < 0)
                {
                    // マイナスの値が来ても、最小値としては0を採用する
                    // (あるいは、マイナス値自体を無視してsensorMinを更新しない)
                    sensorMin = 0;
                }
                else
                {
                    sensorMin = sensorValue;
                }
                if (pattern < 152)
                    old_sensorMin = sensorValue;
            }
        }
    }
    else
    {
        sensorMax = old_sensorMax;
        sensorMin = old_sensorMin;
    }

    // ゼロ除算防止（MaxとMinが同じ場合への対策）
    float divisor = (float)(sensorMax - sensorMin);
    if (divisor == 0.0f)
        divisor = 1.0f; // 安全策

    for (i = 0; i < 5; i++)
    {
        // 【修正3】数式自体はこれでOK（負 - 負 = 正しく計算される）
        // 例: 値が-50, Minが-100の場合 -> (-50 - (-100)) = +50
        float norm = (float)(sensDiff[i] - sensorMin) / divisor;

        // クランプ処理
        if (norm < 0.0)
            norm = 0.0;
        if (norm > 0.6)
            norm = 0.6;

        sensNormalized[i] = (uint16_t)(norm * 1000.0);
    }
}

// パルスからcmを計算
int Get_Distance_cm(void)
{
    return (lEncoderTotal - lEncoderBuff) * CM_PER_PULSE;
}

/************************************************************************/
/* クロスライン検出処理                                                 */
/* 引数　 なし                                                          */
/* 戻り値 0:クロスラインなし 1:あり                                     */
/************************************************************************/
int check_crossline(void)
{
    unsigned char b;
    int ret = 0;

    if (sensLLon == ON && digiSensCC == ON && sensRRon == ON && abs(getServoAngle()) < 15 && abs(Angle_D) < 5)
    // if (digiSensLL == ON && digiSensCC == ON && digiSensRR == ON)
    {
        ret = 1;
    }
    else
    {
        ret = 0;
    }

    return ret;
}

/*************************************************************************/
/* 右ハーフライン検出処理                                               */
/* 引数　 なし                                                          */
/* 戻り値 0:右ハーフラインなし 1:あり                                   */
/************************************************************************/
int check_rightline(void)
{
    unsigned char b;
    int ret = 0;
    static int lane_count = 0;

    if (digiSensCC == ON && sensRRon == ON && abs(getServoAngle()) < 15 && abs(Angle_D) < 5)
    // if (digiSensCC == ON && digiSensRR == ON)
    {
        // crank_count++;
        // lane_count++;
        // if (lane_count > 10)
        // {
        //   ret = 1;
        // }
        ret = 1;
    }
    else
    {
        lane_count = 0;
        ret = 0;
    }

    return ret;
}

/************************************************************************/
/* 左ハーフライン検出処理                                               */
/* 引数　 なし                                                          */
/* 戻り値 0:左ハーフラインなし 1:あり                                   */
/************************************************************************/
int check_leftline(void)
{
    unsigned char b;
    int ret = 0;
    static int lane_count = 0;

    if (digiSensCC == ON && sensLLon == ON && abs(getServoAngle()) < 15 && abs(Angle_D) < 5)
    // if (digiSensCC == ON && digiSensLL == ON)
    {
        // crank_count++;
        // lane_count++;
        // if (lane_count > 10)
        // {
        //   ret = 1;
        // }
        ret = 1;
    }
    else
    {
        lane_count = 0;
        ret = 0;
    }

    return ret;
}

int slopeCheck() // 坂検知
{
    int total_slope = 0;

    if (which_slope)
    {
        for (int i = 0; i < 5; i++)
        {
            total_slope = total_slope + Slope_thr[i];
        }
    }
    else
    {
        for (int i = 0; i < 5; i++)
        {
            total_slope = total_slope + Slope_thr_1[i];
        }
    }

    int average = total_slope / 5; // ← 過去5回の平均

    int diff = anaSensCC_diff - average; // ← 現在値と平均の差

    // 上り坂: 現在値が平均より大きい
    if (diff > SLOPE_UP_START && getServoAngle() <= 10)
        return 1;
    // 下り坂: 現在値が平均より小さい
    else if (diff < -SLOPE_DOWN_START)
        return -1;
    // 平坦: 変化が小さい
    else
        return 0;
}

/************************************************************************/
/* アナログセンサ値取得                                                   */
/* 引数　 なし                                                          */
/* 戻り値 センサ値                                                      */
/************************************************************************/
int getAnalogSensor(void)
{
    int ret;
    if (Trace_position == RIGHT)
        ret = (anaSensCC_diff) - (anaSensRR_diff); /* アナログセンサ情報取得    左大：＋ 　右大：-　  */
    else if (Trace_position == LEFT)
        ret = (anaSensLL_diff) - (anaSensCC_diff); /* アナログセンサ情報取得    左大：＋ 　右大：-　  */
    else
        ret = (anaSensCL_diff) - (anaSensCR_diff); /* アナログセンサ情報取得    左大：＋ 　右大：-　  */
    ;

    return ret;
}

/************************************************************************/
/* サーボモータ制御  トレース用                                            */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                                 */
/************************************************************************/
void servoControl(void)
{
    int i, iRet, iP, iD;
    int kp, kd;

    i = getAnalogSensor(); /* センサ値取得                 */
    kp = data_buff[PROP_GAIN_ADDR];
    kd = data_buff[DIFF_GAIN_ADDR];

    /* サーボモータ用PWM値計算 */
    iP = kp * i;                   // 比例
    iD = kd * (iSensorBefore - i); // 微分(目安はPの5～10倍)
    iRet = iP - iD;

    iRet /= 64; //  <<1 :/2  <<2 :/4  <<3 :/8  <<4 :/16   <<5 :/32  <<6 :/64

    if (iRet > 90)
        iRet = 90; /* マイコンカーが安定したら     */
    if (iRet < -90)
        iRet = -90; /* 上限を70くらいにしてください */

    iServoPwm = -iRet;
    iSensorBefore = i; /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/* サーボモータ2制御  角度制御用                                           */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                               */
/************************************************************************/
void servoControl2(void)
{

    signed int i, j, iRet, iP, iD;
    signed int kp, kd;

    i = iSetAngle;
    j = getServoAngle();

    /* サーボモータ用PWM値計算 */
    iP = 5 * (j - i);              // 比例 10
    iD = 20 * (iAngleBefore2 - j); // 微分(目安はPの5～10倍) 60
    iRet = iP - iD;
    iRet /= 4;

    /* PWMの上限の設定 */
    if (iRet > 90)
        iRet = 90; /* マイコンカーが安定したら     */
    if (iRet < -90)
        iRet = -90; /* 上限を70くらいにしてください */

    iServoPwm2 = iRet;
    iAngleBefore2 = j; /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/* DataFlashのパラメータ読み込み                                        */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void readDataFlashParameter(void)
{
    // データ読み取り
    for (uint16_t i = 0; i < MAX_NUM_ADDR; i++)
    {
        data_buff[i] = EEPROM.read(i);
    }

    // ヘッダー確認
    if (data_buff[HEADER_ADDR] != FLASH_HEADER)
    {
        // デフォルト値
        data_buff[TOTAL_DIST_ADDR] = 15;
        data_buff[START_TIME_ADDR] = 5;
        data_buff[PROP_GAIN_ADDR] = 4;
        data_buff[DIFF_GAIN_ADDR] = 10;
        data_buff[TRG_SPEED_ADDR] = 50;
        data_buff[CORNER_SPEED_ADDR] = 46;
        data_buff[CRANK_SPEED_ADDR] = 27;
        data_buff[LANE_SPEED_ADDR] = 44;
        data_buff[SLOPE_SPEED_ADDR] = 34;
        data_buff[ACCEL_SPEED_ADDR] = 100;
        // data_buff[LANE_ANGLE_L_ADDR]	= 35;
        // data_buff[LANE_ANGLE_R_ADDR]	= 35;
        // data_buff[SENS_L_THOLD1_ADDR]	= (450 >> 8) & 0xFF;
        // data_buff[SENS_L_THOLD2_ADDR]	= (450     ) & 0xFF;
        // data_buff[SENS_R_THOLD1_ADDR]	= (450 >> 8) & 0xFF;
        // data_buff[SENS_R_THOLD2_ADDR]	= (450     ) & 0xFF;

        // パラメーター設定
        writeDataFlashParameter();
    }
}

/************************************************************************/
/* DataFlashへパラメータ書き込み                                        */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void writeDataFlashParameter(void)
{
    // 各値書き込み
    EEPROM.put(HEADER_ADDR, FLASH_HEADER);
    EEPROM.put(TOTAL_DIST_ADDR, data_buff[TOTAL_DIST_ADDR]);
    EEPROM.put(START_TIME_ADDR, data_buff[START_TIME_ADDR]);
    EEPROM.put(PROP_GAIN_ADDR, data_buff[PROP_GAIN_ADDR]);
    EEPROM.put(DIFF_GAIN_ADDR, data_buff[DIFF_GAIN_ADDR]);
    EEPROM.put(TRG_SPEED_ADDR, data_buff[TRG_SPEED_ADDR]);
    EEPROM.put(CORNER_SPEED_ADDR, data_buff[CORNER_SPEED_ADDR]);
    EEPROM.put(CRANK_SPEED_ADDR, data_buff[CRANK_SPEED_ADDR]);
    EEPROM.put(LANE_SPEED_ADDR, data_buff[LANE_SPEED_ADDR]);
    EEPROM.put(SLOPE_SPEED_ADDR, data_buff[SLOPE_SPEED_ADDR]);
    EEPROM.put(ACCEL_SPEED_ADDR, data_buff[ACCEL_SPEED_ADDR]);
}

/************************************************************************/
/* LCDとスイッチを使ったパラメータセット処理                            */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
int lcdProcess(void)
{
    int i;
    char sw = 0; // LCDスイッチ情報用

    // printf("lcd_pattern=%d",lcd_pattern );
    printf(" pattern=%d\n", pattern);

    // スイッチ情報取得
    sw = SwitchGetState();
    //    printf("    sw=%d", sw);

    // メニュー＋１
    if (sw == MENU_UP)
    {
        lcd_pattern++;
        delay(200);

        if (lcd_pattern == 12)
            lcd_pattern = 1;
    }

    // メニュー－１
    if (sw == MENU_DOWN)
    {
        lcd_pattern--;
        delay(200);

        if (lcd_pattern == 0)
            lcd_pattern = 11;
    }

    /* LCD、スイッチ処理 */
    switch (lcd_pattern)
    {
    case 1:
        /* 走行停止距離調整 */
        servoPwmOut(0);
        i = data_buff[TOTAL_DIST_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[TOTAL_DIST_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("01 Stop L[m]=%03d", i);
        LcdPosition(0, 1);
        LcdPrintf("Encoder = %03d   ", lEncoderTotal);
        break;

    case 2:
        /* スタート待ち時間調整 */
        servoPwmOut(0);

        i = data_buff[START_TIME_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[START_TIME_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("02 St time = %03d", i);
        LcdPosition(0, 1);
        // LcdPrintf("L=%1d C=%1d R=%1d %1d   ",
        //           digiSensLL, digiSensCC, digiSensRR
        LcdPrintf("LL=%4d  RR=%4d   ",
                  thrSensLL, thrSensRR
                  /*,
                  SENS_ALL*/
        );
        cnt1 = 0;
        break;

    case 3:
        /* トレース比例制御調整 */
        // servoPwmOut(iServoPwm);
        servoPwmOut(0);

        i = data_buff[PROP_GAIN_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[PROP_GAIN_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("03 Trace kp =%03d", i);
        LcdPosition(0, 1);
        LcdPrintf("L=%4d  R=%4d   ",
                  anaSensCL_diff, anaSensCR_diff);
        // LcdPrintf("L=%4d  R=%4d   ",
        //           ANA_SENS_CR, ANA_SENS_CL);
        // lcdPrintf("cnt1=%4d" , cnt1);

        if (pushsw_get())
        {
            //  パラメータ保存
            writeDataFlashParameter();
            LcdPosition(0, 0);
            LcdPrintf("para set           ");
            LcdPosition(0, 1);
            LcdPrintf("OK!                      ");
            while (1)
            {
                iServo_flag = TRACE;
            }
        }
        break;

    case 4:
        /* トレース微分制御調整 */
        // servoPwmOut(-iServoPwm);
        servoPwmOut(0);
        i = data_buff[DIFF_GAIN_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[DIFF_GAIN_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("04 Trace kd =%03d", i);
        LcdPosition(0, 1);
        // LcdPrintf("courceOut=%2d       ", cource);
        break;

    case 5:
        /* 直線走行目標速度設定 */
        servoPwmOut(0);

        i = data_buff[TRG_SPEED_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 127)
                i = 127;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[TRG_SPEED_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("05 Speed_S = %3d", i);
        LcdPosition(0, 1);
        LcdPrintf("Bar Angle = %4d", BAR_ANGLE);
        break;

    case 6:
        /* カーブ走行目標速度設定 */
        servoPwmOut(0);

        i = data_buff[CORNER_SPEED_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[CORNER_SPEED_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("06 Speed_C = %3d", i);
        LcdPrintf("06 Speed_C = %3d", iEncoder);

        LcdPrintf("06                    ");

        // LcdPosition(0, 1);
        // LcdPrintf("Slope Angle=%4d", SLOPE_ANGLE);
        break;

    case 7:
        /* クランク進入目標速度設定 */
        servoPwmOut(0);

        i = data_buff[CRANK_SPEED_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[CRANK_SPEED_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("07 Speed_CL =%3d", i);
        LcdPosition(0, 1);
        LcdPrintf("");
        break;

    case 8:
        /* レーンチェンジ進入目標速度設定 */
        servoPwmOut(0);

        i = data_buff[LANE_SPEED_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[LANE_SPEED_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("08 Speed_RC =%3d", i);
        LcdPosition(0, 1);
        LcdPrintf("0x%X", dipsw_get());
        break;

    case 9:
        /* 坂進入目標速度設定 */
        servoPwmOut(0);

        i = data_buff[SLOPE_SPEED_ADDR];
        if (sw == DATA_UP)
        {
            i++;
            if (i > 100)
                i = 100;
        }
        if (sw == DATA_DOWN)
        {
            i--;
            if (i < 0)
                i = 0;
        }
        data_buff[SLOPE_SPEED_ADDR] = i;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("08 Speed_SL =%3d", i);
        LcdPosition(0, 1);
        LcdPrintf("0x%X", dipsw_get());
        break;

    case 10:
        /* 設定パラメーター保存 */
        servoPwmOut(0);

        LcdPosition(0, 0);
        LcdPrintf("09 Parameter Set");

        // 設定値保存
        if (pushsw_get())
        {
            cnt1 = 0;
            do
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(500);
                if (cnt1 > 2000)
                {
                    // パラメータ保存
                    writeDataFlashParameter();
                    LcdPosition(0, 1);
                    LcdPrintf("Set Complete    ");
                }
                else
                {
                    LcdPosition(0, 1);
                    LcdPrintf("Setting Now     ");
                }
            } while (pushsw_get());
        }
        else
        {
            LcdPosition(0, 1);
            LcdPrintf("                ");
        }
        break;

    case 11:
        /* モーターテストドライバ基板確認 */
        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("10 Motor_Test   ");
        LcdPosition(0, 1);
        LcdPrintf("SW_1/2 ON!      ");

        if (pushsw_get() == 1 && sw == DATA_UP || pushsw_get() == 1 && DATA_DOWN)
        {
            do
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(200);
                LcdPosition(0, 1);
                LcdPrintf("SW_1/2 OFF!     ");
            } while (pushsw_get() == 1 && sw == DATA_UP || pushsw_get() == 1 && sw == DATA_DOWN);
            delay(15);
            cnt1 = 0;
            while (sw == 0)
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(200);

                servoPwmOut(0);

                // 基板テスト
                motor2_f(100, 100); // 前 （左,右）
                motor2_r(100, 100); // 後（左,右）

                servoPwmOut(100);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test CW   ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", 100);
                delay(2000);

                motor2_f(0, 0); // 前 （左,右）
                motor2_r(0, 0); // 後（左,右）
                servoPwmOut(0);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test STOP ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", 0);
                delay(1000);

                motor2_f(-100, -100); // 前 （左,右）
                motor2_r(-100, -100); // 後（左,右）
                servoPwmOut(-100);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test CCW   ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", -100);
                delay(2000);

                motor2_f(0, 0); // 前 （左,右）
                motor2_r(0, 0); // 後（左,右）
                servoPwmOut(0);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test STOP ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", 0);
                delay(1000);
                if (cnt1 > 3000)
                    break;
            }
            motor_f(0, 0); // 前 （左,右）
            motor_r(0, 0); // 後（左,右）
            do
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(200);
            } while (sw == 0x01 || sw == 0x02);
        }
        break;

    case 12:
        /* モーターテストドライバ基板確認 */

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("10 Motor_Test   ");
        LcdPosition(0, 1);
        LcdPrintf("SW_1 ON!        ");

        if (sw == 0x01 || sw == 0x02)
        {
            do
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(600);
                LcdPosition(0, 1);
                LcdPrintf("SW_1 OFF!       ");
            } while (sw == 0x01 || sw == 0x02);
            delay(15);
            i = 4;
            while (sw == 0 || i < 8)
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(1000);

                servoPwmOut(0);

                // 基板テスト
                if (i < 10)
                    i++;
                else
                    i = 4;
                motor2_f(i * 10, i * 10); // 前 （左,右）
                motor2_r(i * 10, i * 10); // 後（左,右）
                servoPwmOut(i * 10);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test CW   ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", i * 10);
                delay(2000);

                motor2_f(0, 0); // 前 （左,右）
                motor2_r(0, 0); // 後 （左,右）
                servoPwmOut(0);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test STOP ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", 0);
                delay(1000);

                motor2_f(-i * 10, -i * 10); // 前 （左,右）
                motor2_r(-i * 10, -i * 10); // 後（左,右）
                servoPwmOut(-i * 10);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test CCW   ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", -i * 10);
                delay(2000);

                motor2_f(0, 0); // 前 （左,右）
                motor2_r(0, 0); // 後 （左,右）
                servoPwmOut(0);

                LcdPosition(0, 0);
                LcdPrintf("Motor Test STOP ");
                LcdPosition(0, 1);
                LcdPrintf("Power = %4d%%   ", 0);
                delay(1000);
            }
            motor_f(0, 0); // 前 （左,右）
            motor_r(0, 0); // 後 （左,右）
            do
            {
                // スイッチ情報取得
                sw = SwitchGetState();
                delay(200);
            } while (sw == 0x01 || sw == 0x02);
        }
        break;
    }
}

void mtTest()
{
    // 100% CW
    motor2_f(100, 100);
    motor2_r(100, 100);
    servoPwmOut(100);

    LcdPosition(0, 0);
    LcdPrintf("Motor Test CW   ");
    LcdPosition(0, 1);
    LcdPrintf("Power = %4d%%   ", 100);
    delay(2000);

    // 100% STOP
    motor2_f(0, 0);
    motor2_r(0, 0);
    servoPwmOut(0);

    LcdPosition(0, 0);
    LcdPrintf("Motor Test STOP  ");
    LcdPosition(0, 1);
    LcdPrintf("Power = %4d%%   ", -100);
    delay(2000);

    // 100% CCW
    motor2_f(-100, -100);
    motor2_r(-100, -100);
    servoPwmOut(-100);

    LcdPosition(0, 0);
    LcdPrintf("Motor Test CCW   ");
    LcdPosition(0, 1);
    LcdPrintf("Power = %4d%%   ", -100);
    delay(2000);

    // 100% STOP
    motor2_f(0, 0);
    motor2_r(0, 0);
    servoPwmOut(0);

    LcdPosition(0, 0);
    LcdPrintf("Motor Test STOP   ");
    LcdPosition(0, 1);
    LcdPrintf("Power = %4d%%   ", -100);
    delay(2000);
}

// int angleStreatCheck(int i, int jide_angle)
// {
//     static int pattern11count = 0; /*11カウント用			 		 */
//     if (abs(i) < jide_angle)
//     {
//         pattern11count++;
//         if (pattern11count > 55)
//         {
//             pattern11count = 0;
//             return 1;
//         }
//     }
//     else
//     {
//         pattern11count = 0;
//     }
//     return 0;
// }

/**********************************************************************/
/**
 *	入力センサの初期化処理.
 */
void initSens(void)
{
    /* アナログセンサ */
    // ad.useCh(SENS_A_UL); // CN8  9 D68
    ad.useCh(SENS_A_LL); // CN8  8 D67
    ad.useCh(SENS_A_CL); // CN8  6 D65
    ad.useCh(SENS_A_CC); // CN8  5 D64
    ad.useCh(SENS_A_CR); // CN8  4 D63
    ad.useCh(SENS_A_RR); // CN8  3 D62
    // ad.useCh(SENS_A_UR); // CN8  2 D61
    ad.useCh(SENS_A_VR); // CN8  7 D66
    ad.start();
}

/**********************************************************************/
/**
 *	モータ関係 初期化.
 */
void initMotor(void)
{
    /* RRモータ */
    // defineで定義
    MOTOR_INIT_RR;

    /* RLモータ */
    // defineで定義
    MOTOR_INIT_RL;

    /* FRモータ */
    // defineで定義
    MOTOR_INIT_FR;

    /* FLモータ */
    // defineで定義
    MOTOR_INIT_FL;

    /* STモータ */
    // defineで定義
    MOTOR_INIT_ST;
}

/************************************************************************/
/**
 * SDカードファイルオープン.
 */
void SD_file_open(void)
{
    // 通信速度Hz(e6=10の6乗),microSDのCS1(pins_arduino.hで定義)
    if (!SD.begin(2.4e6, CS1))
    {
        // Serial.println("SD初期化に失敗しました");
        return;
    }

    // ログディレクトリの作成
    const char *logDir = "LOG";
    if (!SD.exists(logDir))
    {
        SD.mkdir(logDir);
    }

    int i = 0;
    char renbanPath[32];
    sprintf(renbanPath, "%s/renban.txt", logDir);

    // 連番ファイルの読み取り
    microSD = SD.open(renbanPath, FILE_READ);
    if (microSD)
    {
        int length = microSD.available();
        if (length > 8)
            length = 8;
        microSD.read(filename, length);
        sscanf(filename, "%d", &i);
        if (i < 0 || i >= 99999)
            i = 0;
        microSD.close();
    }

    // 既存の連番ファイルを削除
    if (SD.exists(renbanPath))
        SD.remove(renbanPath);

    // 新しい連番をファイルに書き込み
    microSD = SD.open(renbanPath, FILE_WRITE);
    if (microSD)
    {
        sprintf(filename, "%d", i + 1);
        microSD.println(filename);
        microSD.close();
    }

    // ログファイルのパスを作成（ディレクトリ内）
    // sprintf(filename, "LogData/log%05d.csv", i + 1);
    // sprintf(filename, "LogData/log%05d.csv", i + 1);
    sprintf(filename, "%s/log%05d.csv", logDir, i + 1);
    microSD = SD.open(filename, FILE_WRITE);

    //  //通信速度Hz(e6=10の6乗),microSDのCS1(pins_arduino.hで定義)
    // 	if (SD.begin(2.4e6, CS1))
    // 	{
    //     //microSDへファイルオープン
    // 		microSD = SD.open("log.csv", FILE_WRITE);
    // 	}
}

/************************************************************************/
/**
 * ログをRAMに保存.
 */
void LOG_rec(void)
{
    // if (useBufferA)
    // {
    saveDataA[0][logCt] = digiSensLL << 2 | digiSensCC << 1 | digiSensRR;
    saveDataA[1][logCt] = iEncoder;
    saveDataA[2][logCt] = pattern;
    saveDataA[3][logCt] = lEncoderTotal - lEncoderBuff;
    saveDataA[4][logCt] = getServoAngle();
    saveDataA[5][logCt] = iSetAngle;        // iSetAngle
    saveDataA[6][logCt] = anaSensCL_diff;   // sensNormalized[sLL]
    saveDataA[7][logCt] = anaSensCC_diff;   // sensNormalized[sCC]
    saveDataA[8][logCt] = anaSensCR_diff;   // sensNormalized[sRR]
    saveDataA[9][logCt] = motor_buff_stare; //: PWMステアリング;motor_buff_stare
    saveDataA[10][logCt] = motor_buff_Rl;   //: PWM後左;
    saveDataA[11][logCt] = motor_buff_Fl;   //: PWM前左;
    saveDataA[12][logCt] = motor_buff_Fr;   //: PWM前右;
    saveDataA[13][logCt] = motor_buff_Rr;   //: PWM後右;
    saveDataA[14][logCt] = slopeCheck();
    saveDistA[logCt] = lEncoderTotal; // 再生走行用合計パルス数

    logCt++;

    if (logCt >= LOG_BUFF_SIZE)
    {
        logCt = 0;
        // isWriting = true;
    }

    // 一定数溜まったら書き込み
    // if (logCt >= LOG_BUFF_SIZE && !isWriting) {
    //   isWriting = true;
    //   logCt = 0;
    //   useBufferA = !useBufferA; // バッファ切り替え
    // }
}

/**********************************************************************/
/**
 *	ログ書き出し.
 */
void writeLog(void)
{
    sprintf(str, "d_0=%03d d_1=%03d d_2=%03d d_3=%03d d_4=%03d d_5=%03d "
                 "d_6=%03d d_7=%03d d_8=%03d d_9=%03d d_10=%03d d_11=%03d "
                 "d_12=%03d d_13=%03d d_14=%03d d_15=%03d fin \n\r",
            saveDataA[0][logRd],
            saveDataA[1][logRd],
            saveDataA[2][logRd],
            saveDataA[3][logRd],
            saveDataA[4][logRd],
            saveDataA[5][logRd],
            saveDataA[6][logRd],
            saveDataA[7][logRd],
            saveDataA[8][logRd],
            saveDataA[9][logRd],
            saveDataA[10][logRd],
            saveDataA[11][logRd],
            saveDataA[12][logRd],
            saveDataA[13][logRd],
            saveDistA[logRd], // ⬅ saveDataA[14][logRd] から変更
            saveDataA[14][logRd]);
    microSD.print(str);
    // microSD.flush();
    logRd++;
    if (logRd >= LOG_BUFF_SIZE)
    {
        logRd = 0;
    }
}

/************************************************************************/
/**
 * SDカードファイルクローズ.
 */
void SD_file_close(void)
{
    microSD.close();
    iServo_flag = STOP;
    CPU_LED_2 = OFF;
    CPU_LED_3 = OFF;
}

/***********************************************************************/
/**
 * ログデータ解析，再生走行用データ作成
 */
void Log_Analysis(void)
{
    // 通信速度Hz(e6=10の6乗),microSDのCS1(pins_arduino.hで定義)
    if (!SD.begin(2.4e6, CS1))
    {
        // Serial.println("SD初期化に失敗しました");
        return;
    }

    // 再生データディレクトリの作成
    const char *repDataDir = "REP";
    if (!SD.exists(repDataDir))
    {
        SD.mkdir(repDataDir);
    }

    // ログディレクトリから最新のログファイルを探す
    const char *logDir = "LOG";

    int latestLogNum = 0;
    char renbanPath[32];
    sprintf(renbanPath, "%s/renban.txt", logDir);

    // 実際に存在する最新のログファイルを探す
    // まず連番ファイルから大まかな番号を取得
    microSD = SD.open(renbanPath, FILE_READ);
    if (microSD)
    {
        int length = microSD.available();
        if (length > 8)
            length = 8;
        char buffer[16];
        microSD.read(buffer, length);
        sscanf(buffer, "%d", &latestLogNum);
        microSD.close();

        if (latestLogNum < 1)
            latestLogNum = 1;
    }

    // 実際に存在する最新のファイルを逆順で探す
    bool fileFound = false;
    for (int searchNum = latestLogNum + 10; searchNum >= 1; searchNum--) // 余裕を持って+10から検索
    {
        char testPath[64];
        sprintf(testPath, "%s/log%05d.csv", logDir, searchNum);
        if (SD.exists(testPath))
        {
            latestLogNum = searchNum;
            fileFound = true;
            break;
        }
    }

    if (!fileFound)
    {
        // ファイルが見つからない場合は1から順番に探す
        for (int searchNum = 1; searchNum <= 99999; searchNum++)
        {
            char testPath[64];
            sprintf(testPath, "%s/log%05d.csv", logDir, searchNum);
            if (SD.exists(testPath))
            {
                latestLogNum = searchNum;
            }
            else if (latestLogNum > 0)
            {
                // 連続した番号が途切れたら、それが最新
                break;
            }
        }
    }

    // デバッグ出力
    char debugLine[64];
    sprintf(debugLine, "Latest log number: %d\n", latestLogNum);
    Serial.print(debugLine);

    // 最新のログファイルを開く
    char latestLogPath[64];
    sprintf(latestLogPath, "%s/log%05d.csv", logDir, latestLogNum);

    // デバッグ出力
    char debugLine2[128];
    sprintf(debugLine2, "Opening log file: %s\n", latestLogPath);
    Serial.print(debugLine2);

    microSD = SD.open(latestLogPath, FILE_READ);
    if (!microSD)
    {
        // Serial.println("最新のログファイルが見つかりません");
        return;
    }

    // ここから解析処理を記述
    // microSD変数で最新のログファイルが読み取り可能状態で開かれています

    char line[256];
    char lineCopy[256]; // 文字列コピー用のバッファを追加
    char *token;
    int lineCount = 0;
    uint16_t buff_pattern = 0;
    int16_t buff_angle = 0;
    uint16_t buff_distance = 0;

    // ストレート区間の解析用変数
    bool inStraight = false;
    uint16_t straightStartDistance = 0;

    // 既存ファイルを削除してから新規作成（上書き）
    char outputPath[64];
    sprintf(outputPath, "%s/Log_Rep.csv", repDataDir);

    // 既存ファイルがあれば削除
    if (SD.exists(outputPath))
    {
        SD.remove(outputPath);
    }

    // 新規作成
    File outputFile = SD.open(outputPath, FILE_WRITE);

    if (!outputFile)
    {
        // Serial.println("出力ファイルの作成に失敗しました");
        microSD.close();
        return;
    }

    // CSVヘッダーを書き込み
    char headerLine[64];
    sprintf(headerLine, "start_distance,end_distance\n");
    outputFile.print(headerLine);
    outputFile.flush(); // ヘッダー書き込み後にフラッシュ

    // CSVファイルを1行ずつ読み込み
    while (microSD.available())
    {
        // 1行読み込み（改行文字処理を改善）
        int i = 0;
        while (microSD.available() && i < sizeof(line) - 1)
        {
            char c = microSD.read();
            if (c == '\n')
                break;
            if (c != '\r') // CR文字は無視
                line[i++] = c;
        }
        line[i] = '\0';

        if (strlen(line) == 0)
            continue;

        lineCount++;

        // デバッグ出力（コメントアウト解除で確認可能）
        char debugLine3[128];
        sprintf(debugLine3, "Line %d: %s\n", lineCount, line);
        Serial.print(debugLine3);

        // ヘッダー行をスキップ（1行目）
        if (lineCount == 1)
        {
            // ヘッダー行の内容も確認
            Serial.print("CSV Header: ");
            Serial.println(line);
            continue;
        }

        // 変数を初期化
        buff_pattern = 0;
        buff_angle = 0;
        buff_distance = 0;

        // lineの内容をlineCopyにコピー（strtokは元の文字列を変更するため）
        strcpy(lineCopy, line);

        // データ解析（スペース区切り）- より詳細なデバッグ付き
        token = strtok(lineCopy, " ");
        int columnIndex = 0;

        Serial.print("Parsing columns: ");
        while (token != NULL)
        {
            // 各列の内容を確認
            char columnDebug[64];
            sprintf(columnDebug, "Col[%d]='%s' ", columnIndex, token);
            Serial.print(columnDebug);

            // d_X=YYY形式から値を抽出
            if (strncmp(token, "d_", 2) == 0)
            {
                char *equalPos = strchr(token, '=');
                if (equalPos != NULL)
                {
                    int dataIndex = atoi(token + 2);    // d_の後の数字
                    int dataValue = atoi(equalPos + 1); // =の後の数字

                    switch (dataIndex)
                    {
                    case 2: // d_2 = pattern列
                        buff_pattern = (uint16_t)dataValue;
                        Serial.print("(pattern) ");
                        break;
                    case 4: // d_4 = angle列 (整数値なので10で割る必要があるかもしれません)
                        buff_angle = (int16_t)dataValue;
                        Serial.print("(angle) ");
                        break;
                    case 14: // d_14 = distance列
                        buff_distance = (uint16_t)dataValue;
                        Serial.print("(distance) ");
                        break;
                    }
                }
            }
            token = strtok(NULL, " ");
            columnIndex++;
        }
        Serial.println();

        // 解析結果をデバッグ出力
        char debugLine4[128];
        sprintf(debugLine4, "Parsed -> Pattern: %u, Angle: %.2f, Distance: %u\n", buff_pattern, buff_angle, buff_distance);
        Serial.print(debugLine4);

        // データが正しく取得できているかチェック
        if (buff_pattern == 0 && buff_angle == 0 && buff_distance == 0)
        {
            Serial.println("WARNING: All values are zero - check CSV format and column indices");
        }

        // pattern==11（通常トレース）でアングルがまっすぐの場合のストレート区間解析
        if ((buff_pattern == 11 || buff_pattern == 50) && buff_angle >= -ACCEL_ANGLE && buff_angle <= ACCEL_ANGLE)
        {
            if (!inStraight)
            {
                // ストレート区間開始
                inStraight = true;
                straightStartDistance = buff_distance;
                char debugLine5[64];
                sprintf(debugLine5, "Straight start at: %u\n", straightStartDistance);
                Serial.print(debugLine5);
            }
        }
        else
        {
            if (inStraight)
            {
                // ストレート区間終了 - 加速区間としてCSVに出力
                inStraight = false;
                char debugLine6[128];
                sprintf(debugLine6, "Straight end at: %u\nAcceleration section: %u,%u\n", buff_distance, straightStartDistance, buff_distance);
                Serial.print(debugLine6);

                char outputLine[64];
                sprintf(outputLine, "%u,%u\n", straightStartDistance, buff_distance);
                outputFile.print(outputLine);
                outputFile.flush(); // バッファを強制的に書き込み
            }
        }
    }

    // 最後にストレート区間が終了していない場合の処理
    if (inStraight)
    {
        char debugLine7[64];
        sprintf(debugLine7, "Final straight end at: %u\n", buff_distance);
        Serial.print(debugLine7);
        char outputLine[64];
        sprintf(outputLine, "%u,%u\n", straightStartDistance, buff_distance);
        outputFile.print(outputLine);
        outputFile.flush(); // バッファを強制的に書き込み
    }

    // 解析完了後、ファイルを閉じる
    outputFile.close(); // 出力ファイルを確実にクローズ
    microSD.close();    // 入力ファイルも確実にクローズ

    CPU_LED_2 = OFF;
    CPU_LED_3 = OFF;

    Serial.println("Log analysis completed");
}

void Open_Rep(void)
{
    // 通信速度Hz(e6=10の6乗),microSDのCS1(pins_arduino.hで定義)
    if (!SD.begin(2.4e6, CS1))
    {
        Serial2.println("SD初期化に失敗しました");
        return;
    }

    // repファイルのパス
    const char *repDataDir = "REP";
    char repFilePath[64];
    sprintf(repFilePath, "%s/Log_Rep.csv", repDataDir);

    // デバッグ出力
    char debugLine[128];
    sprintf(debugLine, "Opening rep file: %s\n", repFilePath);
    Serial2.print(debugLine);

    // repファイルの存在確認
    if (!SD.exists(repFilePath))
    {
        Serial2.println("Repファイルが見つかりません");
        straight_section_count = 0;
        return;
    }

    // ファイルを開く
    File repFile = SD.open(repFilePath, FILE_READ);
    if (!repFile)
    {
        Serial2.println("Repファイルのオープンに失敗しました");
        straight_section_count = 0;
        return;
    }

    // 配列を初期化
    straight_section_count = 0;
    memset(straight_sections, 0, sizeof(straight_sections));

    char line[64];
    int lineCount = 0;

    // CSVファイルを1行ずつ読み込み
    while (repFile.available() && straight_section_count < MAX_STRAIGHT_SECTIONS)
    {
        // 1行読み込み
        int i = 0;
        while (repFile.available() && i < sizeof(line) - 1)
        {
            char c = repFile.read();
            if (c == '\n')
                break;
            if (c != '\r') // CR文字は無視
                line[i++] = c;
        }
        line[i] = '\0';

        if (strlen(line) == 0)
            continue;

        lineCount++;

        // ヘッダー行をスキップ（1行目：start_distance,end_distance）
        if (lineCount == 1)
        {
            Serial2.print("CSV Header: ");
            Serial2.println(line);
            continue;
        }

        // データ行の解析（カンマ区切り）
        char *token;
        char lineCopy[64];
        strcpy(lineCopy, line);

        // start_distanceを取得
        token = strtok(lineCopy, ",");
        if (token != NULL)
        {
            straight_sections[straight_section_count].start_distance = (uint16_t)atoi(token);

            // end_distanceを取得
            token = strtok(NULL, ",");
            if (token != NULL)
            {
                straight_sections[straight_section_count].end_distance = (uint16_t)atoi(token);

                // デバッグ出力
                char debugLine2[128];
                sprintf(debugLine2, "Loaded section %d: start=%u, end=%u\n",
                        straight_section_count,
                        straight_sections[straight_section_count].start_distance,
                        straight_sections[straight_section_count].end_distance);
                Serial2.print(debugLine2);

                straight_section_count++;
            }
        }
    }

    // ファイルを閉じる
    repFile.close();

    // // 読み込み結果のデバッグ出力
    // char debugLine3[128];
    // sprintf(debugLine3, "Rep file loaded successfully. Total sections: %d\n", straight_section_count);
    // Serial2.print(debugLine3);

    // // 全セクションの一覧表示（デバッグ用）
    // Serial2.println("=== Loaded Straight Sections ===");
    // for (int i = 0; i < straight_section_count; i++)
    // {
    //     char sectionInfo[64];
    //     sprintf(sectionInfo, "Section %d: %u - %u (length: %u)\n",
    //            i,
    //            straight_sections[i].start_distance,
    //            straight_sections[i].end_distance,
    //            straight_sections[i].end_distance - straight_sections[i].start_distance);
    //     Serial2.print(sectionInfo);
    // }
    // Serial2.println("=== End of Sections ===");
}

/**
 * 直線区間判定関数
 *
 * 現在位置が直線区間内にあるかをチェックし、
 * 物理計算に基づいてACCEL/BRAKE/OFFを返す
 *
 * @param current_dist_pulse 現在の累積走行距離 [パルス]
 * @return ACCEL(加速可能) / BRAKE(減速必要) / OFF(区間外)
 */
int Check_StraightSection(int32_t current_dist_pulse)
{
    // ========== パラメータ設定 ==========
    const float DECEL_ACCEL = 5.0f;         // 減速加速度 [m/s²]    5
    const float SAFETY_TIME_MARGIN = 0.10f; // 安全時間マージン [秒]

    // ========== 直線区間のチェック ==========
    for (int i = 0; i < straight_section_count; i++)
    {
        // 現在位置が区間内かチェック
        if (current_dist_pulse >= straight_sections[i].start_distance &&
            current_dist_pulse <= straight_sections[i].end_distance)
        {
            // --- 1. 残り距離の計算 [パルス] ---
            int32_t remaining_pulse = straight_sections[i].end_distance - current_dist_pulse;

            // --- 2. 現在速度の計算 [m/s] ---
            // 修正点: 事前計算済みの定数PULSE_TO_MSを使用
            // 旧: iEncoder * ((TIRE_DIAMETER * PI) / (CONTROL_PERIOD * ENC_PULSE_REV))
            // 新: iEncoder * PULSE_TO_MS
            float v_now = iEncoder * PULSE_TO_MS;

            // --- 3. 次のコーナー目標速度 [m/s] ---
            // 修正点: DATA_TO_MSマクロで明示的に変換
            // 旧: data_buff[CORNER_SPEED_ADDR] / 10
            // 新: DATA_TO_MS(data_buff[CORNER_SPEED_ADDR])
            float v_next = DATA_TO_MS(data_buff[CORNER_SPEED_ADDR]);

            // --- 4. 早期リターン: すでに十分遅い場合 ---
            // 修正点: v_now <= v_next の場合は即座にACCEL返却
            // 旧: brake_dist_m = 0.0f のまま計算継続（無駄）
            // 新: 早期リターンで効率化
            if (v_now <= v_next)
            {
                return ACCEL; // すでに目標速度以下なので加速可能
            }

            // --- 5. 必要なブレーキ距離の計算 [m] ---
            // 等加速度運動の公式: v² - v₀² = 2as
            // → s = (v² - v₀²) / (2a)
            // ここでは減速なので: s = (v_now² - v_next²) / (2 × DECEL_ACCEL)
            float brake_dist_m = (v_now * v_now - v_next * v_next) / (2.0f * DECEL_ACCEL);

            // --- 6. 安全マージンの追加 ---
            // 修正点: 固定0.1mから動的マージンに変更
            // 旧: brake_dist_m += 0.1f; （速度に関係なく常に10cm）
            // 新: brake_dist_m += v_now * SAFETY_TIME_MARGIN; （速度に比例）
            //
            // 例: v_now = 1.0m/s → マージン = 0.15m
            //     v_now = 2.0m/s → マージン = 0.30m
            float safety_margin_m = v_now * SAFETY_TIME_MARGIN;
            brake_dist_m += safety_margin_m;

            // --- 7. ブレーキ距離をパルス数に変換 ---
            int32_t brake_dist_pulse = (int32_t)(brake_dist_m / M_PER_PULSE);

            // デバッグ用バッファ（オプション）
            nomal_buff = v_now;

            // --- 8. 判定 ---
            if (remaining_pulse <= brake_dist_pulse)
            {
                // 残り距離 ≤ ブレーキ距離 → 今すぐ減速開始
                return BRAKE;
            }
            else
            {
                // 残り距離 > ブレーキ距離 → まだ加速可能
                return ACCEL;
            }
        }
    }

    // どの直線区間にも該当しない
    return OFF;
}

/************************************************************************/
/* サーボ角度取得                                                       */
/* 引数　 なし                                                          */
/* 戻り値 サーボ角度                                                */
/************************************************************************/
int getServoAngle(void)
{
    // タイマ割り込み内で安全に計算された角度を返すだけ
    return g_current_servo_angle;
}
/************************************************************************/
/**
 * アングルフィルタ.
 */
short Dig_M(short angle)
{
    static short ang_buf[3];
    unsigned short MIN, MID, MAX, tmp;

    ang_buf[2] = ang_buf[1];
    ang_buf[1] = ang_buf[0];
    ang_buf[0] = angle;

    MIN = ang_buf[0];
    MID = ang_buf[1];
    MAX = ang_buf[2];

    if (MAX < MID)
    {
        tmp = MAX;
        MAX = MID;
        MID = tmp;
    }
    if (MAX < MIN)
    {
        tmp = MAX;
        MAX = MIN;
        MIN = tmp;
    }
    if (MID < MIN)
    {
        tmp = MIN;
        MIN = MID;
        MID = tmp;
    }

    return MID;
}

/************************************************************************/
/**
 * アングル速度.
 */
short Ang(void)
{
    static short Ang_B;
    short i;
    short ret;

    // i = getServoAngle();
    i = g_current_servo_angle;

    // if (i < 0)
    //     i = i * -1;
    ret = i - Ang_B;
    Ang_B = i;

    return ret;
}
/************************************************************************/
/**
 * オートセット用速度制御
 */
void PDtrace_Control_S(short Dig, short target_speed_ms)
{
    static float prev_current_speed_pulse = 0;
    static float K_MS_TO_PULSE = (CONTROL_PERIOD * ENC_PULSE_REV) / (TIRE_DIAMETER * PI);

    long i, iP, iD;
    int PWM;
    int DEF_PWM;

    int current_pulse = iEncoder;
    float target_pulse = (target_speed_ms)*K_MS_TO_PULSE;

    const int Ofset = 25;  // 20
    const int Gain = 60;   // 30
    const int P_gain = 20; // 20
    const int D_gain = 8;  // 10

    // ========== PD制御 ==========
    iP = (long)((target_pulse - (float)current_pulse) * P_gain);
    iD = (long)(((float)current_pulse - prev_current_speed_pulse) * D_gain);
    prev_current_speed_pulse = (float)current_pulse;

    PWM = (iP - iD) * Gain / 100;
    PWM += Ofset;

    // PWM制限
    if (PWM > 100)
        PWM = 100;
    if (PWM < -100)
        PWM = -100;

    motor_f(PWM, PWM);
    motor_r(PWM, PWM);
}

/************************************************************************/
/**
 * トレース時のモーター制御（PD制御）
 *
 * @param Dig ステアリング角度 (-120 ~ 120程度)
 * @param target_speed_ms 目標制御値（無単位）
 *   【重要】この値は厳密な「m/s」ではなく、制御用の目標値として使用
 *   - コーナー時: data_buff値 × 係数 + Angle_D_GF (例: 7.5 - 20 = -12.5)
 *   - 直線加速時: 100 などの大きな値（全力加速を指示）
 *   - 通常走行時: data_buff値そのまま (例: 15)
 *
 *   この値は内部で以下のように変換される:
 *   target_pulse = target_speed_ms * K_MS_TO_PULSE
 *   ※ K_MS_TO_PULSE = 14.468 なので、例えば target_speed_ms=100 なら
 *     target_pulse = 1446.8 という非現実的な大きな目標値になり、
 *     結果としてPWMが常に最大に近い値となり全力加速が実現される
 *
 * @param boost_trig ブーストトリガー（現在未使用）
 */
void PDtrace_Control(short Dig, short target_speed_ms, char boost_trig)
{
    static float prev_current_speed_pulse = 0;
    static float K_MS_TO_PULSE = (CONTROL_PERIOD * ENC_PULSE_REV) / (TIRE_DIAMETER * PI);

    long i, iP, iD;
    int PWM;
    int DEF_PWM;

    int current_pulse = iEncoder;
    // float target_pulse = (target_speed_ms / 10) * K_MS_TO_PULSE;
    float target_pulse = (target_speed_ms / 10.0f) * K_MS_TO_PULSE;

    // ========== カーブ制御パラメータ（調整可能） ==========
    // 外輪側の速度倍率（100% = PWMそのまま）
    const int OUTER_FRONT_RATIO = 100; // 外輪前: 100%
    const int OUTER_REAR_RATIO = 100;  // 外輪後: 95%（やや遅く）           AI:98

    // 内輪側の速度倍率
    const int INNER_FRONT_RATIO = 68; // 内輪前: 75%（旧r2=80から微調整）   AI:67
    const int INNER_REAR_RATIO = 71;  // 内輪後: 85%（旧r3=90から微調整）   AI:70

    // 深いカーブ用の追加減速
    const int SHARP_CURVE_REDUCTION = 0; // 深いカーブ時の追加減速量

    // ブレーキ比率（PWM < 0の時）
    const int F_BrakeRatio = 100; // ストレート前輪ブレーキ: 80%（旧100から軽く）
    const int R_BrakeRatio = 100; // ストレート後輪ブレーキ: 70%（旧100から軽く）

    const int FC_BrakeRatio = 20; // コーナー前輪ブレーキ: 80%（旧100から軽く）
    const int RC_BrakeRatio = 20; // コーナー後輪ブレーキ: 70%（旧100から軽く）

    // オフセット
    const int Ofset = 30;   // 20
    const int F_Ofset = 25; // 40

    // const int Gain = 80;    // 30
    // const int P_gain = 20;  // 20
    // const int D_gain = 8;   // 10
    int Gain = 80;
    const int P_gain = 20;
    const int D_gain = 8;

    if (abs(Dig) > 30) {
        Gain = 30;  // 深いコーナー：自然な減速を許容し、暴走を防ぐ
    } else if (abs(Dig) > 15) {
        Gain = 50;  // 浅いコーナー
    } else {
        Gain = 80;  // 直線：今まで通り強く速度を維持
    }


    int RR, RF, LR, LF;

    // ========== PD制御 ==========
    iP = (long)((target_pulse - (float)current_pulse) * P_gain);
    iD = (long)(((float)current_pulse - prev_current_speed_pulse) * D_gain);
    prev_current_speed_pulse = (float)current_pulse;

    // 【追加】コーナー中の自然な減速時に、iDが「加速」に働くのを防ぐ
    if (abs(Dig) > 15 && iD < 0) {
        iD = 0; // コーナーで速度が落ちた時は、ムキになって加速（D項の反発）しない
    }

    PWM = (iP - iD) * Gain / 100;
    PWM += Ofset;

    // PWM制限
    if (PWM > 100)
        PWM = 100;
    if (PWM < -100)
        PWM = -100;

    // 停止処理
    if (pattern >= 201)
    {
        if (PWM < -30)
            PWM = -30;
        if (PWM > 20)
            PWM = 20;
    }

    // ========== ステアリング角度に応じた制御 ==========
    i = abs((Dig * 3) / 10);
    DEF_PWM = PWM * i / 40;
    if (DEF_PWM > 100)
        DEF_PWM = 100;
    if (DEF_PWM < -100)
        DEF_PWM = -100;

    // ========== 右カーブ（Dig < -7） ==========
    if (Dig < -7)
    {
        if (PWM > 0) // 加速または定速
        {
            // 深い右カーブ（Dig < -35）
            if (Dig < -35)
            {
                // 基本PWMから追加減速
                int base_pwm = PWM - SHARP_CURVE_REDUCTION;
                if (base_pwm < 10)
                    base_pwm = 10; // 最低速度保証

                // 外輪（右側）: ほぼ基本速度
                LF = base_pwm * OUTER_FRONT_RATIO / 100;
                LR = base_pwm * OUTER_REAR_RATIO / 100;

                // 内輪（左側）: 減速
                RF = base_pwm * INNER_FRONT_RATIO / 100 - (DEF_PWM * 30 / 100);
                RR = base_pwm * INNER_REAR_RATIO / 100 - (DEF_PWM * 5 / 100);
            }
            // 浅い右カーブ（-35 <= Dig < -7）
            else
            {
                // 外輪（右側）
                LF = PWM * OUTER_FRONT_RATIO / 100;
                LR = PWM * OUTER_REAR_RATIO / 100;

                // 内輪（左側）: 速度差を付ける
                RF = PWM * INNER_FRONT_RATIO / 100; // 30
                RR = PWM * INNER_REAR_RATIO / 100;
            }
        }
        else // PWM <= 0（ブレーキ）
        {
            // ブレーキもPWMベースで制御
            RR = PWM * RC_BrakeRatio / 100;
            RF = PWM * FC_BrakeRatio / 100;
            LR = PWM * RC_BrakeRatio / 100;
            LF = PWM * FC_BrakeRatio / 100;
        }
    }

    // ========== 左カーブ（Dig > 7） ==========
    else if (Dig > 7)
    {
        if (PWM > 0) // 加速または定速
        {
            // 深い左カーブ（Dig > 35）
            if (Dig > 35)
            {
                // 基本PWMから追加減速
                int base_pwm = PWM - SHARP_CURVE_REDUCTION;
                if (base_pwm < 10)
                    base_pwm = 10; // 最低速度保証

                // 外輪（左側）: ほぼ基本速度
                RF = base_pwm * OUTER_FRONT_RATIO / 100;
                RR = base_pwm * OUTER_REAR_RATIO / 100;

                // 内輪（右側）: 減速
                LF = base_pwm * INNER_FRONT_RATIO / 100 - (DEF_PWM * 30 / 100);
                LR = base_pwm * INNER_REAR_RATIO / 100 - (DEF_PWM * 5 / 100);
            }
            // 浅い左カーブ（7 < Dig <= 35）
            else
            {
                // 外輪（左側）
                RF = PWM * OUTER_FRONT_RATIO / 100;
                RR = PWM * OUTER_REAR_RATIO / 100;

                // 内輪（右側）: 速度差を付ける
                LF = PWM * INNER_FRONT_RATIO / 100; // 30
                LR = PWM * INNER_REAR_RATIO / 100;
            }
        }
        else // PWM <= 0（ブレーキ）
        {
            // ブレーキもPWMベースで制御
            RR = PWM * RC_BrakeRatio / 100;
            RF = PWM * FC_BrakeRatio / 100;
            LR = PWM * RC_BrakeRatio / 100;
            LF = PWM * FC_BrakeRatio / 100;
        }
    }

    // ========== 直進（-7 <= Dig <= 7） ==========
    else
    {
        if (PWM > 0)
        {
            RF = PWM + F_Ofset;
            RR = PWM;
            LF = PWM + F_Ofset;
            LR = PWM;
        }
        else
        {
            LR = PWM * R_BrakeRatio / 100;
            LF = PWM * F_BrakeRatio / 100;
            RR = PWM * R_BrakeRatio / 100;
            RF = PWM * F_BrakeRatio / 100;
        }
    }

    motor_f(LF, RF);
    motor_r(LR, RR);
}
