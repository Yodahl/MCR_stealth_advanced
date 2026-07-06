/**********************************************************************/
/*
 * MCR Advanced クラス走行プログラム (RMC-RA4M1)
 *
 * ファイル構成:
 *   main.ino           グローバル変数 / setup() / loop()の走行パターン状態遷移
 *   isr.ino            0.2ms×5ステップのタイマ割り込み(センサ読取・サーボ・停止監視)
 *   sensor.ino         センサ正規化/2値化, ライン・坂検出, サーボ角取得, ゲイン動的補正
 *   motor.ino          前後モータ/ステアのPWM出力, DIPSW・プッシュSW入力
 *   servo.ino          トレース用サーボPD制御, 速度PD制御(PDtrace)
 *   params_lcd.ino     走行パラメータの内蔵Flash保存とLCDメニュー
 *   logging.ino        走行ログ(RAMリングバッファ→microSD)
 *   replay.ino         再生走行(ログ解析・直線区間読込・加減速判定)
 *   personal_setting.h ピン割当・PWM周期・車体固有定数(VR_CENTER等)
 *
 * 走行パターン(pattern)の主な流れ:
 *   0-5: 起動･スタートゲート待ち → 9: スタート待機 → 10: 発進 → 11: 通常トレース
 *   50-52: 坂 / 101-120: クランク / 151-170: レーンチェンジ
 *   220-235: 停止処理･ログクローズ･再生走行データ作成
 */
/**********************************************************************/
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
char filename[32];
volatile uint16_t logCt = 0;
volatile uint16_t logRd = 0;
volatile int8_t motor_buff_Fl, motor_buff_Fr, motor_buff_Rl, motor_buff_Rr, motor_buff_stare;

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
void servoPwmOut(int pwm);
int check_crossline(void);
int check_rightline(void);
int check_leftline(void);

int Get_Distance_cm(void);


int getServoAngle(void);
int getAnalogSensor(void);
void servoControl(void);
void servoControl2(void);
void readDataFlashParameter(void);
void writeDataFlashParameter(void);
int lcdProcess(void);
int slopeCheck(void);

float calculateServoGainCorrection(void); // 動的ゲイン補正の計算関数

// ブレーキ用
short Dig_M(short angle);
short Ang(void);
void PDtrace_Control(short Dig, short target_speed_ms, char boost_trig = false);
void PDtrace_Control_S(short Dig, float target_speed_ms);


// ログ関係
void SD_file_open(void);
void SD_file_close(void);
void writeLog(void);
void LOG_rec(void);

// 再生走行関係
void Log_Analysis(void);
void Open_Rep(void);
int Check_StraightSection(int32_t current_dist_pulse);

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
volatile uint16_t anaSensRR_on;
volatile uint16_t anaSensCR_on;
volatile uint16_t anaSensCC_on;
volatile uint16_t anaSensCL_on;
volatile uint16_t anaSensLL_on;
// LED OFF時のセンサ値
volatile uint16_t anaSensRR_off;
volatile uint16_t anaSensCR_off;
volatile uint16_t anaSensCC_off;
volatile uint16_t anaSensCL_off;
volatile uint16_t anaSensLL_off;
// LED ON/OFFの差分
volatile int16_t anaSensRR_diff;
volatile int16_t anaSensCR_diff;
volatile int16_t anaSensCC_diff;
volatile int16_t anaSensCL_diff;
volatile int16_t anaSensLL_diff;

// 2値化
volatile uint8_t digiSensRR;
volatile uint8_t digiSensCR;
volatile uint8_t digiSensCC;
volatile uint8_t digiSensCL;
volatile uint8_t digiSensLL;

// 正規化用 最大・最小値
volatile uint16_t sensorMax;
volatile uint16_t sensorMin;
volatile uint16_t old_sensorMax;
volatile uint16_t old_sensorMin;

volatile uint16_t sensNormalized[5];

volatile bool LED_flag = false;

volatile int g_current_servo_angle = 0;
volatile int g_current_raw_adc = 0;


volatile int16_t pattern = 0; // マイコンカー動作パターン
volatile int16_t old_pattern = 0;

/*
 *	タイマカウント
 *	@note
 *		割り込み内でインクリメント
 */
volatile uint64_t cnt1;          // タイマ用
volatile uint64_t cnt2;          // タイマ用
volatile uint64_t check_sen_cnt; // 脱輪(センサ)停止判定用
volatile uint64_t check_enc_cnt; // 脱輪(エンコーダ)停止判定用
volatile uint64_t cnt_lcd;       // LCD処理で使用
volatile uint64_t safety_cnt;    // 同一パターン滞留の異常停止判定用

/*
 *	走行モード・時間処理等
 */
volatile int8_t crankDirection = 'N'; // クランクの方向 R:右 L:左
volatile int8_t laneDirection = 'N';  // レーンの方向 R:右 L:左
volatile int8_t slopeTotalCount = 0;  // 坂通過数

volatile int laneClearTime = 0;  // レーン後のブレーキ防止
volatile int crankClearTime = 0; // クランク後のブレーキ防止

volatile int8_t lineCCcount = 0;
volatile int8_t linellcount = 0;
volatile int8_t linerrcount = 0;

volatile int8_t anglecount = 0;

volatile int8_t slopecount = 0;
volatile bool isSlope = false;

volatile int8_t Trace_position = CENTER;


volatile int8_t sensCCon = OFF;
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
volatile int8_t iServo_flag = STOP; // STOP:PWM＝0　TRACE:ライン追従　ANGLE:角度指定

/*
 *	サーボ関連2
 */
volatile int16_t iSetAngle;
volatile int16_t iAngleBefore2;
volatile int16_t iServoPwm2;

/*
 * 動的ゲイン補正関連
 */
volatile int32_t ref_sensor_sum = 0;           // 基準センサ合計値(pattern2中の平均)
volatile bool ref_initialized = false;         // 基準値初期化フラグ
volatile int32_t ref_sum_acc = 0;              // 基準値平均用アキュムレータ
volatile uint16_t ref_sum_cnt = 0;             // 基準値平均用サンプル数
volatile float current_gain_correction = 1.0f; // 現在の補正係数


/*
 *	DataFlash関係
 */
uint8_t data_buff[16];

// トレース用
volatile short Angle_D;    // 舵角(絶対値)の1ms毎の変化量
volatile short Angle_D_GF; // 舵角変化による減速補正値(負のみ)

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
    signed int i;

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
        delay(100);

        CPU_LED_2 = OFF;
        delay(100);
    }

    while (1)
    {


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
                iAngle0 = VR_CENTER_GET(); // EEPROM保存のセンター値(LCDメニュー12で設定)
                LED_flag = true;
                iServo_flag = STOP;
                pattern = 2;

                break;
            }
            break;

        case 2:
            if (cnt1 > 100)
            {
                pattern = 3;
                cnt1 = 0;
            }
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
                    PDtrace_Control_S(i, 0.1);
                }
            }

            break;

        case 4: // ゲートが見えなくなるまで前進
            i = getServoAngle();
            iSetAngle = 0;
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
                PDtrace_Control_S(i, 0.05);
                cnt1 = 0;
            }

            break;

        case 5: // ゲート開閉確認走行開始
            if (sensRRon == ON && iEncoder == 0)
            {
                CPU_LED_2 = OFF;
                cnt1 = 0;
                check_sen_cnt = 0;
                check_enc_cnt = 0;
                cnt1 = 0;
                pattern = 9;
                break;
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
            motor_f(100, 100);
            motor_r(100, 100);
            if (lEncoderTotal >= CM_TO_PULSE(10)) // 発進判定 10cm(旧150パルス)
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
            if (Get_Distance_cm() > 30 && !SLOPE_flag && slopeTotalCount != 0)
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
                    pattern = 11;
                    lEncoderBuff = lEncoderTotal;
                }
            }
            else
            {
                anglecount = 0;
            }


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
            SLOPE_flag = false;
            PDtrace_Control(i, data_buff[SLOPE_SPEED_ADDR]);

            if (slopeCheck() == -1 && Get_Distance_cm() > 90)
            { // 坂登終わり検知
                pattern = 52;
                lEncoderBuff = lEncoderTotal;
            }
            break;

        case 52:
            iServo_flag = TRACE; // ライントレース
            Trace_position = CENTER;
            i = getServoAngle();
            iSetAngle = 0;
            PDtrace_Control(i, data_buff[TRG_SPEED_ADDR]);
            if (slopeCheck() == 1 && Get_Distance_cm() > 200)
            {
                pattern = 11;
                SLOPE_flag = false;
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
            if(abs(i) > 25){
                pattern = 11; // クランク誤読み防止
                lEncoderBuff = lEncoderTotal;
                break;
            }
            if (Get_Distance_cm() > 35) // 誤読み防止(350mm)
            {
                cnt1 = 0;
                lEncoderBuff = lEncoderTotal;
                pattern = 106;
                break;
            }

            break;


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

            if (Get_Distance_cm() >= 69) // クランク探索タイムアウト 69cm(旧1000パルス)
            {
                pattern = 11; // 通常に戻す
                break;
            }
            break;

        case 108: // クランク処理	 　ハーフライン検出後
            i = getServoAngle();
            if (crankDirection == 'L') // クランク方向　左
            {
                iSetAngle = CRANK_ANGLE_L / 2;
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
                iSetAngle = -CRANK_ANGLE_R / 2;
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
            }
            break;

        case 120:
            /* 少し時間が経つまで待つ */
            i = getServoAngle(); // ステアリング角度取得
            iServo_flag = TRACE;
            motor_r(100, 100);
            motor_f(100, 100);
            if (abs(i) < 15 && Get_Distance_cm() >= 20)
            {
                cnt1 = 0;
                pattern = 11;
                lEncoderBuff = lEncoderTotal;
                crankDirection = 0; // クランク方向クリア
                laneDirection = 0;  // レーン方向クリア
                crankClearTime = 50;
                break;
            }

            break;

            /************************************************************************/
            /* レーンチェンジの処理*/
            /************************************************************************/
        case 151: // ハーフライン後の処理１（速度制御）

            iSetAngle = 0;
            i = getServoAngle();

            if(abs(i) > 25){
                pattern = 11; // 誤読み防止
                lEncoderBuff = lEncoderTotal;
                break;
            }

            if (Get_Distance_cm() < 5)
            { // 左右に振られる対処
                iServo_flag = STOP;
                if (check_crossline())
                { /* クロスラインチェック         */
                    cnt1 = 0;
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

                if (sensCCon == ON) // CL
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

                if (sensCCon == ON) // CR
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

                if (sensRRon == ON && Get_Distance_cm() >= 1) // CR
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
                motor_r(10, 10);           // 後（左,右）

                if (sensLLon == ON && Get_Distance_cm() >= 1) // CL
                {
                    pattern = 166;
                    cnt1 = 0;
                }
            }
            break;

        case 166: // 最内センサ　黒反応後の処理（大カウンター）　最内センサ　白反応時待ち
            if (laneDirection == 'L')
            {                                  // レーン方向　左
                iSetAngle = -((LANE_ANGLE_L)); // カウンター　
                iServo_flag = ANGLE;           // 2角度制御 3:割込制御無
                motor_f(10, 10);               // 前 （左,右）
                motor_r(10, 10);               // 後（左,右)
                if (digiSensCC == ON && cnt1 >= 10)
                {
                    pattern = 168;
                }
            }
            else if (laneDirection == 'R')
            {                                 // レーン方向　右　カウンター処理
                iSetAngle = ((LANE_ANGLE_R)); // カウンター
                iServo_flag = ANGLE;          // 2角度制御 3:割込制御無
                motor_f(10, 10);              // 前 （左,右）
                motor_r(10, 10);              // 後（左,右)
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
                    iSetAngle = -((LANE_ANGLE_L / 2)); /* +で左 -で右に曲がります */
                    iServo_flag = ANGLE;               // 2角度制御 3:割込制御無
                    motor_f(10, 10);                   // 前 （左,右）
                    motor_r(10, 10);                   // 後（左,右
                }
            }

            else if (laneDirection == 'R')
            { // レーン方向　右　カウンター処理
                if (sensRRon)
                {
                    iSetAngle = ((LANE_ANGLE_R / 2)); /* +で左 -で右に曲がります */
                    iServo_flag = ANGLE;              // 2角度制御 3:割込制御無
                    motor_f(10, 10);                  // 前 （左,右）
                    motor_r(10, 10);                  // 後（左,右)
                }
            }

            if (digiSensCC == ON)
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
            if (Get_Distance_cm() >= 20 && abs(anaSensCL_diff - anaSensCR_diff) < 80) // 100
            {
                cnt1 = 0;
                pattern = 11;
                crankDirection = 0; // クランク方向クリア
                laneDirection = 0;  // レーン方向クリア
                laneClearTime = 200;
                break;
            }
            break;

        case 220:
            i = getServoAngle();
            iServo_flag = TRACE;
            PDtrace_Control_S(i, 2.0);
            if (iEncoder * PULSE_TO_MS <= 2)
            {
                lEncoderBuff = lEncoderTotal;
                pattern = 230;
                break;
            }
            break;

        case 230:
            i = getServoAngle();
            iServo_flag = TRACE;
            PDtrace_Control_S(i, 0.8);
            if (Get_Distance_cm() >= 300)
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
            if (iEncoder * PULSE_TO_MS < 1)
            {
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
            motor_f(0, 0);
            motor_r(0, 0);
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

        case 235: // 再生走行データ作成完了（待機）
            break;

        default:
            break;
        }
    }
}
