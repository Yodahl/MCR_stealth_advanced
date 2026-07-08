/**********************************************************************/
/* サーボ制御(PD) と 速度制御(PDtrace)
/**********************************************************************/

/************************************************************************/
/* 舵角に応じたコーナー目標速度（case 11の減速テーブル）                */
/* 引数   angle_i: サーボ角（|angle_i| > 10 のとき呼ぶこと）            */
/* 戻り値 PDtrace_Controlへ渡す目標値                                   */
/* @note  case 11 と case 52（坂下り後）で共用。                        */
/*        Angle_D_GF（切り込み中の減速補正）もここで更新する。          */
/************************************************************************/
float cornerTargetSpeed(int angle_i)
{
    // 舵角が戻り方向なら補正なし、切り込み中は負の補正（減速方向）
    if (Angle_D > 0)
        Angle_D_GF = 0;
    else
        Angle_D_GF = Angle_D;

    if (abs(angle_i) > 110)
        return (data_buff[CORNER_SPEED_ADDR] * 80.0f / 100.0f) + Angle_D_GF; // 50
    if (abs(angle_i) > 80)
        return (data_buff[CORNER_SPEED_ADDR] * 86.0f / 100.0f) + Angle_D_GF; // 65
    if (abs(angle_i) > 68)
        return (data_buff[CORNER_SPEED_ADDR] * 94.0f / 100.0f) + Angle_D_GF; // 80
    if (abs(angle_i) > 47)
        return (data_buff[CORNER_SPEED_ADDR] * 96.0f / 100.0f) + Angle_D_GF; // 88
    if (abs(angle_i) > 20)
        return (data_buff[CORNER_SPEED_ADDR] * 98.0f / 100.0f) + Angle_D_GF;
    return (data_buff[CORNER_SPEED_ADDR]) + Angle_D_GF;
}

/************************************************************************/
/* クランク進入（case 101/106）の目標速度                               */
/* 戻り値 CRANK_SPEED設定値とCRANK_TOP_SPEED(m/s×10)の低い方           */
/* @note  設定値がCRANK_TOP_SPEEDより高いと、108のバンバン制御に        */
/*        任せるまで減速が始まらず高速進入する（LOG00308で2.63m/s進入・ */
/*        フルブレーキを確認）。クロスライン通過直後から2.2m/sへ        */
/*        落とし始めるよう上限を掛ける。                                */
/************************************************************************/
short crankEntryTarget(void)
{
    short t = data_buff[CRANK_SPEED_ADDR];
    short cap = (short)(CRANK_TOP_SPEED * 10.0f);
    return (t > cap) ? cap : t;
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

    // ========== センサ値を取得 ==========
    i = getAnalogSensor(); /* センサ値取得 */

    kp = data_buff[PROP_GAIN_ADDR];
    kd = data_buff[DIFF_GAIN_ADDR];

#if SERVO_GAIN_CORR_MODE == 2
    // ========== 旧実装: センサ値と（舵角<20時）ゲインの両方に補正 ==========
    float sensor_correction = calculateServoGainCorrection();

    // センサ値そのものに補正を適用
    i = (int)(i * sensor_correction);

    // 注意: abs() を取っていないため右切り(負角)では常に成立し、左切り20以上でのみ
    //       補正が外れる左右非対称な条件になっている（旧実装のまま）
    if (getServoAngle() < 20)
    {
        kp = (int)(kp * sensor_correction);
        kd = (int)(kd * sensor_correction);
    }
#endif

    /* サーボモータ用PWM値計算 */
    iP = kp * i;                   // 比例
    iD = kd * (iSensorBefore - i); // 微分
    iRet = iP - iD;

#if SERVO_GAIN_CORR_MODE == 1
    // ========== 補正の一本化: P・D項の合計に1回だけ適用 ==========
    // （kp/kdへの整数乗算だと kp=4 程度では丸め誤差が大きいため結果に掛ける）
    iRet = (int)(iRet * calculateServoGainCorrection());
#endif

    iRet /= 64;

    if (iRet > 90)
        iRet = 90;
    if (iRet < -90)
        iRet = -90;

    iServoPwm = -iRet;
    iSensorBefore = i;
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
    iP = 8 * (j - i);              // 比例 8
    iD = 60 * (iAngleBefore2 - j); // 微分(目安はPの5～10倍) 60
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
/**
 * オートセット用速度制御
 */
void PDtrace_Control_S(short Dig, float target_speed_ms)
{
    static float prev_current_speed_pulse = 0;
    static float K_MS_TO_PULSE = (CONTROL_PERIOD * ENC_PULSE_REV) / (TIRE_DIAMETER * PI);

    long iP, iD;
    int PWM;

    int current_pulse = iEncoder;
    float target_pulse = (target_speed_ms)*K_MS_TO_PULSE;

    const int Ofset = 25;  // 20
    const int Gain = 70;   // 30
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
    float target_pulse = (target_speed_ms / 10.0f) * K_MS_TO_PULSE;

    // ========== カーブ制御パラメータ（調整可能） ==========
    // 外輪側の速度倍率（100% = PWMそのまま）
    const int OUTER_FRONT_RATIO = 100; // 外輪前
    const int OUTER_REAR_RATIO = 100;  // 外輪後

    // 内輪側の速度倍率
    const int INNER_FRONT_RATIO = 68; // 内輪前
    const int INNER_REAR_RATIO = 71;  // 内輪後

    // 深いカーブ用の追加減速
    const int SHARP_CURVE_REDUCTION = 0; // 深いカーブ時の追加減速量

    // ブレーキ比率（PWM < 0の時）
    const int F_BrakeRatio = 100; // ストレート前輪ブレーキ
    const int R_BrakeRatio = 100; // ストレート後輪ブレーキ

    const int FC_BrakeRatio = 60; // コーナー前輪ブレーキ
    const int RC_BrakeRatio = 40; // コーナー後輪ブレーキ

    // オフセット
    const int Ofset = 30;   // 20
    const int F_Ofset = 25; // 40

    const int Gain = 70;   // 30
    const int P_gain = 20; // 20
    const int D_gain = 8;  // 10

    int RR, RF, LR, LF;

    // ========== PD制御 ==========
    iP = (long)((target_pulse - (float)current_pulse) * P_gain);
    iD = (long)(((float)current_pulse - prev_current_speed_pulse) * D_gain);
    prev_current_speed_pulse = (float)current_pulse;

    // 【追加】コーナー中の自然な減速時に、iDが「加速」に働くのを防ぐ
    if (abs(Dig) > 15 && iD < 0)
    {
        iD = 0; // コーナーで速度が落ちた時は、ムキになって加速（D項の反発）しない
    }

    PWM = (iP - iD) * Gain / 100;
    PWM += Ofset;

    // PWM制限
    if (PWM > Motor_Max_PWM)
        PWM = Motor_Max_PWM;
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
    if (Dig < -7 && pattern < 60)
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
    else if (Dig > 7 && pattern < 60)
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
