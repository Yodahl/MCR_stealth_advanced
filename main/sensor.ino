/**********************************************************************/
/* センサ処理 (正規化, 2値化, ライン/坂検出, サーボ角取得, ゲイン動的補正)
/**********************************************************************/


/************************************************************************/
/* 差分の正規化                                                          */
/* 引数　 なし                                                          */
/* 戻り値 なし                                                          */
/************************************************************************/
void Diff_Nomal(void)
{
    int i;

    // 初期値: Max は最低 500、Min は最大 200 から更新される（正規化レンジの下駄）
    sensorMax = 500;
    sensorMin = 200;

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

            if (sensorValue < sensorMin)
            {
                // 負値はノイズとみなし最小値としては0を採用
                if (sensorValue < 0)
                {
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
    int ret = 0;

    if (sensLLon == ON && digiSensCC == ON && sensRRon == ON && abs(getServoAngle()) < 15 && abs(Angle_D) < 5)
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
    int ret = 0;

    if (digiSensCC == ON && sensRRon == ON && abs(getServoAngle()) < 15 && abs(Angle_D) < 5)
    {
        ret = 1;
    }
    else
    {
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
    int ret = 0;

    if (digiSensCC == ON && sensLLon == ON && abs(getServoAngle()) < 15 && abs(Angle_D) < 5)
    {
        ret = 1;
    }
    else
    {
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

/************************************************************
 * センサ距離に基づく動的ゲイン補正係数を計算
 * 坂でセンサが近くなるとゲインを下げ、遠くなるとゲインを上げる
 * 基準値(ref_sensor_sum)はISR側でpattern2中の平均から初期化される
 *
 * @return 補正係数
 *************************************************************/
float calculateServoGainCorrection(void)
{
#if SERVO_GAIN_CORR_MODE == 0
    return 1.0f;
#else
    if (!ref_initialized)
        return 1.0f;

    // センサ出力の合計値（坂で路面との距離が変わると増減する）
    int32_t current_sensor_sum = anaSensLL_diff + anaSensCL_diff +
                                 anaSensCC_diff + anaSensCR_diff + anaSensRR_diff;

    // 基準値からの乖離
    // 大きい（近い）→ 補正係数小さい（ゲイン低下）
    // 小さい（遠い）→ 補正係数大きい（ゲイン上昇）
    int32_t sensor_diff = current_sensor_sum - ref_sensor_sum;

#if SERVO_GAIN_CORR_MODE == 2
    // 旧実装の感度・上下限
    float correction = 1.0f + (sensor_diff * 0.01f);
    if (correction > 2.0f)
        correction = 2.0f;
    if (correction < 0.2f)
        correction = 0.2f;
#else
    float correction = 1.0f + (sensor_diff * SERVO_GAIN_CORR_COEF);
    if (correction > SERVO_GAIN_CORR_MAX)
        correction = SERVO_GAIN_CORR_MAX;
    if (correction < SERVO_GAIN_CORR_MIN)
        correction = SERVO_GAIN_CORR_MIN;
#endif

    // グローバル変数に保存（LCD表示・ログ用）
    current_gain_correction = correction;

    return correction;
#endif
}

/************************************************************************/
/* アナログセンサ値取得（スムージング+補正版）
 * 引数　 なし
 * 戻り値 センサ値
 */
int getAnalogSensor(void)
{
    int ret;

    // ========== 基本的なセンサ選択ロジック（既存） ==========
    if (Trace_position == RIGHT)
    {
        ret = (anaSensCC_diff) - (anaSensRR_diff);
    }
    else if (Trace_position == LEFT)
    {
        ret = (anaSensLL_diff) - (anaSensCC_diff);
    }
    else
    {
        ret = (anaSensCL_diff) - (anaSensCR_diff);
    }

#if SERVO_GAIN_CORR_MODE == 2
    // ========== センサ合計値による応答性スムージング（旧実装のみ） ==========
    // モード1では補正をservoControlのP・D項へ一本化したためここでは何もしない
    int32_t sensor_sum = anaSensLL_diff + anaSensCL_diff +
                         anaSensCC_diff + anaSensCR_diff + anaSensRR_diff;

    // センサ合計値が大きい（近い）場合、センサ値を少し減衰させる
    if (sensor_sum > ref_sensor_sum + 100)
    {
        ret = (ret * 85) / 100; // 近い状態：応答を弱める
    }
    else if (sensor_sum < ref_sensor_sum - 100)
    {
        ret = (ret * 115) / 100; // 遠い状態：応答を強める
    }
#endif

    return ret;
}


/**********************************************************************/
/**
 *	入力センサの初期化処理.
 */
void initSens(void)
{
    /* アナログセンサ */
    ad.useCh(SENS_A_LL); // CN8  8 D67
    ad.useCh(SENS_A_CL); // CN8  6 D65
    ad.useCh(SENS_A_CC); // CN8  5 D64
    ad.useCh(SENS_A_CR); // CN8  4 D63
    ad.useCh(SENS_A_RR); // CN8  3 D62
    ad.useCh(SENS_A_VR); // CN8  7 D66
    ad.start();
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

    i = g_current_servo_angle;

    if (i < 0)
        i = i * -1;
    ret = i - Ang_B;
    Ang_B = i;

    return ret;
}
