/**********************************************************************/
/* タイマ割り込み(0.2ms×5ステップ=1ms周期) センサ読取/サーボ制御/ライン検出/停止監視
/**********************************************************************/


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


        if (laneClearTime > 0)
        {
            laneClearTime--;
        }

        if (crankClearTime > 0)
        {
            crankClearTime--;
        }

        // 他センサとの混線を防ぐため、赤外LED消灯中のこのタイミングでポテンショを読む
        g_current_raw_adc = BAR_ANGLE;
        if (g_current_raw_adc < 1000)
        {
            // 取得した生データをフィルタに通し、グローバル変数に保存
            g_current_servo_angle = Dig_M(g_current_raw_adc) - iAngle0;
        }

        // --- センサ値取得(OFF値: 外乱光) ---
        // 5ch一括スキャン（1chずつのgetDataDualよりソフト待機が少ない）
        {
            static const uint8_t lineSensCh[5] = {SENS_A_RR, SENS_A_CR, SENS_A_CC, SENS_A_CL, SENS_A_LL};
            int16_t v[5];
            ad.scanDual(lineSensCh, v, 5);
            anaSensRR_off = v[0];
            anaSensCR_off = v[1];
            anaSensCC_off = v[2];
            anaSensCL_off = v[3];
            anaSensLL_off = v[4];
        }

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
        {
            static const uint8_t lineSensCh[5] = {SENS_A_RR, SENS_A_CR, SENS_A_CC, SENS_A_CL, SENS_A_LL};
            int16_t v[5];
            ad.scanDual(lineSensCh, v, 5);
            anaSensRR_on = v[0];
            anaSensCR_on = v[1];
            anaSensCC_on = v[2];
            anaSensCL_on = v[3];
            anaSensLL_on = v[4];
        }

        // --- LED消灯 (Duty 20%確保のため即消灯) ---
        INFRARED_LED = OFF;

        // --- 差分計算 ---
        anaSensRR_diff = anaSensRR_on - anaSensRR_off;
        anaSensCR_diff = anaSensCR_on - anaSensCR_off;
        anaSensCC_diff = anaSensCC_on - anaSensCC_off;
        anaSensCL_diff = anaSensCL_on - anaSensCL_off;
        anaSensLL_diff = anaSensLL_on - anaSensLL_off;

        // --- 差分の正規化 ---
        Diff_Nomal();

        // --- 2値化 ---
        digiSensRR = ((sensNormalized[sRR] > THR_M_Sens) ? ON : OFF);
        digiSensCR = ((sensNormalized[sCR] > THR_Sens) ? ON : OFF);
        digiSensCC = ((sensNormalized[sCC] > THR_Sens) ? ON : OFF);
        digiSensCL = ((sensNormalized[sCL] > THR_Sens) ? ON : OFF);
        digiSensLL = ((sensNormalized[sLL] > THR_M_Sens) ? ON : OFF);

#if SERVO_GAIN_CORR_MODE != 0
        // --- ゲイン動的補正の基準値サンプリング ---
        // スタート待ち(pattern2, 約100ms)中のセンサ合計を平均して基準値にする。
        // 旧実装はservoControl(TRACE時のみ実行)内でpattern2を待っていたため
        // 一度も初期化されなかった（補正が不発だった）のをここで修正。
        if (pattern == 2)
        {
            ref_sum_acc += (int32_t)(anaSensLL_diff + anaSensCL_diff +
                                     anaSensCC_diff + anaSensCR_diff + anaSensRR_diff);
            ref_sum_cnt++;
        }
        else if (!ref_initialized && ref_sum_cnt > 0)
        {
            ref_sensor_sum = ref_sum_acc / ref_sum_cnt;
            ref_initialized = true;
        }
#endif
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

        if (digiSensCC == ON)
        {
            lineCCcount++;
            if (lineCCcount > 4)
            {
                sensCCon = ON;
            }
        }
        else
        {
            lineCCcount = 0;
            sensCCon = OFF;
        }

        if (digiSensLL == ON)
        {
            linellcount++;
            if (linellcount > 4) // 45
            {
                sensLLon = ON;
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
            if (linerrcount > 4)
            {
                sensRRon = ON;
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

        break;

        /* ----------------------------------------------------------------
            case 4: [0.6ms - 0.8ms]
         ---------------------------------------------------------------- */
    case 4:
        Angle_D = Ang();

        // スタートから41cm(旧600パルス)まではライン検出を無効化
        if ((pattern == 11 || pattern == 50) && lEncoderTotal > CM_TO_PULSE(41))
        {
            if (isSlope && SLOPE_flag && !check_leftline() && !check_rightline() && !check_crossline())
            {
                // 坂走行処理へ	のぼるくん
                slopeTotalCount++;
                pattern = 50;
                lEncoderBuff = lEncoderTotal;
            }

            // クロスラインチェック
            if (check_crossline() && SLOPE_flag) //&& crankClearTime == 0 && laneClearTime == 0
            {
                cnt1 = 0;
                pattern = 101;
                lEncoderBuff = lEncoderTotal;
            }

            // 左ハーフラインチェック
            if (check_leftline() && SLOPE_flag) //&& crankClearTime == 0 && laneClearTime == 0
            {
                cnt1 = 0;
                pattern = 151;
                laneDirection = 'L';
                Trace_position = RIGHT;
                lEncoderBuff = lEncoderTotal;
            }
            // 右ハーフラインチェック
            if (check_rightline() && SLOPE_flag) //&& crankClearTime == 0 && laneClearTime == 0
            {
                cnt1 = 0;
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
            break;
        case 2:
            // 再生走行の加減速判定
            // 入力(lEncoderTotal, iEncoder)はcase 1で10ms毎にしか更新されないため、
            // 1ms毎(旧: タイマcase 4)に呼んでも結果は同じ。エンコーダ更新直後の
            // このタイミングで10ms毎に1回だけ判定する。
            if (Cheat_flag)
            {
                mode = Check_StraightSection(lEncoderTotal);
            }
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
                // 【バグ修正】旧コードは slope_thr_cnt が 5 のまま書き込みを行い
                // Slope_thr[5]（配列外）へ書いていた。インクリメント後に折り返す。
                if (which_slope)
                {
                    Slope_thr[slope_thr_cnt] = anaSensCC_diff;
                    slope_thr_cnt++;
                    if (slope_thr_cnt >= 5)
                    {
                        slope_thr_cnt = 0;
                        which_slope = false;
                    }
                }
                else
                {
                    Slope_thr_1[slope_thr_cnt_1] = anaSensCC_diff;
                    slope_thr_cnt_1++;
                    if (slope_thr_cnt_1 >= 5)
                    {
                        slope_thr_cnt_1 = 0;
                        which_slope = true;
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
                if (check_sen_cnt >= 400 && !(dipsw_get() & 0x01)) // 1000
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
                if (check_enc_cnt >= 200) // 2000
                {
                    pattern = 231;
                }
            }
            else
            {
                check_enc_cnt = 0;
            }
            /* 途中で停止処理 */
            if (pushsw_get() == ON && lEncoderTotal > CM_TO_PULSE(69)) // 旧1000パルス
            {
                pattern = 231;
                cnt1 = 0;
            }

            if (pattern == old_pattern && pattern > 100)
            {
                safety_cnt++;

                // このブロックは1ms周期で実行されるため 600カウント = 0.6秒 で異常停止
                // （旧コメントの「6秒」は誤り。意図が6秒なら要修正）
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
