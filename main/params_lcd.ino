/**********************************************************************/
/* パラメータ保存(内蔵Flash) と LCDメニュー
/**********************************************************************/


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
        // ステアセンターの初期値は VR_CENTER(車体既定)を2バイトに分けて保存
        data_buff[VR_CENTER_H_ADDR] = (VR_CENTER >> 8) & 0xFF;
        data_buff[VR_CENTER_L_ADDR] = VR_CENTER & 0xFF;

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
    EEPROM.put(VR_CENTER_H_ADDR, data_buff[VR_CENTER_H_ADDR]);
    EEPROM.put(VR_CENTER_L_ADDR, data_buff[VR_CENTER_L_ADDR]);
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

    printf(" pattern=%d\n", pattern);

    // スイッチ情報取得
    sw = SwitchGetState();

    // メニュー＋１
    if (sw == MENU_UP)
    {
        lcd_pattern++;
        delay(200);

        if (lcd_pattern == 13)
            lcd_pattern = 1;
    }

    // メニュー－１
    if (sw == MENU_DOWN)
    {
        lcd_pattern--;
        delay(200);

        if (lcd_pattern == 0)
            lcd_pattern = 12;
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
        LcdPrintf("LL=%4d  RR=%4d   ",
                  anaSensLL_diff, anaSensRR_diff);
        cnt1 = 0;
        break;

    case 3:
        /* トレース比例制御調整 */
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

        // 【バグ修正】旧: `... && DATA_DOWN`（定数のため常に真＝プッシュSWだけで発動していた）
        if ((pushsw_get() == 1 && sw == DATA_UP) || (pushsw_get() == 1 && sw == DATA_DOWN))
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
    {
        /* ステアセンター設定 */
        /* ハンドルを真っすぐ保持した状態でSETスイッチを押すと現在値を取り込む。
           DATA_UP/DOWNで±1微調整。ポテンショ値は割り込みで安全に読んだ
           g_current_raw_adc を使う(メニューからのADC直読みによる混線を回避)。 */
        servoPwmOut(0);

        int live = g_current_raw_adc; // 現在のポテンショ値(0..1023)
        int center = VR_CENTER_GET();

        if (sw == SET) // SETスイッチ: 現在値をセンターとして取り込み
            center = live;
        if (sw == DATA_UP && center < 1023)
            center++;
        if (sw == DATA_DOWN && center > 0)
            center--;

        data_buff[VR_CENTER_H_ADDR] = (center >> 8) & 0xFF;
        data_buff[VR_CENTER_L_ADDR] = center & 0xFF;

        /* LCD処理 */
        LcdPosition(0, 0);
        LcdPrintf("12 Center=%4d ", center);
        LcdPosition(0, 1);
        LcdPrintf("Bar=%4d SET=cap", live);
        break;
    }
    }
    return 0;
}
