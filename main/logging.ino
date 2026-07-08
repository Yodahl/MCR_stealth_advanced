/**********************************************************************/
/* 走行ログ (RAMリングバッファ → microSD)
/**********************************************************************/


/************************************************************************/
/**
 * SDカードファイルオープン.
 */
void SD_file_open(void)
{
    // 通信速度Hz(e6=10の6乗),microSDのCS1(pins_arduino.hで定義)
    if (!SD.begin(2.4e6, CS1))
    {
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
    sprintf(filename, "%s/log%05d.csv", logDir, i + 1);
    microSD = SD.open(filename, FILE_WRITE);

}

/************************************************************************/
/**
 * ログをRAMに保存.
 */
void LOG_rec(void)
{
    saveDataA[0][logCt] = sensLLon << 2 | digiSensCC << 1 | sensRRon;
    saveDataA[1][logCt] = iEncoder;
    saveDataA[2][logCt] = pattern;
    saveDataA[3][logCt] = lEncoderTotal - lEncoderBuff;
    saveDataA[4][logCt] = getServoAngle();
    saveDataA[5][logCt] = iSetAngle;        // iSetAngle
    saveDataA[6][logCt] = anaSensLL_diff;   // sensNormalized[sLL]
    saveDataA[7][logCt] = anaSensCC_diff;   // sensNormalized[sCC]
    saveDataA[8][logCt] = anaSensRR_diff;   // sensNormalized[sRR]
    saveDataA[9][logCt] = motor_buff_stare; //: PWMステアリング;motor_buff_stare
    saveDataA[10][logCt] = motor_buff_Rl;   //: PWM後左;
    saveDataA[11][logCt] = motor_buff_Fl;   //: PWM前左;
    saveDataA[12][logCt] = motor_buff_Fr;   //: PWM前右;
    saveDataA[13][logCt] = motor_buff_Rr;   //: PWM後右;
    saveDataA[14][logCt] = slopeCheck();    //
    saveDistA[logCt] = lEncoderTotal;       // 再生走行用合計パルス数

    logCt++;

    if (logCt >= LOG_BUFF_SIZE)
    {
        logCt = 0;
    }

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
