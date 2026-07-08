/**********************************************************************/
/* 再生走行 (ログ解析, 直線区間読み込み, 加減速判定)
/**********************************************************************/


/***********************************************************************/
/**
 * ログデータ解析，再生走行用データ作成
 */
void Log_Analysis(void)
{
    // 通信速度Hz(e6=10の6乗),microSDのCS1(pins_arduino.hで定義)
    if (!SD.begin(2.4e6, CS1))
    {
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

    // 最新のログファイルを開く
    char latestLogPath[64];
    sprintf(latestLogPath, "%s/log%05d.csv", logDir, latestLogNum);
    Serial2.print("Log_Analysis: ");
    Serial2.println(latestLogPath);

    microSD = SD.open(latestLogPath, FILE_READ);
    if (!microSD)
    {
        return;
    }

    char line[256];
    int lineCount = 0;
    uint16_t buff_pattern = 0;
    int16_t buff_angle = 0;
    uint32_t buff_distance = 0; // 累積パルス（uint16だと約45mで破綻するためuint32）

    // ストレート区間の解析用変数
    bool inStraight = false;
    uint32_t straightStartDistance = 0;

    // 一旦RAMに全区間を貯めて、後処理（結合・フィルタ）をしてから書き出す
    uint32_t rawStart[REP_RAW_MAX];
    uint32_t rawEnd[REP_RAW_MAX];
    int rawCount = 0;

    // CSVファイルを1行ずつ読み込み
    while (microSD.available())
    {
        // 1行読み込み
        int i = 0;
        while (microSD.available() && i < (int)sizeof(line) - 1)
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

        // ヘッダー行をスキップ（1行目）
        if (lineCount == 1)
            continue;

        // 変数を初期化
        buff_pattern = 0;
        buff_angle = 0;
        buff_distance = 0;

        // データ解析（スペース区切りの "d_X=YYY" 形式）
        char *token = strtok(line, " ");
        while (token != NULL)
        {
            if (strncmp(token, "d_", 2) == 0)
            {
                char *equalPos = strchr(token, '=');
                if (equalPos != NULL)
                {
                    int dataIndex = atoi(token + 2); // d_の後の数字

                    switch (dataIndex)
                    {
                    case 2: // d_2 = pattern列
                        buff_pattern = (uint16_t)atoi(equalPos + 1);
                        break;
                    case 4: // d_4 = angle列
                        buff_angle = (int16_t)atoi(equalPos + 1);
                        break;
                    case 14: // d_14 = 累積パルス列
                        buff_distance = (uint32_t)atol(equalPos + 1);
                        break;
                    }
                }
            }
            token = strtok(NULL, " ");
        }

        // pattern==11/50（トレース中）でアングルがまっすぐの場合のストレート区間解析
        if ((buff_pattern == 11 || buff_pattern == 50) && buff_angle >= -ACCEL_ANGLE && buff_angle <= ACCEL_ANGLE)
        {
            if (!inStraight)
            {
                inStraight = true;
                straightStartDistance = buff_distance;
            }
        }
        else
        {
            if (inStraight)
            {
                inStraight = false;
                if (rawCount < REP_RAW_MAX)
                {
                    rawStart[rawCount] = straightStartDistance;
                    rawEnd[rawCount] = buff_distance;
                    rawCount++;
                }
            }
        }
    }

    // 最後にストレート区間が終了していない場合の処理
    if (inStraight && rawCount < REP_RAW_MAX)
    {
        rawStart[rawCount] = straightStartDistance;
        rawEnd[rawCount] = buff_distance;
        rawCount++;
    }
    microSD.close();

    // ===== 後処理1: 近接区間の結合 =====
    // 舵角が一瞬だけACCEL_ANGLEを超えて分断された直線を繋ぐ
    // （ギャップがREP_MERGE_GAP以下なら1本の直線として扱う）
    int mergedCount = 0;
    for (int i = 0; i < rawCount; i++)
    {
        if (mergedCount > 0 &&
            rawStart[i] - rawEnd[mergedCount - 1] <= (uint32_t)CM_TO_PULSE(REP_MERGE_GAP_CM))
        {
            rawEnd[mergedCount - 1] = rawEnd[i]; // 直前の区間に吸収
        }
        else
        {
            rawStart[mergedCount] = rawStart[i];
            rawEnd[mergedCount] = rawEnd[i];
            mergedCount++;
        }
    }

    // ===== 後処理2: 微小区間の除去と書き出し =====
    // 短すぎる区間は加速する前にブレーキ判定になるだけなので出力しない
    char outputPath[64];
    sprintf(outputPath, "%s/Log_Rep.csv", repDataDir);
    if (SD.exists(outputPath))
    {
        SD.remove(outputPath);
    }

    File outputFile = SD.open(outputPath, FILE_WRITE);
    if (!outputFile)
    {
        return;
    }
    outputFile.print("start_distance,end_distance\n");

    int written = 0;
    for (int i = 0; i < mergedCount; i++)
    {
        if (rawEnd[i] - rawStart[i] < (uint32_t)CM_TO_PULSE(REP_MIN_LENGTH_CM))
            continue;

        char outputLine[64];
        sprintf(outputLine, "%lu,%lu\n", (unsigned long)rawStart[i], (unsigned long)rawEnd[i]);
        outputFile.print(outputLine);
        written++;
    }
    outputFile.close();

    char summary[80];
    sprintf(summary, "sections raw=%d merged=%d written=%d\n", rawCount, mergedCount, written);
    Serial2.print(summary);

    CPU_LED_2 = OFF;
    CPU_LED_3 = OFF;
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
        // （uint16へのキャストは約45m=65535パルス超で破綻するためuint32で読む）
        token = strtok(lineCopy, ",");
        if (token != NULL)
        {
            straight_sections[straight_section_count].start_distance = (uint32_t)atol(token);

            // end_distanceを取得
            token = strtok(NULL, ",");
            if (token != NULL)
            {
                straight_sections[straight_section_count].end_distance = (uint32_t)atol(token);

                // デバッグ出力
                char debugLine2[128];
                sprintf(debugLine2, "Loaded section %d: start=%lu, end=%lu\n",
                        straight_section_count,
                        (unsigned long)straight_sections[straight_section_count].start_distance,
                        (unsigned long)straight_sections[straight_section_count].end_distance);
                Serial2.print(debugLine2);

                straight_section_count++;
            }
        }
    }

    // ファイルを閉じる
    repFile.close();
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
            float v_now = iEncoder * PULSE_TO_MS;

            // --- 3. 次のコーナー目標速度 [m/s] ---
            float v_next = DATA_TO_MS(data_buff[CORNER_SPEED_ADDR]);

            // --- 4. すでに目標速度以下なら加速可能 ---
            if (v_now <= v_next)
            {
                return ACCEL; // すでに目標速度以下なので加速可能
            }

            // --- 5. 必要なブレーキ距離の計算 [m] ---
            // 等加速度運動の公式: v² - v₀² = 2as
            // → s = (v² - v₀²) / (2a)
            // ここでは減速なので: s = (v_now² - v_next²) / (2 × DECEL_ACCEL)
            float brake_dist_m = (v_now * v_now - v_next * v_next) / (2.0f * DECEL_ACCEL);

            // --- 6. 安全マージンの追加（速度に比例） ---
            float safety_margin_m = v_now * SAFETY_TIME_MARGIN;
            brake_dist_m += safety_margin_m;

            // --- 7. ブレーキ距離をパルス数に変換 ---
            int32_t brake_dist_pulse = (int32_t)(brake_dist_m / M_PER_PULSE);

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
