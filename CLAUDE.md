# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

マイコンカーラリー（MCR）Advanced クラス用の自律走行マシンのファームウェア。実際の競技車両（岡谷杯等）で使われており、**挙動を変える変更は実車検証なしにマージしないこと**。コメント・コミットメッセージは日本語。

- ボード: RMC-RA4M1（Renesas RA4M1, Arduino環境）
- スケッチ: `main/` フォルダ
- PC側ツール: `MCRLoger*.HTML`（走行ログCSVをブラウザで可視化。**ver4が最新**＝再生機能付き: 車両パネル・時系列グラフ・推定軌跡マップ。完全自己完結でオフライン動作。ヘッドレスChrome＋fetchフックで自動検証可能）
- `LOG_REP.CSV` / `sample.csv`: 再生走行データとロガー用のサンプル

## ビルド

arduino-cli は単体インストールされていないが、Arduino IDE 同梱のものを使う:

```bash
CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
"$CLI" compile --fqbn rmc_ra4m1_20:ra4m1_20:rmc_ra4m1_20 --libraries ~/Library/Arduino15/libraries main
```

- `--libraries ~/Library/Arduino15/libraries` が必須（SD.h がここにある）。
- EEPROMライブラリのアーキテクチャ警告は既知で無害。
- テストは存在しない。検証はコンパイル＋実車走行のみ。リファクタ時は「コンパイル後のバイナリ一致」や「コメント除去・正規化したソースの関数単位diff」で挙動不変を確認する手法が有効（過去に実績あり）。

## アーキテクチャ

制御の実体は **1msサイクルのタイマ割り込み**（`isr.ino` の `timerCallback`、0.2ms×5ステップ）と、**`loop()` 内の `pattern` 状態機械**（`main.ino`）の2層構造。グローバル `volatile` 変数で両者が通信する。

- 割り込み側（毎1ms）: 赤外LEDのON/OFF差分でアナログセンサ5個を読み（外乱光除去）、正規化・2値化、サーボPWM出力、クロス/ハーフライン検出による `pattern` の強制遷移、脱輪・滞留時の安全停止、10ms毎のエンコーダ集計とログ記録。
- `loop()` 側: `pattern` に応じた速度指示・遷移条件の判定。主な流れ:
  - 0〜5: 起動・スタートゲート待ち → 9: スタート待機 → 10: 発進 → 11: 通常トレース
  - 50〜52: 坂 / 101〜120: クランク / 151〜170: レーンチェンジ / 220〜235: 停止・ログクローズ

ファイル分担（`main/`）:

| ファイル | 役割 |
|---|---|
| `main.ino` | グローバル変数・プロトタイプ・setup()・loop()状態機械 |
| `isr.ino` | タイマ割り込み本体 |
| `sensor.ino` | 正規化(`Diff_Nomal`)・ライン/坂検出・サーボ角取得・距離ゲイン動的補正 |
| `motor.ino` | 4輪+ステアのPWM出力（レジスタ直叩き）、DIPSW/プッシュSW |
| `servo.ino` | トレース用サーボPD制御、速度PD制御(`PDtrace_Control`) |
| `params_lcd.ino` | LCDメニューによる走行パラメータ調整と内蔵Flash(EEPROM)保存 |
| `logging.ino` | 10ms毎ログ→RAMリングバッファ→microSD (`LOG/logNNNNN.csv`) |
| `replay.ino` | 再生走行: ログ解析→`REP/Log_Rep.csv`→直線区間で加減速判定 |
| `personal_setting.h` | ピン割当・PWM周期・車体固有定数。`VR_CENTER` はステアセンターの初期値/フォールバック（実運用値はEEPROM保存・LCDメニュー12で設定） |
| `mcr_gpt_lib` / `mcr_ad_lib` | RA4M1のGPT(PWM/エンコーダ)・ADC低レイヤ（ベンダ由来。`scanDual`=複数ch一括スキャンのみ独自追加） |
| `lcd_lib` / `switch_lib` / `i2c_eeprom_lib` | LCD・スイッチ・I2C EEPROM（i2c_eepromは現在未使用） |

### 再生走行（記憶走行）の仕組み

1周目のログを走行後に `Log_Analysis()` で解析し、pattern==11/50 かつ舵角±20以内の区間を「直線」として抽出→**近接区間の結合（`REP_MERGE_GAP_CM`）と微小区間の除去（`REP_MIN_LENGTH_CM`）**の後処理をして `REP/Log_Rep.csv` に保存。次回起動時に DIPSW2 ON でスタートすると `Open_Rep()` が読み込み、`Check_StraightSection()` がエンコーダ累積パルスと照合して ACCEL/BRAKE/OFF を10ms毎に判定、直線で全力加速・コーナー手前でブレーキする（`Cheat_flag`）。距離の単位は全てエンコーダパルス（1m ≒ 1447パルス、`METER`）。ブレーキ開始点は `Check_StraightSection` 内の `DECEL_ACCEL`（想定減速度）と `SAFETY_TIME_MARGIN` で決まる（速度と安定性のトレードオフ調整箇所）。

### ステアセンター（VR_CENTER）

ステア中立位置のポテンショ値。以前はコンパイル時 `#define VR_CENTER`（524）固定だったが、EEPROMパラメータ化した。0〜1023の値を `VR_CENTER_H_ADDR`(0x0B)/`VR_CENTER_L_ADDR`(0x0C) の2バイトに分割保存し、`VR_CENTER_GET()`（personal_setting.h）で復元。run-start（main.ino case1）で `iAngle0 = VR_CENTER_GET()` として使い、LCDメニュー**12番「Center」**で設定する（ハンドルを直進で保持→`SET`スイッチで現在の `g_current_raw_adc` を取り込み、`DATA_UP/DOWN`で±1微調整。保存は他パラメータ同様に走行開始時か「10 Parameter Set」で永続化）。`#define VR_CENTER` はEEPROM未初期化時のデフォルト値としてのみ残存。自動取得方式は「センターでない値を掴む」ため採用しない方針（固定値＋現場でLCD再測定が結論）。

### 単位系の注意

- 速度: `iEncoder`（10ms毎のパルス数）× `PULSE_TO_MS`(0.0691) = m/s
- `data_buff[]` の速度パラメータは 0.1m/s 単位（`DATA_TO_MS` で変換）
- `PDtrace_Control` の第2引数は厳密な m/s ではなく制御目標値（100で実質全力加速）
- **距離のしきい値（人が編集する値）は cm で書く**: 生パルスと比較する場合は `CM_TO_PULSE(cm)`（personal_setting.h）を通すか、`Get_Distance_cm()` と比較する。内部計算（`Check_StraightSection` 等）はパルスのままで良い

## サーボゲイン動的補正（SERVO_GAIN_CORR_MODE）

坂などでのセンサ-路面距離変化を補正する機能。`personal_setting.h` の `SERVO_GAIN_CORR_MODE` で切替: 0=無効 / 1=P・D項へ一括適用（既定・一本化） / 2=旧実装（三重がけ・テスト用）。**旧実装は基準値が初期化されないバグで実質不発**（`getAnalogSensor` の常時0.85倍減衰のみ効いていた）だった。現在は基準値をスタート待ち(pattern2)中の平均でISR側で初期化する。モード0/1では0.85倍減衰が無くなるため実効ゲインが約18%上がる → 実車テストはトレースkpを1段下げから。

## 既知の要注意箇所（2026-07時点・未修正）

- `servoControl()` のモード2（旧実装）は `getServoAngle() < 20` に abs() がなく左右非対称（旧実装再現のため温存）
- 同一パターン滞留の安全停止 `safety_cnt >= 600` は**0.6秒**（1msブロック内。コメントの旧記述「6秒」は誤りだった）。**0.6秒で良いというのがユーザーの判断（2026-07確認済み）なので変更しないこと**。ただし走行ログで pattern>100 の直後に 231 へ飛ぶ誤停止を見つけたら、ここが原因の可能性をユーザーに伝える
- ログのリングバッファはSD書き込みが遅延すると未書込データを上書きする
- `lcdProcess` のADC読み（`BAR_ANGLE`等）はISRのADCスキャンと排他されていない（表示のみ実害なし・従来から）

過去に修正済み: 坂検知 `Slope_thr[5]` の配列外書き込み（メモリ破壊）、LCDメニュー case11 のスイッチ判定、再生走行距離のuint16切り詰め（45m超で破綻）。
