# WASA CanSat REGULUS

ESP32-S3を使用するCanSat搭載ソフトウェアです。FreeRTOS版とbaremetal版があり、センサードライバー、ESP-NOW通信、WCPPテレメトリ、ミッション状態管理を共有します。

現段階ではモーター仕様が未確定のため、分離機構および走行モーターは一切駆動しません。状態4・5への遷移とサブ状態判定は行いますが、出力は安全停止のままです。

## 対象ハードウェア

- MCU: ESP32-S3-WROOM-1 N16R8
- IMU: QMI8658
- 磁気センサー: MMC5603
- 気圧センサー: HP203B
- GNSS: MAX-M10M
- 無線: ESP-NOW

ピン定義は `include/BoardConfig.h` を参照してください。

## ミッション状態

状態番号は地上局との通信仕様です。番号を変更する場合は地上局も同時に更新してください。

| 値 | 名前 | 動作 |
|---:|---|---|
| 0 | BOOT | 起動状態。地上局から開発／ミッションモードが選択されるまで待機 |
| 1 | LAUNCH_STANDBY | 打ち上げ待機。通常周期でセンサー取得とテレメトリ送信 |
| 2 | LAUNCH | 打ち上げ検知後。IMU、磁気、気圧、テレメトリを高速化 |
| 3 | DEPLOYED | ロケット放出後のパラシュート降下 |
| 4 | LANDED | 着陸後の分離フェーズ。分離機構は駆動しない |
| 5 | RUNNING | 走行フェーズ。サブ状態判定のみ行い、モーターは駆動しない |
| 6 | FINISHED | ゴール到達通知。一定時間後に状態0へ戻る |
| 7 | LOW_BATTERY | 低電圧フェイルセーフ。必要最小限のGPSテレメトリだけを低頻度送信 |

### 状態5のサブ状態

| 値 | 対応 | 名前 | 判定 |
|---:|---|---|---|
| 0 | — | NONE | 状態5以外 |
| 1 | 5a | GPS_NAVIGATION | GPSによるゴール接近 |
| 2 | 5b | RSSI_HOMING | 有効なRSSIが閾値以上 |
| 3 | 5c | STACK_ESCAPE | GPS位置、RSSI、方位が一定時間ほぼ変化しない |

5cは現在、地上局へ状態を通知して一定時間保持するだけです。脱出用モーター動作は実装していません。

## 運用モードとアップリンク

運用モードはミッション状態とは別フィールドです。

| 値 | 名前 |
|---:|---|
| 0 | UNSELECTED |
| 1 | DEVELOPMENT |
| 2 | MISSION |

地上局はWCPPコマンドの `AC` フィールドへ次の値を設定します。

| AC | コマンド |
|---:|---|
| 1 | 開発モードを選択 |
| 2 | ミッションモードを選択 |
| 3 | 状態1から状態2へLAUNCH |
| 4 | Low Battery解除条件成立後、状態7から状態0へ戻る |
| 5 | 飛行中再起動後、着地確認済みとして状態7から状態4へ復帰 |

コマンド値は MissionCommand enumで定義されています。モード選択は状態0でだけ受け付けます。ミッションモードを選択すると状態1へ遷移します。状態1では AC=3 を受けると状態2へ進みます。また、地上局コマンドが届かない場合に備え、合成加速度15 m/s²以上が300 ms継続した場合も自動的に状態2へ遷移します。開発モードでは状態0を維持します。

`AC=5` は `FAILSAFE / UNSAFE_FLIGHT_RESTART` かつミッションモードの場合だけ受理します。落下中に送ると状態4扱いになるため、地上局オペレーターが実際の着地を確認してから二段階操作で送信します。Low BatteryまたはSensor Failureによる状態7では拒否します。

## WCPPテレメトリ

パケットIDは2、コンポーネントIDは0です。

ミッション管理に関係するフィールド：

| フィールド | 内容 |
|---|---|
| `ST` | ミッション状態 0～7 |
| `SS` | 走行サブ状態 0～3 |
| `MO` | 運用モード 0～2 |
| `TI` | 起動後時刻 [s] |
| `LA`, `LO` | 緯度・経度 |
| `SA` | 捕捉衛星数 |

通常パケットには気圧、温度、IMU、磁気、姿勢、有効フラグも含まれます。状態7では時刻、状態、サブ状態、モード、GPS、衛星数だけを送信します。

## 状態遷移

FreeRTOS版とbaremetal版はともに `MissionController` を使用します。

主な自動判定：

1. 状態1で合成加速度が閾値を一定時間超えると状態2
2. 状態2で最短時間経過後、自由落下相当の加速度を検知すると状態3
3. 状態3で鉛直速度、加速度、角速度が一定時間安定すると状態4
4. 状態4で安全待機後、モーターを停止したまま状態5
5. 状態5で設定済みゴール座標の範囲内に一定時間留まると状態6
6. 状態6の通知時間経過後、運用モードを未選択に戻して状態0
7. 有効なバッテリー電圧が低電圧閾値以下で継続すると状態7

バッテリー電圧が0 V、NaN、未計測の場合は状態7へ遷移しません。

## ゴール座標の設定

誤った座標でミッション終了しないよう、デフォルトではゴール判定を無効にしています。実機用ビルドフラグへ次を追加すると有効になります。

```ini
build_flags =
    ${env.build_flags}
    -D MISSION_GOAL_LATITUDE=35.000000
    -D MISSION_GOAL_LONGITUDE=139.000000
```

座標を設定しない場合も、状態5のGPS／RSSI／スタック判定は継続しますが、状態6へは自動遷移しません。

## バッテリー電圧とRSSI

`update_system_status(float battery_voltage, int rssi)` で共有データへ入力します。

- バッテリー電圧: V単位。0以下は未計測扱い
- RSSI: dBm単位の負値。0以上は未計測扱い

現在の基板入力処理にはバッテリーADC取得がまだ接続されていません。また、使用中のArduino ESP32 2.xのESP-NOW受信コールバックからRSSIを取得する処理も未接続です。このため5b・5cは有効なRSSIが入力されるまで誤発動しません。

## NVS復元

- 運用モードは選択時に保存
- ミッション状態は状態遷移時だけ保存
- センサー更新周期ごとのNVS書き込みは行わない
- 状態0、1、4、5、6、7は再起動後に復元可能
- 飛行中の状態2、3から再起動した場合は、安全側として状態7へ移行
- 飛行中再起動の状態7は自動解除せず、着地確認後の `AC=5` だけで状態4へ復帰
- Low Batteryの状態7は電圧回復条件成立後の `AC=4` で状態0へ戻る

走行サブ状態は保存せず、状態5の再開時にセンサー値から再判定します。

## ビルド

PlatformIOを使用します。

```powershell
pio run -e freertos_sensors
pio run -e freertos_flight
pio run -e baremetal
pio run -e baremetal_espnow
```

| 環境 | 内容 |
|---|---|
| `freertos_sensors` | FreeRTOS、実センサー、ESP-NOWなし |
| `freertos_flight` | FreeRTOS、実センサー、ESP-NOWあり |
| `baremetal` | superloop、実センサー、ESP-NOWなし |
| `baremetal_espnow` | superloop、実センサー、ESP-NOWあり |

bring-up用に i2c_scan、imu_test、mag_test、baro_test、gnss_test、motor_test も定義されています。通常ファームウェアからモーターテストは参照されません。

### flight_mockの状態遷移試験

NVSを消去して freertos_flight_mock を書き込み、地上局から AC=2 でミッションモードを選択すると状態1になります。状態1で AC=3 を送ると直ちに状態2へ進みます。手動LAUNCHを送らない場合は、10秒後にmockが打上げ加速度を生成して自動判定経路を試験します。その後は上昇、放出、パラシュート降下、着陸を模擬して状態5まで進みます。

## ディレクトリ構成

- `include/MissionState.h`: 状態、サブ状態、モード、コマンドの通信値
- `include/MissionConfig.h`: 遷移閾値と周期
- `src/MissionController.cpp`: FreeRTOS／baremetal共通状態機械
- `src/tasks/`: FreeRTOSタスク
- `src/baremetal/main.cpp`: baremetal superloop
- `src/communication/`: ESP-NOWとWCPP
- `src/drivers/`: センサードライバー
- `src/CanSatData.cpp`: タスク／superloop間の共有データ

## 未完了・実機確認待ち

- バッテリーADC回路に合わせた電圧取得と換算
- ESP-NOW受信RSSIの取得方法確定と共有データへの接続
- ゴール座標、ゴール半径、RSSI閾値、スタック判定値の実験調整
- パラシュート分離機構
- GPS走行、RSSIホーミング、スタック脱出のモーター制御
- 基板上でのESP-NOW送受信確認
- 実飛行ログに基づく打ち上げ・放出・着陸閾値の再調整

## ESP-NOW受信機を試験基板へ書き込む

OneDriveのArduino ReceiverをESP32-S3試験基板向けの espnow_receiver 環境として移植しています。ESP-NOWチャンネルは1、USBシリアルは115200 baud、受信表示LEDはGPIO21です。

ビルド：

    pio run -e espnow_receiver

自動検出したポートへ書き込み：

    pio run -e espnow_receiver -t upload

複数のESP32が接続されている場合は、先にポートを確認して明示します。

    pio device list
    pio run -e espnow_receiver -t upload --upload-port COM5

ReceiverはUSBへWCPPバイナリを直接出力します。地上局を起動している間は同じCOMポートをPlatformIOのシリアルモニターで同時に開かないでください。
## Goal navigation and motor sequence

The ground station sends WCPP action `AC=6` with Float64 `GL` (latitude) and `GO` (longitude). The vehicle validates the coordinate, stores it in ESP32 NVS, and echoes `GC`, `GL`, `GO`, `GD`, `MC`, and `PS` in normal telemetry.

State 4 writes the separation-attempt flag to NVS before driving both TB6612FNG channels forward for 10 seconds. A reboot never repeats an attempted separation; it resumes at State 5 with `PS=2` (complete) or `PS=3` (aborted). State 5 stops unless the goal, GNSS fix, and magnetometer heading are valid. It drives straight inside the heading deadband, turns with only the outside wheel, stops inside 5 m, confirms for 5 seconds, and enters State 6. State 7 always stops both motors.

Motor mapping is defined in `include/BoardConfig.h`. Current defaults assume motor A is the right wheel and both channels move forward with IN1 high / IN2 low. Confirm wheel assignment and polarity with the vehicle lifted before flight; change only these booleans if wiring differs.

Telemetry additions: `GC` goal configured, `GL` goal latitude, `GO` goal longitude, `GD` distance metres, `MC` motor state (`0 stop`, `1 forward`, `2 left`, `3 right`), and `PS` separation state (`0 not started`, `1 active`, `2 complete`, `3 aborted`).
## Manual phase recovery commands

WCPP `AC=7` (`CONFIRM_DEPLOYED`) is accepted only in Mission Mode while State 2 is active and advances to State 3. `AC=8` (`CONFIRM_LANDED`) is accepted in Mission Mode from State 2 or 3, or in Development Mode from State 0 for a lifted ground test. Entering State 4 immediately starts the one-time 10 second separation motor sequence. Invalid mode/state combinations are rejected by the vehicle. Flight-restart State 7 recovery continues to use `AC=5` (`RESUME_LANDED`).