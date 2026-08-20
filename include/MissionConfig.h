#pragma once

#include <Arduino.h>

// 実験結果に応じて調整するミッション判定値を一か所にまとめる。
// 現在値は暫定値であり、実飛行前に加速度・気圧ログから再評価すること。
namespace MissionConfig {
constexpr uint32_t TASK_PERIOD_MS = 50;

// 打ち上げ判定：合成加速度が閾値を一定時間超えた場合。
constexpr float LAUNCH_ACCEL_MPS2 = 15.0f;
constexpr uint32_t LAUNCH_CONFIRM_MS = 300;

// 放出推定：打ち上げ直後の誤判定を避け、低加速度状態の継続を確認する。
constexpr uint32_t MIN_LAUNCH_TO_DEPLOY_MS = 3000;
constexpr float DEPLOY_FREEFALL_ACCEL_MPS2 = 4.0f;
constexpr uint32_t DEPLOY_CONFIRM_MS = 400;

// 降下判定：気圧高度から計算した鉛直速度を使用する。
constexpr float DESCENT_SPEED_MPS = -0.8f;
constexpr uint32_t DESCENT_CONFIRM_MS = 1000;

// 着地判定：高度変化・加速度・角速度が同時に安定したことを確認する。
constexpr uint32_t MIN_DESCENT_TO_LANDED_MS = 5000;
constexpr float LANDED_MAX_VERTICAL_SPEED_MPS = 0.35f;
constexpr float LANDED_MIN_ACCEL_MPS2 = 7.0f;
constexpr float LANDED_MAX_ACCEL_MPS2 = 12.5f;
constexpr float LANDED_MAX_GYRO_DPS = 20.0f;
constexpr uint32_t LANDED_CONFIRM_MS = 3000;

// 気圧ノイズを抑える一次ローパスフィルタの係数。
constexpr float VERTICAL_SPEED_FILTER_ALPHA = 0.15f;
}