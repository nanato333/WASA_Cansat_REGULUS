#pragma once

#include <Arduino.h>

// 地上局のSTフィールドと共通で使用するミッション状態。
// 数値は通信仕様になるため、既存値を変更するときは地上局側も合わせる。
enum class MissionState : uint8_t {
    ROCKET_LOADED = 0,
    LAUNCH = 1,
    DEPLOYED = 2,
    DESCENT = 3,
    LANDED = 4,
    RUNNING = 5,  // 将来の走行用。現在の実飛行判定では自動遷移しない。
    STOPPED = 6
};

const char *mission_state_name(MissionState state);