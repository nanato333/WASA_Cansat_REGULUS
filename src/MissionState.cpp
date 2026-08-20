#include "MissionState.h"

const char *mission_state_name(MissionState state) {
    switch (state) {
        case MissionState::ROCKET_LOADED: return "ROCKET_LOADED";
        case MissionState::LAUNCH: return "LAUNCH";
        case MissionState::DEPLOYED: return "DEPLOYED";
        case MissionState::DESCENT: return "DESCENT";
        case MissionState::LANDED: return "LANDED";
        case MissionState::RUNNING: return "RUNNING";
        case MissionState::STOPPED: return "STOPPED";
        default: return "UNKNOWN";
    }
}