#pragma once

#include "CanSatData.h"
#include "MotorController.h"

class MissionController {
public:
    void begin();
    void update(const CanSatData_t &data, uint32_t now);
    bool handleCommand(uint8_t command, uint32_t now, bool hasGoal=false, double goalLatitude=0, double goalLongitude=0);
    MissionState state() const { return state_; }

private:
    void transitionTo(MissionState next, uint32_t now);
    void enterFailsafe(FailsafeReason reason, uint32_t now);
    void setSubState(MissionSubState next, uint32_t now);
    bool conditionHeld(bool condition, uint32_t now, uint32_t requiredMs);
    void updateVerticalSpeed(const CanSatData_t &data, uint32_t now);
    void updateRunningState(const CanSatData_t &data, uint32_t now);
    MissionSubState preferredRunningSubState(const CanSatData_t &data) const;

    MissionState state_ = MissionState::BOOT;
    MissionSubState subState_ = MissionSubState::NONE;
    OperationMode mode_ = OperationMode::UNSELECTED;
    FailsafeReason failsafeReason_ = FailsafeReason::NONE;
    uint32_t stateStartedMs_ = 0;
    uint32_t subStateStartedMs_ = 0;
    uint32_t conditionStartedMs_ = 0;
    uint32_t lowBatteryStartedMs_ = 0;
    uint32_t failsafeRecoveryStartedMs_ = 0;
    bool failsafeRecoveryReady_ = false;
    uint32_t previousAltitudeMs_ = 0;
    bool altitudeInitialized_ = false;
    float previousAltitude_ = 0.0f;
    float filteredVerticalSpeed_ = 0.0f;

    bool motionBaselineInitialized_ = false;
    double motionLatitude_ = 0.0;
    double motionLongitude_ = 0.0;
    float motionHeading_ = 0.0f;
    int motionRssi_ = 0;
    uint32_t motionBaselineMs_ = 0;
    uint32_t stackStartedMs_ = 0;
    uint32_t goalStartedMs_ = 0;
    MotorController motor_;
    bool goalConfigured_=false, separationAttempted_=false, separationCompleted_=false, separationRunning_=false;
    double goalLatitude_=0, goalLongitude_=0;
    uint32_t separationStartedMs_=0;
    uint8_t separationStatus_=0;
};
