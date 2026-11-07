#include <Arduino.h>
#include "tasks/MissionTask.h"
#include "tasks/TaskHealth.h"
#include "MissionController.h"
#include "MissionConfig.h"
#ifdef ENABLE_RC
#include "BoardConfig.h"
#include "MotorController.h"
#include <math.h>

namespace {
constexpr uint8_t ARM=9, STOP=10, FORWARD=11, LEFT=12, RIGHT=13, NEUTRAL=14;
constexpr uint32_t COMMAND_TIMEOUT_MS=300;
constexpr float RC_LOW_VOLTAGE=3.0f;
constexpr uint32_t RC_LOW_CONFIRM_MS=200;
constexpr float RC_RECOVERY_VOLTAGE=3.3f;
constexpr uint32_t RC_RECOVERY_MS=2000;
portMUX_TYPE rcMux=portMUX_INITIALIZER_UNLOCKED;
uint8_t pendingAction=0;
int8_t pendingLeft=0, pendingRight=0;
uint32_t pendingAt=0;
}

bool submit_mission_command(const WcppTelemetry::Command &command) {
    if (command.action < ARM || command.action > WcppTelemetry::RC_DRIVE) return false;
    portENTER_CRITICAL(&rcMux);
    if (pendingAction != STOP || command.action == STOP) {
        pendingAction=command.action;
        pendingLeft=command.left;
        pendingRight=command.right;
        pendingAt=millis();
    }
    portEXIT_CRITICAL(&rcMux);
    return true;
}

bool submit_mission_command(uint8_t action) {
    WcppTelemetry::Command command{};
    command.action=action;
    return submit_mission_command(command);
}

void MissionTask(void *) {
    MotorController motor;
    motor.begin();
    pinMode(BoardConfig::LED_PIN, OUTPUT);
    digitalWrite(BoardConfig::LED_PIN, LOW);
    bool armed=false, lowLocked=false;
    bool lowTiming=false;
    uint32_t lowStarted=0;
    bool recoveryTiming=false;
    uint32_t recoveryStarted=0;
    uint32_t lastCommand=0;
    uint8_t lastReported=255;
    // MO=3 identifies the dedicated RC firmware to the ground station.
    update_mission_status(MissionState::BOOT, MissionSubState::NONE, static_cast<OperationMode>(3));
    for (;;) {
        portENTER_CRITICAL(&rcMux);
        const uint8_t action=pendingAction;
        const int8_t left=pendingLeft, right=pendingRight;
        const uint32_t received=pendingAt;
        pendingAction=0;
        portEXIT_CRITICAL(&rcMux);
        const uint32_t now=millis();
        CanSatData_t data{};
        const bool available=get_cansat_data(&data);
        const float voltage=data.sys.battery_voltage;
        const bool measured=available && isfinite(voltage) && voltage>0 && voltage<=5.0f;
        const bool belowThreshold=!lowLocked && measured && voltage<RC_LOW_VOLTAGE;
        if (!belowThreshold) {
            lowTiming=false;
        } else if (!lowTiming) {
            lowTiming=true;
            lowStarted=now;
        }
        if (lowTiming && now-lowStarted>=RC_LOW_CONFIRM_MS) {
            lowLocked=true;
            motor.stop();
            update_failsafe_reason(FailsafeReason::LOW_BATTERY);
            update_mission_status(MissionState::FAILSAFE, MissionSubState::NONE, static_cast<OperationMode>(3));
        }
        bool justRecovered=false;
        if (lowLocked) {
            if (!measured || voltage<RC_RECOVERY_VOLTAGE) {
                recoveryTiming=false;
            } else if (!recoveryTiming) {
                recoveryTiming=true;
                recoveryStarted=now;
            } else if (now-recoveryStarted>=RC_RECOVERY_MS) {
                lowLocked=false;
                recoveryTiming=false;
                justRecovered=true;
                update_failsafe_reason(FailsafeReason::NONE);
                update_mission_status(MissionState::BOOT, MissionSubState::NONE, static_cast<OperationMode>(3));
            }
        }
        if (lowLocked || justRecovered || !measured || action==STOP || now-lastCommand>=COMMAND_TIMEOUT_MS) {
            armed=false;
            motor.stop();
        }
        if (!lowLocked && !justRecovered && measured && now-received<COMMAND_TIMEOUT_MS) {
            if (action==ARM) {
                motor.stop();
                armed=true;
                lastCommand=received;
            } else if (armed && action>=FORWARD && action<=WcppTelemetry::RC_DRIVE) {
                lastCommand=received;
                switch (action) {
                    case FORWARD: motor.driveRc(15, 15); break;
                    case LEFT: motor.driveRc(0, 15); break;
                    case RIGHT: motor.driveRc(15, 0); break;
                    case WcppTelemetry::RC_DRIVE: motor.driveRc(left, right); break;
                    default: motor.stop(); break;
                }
            }
        }
        digitalWrite(BoardConfig::LED_PIN, lowLocked && (now/250)%2 ? HIGH : LOW);
        const auto drive=static_cast<uint8_t>(motor.state());
        if (drive!=lastReported && update_navigation_status(false,0,0,-1,drive,0)) lastReported=drive;
        task_health_heartbeat(TaskId::MISSION);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#else
namespace{volatile bool pending=false;WcppTelemetry::Command queued;}
bool submit_mission_command(const WcppTelemetry::Command&c){if(!is_valid_mission_command(c.action))return false;noInterrupts();queued=c;pending=true;interrupts();return true;}
bool submit_mission_command(uint8_t c){WcppTelemetry::Command x;x.action=c;return submit_mission_command(x);}
void MissionTask(void*){MissionController m;m.begin();for(;;){uint32_t now=millis();if(pending){noInterrupts();auto c=queued;pending=false;interrupts();m.handleCommand(c.action,now,c.hasGoalCoordinate,c.goalLatitude,c.goalLongitude);}CanSatData_t d{};if(get_cansat_data(&d))m.update(d,now);task_health_heartbeat(TaskId::MISSION);vTaskDelay(pdMS_TO_TICKS(MissionConfig::TASK_PERIOD_MS));}}
#endif
