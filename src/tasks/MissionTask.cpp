#include <Arduino.h>
#include "tasks/MissionTask.h"
#include "tasks/TaskHealth.h"
#include "MissionController.h"
#include "MissionConfig.h"
namespace{volatile bool pending=false;WcppTelemetry::Command queued;}
bool submit_mission_command(const WcppTelemetry::Command&c){if(!is_valid_mission_command(c.action))return false;noInterrupts();queued=c;pending=true;interrupts();return true;}
bool submit_mission_command(uint8_t c){WcppTelemetry::Command x;x.action=c;return submit_mission_command(x);}
void MissionTask(void*){MissionController m;m.begin();for(;;){uint32_t now=millis();if(pending){noInterrupts();auto c=queued;pending=false;interrupts();m.handleCommand(c.action,now,c.hasGoalCoordinate,c.goalLatitude,c.goalLongitude);}CanSatData_t d{};if(get_cansat_data(&d))m.update(d,now);task_health_heartbeat(TaskId::MISSION);vTaskDelay(pdMS_TO_TICKS(MissionConfig::TASK_PERIOD_MS));}}
