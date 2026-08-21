#include <Arduino.h>
#include "tasks/MissionTask.h"
#include "tasks/TaskHealth.h"
#include "MissionController.h"
#include "MissionConfig.h"
namespace{volatile bool commandPending=false;volatile uint8_t pendingCommand=0;}
bool submit_mission_command(uint8_t c){if(!is_valid_mission_command(c))return false;noInterrupts();pendingCommand=c;commandPending=true;interrupts();return true;}
void MissionTask(void*){MissionController controller;controller.begin();for(;;){uint32_t now=millis();if(commandPending){noInterrupts();uint8_t c=pendingCommand;commandPending=false;interrupts();controller.handleCommand(c,now);}CanSatData_t d{};if(get_cansat_data(&d))controller.update(d,now);task_health_heartbeat(TaskId::MISSION);vTaskDelay(pdMS_TO_TICKS(MissionConfig::TASK_PERIOD_MS));}}
