#include "MissionController.h"
#include <Preferences.h>
#include <math.h>
#include "MissionConfig.h"

namespace {
constexpr char NS[]="mission", K_STATE[]="state", K_MODE[]="mode", K_FS[]="failsafe";
constexpr char K_GSET[]="goal_set",K_GLAT[]="goal_lat",K_GLON[]="goal_lon",K_STRY[]="sep_try",K_SDONE[]="sep_done";
constexpr double R=6371000.0;
void saveByte(const char*k,uint8_t v){Preferences p;if(p.begin(NS,false)){p.putUChar(k,v);p.end();}}
void saveBool(const char*k,bool v){Preferences p;if(p.begin(NS,false)){p.putBool(k,v);p.end();}}
float distanceM(double a,double b,double c,double d){double p=a*DEG_TO_RAD,q=c*DEG_TO_RAD,x=(c-a)*DEG_TO_RAD,y=(d-b)*DEG_TO_RAD,z=sin(x/2)*sin(x/2)+cos(p)*cos(q)*sin(y/2)*sin(y/2);return R*2*atan2(sqrt(z),sqrt(1-z));}
float bearing(double a,double b,double c,double d){double y=sin((d-b)*DEG_TO_RAD)*cos(c*DEG_TO_RAD),x=cos(a*DEG_TO_RAD)*sin(c*DEG_TO_RAD)-sin(a*DEG_TO_RAD)*cos(c*DEG_TO_RAD)*cos((d-b)*DEG_TO_RAD);float v=atan2(y,x)*RAD_TO_DEG;return v<0?v+360:v;}
float signedAngle(float v){while(v>180)v-=360;while(v<-180)v+=360;return v;}
bool restorable(MissionState s){return s==MissionState::BOOT||s==MissionState::LAUNCH_STANDBY||s==MissionState::LANDED||s==MissionState::RUNNING||s==MissionState::FINISHED||s==MissionState::FAILSAFE;}
}

void MissionController::begin(){
 motor_.begin(); Preferences p;uint8_t s=0,m=0,f=0;
 if(p.begin(NS,true)){s=p.getUChar(K_STATE,0);m=p.getUChar(K_MODE,0);f=p.getUChar(K_FS,0);goalConfigured_=p.getBool(K_GSET,MissionConfig::GOAL_COORDINATE_CONFIGURED);goalLatitude_=p.getDouble(K_GLAT,MissionConfig::GOAL_LATITUDE);goalLongitude_=p.getDouble(K_GLON,MissionConfig::GOAL_LONGITUDE);separationAttempted_=p.getBool(K_STRY,false);separationCompleted_=p.getBool(K_SDONE,false);p.end();}
 mode_=m<=2?(OperationMode)m:OperationMode::UNSELECTED;MissionState candidate=s<=7?(MissionState)s:MissionState::BOOT;
 state_=restorable(candidate)?candidate:MissionState::FAILSAFE;failsafeReason_=state_==MissionState::FAILSAFE?(restorable(candidate)?(FailsafeReason)f:FailsafeReason::UNSAFE_FLIGHT_RESTART):FailsafeReason::NONE;
 if(state_==MissionState::LANDED&&separationAttempted_){state_=MissionState::RUNNING;separationStatus_=separationCompleted_?2:3;saveByte(K_STATE,(uint8_t)state_);}
 subState_=state_==MissionState::RUNNING?MissionSubState::GPS_NAVIGATION:MissionSubState::NONE;stateStartedMs_=subStateStartedMs_=millis();
 update_mission_status(state_,subState_,mode_);update_failsafe_reason(failsafeReason_);update_navigation_status(goalConfigured_,goalLatitude_,goalLongitude_,-1,(uint8_t)motor_.state(),separationStatus_);
}

bool MissionController::handleCommand(uint8_t c,uint32_t now,bool hasGoal,double lat,double lon){
 if(!is_valid_mission_command(c))return false;MissionCommand cmd=(MissionCommand)c;
 if(cmd==MissionCommand::SET_GOAL_COORDINATE){if(!hasGoal||!isfinite(lat)||!isfinite(lon)||lat<-90||lat>90||lon<-180||lon>180||!(state_==MissionState::BOOT||state_==MissionState::LAUNCH_STANDBY||state_==MissionState::LANDED))return false;Preferences p;if(!p.begin(NS,false))return false;p.putBool(K_GSET,true);p.putDouble(K_GLAT,lat);p.putDouble(K_GLON,lon);p.end();goalConfigured_=true;goalLatitude_=lat;goalLongitude_=lon;update_navigation_status(true,lat,lon,-1,(uint8_t)motor_.state(),separationStatus_);return true;}
 if(cmd==MissionCommand::RESUME_LANDED){if(state_!=MissionState::FAILSAFE||failsafeReason_!=FailsafeReason::UNSAFE_FLIGHT_RESTART||mode_!=OperationMode::MISSION)return false;failsafeReason_=FailsafeReason::NONE;saveByte(K_FS,0);transitionTo(MissionState::LANDED,now);return true;}
 if(cmd==MissionCommand::CLEAR_FAILSAFE){if(state_!=MissionState::FAILSAFE||failsafeReason_!=FailsafeReason::LOW_BATTERY||!failsafeRecoveryReady_)return false;mode_=OperationMode::UNSELECTED;failsafeReason_=FailsafeReason::NONE;saveByte(K_MODE,0);saveByte(K_FS,0);transitionTo(MissionState::BOOT,now);return true;}
 if(cmd==MissionCommand::START_LAUNCH){if(mode_!=OperationMode::MISSION||state_!=MissionState::LAUNCH_STANDBY)return false;transitionTo(MissionState::LAUNCH,now);return true;}
 if(state_!=MissionState::BOOT)return false;mode_=cmd==MissionCommand::SELECT_MISSION_MODE?OperationMode::MISSION:OperationMode::DEVELOPMENT;saveByte(K_MODE,(uint8_t)mode_);separationAttempted_=separationCompleted_=false;saveBool(K_STRY,false);saveBool(K_SDONE,false);update_mission_status(state_,subState_,mode_);if(mode_==OperationMode::MISSION)transitionTo(MissionState::LAUNCH_STANDBY,now);return true;
}

bool MissionController::conditionHeld(bool c,uint32_t n,uint32_t ms){if(!c){conditionStartedMs_=0;return false;}if(!conditionStartedMs_)conditionStartedMs_=n;return n-conditionStartedMs_>=ms;}
void MissionController::setSubState(MissionSubState s,uint32_t n){if(s==subState_)return;subState_=s;subStateStartedMs_=n;update_mission_status(state_,subState_,mode_);}
void MissionController::enterFailsafe(FailsafeReason r,uint32_t n){motor_.stop();separationRunning_=false;failsafeReason_=r;saveByte(K_FS,(uint8_t)r);transitionTo(MissionState::FAILSAFE,n);}
void MissionController::transitionTo(MissionState s,uint32_t n){if(s==state_)return;if(s!=MissionState::LANDED&&s!=MissionState::RUNNING)motor_.stop();state_=s;stateStartedMs_=n;conditionStartedMs_=0;subState_=s==MissionState::RUNNING?MissionSubState::GPS_NAVIGATION:MissionSubState::NONE;update_mission_status(state_,subState_,mode_);update_failsafe_reason(failsafeReason_);saveByte(K_STATE,(uint8_t)s);}
void MissionController::updateVerticalSpeed(const CanSatData_t&d,uint32_t n){if(!d.baro.is_valid){altitudeInitialized_=false;return;}if(!altitudeInitialized_){altitudeInitialized_=true;previousAltitude_=d.baro.altitude;previousAltitudeMs_=n;return;}if(n==previousAltitudeMs_)return;float v=(d.baro.altitude-previousAltitude_)/((n-previousAltitudeMs_)/1000.f);filteredVerticalSpeed_+=MissionConfig::VERTICAL_SPEED_FILTER_ALPHA*(v-filteredVerticalSpeed_);previousAltitude_=d.baro.altitude;previousAltitudeMs_=n;}
MissionSubState MissionController::preferredRunningSubState(const CanSatData_t&)const{return MissionSubState::GPS_NAVIGATION;}

void MissionController::updateRunningState(const CanSatData_t&d,uint32_t n){
 if(!goalConfigured_||!d.gnss.is_valid||!d.gnss.fix||!d.mag.is_valid){motor_.stop();goalStartedMs_=0;update_navigation_status(goalConfigured_,goalLatitude_,goalLongitude_,-1,(uint8_t)motor_.state(),separationStatus_);return;}
 float dist=distanceM(d.gnss.latitude,d.gnss.longitude,goalLatitude_,goalLongitude_);if(dist<=MissionConfig::GOAL_RADIUS_M){motor_.stop();if(!goalStartedMs_)goalStartedMs_=n;if(n-goalStartedMs_>=MissionConfig::GOAL_CONFIRM_MS)transitionTo(MissionState::FINISHED,n);}else{goalStartedMs_=0;float e=signedAngle(bearing(d.gnss.latitude,d.gnss.longitude,goalLatitude_,goalLongitude_)-d.mag.heading);if(e>MissionConfig::NAVIGATION_HEADING_DEADBAND_DEG)motor_.turnRight();else if(e<-MissionConfig::NAVIGATION_HEADING_DEADBAND_DEG)motor_.turnLeft();else motor_.forward();}update_navigation_status(true,goalLatitude_,goalLongitude_,dist,(uint8_t)motor_.state(),separationStatus_);
}

void MissionController::update(const CanSatData_t&d,uint32_t n){
 bool measured=isfinite(d.sys.battery_voltage)&&d.sys.battery_voltage>=MissionConfig::BATTERY_VALID_MIN_VOLTAGE&&d.sys.battery_voltage<=MissionConfig::BATTERY_VALID_MAX_VOLTAGE,low=measured&&d.sys.battery_voltage<=MissionConfig::LOW_BATTERY_VOLTAGE;
 if(low){if(!lowBatteryStartedMs_)lowBatteryStartedMs_=n;if(n-lowBatteryStartedMs_>=MissionConfig::LOW_BATTERY_CONFIRM_MS){enterFailsafe(FailsafeReason::LOW_BATTERY,n);return;}}else lowBatteryStartedMs_=0;
 if(state_==MissionState::FAILSAFE){motor_.stop();bool ok=measured&&d.sys.battery_voltage>=MissionConfig::FAILSAFE_RECOVERY_VOLTAGE;if(!ok){failsafeRecoveryStartedMs_=0;failsafeRecoveryReady_=false;}else{if(!failsafeRecoveryStartedMs_)failsafeRecoveryStartedMs_=n;failsafeRecoveryReady_=n-failsafeRecoveryStartedMs_>=MissionConfig::FAILSAFE_RECOVERY_CONFIRM_MS;}return;}if(mode_!=OperationMode::MISSION){motor_.stop();return;}
 updateVerticalSpeed(d,n);float a=sqrtf(d.imu.ax*d.imu.ax+d.imu.ay*d.imu.ay+d.imu.az*d.imu.az),g=sqrtf(d.imu.gx*d.imu.gx+d.imu.gy*d.imu.gy+d.imu.gz*d.imu.gz);
 switch(state_){case MissionState::LAUNCH_STANDBY:if(conditionHeld(d.imu.is_valid&&a>=MissionConfig::LAUNCH_ACCEL_MPS2,n,MissionConfig::LAUNCH_CONFIRM_MS))transitionTo(MissionState::LAUNCH,n);break;case MissionState::LAUNCH:if(conditionHeld(d.imu.is_valid&&n-stateStartedMs_>=MissionConfig::MIN_LAUNCH_TO_DEPLOY_MS&&a<=MissionConfig::DEPLOY_FREEFALL_ACCEL_MPS2,n,MissionConfig::DEPLOY_CONFIRM_MS))transitionTo(MissionState::DEPLOYED,n);break;case MissionState::DEPLOYED:if(conditionHeld(n-stateStartedMs_>=MissionConfig::MIN_DEPLOYED_TO_LANDED_MS&&d.baro.is_valid&&d.imu.is_valid&&altitudeInitialized_&&fabsf(filteredVerticalSpeed_)<=MissionConfig::LANDED_MAX_VERTICAL_SPEED_MPS&&a>=MissionConfig::LANDED_MIN_ACCEL_MPS2&&a<=MissionConfig::LANDED_MAX_ACCEL_MPS2&&g<=MissionConfig::LANDED_MAX_GYRO_DPS,n,MissionConfig::LANDED_CONFIRM_MS))transitionTo(MissionState::LANDED,n);break;case MissionState::LANDED:if(!separationRunning_){separationAttempted_=true;saveBool(K_STRY,true);separationRunning_=true;separationStartedMs_=n;separationStatus_=1;motor_.forward();}if(n-separationStartedMs_>=MissionConfig::SEPARATION_HOLD_MS){motor_.stop();separationRunning_=false;separationCompleted_=true;separationStatus_=2;saveBool(K_SDONE,true);transitionTo(MissionState::RUNNING,n);}update_navigation_status(goalConfigured_,goalLatitude_,goalLongitude_,-1,(uint8_t)motor_.state(),separationStatus_);break;case MissionState::RUNNING:updateRunningState(d,n);break;case MissionState::FINISHED:motor_.stop();if(n-stateStartedMs_>=MissionConfig::FINISHED_HOLD_MS){mode_=OperationMode::UNSELECTED;saveByte(K_MODE,0);transitionTo(MissionState::BOOT,n);}break;default:motor_.stop();break;}
}
