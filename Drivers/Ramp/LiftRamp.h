/*
 * LiftRamp.h
 *
 *  Created on: Mar 30, 2023
 *      Author: harsha
 */

#ifndef LIFTRAMP_H_
#define LIFTRAMP_H_

#include "stm32g4xx_hal.h"
#include "Struct.h"

#define RAMP_UP 0
#define RAMP_DOWN 1
#define RAMP_STEADY 2
#define RAMP_OVER 3
#define RAMP_WAIT 4
#define EOS_RAMPDOWN 5
#define BOS_RAMPUP 6
#define LIFT_CONFIRM_STOP 7
#define RAMP_CHANGE 8

typedef struct LiftRampDutyStruct {
  uint16_t finalTargetDuty;
  uint16_t currentDuty;
  float currentDutyF;
  float dDuty_F_RU;
  float dDuty_F_RD;
  float dDuty_F_EOS;
  float dDuty_F_BOS;
  int16_t steadyRunTime_s;
  uint16_t rampUpTime_ms;
  uint16_t rampDownTime_ms;
  uint16_t EOS_DownTime_ms;
  uint16_t BOS_UpTime_ms;
  uint16_t ramp_callingTime_ms;
  uint8_t rampPhase;
}LiftRampDuty;


extern LiftRampDuty liftRampDuty;
void InitLiftRampDuty(LiftRampDuty *r);
void SetupLiftRampDuty(LiftRampDuty *ramp,uint16_t targetDuty,uint16_t rampUpTime,uint16_t rampDownTime,int16_t rampSteadyTime,uint16_t BOS_rampUpTime, uint16_t EOS_rampDownTime);
void StartLiftRampDuty(LiftRampDuty *ramp);
void StopLiftRampDuty(LiftRampDuty *ramp);
void ResetLiftRampDuty(LiftRampDuty *ramp);
void ExecLiftRampDuty(LiftRampDuty *ramp,TimerTypeDef *t);

/*
 * REMOVE LIFT RPM CLOSED LOOP SINCE WE DONT WANT TO USE IT. WE also remove the LiftRAMPRPM.c file so that
 * it doesnt build and take space.
typedef struct LiftRampRPMStruct {
  uint16_t finalTargetRPM;
  float instTargetRPM_F;
  uint16_t instTargetRPM_s16M;
  float currentDutyF;
  float dRPM_F_RU;
  float dRPM_F_RD;
  float dRPM_F_BOS_RU;
  float dRPM_F_EOS_RD;
  int16_t steadyRunTime_s;
  uint16_t rampUpTime_ms;
  uint16_t rampDownTime_ms;
  uint16_t EOS_DownTime_ms;
  uint16_t BOS_UpTime_ms;
  uint16_t ramp_callingTime_ms;
  uint8_t rampPhase;
}LiftRampRPM;



//extern LiftRampRPM liftRampRpm;
void InitLiftRampRPMStruct(LiftRampRPM *ramp,uint16_t targetRPM,uint16_t rampUpTime,uint16_t rampDownTime,int16_t rampSteadyTime,uint16_t BOS_rampUpTime, uint16_t EOS_rampDownTime);
void StartLiftRampRPM(LiftRampRPM *ramp);
void StopLiftRampRPM(LiftRampRPM *ramp);
void ResetLiftRampRPM(LiftRampRPM *ramp);
void ExecLiftRampRPM(LiftRampRPM *ramp,TimerTypeDef *t);

*/

#endif /* RAMP_LIFTRAMP_H_ */
