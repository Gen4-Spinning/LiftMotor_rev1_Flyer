/*
 * PosCntrl.h
 *
 *  Created on: Mar 25, 2023
 *      Author: harsha
 */

#ifndef POSRAMP_H_
#define POSRAMP_H_

#define CALLING_TIME_MS 40
#define MIN_VELOCITY 1

#define VELOCITY_RAMPUP 1
#define VELOCITY_CRUISE 2
#define VELOCITY_RAMPDOWN 3
#define VELOCITY_PAUSE 4
#define POS_LOOP_OVER 5
#define POS_LOOP_IDLE 6
#define POS_CYCLES_OVER 7

typedef struct PosRampStruct{

	uint16_t rampUpTimems_Const;
	uint16_t rampDownTimems_Const;
	float callingTimeSec_Const;

	float 	 moveDistance_mm;
	float    moveTime_s;
	uint8_t  moveDirection;

	float 	 currentMoveTarget_mm;
	float    currentMoveVelocity;

	float    pkVelocity;

	char 	 rampPhase;
	float    dist_cruisePhase;
	float    dist_RU;
	float    dist_RD;

	float    dV_RU;
	float    dV_RD;
	float    dS_cruise;

	float    endDist_RU;
	float    endDist_cruise;
	float    endDist_RD;

}PosRamp;


extern PosRamp posRamp;

void posRamp_Reset(PosRamp *p);
void posRamp_SetupRampTimes(PosRamp *p,uint16_t rampUpms,uint16_t rampDownms);
void posRamp_SetupMove(PosRamp *p,float targetDistance, float targetTime,uint8_t direction);

void posRamp_updateRDTime(PosRamp *p , uint16_t RD_time);

void posRamp_Start(PosRamp *p);
void posRamp_Stop(PosRamp *p);
void posRamp_Exec(PosRamp *p);







#endif /* POSRAMP_H_ */
