/*
 * PID.h
 *
 *  Created on: Mar 11, 2023
 *      Author: harsha
 */

#ifndef PID_H_
#define PID_H_

#include "Struct.h"
#include "EncSpeed.h"
#include "PosCntrl_CL.h"
#include "PosRamp.h"
#include "Constants.h"

typedef struct PID_struct{

	uint16_t FF_percent;
	float Kp_constant;
	float Ki_constant;
	float FF_constant;
	uint16_t startOffsetConstant;

	int16_t error_s16;
	float errorF;
	float Kp_term;
	float Ki_term;

	uint16_t feedForwardTerm;
	uint16_t startOffsetTerm;
	char antiWindup;
	int16_t pwm;

}PID_Typedef;


void InitializePID_TypeDef(PID_Typedef *p);
void setupPID(PID_Typedef *p,float Kp,float Ki,uint16_t FF_Percent,uint16_t so);
void setupPID_LiftMotors(PID_Typedef *p,float Kp,float Ki,uint16_t FF_Percent,uint16_t so);
void ExecPID_PosLift(PID_Typedef *p, PosRamp *posRamp,PosCL_TypeDef *posCL);
void resetPID(PID_Typedef *p);

extern PID_Typedef PIDpos;


#endif /* PID_H_ */
