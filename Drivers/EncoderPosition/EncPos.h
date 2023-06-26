/*
 * EncPos.h
 *
 *  Created on: 19-Apr-2023
 *      Author: harsha
 */

#ifndef ENCPOS_H_
#define ENCPOS_H_

#include "EncSpeed.h"
#include "PosCntrl_CL.h"
#include "PosCntrl_OL.h"


#define LIFT_MOTOR_GB 15.3 //12.5
#define LEAD_SCREW_PITCH 4
#define CONTROL_LOOP_TIME 0.040 // 40 ms

typedef struct EncPos{
	float rps;
	float strokeVelocity_mm_sec;
	float strokeMovement_mm;
	float error_with_GB;
}EncPos_TypeDef;

extern EncPos_TypeDef encPos;

void EncPos_CalculateMovement(EncPos_TypeDef *encPos,EncSpeed_TypeDef *encSpeed);
void EncPos_ZeroMovement(EncPos_TypeDef *encPos);
void encPos_CalculateErrorWithGB_OL(EncPos_TypeDef *encPos,PosOL_TypeDef *p);
void encPos_CalculateErrorWithGB_CL(EncPos_TypeDef *encPos,PosCL_TypeDef *p);

#endif /* ENCODERPOSITION_ENCPOS_H_ */
