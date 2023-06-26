/*
 * EncPos.c
 *
 *  Created on: 19-Apr-2023
 *      Author: harsha
 */

#include "EncPos.h"

void EncPos_CalculateMovement(EncPos_TypeDef *encPos,EncSpeed_TypeDef *encSpeed){
	encPos->rps = encSpeed->speedRPM/60.0;
	encPos->strokeVelocity_mm_sec = (encPos->rps/LIFT_MOTOR_GB) * LEAD_SCREW_PITCH;
	encPos->strokeMovement_mm  += encPos->strokeVelocity_mm_sec * CONTROL_LOOP_TIME;
}


void EncPos_ZeroMovement(EncPos_TypeDef *encPos){
	encPos->strokeMovement_mm = 0;
	encPos->error_with_GB = 0;
}


void encPos_CalculateErrorWithGB_OL(EncPos_TypeDef *encPos,PosOL_TypeDef *p){
	encPos->error_with_GB = p->currentMoveDist_mm - encPos->strokeMovement_mm;
}

void encPos_CalculateErrorWithGB_CL(EncPos_TypeDef *encPos,PosCL_TypeDef *p){
	encPos->error_with_GB = p->currentMoveDist_mm - encPos->strokeMovement_mm;
}

