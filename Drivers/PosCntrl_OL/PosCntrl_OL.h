/*
 * posCntrl_OL.h
 *
 *  Created on: 31-Mar-2023
 *      Author: harsha
 */

#ifndef POSCNTRL_OL_H_
#define POSCNTRL_OL_H_

#include "stdio.h"
#include "Constants.h"

#define MOTOR_ENCODER_MM_PER_ROT 0.16
#define MOTOR_ENCODER_CNTS_PER_MM 512  //2048 counts per rotation.

typedef struct PositioningOpenLoop_struct {
	uint16_t moveDuty;
	uint8_t moveDirection;
	float moveDist_mm;

	float GB_absEndPosition;
	float GB_absStartPosition;
    float GB_absCurrentPosition;

    float currentMoveDist_mm;

    char targetSettingError;
    uint8_t targetReached;

} PosOL_TypeDef;

void posOL_Reset(PosOL_TypeDef *p);
void posOL_SetupMove(PosOL_TypeDef *p,float startPosition,float moveDist,uint8_t moveDir,uint16_t moveDuty);
void posOL_ClearMove(PosOL_TypeDef *p);
void posOL_CheckTargetReached(PosOL_TypeDef *p);
void posOL_CalcMoveDistance(PosOL_TypeDef *p,float currentDist);



#endif /* POSCNTRL_OL_H_ */
