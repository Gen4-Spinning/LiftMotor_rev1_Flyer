/*
 * PosCntrl_CL.c
 *
 *  Created on: 31-Mar-2023
 *      Author: harsha
 */


#include "PosCntrl_CL.h"

void posCL_Reset(PosCL_TypeDef *p){
	p->moveTime = 0;
	p->moveDirection = 0;
	p->moveDist_mm = 0;

	p->GB_absEndPosition = 0;
	p->GB_absStartPosition = 0;
	p->GB_absCurrentPosition = 0;

	p->currentMoveDist_mm = 0;
	p->positioningError_mm = 0;
	p->remainingDistance_mm = 0;
	p->pkVelocity = 0 ;

	p->targetSettingError = 0;
	p->targetReached = 0;
}


void posCL_SetupMove(PosCL_TypeDef *p,float startPosition,float moveDist,uint8_t moveDir,float moveTime){
	p->moveTime = moveTime;
	p->moveDist_mm = moveDist;
	p->moveDirection = moveDir;
	p->GB_absStartPosition = startPosition;
	if (p->moveDirection == MOVEUP){
		p->GB_absEndPosition = startPosition + moveDist;
	}
	if (p->moveDirection == MOVEDOWN){
		p->GB_absEndPosition = startPosition - moveDist;
	}
	//TODO set the error if the limits are too much.

	p->currentMoveDist_mm = 0;
	p->positioningError_mm = 0;
	p->remainingDistance_mm = p->moveDist_mm;
	p->targetReached = 0;
}

void posCL_ClearMove(PosCL_TypeDef *p){
	p->moveTime = 0;
	p->moveDist_mm = 0;
	p->moveDirection = 0;
	p->positioningError_mm = 0;
}

void posCL_setPkVelocity(PosCL_TypeDef *p,float velocity){
	p->pkVelocity = velocity;
}

void posCL_CalcMoveDistance(PosCL_TypeDef *p,float currentDist){

	p->GB_absCurrentPosition = currentDist;

	if (p->moveDirection == MOVEUP){
		p->currentMoveDist_mm = p->GB_absCurrentPosition - p->GB_absStartPosition;
	}else if(p->moveDirection == MOVEDOWN){
		p->currentMoveDist_mm = p->GB_absStartPosition - p->GB_absCurrentPosition;
	}
	p->remainingDistance_mm = p->moveDist_mm - p->currentMoveDist_mm;
}

float RecalculateTime_OnResume(PosCL_TypeDef *p,LiftRunMgmtTypeDef *lrm){
	float steadyState_distance = 0;
	float steadyStateTime = 0;
	float totalTime = 0;


	float rampUp_distance = 0.5 * lrm->rampUpTime_ms/1000.0 * p->pkVelocity;
	float rampDown_distance = 0.5 * lrm->directionChangeRamp_ms/1000.0 * p->pkVelocity;
	if (p->remainingDistance_mm > (rampUp_distance + rampDown_distance)){
		steadyState_distance = p->remainingDistance_mm - rampUp_distance - rampDown_distance;
		steadyStateTime = steadyState_distance/p->pkVelocity;
		totalTime = steadyStateTime + lrm->rampUpTime_ms/1000.0 + lrm->directionChangeRamp_ms/1000.0;
	}else{
		//TODO - need to see what to do.
		totalTime = lrm->rampUpTime_ms/1000.0 + lrm->directionChangeRamp_ms/1000.0;
	}

	return totalTime;
}


