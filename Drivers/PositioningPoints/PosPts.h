/*
 * PosPts.h
 *
 *  Created on: 31-Mar-2023
 *      Author: harsha
 */

#ifndef POSPTS_H_
#define POSPTS_H_

#include "GB.h"
#include "Constants.h"

#define MIN_POSITION_FROM_HOMING_POSITION_MM 8
#define MAX_POSITION_FROM_HOMING_POSITION_MM 300
#define HOMING_POSITION_DUTY_CYCLE 12.5

#define HOMING_VELOCITY 2 // mm/s
#define MIN_ZERO_POSITION 1500 //is usually around 900

typedef struct PosPointsStruct {

	uint16_t homingPositionCnts;
	float homingPositionDutyCycle;
	uint16_t maxLimit_positionCnts;
	uint16_t minLimit_positionCnts;

	float homingDistance;
	uint16_t homingDirection;
	float homingTime;
	uint8_t homingControlType;
	uint16_t alreadyAtHome_Flag;
} PosPoints;

extern PosPoints posPts;

void initPosPts(PosPoints *p);
void setupLeadScrewLimitsAndHoming(PosPoints *p,uint16_t zeroPosition);
uint8_t checkWithinLeadScrewLimits(PosPoints *p,uint16_t currentPosition);
void calculateHomeMove(PosPoints *p,uint16_t currentPosition);
void resetHomeCalculations(PosPoints *p);
uint8_t checkHomingPosition(uint16_t homingPosCnts);



#endif /* POSPTS_H_ */
