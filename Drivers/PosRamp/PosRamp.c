/*
 * PosCntrl.c
 *
 *  Created on: Mar 25, 2023
 *      Author: harsha
 */

#include "stdio.h"
#include "PosRamp.h"

void posRamp_Reset(PosRamp *p){
	p->rampPhase = POS_LOOP_IDLE;
	p->callingTimeSec_Const = CALLING_TIME_MS/1000.0;
	p->currentMoveTarget_mm = 0;
	p->currentMoveVelocity = 0;
	p->dS_cruise = 0;
	p->dV_RD = 0;
	p->dV_RU = 0;
	p->dist_RD = 0;
	p->dist_RU = 0;
	p->dist_cruisePhase = 0;
	p->endDist_RD = 0;
	p->moveDistance_mm = 0;
	p->moveTime_s = 0;
	p->moveDirection = 0;
	p->pkVelocity = 0;

}

void posRamp_SetupRampTimes(PosRamp *p,uint16_t rampUpms,uint16_t rampDownms){
	p->rampUpTimems_Const = rampUpms;
	p->rampDownTimems_Const = rampDownms;
	p->callingTimeSec_Const = (float)CALLING_TIME_MS/1000.0;
}


//Trapezoidal Trajectory ( No Jerk Control - No S profile )
void posRamp_SetupMove(PosRamp *p,float targetDistance, float targetTime,uint8_t direction){

	p->moveDistance_mm = targetDistance;
    p->moveTime_s = targetTime;
    p->moveDirection = direction;

	float steadyStateTime_s = p->moveTime_s - ((float)(p->rampUpTimems_Const)/1000) - ((float)(p->rampDownTimems_Const)/1000);

	float temp = (0.5*(float)(p->rampUpTimems_Const + p->rampDownTimems_Const)/1000) + steadyStateTime_s;
	p->pkVelocity = p->moveDistance_mm/temp;

	uint16_t rampUp_steps = p->rampUpTimems_Const/CALLING_TIME_MS;
	uint16_t rampDown_steps = p->rampDownTimems_Const/CALLING_TIME_MS;
	uint16_t cruise_steps =  (uint16_t)((steadyStateTime_s*1000)/CALLING_TIME_MS);

	p->dV_RU = p->pkVelocity/rampUp_steps;
	p->dV_RD = p->pkVelocity/rampDown_steps;

	p->dist_RU = 0.5 * p->pkVelocity * (float)p->rampUpTimems_Const/1000.0;
	p->dist_RD = 0.5 * p->pkVelocity * (float)p->rampDownTimems_Const/1000.0;;
	p->dist_cruisePhase = p->moveDistance_mm - p->dist_RU - p->dist_RD ;

	p->dS_cruise = p->dist_cruisePhase/cruise_steps;

	p->endDist_cruise = p->dist_cruisePhase + p->dist_RU;
	p->endDist_RD = p->moveDistance_mm;

}


void posRamp_Start(PosRamp *p){
	p->rampPhase = VELOCITY_RAMPUP;
}
void posRamp_Stop(PosRamp *p){
	p->rampPhase = POS_LOOP_OVER;
}

void posRamp_updateRDTime(PosRamp *p , uint16_t RD_time){
	p->rampDownTimems_Const = RD_time;
	uint16_t rampDown_steps = p->rampDownTimems_Const/CALLING_TIME_MS;
	p->dV_RD = p->pkVelocity/rampDown_steps;
}


void posRamp_Exec(PosRamp *p){
	if (p->rampPhase == VELOCITY_RAMPUP){
		if(p->currentMoveVelocity < p->pkVelocity){
			p->currentMoveVelocity += p->dV_RU;
			if (p->currentMoveVelocity >= p->pkVelocity){
				p->currentMoveVelocity = p->pkVelocity;
				p->rampPhase =  VELOCITY_CRUISE;
			}
			float dS_RU =p->currentMoveVelocity * p->callingTimeSec_Const;
			p->currentMoveTarget_mm += dS_RU;
		}
	}
	else if (p->rampPhase == VELOCITY_CRUISE){
		if (p->currentMoveTarget_mm < p->endDist_cruise){
			p->currentMoveTarget_mm += p->dS_cruise;
			if (p->currentMoveTarget_mm >= p->endDist_cruise){
				p->currentMoveTarget_mm = p->endDist_cruise;
				p->rampPhase =  VELOCITY_RAMPDOWN;
			}
		}
	}
	else if (p->rampPhase == VELOCITY_PAUSE){
		if(p->currentMoveVelocity > 0){
			p->currentMoveVelocity -= p->dV_RD;
		}else{
			p->currentMoveVelocity = 0;
			p->rampPhase =  POS_LOOP_OVER;
		}
		float dS_RD =p->currentMoveVelocity * p->callingTimeSec_Const;
		p->currentMoveTarget_mm += dS_RD;
	}

	else if (p->rampPhase == VELOCITY_RAMPDOWN){
			if(p->currentMoveVelocity > MIN_VELOCITY){
				p->currentMoveVelocity -= p->dV_RD/2;
			}else{
				p->currentMoveVelocity = MIN_VELOCITY;
			}
			float dS_RD =p->currentMoveVelocity * p->callingTimeSec_Const;
			p->currentMoveTarget_mm += dS_RD;
			if (p->currentMoveTarget_mm >= p->endDist_RD){
				//p->currentMoveTarget_mm = p->dist_cruisePhase;
				p->rampPhase =  POS_LOOP_OVER;
			}
		}
	else{
	}
}
