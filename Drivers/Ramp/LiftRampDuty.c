#include "LiftRamp.h"
#include "sixSector.h"
#include "Constants.h"

void InitLiftRampDuty(LiftRampDuty *r){
	r->ramp_callingTime_ms = 200;
	r->rampPhase = RAMP_WAIT;
}

void SetupLiftRampDuty(LiftRampDuty *ramp,uint16_t targetDuty,uint16_t rampUpTime,uint16_t rampDownTime,int16_t rampSteadyTime,uint16_t BOS_rampUpTime, uint16_t EOS_rampDownTime){
	uint16_t totalSteps  = 0;
	ramp->ramp_callingTime_ms = 200;
	ramp->rampUpTime_ms = rampUpTime;
	ramp->rampDownTime_ms = rampDownTime;
	ramp->steadyRunTime_s = rampSteadyTime; // in Seconds!
	ramp->finalTargetDuty = targetDuty;
	ramp->EOS_DownTime_ms = EOS_rampDownTime;
	ramp->BOS_UpTime_ms = BOS_rampUpTime;
	ramp->currentDutyF = 0;
	ramp->currentDuty = 0;
	//For RampUp
	totalSteps = ramp->rampUpTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RU = ((float)ramp->finalTargetDuty)/totalSteps;

	//For RampDown
	totalSteps = ramp->rampDownTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_RD = ((float)ramp->finalTargetDuty)/totalSteps;

	//for EndOfStroke RampDown
	totalSteps = ramp->EOS_DownTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
		totalSteps = 1;
	}
	ramp->dDuty_F_EOS = ((float)ramp->finalTargetDuty)/totalSteps;

	//for BeginningOfStroke RampUp
	totalSteps = ramp->BOS_UpTime_ms/ramp->ramp_callingTime_ms;
	if (totalSteps == 0){
			totalSteps = 1;
		}
	ramp->dDuty_F_BOS = ((float)ramp->finalTargetDuty)/totalSteps;

	ramp->rampPhase = RAMP_WAIT;
}
void StartLiftRampDuty(LiftRampDuty *ramp){
	ramp->rampPhase = RAMP_UP;
}
void StopLiftRampDuty(LiftRampDuty *ramp){
	ramp->rampPhase = RAMP_OVER;
	ramp->currentDutyF = 0;
	ramp->currentDuty = 0;
}

void ResetLiftRampDuty(LiftRampDuty *ramp){
	ramp->ramp_callingTime_ms = 20;
	ramp->rampPhase = RAMP_WAIT;
}


void ExecLiftRampDuty(LiftRampDuty *ramp,TimerTypeDef *t){
	if (ramp->rampPhase == RAMP_UP){
		if(ramp->currentDuty <= ramp->finalTargetDuty){
			ramp->currentDutyF += ramp->dDuty_F_RU;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty >= ramp->finalTargetDuty){
				ramp->currentDuty  = ramp->finalTargetDuty;
				ramp->rampPhase = RAMP_STEADY;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
		}
	}

	else if (ramp->rampPhase == RAMP_STEADY){
		if (ramp->steadyRunTime_s != RUN_FOREVER){
			if (t->tim16_oneSecTimer >= ramp->steadyRunTime_s){
				ramp->rampPhase = RAMP_DOWN;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
		}
	}

	else if (ramp->rampPhase == EOS_RAMPDOWN){
			if(ramp->currentDuty >= 0){
				ramp->currentDutyF -= ramp->dDuty_F_EOS;
				ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
				if(ramp->currentDuty <= 0){
					ramp->currentDuty  = 0;
					ramp->rampPhase = LIFT_CONFIRM_STOP;
					t->tim16_oneSecTimer = 0;
					t->tim16_20msTimer = 0;
				}
			}
		}


	else if (ramp->rampPhase == BOS_RAMPUP){
		if(ramp->currentDuty <= ramp->finalTargetDuty){
			ramp->currentDutyF += ramp->dDuty_F_BOS;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty >= ramp->finalTargetDuty){
				ramp->currentDuty  = ramp->finalTargetDuty;
				ramp->rampPhase = RAMP_STEADY;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
		}
	}

	else if (ramp->rampPhase == RAMP_DOWN){
		if(ramp->currentDuty >= 0){
			ramp->currentDutyF -= ramp->dDuty_F_RD;
			ramp->currentDuty = (uint16_t)(ramp->currentDutyF);
			if(ramp->currentDuty <= 0){
				ramp->currentDuty  = 0;
				ramp->rampPhase = RAMP_OVER;
				t->tim16_oneSecTimer = 0;
				t->tim16_20msTimer = 0;
			}
		}
	}
	else{
		//Do Nothing
	}
}

