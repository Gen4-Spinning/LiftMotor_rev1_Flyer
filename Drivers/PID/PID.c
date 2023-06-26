/*
 * PID.c
 *
 *  Created on: Mar 11, 2023
 *      Author: harsha
 */

#include "PID.h"

void InitializePID_TypeDef(PID_Typedef *p){
	p->FF_percent = 0;
	p->Kp_constant = 0;
	p->Ki_constant = 0;
	p->FF_constant = 0;
	p->startOffsetConstant = 0;

	p->error_s16 = 0;
	p->errorF = 0;
	p->Kp_term = 0;
	p->Ki_term = 0;
	p->feedForwardTerm = 0;
	p->startOffsetTerm = 0;
	p->antiWindup = 0;
	p->pwm = 0;
}

void setupPID(PID_Typedef *p,float Kp,float Ki,uint16_t FF_Percent,uint16_t so){
	p->FF_percent = FF_Percent;// rpms16 : rpm = 64000:2000
	p->FF_constant = (float)((p->FF_percent * 10.0)/32000); //32000 so that it works with rpms16
	p->Ki_constant = Ki;
	p->Kp_constant = Kp;
	p->startOffsetConstant = so;
}

void setupPID_LiftMotors(PID_Typedef *p,float Kp,float Ki,uint16_t FF_Percent,uint16_t so){
	p->FF_percent = FF_Percent;
	// max Velocity for the lift is 5.8/sec(see Roving width = 3, and Flyer RPM = 850, see Flyer excel).
	//that velocity becomes 100% duty cycle, ie 1500. So, (velocity/6) * 1500 * FFpercent/100 = FFduty
	p->FF_constant = ((float)(p->FF_percent) * 2.5 );
	p->Ki_constant = Ki;
	p->Kp_constant = Kp;
	p->startOffsetConstant = so;
}

void resetPID(PID_Typedef *p){
	p->error_s16 = 0;
	p->errorF = 0;
	p->Ki_term  = 0;
	p->Kp_term = 0;
	p->feedForwardTerm = 0;
	p->startOffsetTerm = 0;
	p->pwm = 0;
	p->antiWindup= 0;
}

void ExecPID_PosLift(PID_Typedef *p, PosRamp *posRamp,PosCL_TypeDef *posCL){
	float temp_Ki_term;
	float temp ;

	p->errorF = posCL->currentMoveDist_mm - posRamp->currentMoveTarget_mm;
	posCL->positioningError_mm = p->errorF;
	// if error is -ve we are lagging, and we want to add PWM. so we invert the sign here
	// if error is positive we are faster than the target and we want to slow down. again invert the sign
	p->errorF = -1.0*p->errorF;
	p->Kp_term = p->errorF * p->Kp_constant;
	temp = ((float)p->errorF) * p->Ki_constant;
	temp_Ki_term = p->Ki_term + temp;
	if ((temp_Ki_term > 32000) || (temp_Ki_term < -32000) || (p->pwm >= MAX_PWM)){
		//dont increment the Ki Term
		p->antiWindup= 1;
	}else{
		p->Ki_term = temp_Ki_term;
		p->antiWindup = 0;
	}

	p->feedForwardTerm = posRamp->currentMoveVelocity * p->FF_constant;
	p->startOffsetTerm =  p->startOffsetConstant;

	p->pwm = p->feedForwardTerm + (int16_t)p->Kp_term + (int16_t)p->Ki_term + p->startOffsetTerm;

	if (p->pwm > MAX_PWM){
		p->pwm = MAX_PWM;
	}
	if (p->pwm < 0){
		p->pwm = 0;
	}
}


/*
void ExecPID(PID_Typedef *p,RampRPM *r,EncSpeed_TypeDef *eS){
	p->error_s16 = r->instTargetRPM_s16M - eS->speed_s16;
	p->errorF = r->instTargetRPM_F - eS->speedRPM;

	p->Kp_term = p->error_s16 * p->Kp_constant;

	temp = ((float)p->error_s16) * p->Ki_constant;
	temp_Ki_term = p->Ki_term + temp;
	if ((temp_Ki_term > 32000) || (temp_Ki_term < -32000) || (p->pwm >= MAX_PWM)){
		//dont increment the Ki Term
		p->antiWindup= 1;
	}else{
		p->Ki_term = temp_Ki_term;
		p->antiWindup = 0;
	}

	p->feedForwardTerm = r->instTargetRPM_s16M * p->FF_constant;
	p->pwm = p->feedForwardTerm + p->Kp_term + (int16_t)p->Ki_term + p->startOffsetConstant;

	if (p->pwm > MAX_PWM){
		p->pwm = MAX_PWM;
	}
	if (p->pwm < 0){
		p->pwm = 0;
	}

}
*/
