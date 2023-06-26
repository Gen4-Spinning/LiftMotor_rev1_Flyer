/*
 * Init_TypeDefs.c
 *
 *  Created on: 10-Mar-2023
 *      Author: harsha
 */

#include "Struct.h"
#include "Constants.h"



void InitializeTimer_TypeDef(TimerTypeDef *t)
{
	t->tim16_20msTimer=0;
	t->tim16_oneSecTimer=0;

}

void InitializeLiftRunMgmt_TypeDef(LiftRunMgmtTypeDef *rm)
{
	rm->runType=NOT_RUNNING;
	rm->controlType = 0;
	rm->logReturn = NO_LOG;
	rm->distance = 0;
	rm->time = 0;
	rm->direction = 0;
	rm->duty = 0;
	// last two for running production run
	rm->rampUpTime_ms= 0;
	rm->rampDownTime_ms = 0;
}

void InitializeSettingsObj(settingVar *stV)
{
	  stV->Kp=1.01;
	  stV->Ki=1.58;
	  stV->start_offset=1560;
	  stV->ff_percent=0.25;
	  stV->MOTID=2;
	  stV->AMS_offset_index=1590;

}

void InitializeSetup_TypeDef(setup_typeDef *s){
	s->eepromPWMValsGood = 0;
	s->eepromMotorValsGood = 0;
	s->eepromHomingPositionGood = 0;
	s->defaults_eepromWriteFailed=0;
	s->encoderSetupOK=0;
	s->encoderMagErrorSetupOK = 0;
	s->encoderZeroValueOK = 0;
}

void InitializeRunTime_TypeDef(runtimeVarsTypeDef *r)
{
	r->motor_state = IDLE_STATE;
	r->runmode=0;
	r->targetPosition = 0;
	r->presentPosition = 0;
	r->liftDirection=0;
	r->presentRPM=0;
	r->appliedDuty=0;
	r->FETtemp=0;
	r->MOTtemp=0;
	r->busCurrentADC=0;
	r->busVoltageADC=0;
	r->currentAmps = 0;
	r->voltageVolts =0;
	r->proportionalTerm = 0;
	r->IntegralTerm = 0;
	r->startOffsetTerm = 0;
	r->feedforwardTerm = 0;
}

void InitializeState_TypeDef(StateTypeDef *s){
	s->motorState = 0;
	s->motorSetupFailed = 0;
	s->errorMsgSentOnce = 0 ;
	s->CAN_MSG = 0;
	s->oneTime = 1;
	s->CAN_ID = 0;
	s->RM_state = 0;
	s->recievedStopCommand = 0;
}

