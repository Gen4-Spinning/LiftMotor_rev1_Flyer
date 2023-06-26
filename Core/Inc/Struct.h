/*
 * Struct.h
 *
 *  Created on: 07-Mar-2023
 *      Author: Jonathan
 */

#ifndef INC_STRUCT_H_
#define INC_STRUCT_H_

#include "stm32g4xx_hal.h"
#include "main.h"

typedef struct StateStruct
{
	char CAN_ID;
	char motorSetupFailed;
	char errorMsgSentOnce;
	uint8_t CAN_MSG;
	uint8_t RM_state;
	uint8_t recievedStopCommand;
	char motorState;
	char oneTime;
	char sendStrokeOver;
}StateTypeDef;

typedef struct TimerStruct
{
	char tim16_20msTimer;
	uint16_t tim16_oneSecTimer;;

}TimerTypeDef;


// Struct is first point of contact with variables from the CAN
typedef struct LiftRunMgmt
{	char runType; // Normal or Diagnosis
	char controlType;//open/closed
	char logReturn ; //runTime/analysis/None
	float distance;
	float time;

	uint8_t direction;
	uint16_t duty;

	// last three for running production run
	uint16_t rampUpTime_ms;
	uint16_t rampDownTime_ms;
	uint16_t directionChangeRamp_ms;
}LiftRunMgmtTypeDef;


typedef struct settingVarStruct {
  float Kp;
  float Ki;
  uint16_t start_offset;
  uint16_t ff_percent;
  uint8_t MOTID; 				// FOR CAN
  uint16_t AMS_offset_index; 		// for the AMS Chip
  uint16_t default_direction;
}settingVar;

typedef struct motorSetupStruct{
	char eepromPWMValsGood; // we got good readings when we started from the eeprom
	char eepromMotorValsGood; // we got good readings when we started from the eeprom
	char eepromHomingPositionGood;
	char defaults_eepromWriteFailed; // we got bad readings but the defaults were written properly
	char encoderSetupOK;
	char encoderZeroValueOK;
	char encoderMagErrorSetupOK;
}setup_typeDef;

typedef struct Errors_Struct
{	char overcurrent;
	char overvoltage;
	char undervoltage;
	char motorThermistorFault;
	char fetThermistorFault;
	char motorOvertemperature;
	char fetOvertemperature;
	char eepromWriteError;
	char eepromBadValueError;
	char trackingError;
	char motorEncoderSetupError;
	char liftPosTrackingError;
	char liftSynchronicityError;
	char liftOutOfBoundsError;
	char eerpomBadHomingPos;
	char errorFlag;//this flag is set when any of the critical error shows up
}ErrorsTypeDef;

typedef struct runtimeVarStruct {
  uint8_t motor_state;
  uint8_t runmode;//open loop or closed loop
  float targetPosition;
  float presentPosition;
  uint8_t liftDirection;
  uint16_t presentRPM;
  uint16_t appliedDuty;
  uint8_t FETtemp;
  uint8_t MOTtemp;
  uint16_t busCurrentADC;
  uint16_t busVoltageADC;
  float currentAmps;
  float prevcurrentAmps;
  float voltageVolts;
  int16_t proportionalTerm;
  int16_t IntegralTerm;
  uint16_t feedforwardTerm;
  uint16_t startOffsetTerm;
  uint16_t motorError;//this variable is sent on the canbus to motherboard
}runtimeVarsTypeDef;

typedef struct Dbg_MultiStroke_oneSideRun
{	char dbg_multiStroke_On;
	uint8_t targetLayers;
	uint8_t currentLayer;
	uint8_t strokeOver;
	uint16_t uart_ms_timer;
}Dbg_multiStroke;

extern setup_typeDef setup;
extern settingVar settingVarObj;
extern ErrorsTypeDef E;
extern runtimeVarsTypeDef R;
extern StateTypeDef S;
extern LiftRunMgmtTypeDef LRM;
extern TimerTypeDef T;
extern Dbg_multiStroke Dbg_multiStrk;

void InitializeSettingsObj(settingVar *stV);
void InitializeRunTime_TypeDef(runtimeVarsTypeDef *runtimeVarObj);
void InitializeState_TypeDef(StateTypeDef *s);
void InitializeLiftRunMgmt_TypeDef(LiftRunMgmtTypeDef *rm);
void InitializeTimer_TypeDef(TimerTypeDef *t);
void InitializeSetup_TypeDef(setup_typeDef *s);

#endif /* INC_STRUCT_H_ */
