/*
 * CAN_Motor.c
 *
 *  Created on: 10-Mar-2023
 *      Author: harsha
 */

#include "CAN_Motor.h"
#include "stdio.h"
extern char UART_buffer[50];
extern UART_HandleTypeDef huart3;
extern uint8_t enable_overRideLiftLimitError;

void FDCAN_liftRunDataFromMotor(void)
{
	TxHeader.Identifier =(0xE0C01<<8)|S.CAN_ID;//set to transmit runtime data frame from flyer to motherboard
	TxHeader.DataLength = FDCAN_DLC_BYTES_16;

	TxData[0]=(uint16_t)(R.targetPosition*100.0)>>8;
	TxData[1]=(uint16_t)(R.targetPosition*100.0);
	TxData[2]=(uint16_t)(R.presentPosition*100.0)>>8;
	TxData[3]=(uint16_t)(R.presentPosition*100.0);
	TxData[4]=(R.presentRPM)>>8;
	TxData[5]=R.presentRPM;
	TxData[6]=(R.appliedDuty)>>8;
	TxData[7]=R.appliedDuty;
	TxData[8]=R.FETtemp;
	TxData[9]=R.MOTtemp;
	TxData[10]=(R.busCurrentADC)>>8;
	TxData[11]=R.busCurrentADC;
	TxData[12]=(R.busVoltageADC)>>8;
	TxData[13]=R.busVoltageADC;
	TxData[14]=R.liftDirection;

	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>1){
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	}
}


void FDCAN_GBData_FromMotor(void)
{
	TxHeader.Identifier =(0xFFFFF<<8)|S.CAN_ID;//set to transmit runtime data frame from flyer to motherboard
	TxHeader.DataLength = FDCAN_DLC_BYTES_4;

	TxData[0]=(uint16_t)(GB.rawPwmCnt)>>8;
	TxData[1]=(uint16_t)(GB.rawPwmCnt);
	TxData[2]=(uint16_t)(GB.PWM_cnts)>>8;
	TxData[3]=(uint16_t)(GB.PWM_cnts);

	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>1){
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	}
}


void FDCAN_liftAnalysisDataFromMotor(void)
{
	TxHeader.Identifier =(0xA0801<<8)|S.CAN_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_16;

	TxData[0]=(uint16_t)(R.targetPosition*100.0)>>8;
	TxData[1]=(uint16_t)(R.targetPosition*100.0);
	TxData[2]=(uint16_t)(R.presentPosition*100.0)>>8;
	TxData[3]=(uint16_t)(R.presentPosition*100.0);
	TxData[4]=(R.presentRPM)>>8;
	TxData[5]=R.presentRPM;
	TxData[6]=(R.appliedDuty)>>8;
	TxData[7]=R.appliedDuty;
	TxData[8]=(R.proportionalTerm)>>8;
	TxData[9]=R.proportionalTerm;
	TxData[10]=(R.IntegralTerm)>>8;
	TxData[11]=R.IntegralTerm;
	TxData[12]=(R.feedforwardTerm)>>8;
	TxData[13]=R.feedforwardTerm;
	TxData[14]=(R.feedforwardTerm)>>8;
	TxData[15]=R.feedforwardTerm;

	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)>1){
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	}
}

void FDCAN_errorFromMotor(void)
{
	TxHeader.Identifier =(0x60201<<8)|S.CAN_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	TxData[0]=(R.motorError)>>8;
	TxData[1]=(R.motorError);
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)==0);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_parseForMotor(uint8_t my_address)
{

	functionID=((RxHeader.Identifier)&0xFF0000)>>16;
	source_address=(RxHeader.Identifier)&0xFF;

	switch (functionID) {

		case MOTORSTATE_FUNCTIONID:
			S.CAN_MSG=RxData[0];
			FDCAN_ACKresponseFromMotor(my_address);
			//sprintf(UART_buffer,"\r\n CAN-MotorState-%02d",RxData[0]);
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,20);
			break;

		case LIFT_DIAGNOSTICSDATA_FUNCTIONID:
			FDCAN_ACKresponseFromMotor(my_address);
			FDCAN_parseDiagnosticData(&LRM);
			LRM.runType = DIAGNOSIS_RUN;
			LRM.logReturn = RUNTIME_LOG;
			//For Diagnosis ramp up and ramp down time for lift is hardcoded.
			LRM.rampUpTime_ms = 1000;
			LRM.rampDownTime_ms = 1000;
			LRM.directionChangeRamp_ms = 0;
			S.RM_state = RECEIVED_RAMP_SETTINGS;
			//sprintf(UART_buffer,"\r\n CAN-Diagnostics Setup");
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,23);
			break;

		//setup is used only once. whenever we get start use rampUp, when we get rampDown use ramp Down
		//for stroke over use direction change ms.
		case LIFT_SETUP_FUNCTIONID:
			FDCAN_ACKresponseFromMotor(my_address);
			FDCAN_parseLiftSetupData(&LRM);
			LRM.runType = NORMAL_RUN;
			LRM.controlType=CLOSED_LOOP;
			LRM.logReturn = RUNTIME_LOG;
			S.RM_state = RECEIVED_RAMP_SETTINGS;
			//sprintf(UART_buffer,"\r\n CAN-Lift Run Setup");
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,20);
			break;

		case LIFT_NEW_STROKE_FUNCTIONID:
			FDCAN_ACKresponseFromMotor(my_address);
			FDCAN_parseNewStrokeData(&LRM);
			S.CAN_MSG=NEW_LAYER;
			LRM.runType = NORMAL_RUN;
			LRM.controlType=CLOSED_LOOP;
			LRM.logReturn = RUNTIME_LOG;
			//sprintf(UART_buffer,"\r\n CAN-Lift NewStroke");
			//HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,20);
			break;

		case LIFT_REQUEST_FOR_GB_DATA:
			GB.MB_request  = RxData[0];
			enable_overRideLiftLimitError=1;
		default:
			break;
	}
}


void FDCAN_driveresponseFromMotor(uint8_t source)
{
	TxHeader.Identifier =(0xA0401<<8)|source;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=source;
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)==0);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_ACKresponseFromMotor(uint8_t source)
{
	TxHeader.Identifier =(0x060F01<<8)|source;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=1;
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)==0);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_GBresponseFromMotor(uint8_t source,GB_TypeDef *gb){
	TxHeader.Identifier =(0xA1701<<8)|source;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxData[0]=gb->PWM_cnts >> 8;
	TxData[1]=gb->PWM_cnts ;
	TxData[2]=(uint16_t)(gb->PWM_dutyCycle*100) >> 8 ;
	TxData[3]=(uint16_t)(gb->PWM_dutyCycle*100);
	TxData[4]=(uint16_t)(gb->absPosition*100) >> 8 ;
	TxData[5]=(uint16_t)(gb->absPosition*100) ;
	TxData[6]=1;
	TxData[7]=1;
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)==0);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}
void FDCAN_parseDiagnosticData(LiftRunMgmtTypeDef *l)
{
	l->controlType=RxData[0];
	l->distance = ((RxData[1]<<8)|(RxData[2]))/100.0;
	if (l->controlType == CLOSED_LOOP){
		l->time = ((RxData[3]<<8)|(RxData[4]))/100.0;
	}else if (l->controlType == OPEN_LOOP){
		l->duty = (RxData[3]<<8)|(RxData[4]);
	}
	l->direction = RxData[5];
}

void FDCAN_parseLiftSetupData(LiftRunMgmtTypeDef *l){
	l->distance = ((RxData[0]<<8)|(RxData[1]))/100.0;
	l->time = ((RxData[2]<<8)|(RxData[3]))/100.0;
	l->direction = RxData[4];
	//TODO - what happens if rampup time and rampDown time is 0?
	l->rampUpTime_ms = RxData[5] * 1000;
	l->rampDownTime_ms = RxData[6]*1000;
	l->directionChangeRamp_ms  = ((RxData[7]<<8)|(RxData[8]));
}

void FDCAN_parseNewStrokeData(LiftRunMgmtTypeDef *l){
	l->distance = ((RxData[0]<<8)|(RxData[1]))/100.0;
	l->time = ((RxData[2]<<8)|(RxData[3]))/100.0;
	l->direction = RxData[4];
}

void FDCAN_HomingDone(uint8_t source,uint8_t command){
	TxHeader.Identifier =(0x0A1301<<8)|source;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=command;
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)==0);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

void FDCAN_StrokeOver(uint8_t source,LiftRunMgmtTypeDef *lrm){
	TxHeader.Identifier =(0x0A0E01<<8)|source;
	TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	TxData[0] = 1;
	TxData[1]= lrm->direction;
	while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)==0);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}
void FDCAN_sendDiagDoneFrame(void){
	TxHeader.Identifier =(0xA1401<<8)|S.CAN_ID;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxData[0]=1;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

