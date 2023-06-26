/*
 * CAN_Motor.h
 *
 *  Created on: 10-Mar-2023
 *      Author: harsha
 */

#ifndef CAN_MOTOR_H_
#define CAN_MOTOR_H_


#include "Struct.h"
#include "FDCAN.h"
#include "Constants.h"
#include "GB.h"

void FDCAN_driveresponseFromMotor(uint8_t source);
//void FDCAN_liftruntimedataFromMotor(LiftControllerTypeDef *L);
void FDCAN_liftRunDataFromMotor(void);
void FDCAN_GBresponseFromMotor(uint8_t source,GB_TypeDef *gb);
void FDCAN_liftAnalysisDataFromMotor(void);
void FDCAN_errorFromMotor(void);

void FDCAN_parseForMotor(uint8_t my_address);
void FDCAN_parseDiagnosticData(LiftRunMgmtTypeDef *l);
//void FDCAN_parseChangeTargetFrame(LiftRunMgmtTypeDef *r);
void FDCAN_ACKresponseFromMotor(uint8_t source);
void FDCAN_HomingDone(uint8_t source,uint8_t command);
void FDCAN_StrokeOver(uint8_t source,LiftRunMgmtTypeDef *lrm);
void FDCAN_parseLiftSetupData(LiftRunMgmtTypeDef *l);
void FDCAN_parseNewStrokeData(LiftRunMgmtTypeDef *l);
void FDCAN_sendDiagDoneFrame(void);


#endif /* CAN_MOTOR_H_ */
