/*
 * GB.c
 *
 *  Created on: Mar 30, 2023
 *      Author: harsha
 */


#include "GB.h"
#include "Struct.h"
#include "math.h"
#include "EepromFns.h"
#include "CAN_Motor.h"

extern uint8_t disable_overRideLiftLimitError;

void init_GB(GB_TypeDef *gb){
	gb->ErrorFlag = 0;


	gb->PWM_cnts = 0;
	gb->previousPWM_cnt = 0;
	gb->PWM_dutyCycle = 0;
	gb->absPosition = 0;

	gb->rawPwmCnt = 0;
	gb->deltaPwmCnt = 0;
	gb->firstReading = 0;
	gb->noisyRdngsPerSec = 0;
	gb->msTimer = 0;
	gb->maxNoiseRdngsSinceStart= 0;

	gb->MB_request = 0;
	gb->MB_requestTimer = 0;

}

uint8_t check_GB_Encoder_Health(void){
	if (GB.PWM_dutyCycle <= GB_ENCODER_ERROR_DUTY_CYCLE){
		return 1;
	}else{
		return 0;
	}
}


void send_GB_Request(void){
  if (GB.MB_request == GB_REQUEST_SINGLE_SHOT){
	  FDCAN_GBresponseFromMotor(S.CAN_ID,&GB);
	  GB.MB_request = GB_REQUEST_NONE;
	  disable_overRideLiftLimitError=1;
  }else if (GB.MB_request == GB_REQUEST_CONTINUOUS){
	  if (fabs(GB.MB_requestTimer - T.tim16_20msTimer) > 1){
		  FDCAN_GBresponseFromMotor(S.CAN_ID,&GB);
		  GB.MB_requestTimer = T.tim16_20msTimer;
	  }
  }else if (GB.MB_request == GB_REQUEST_STOP){
	  GB.MB_request = GB_REQUEST_NONE;
	  disable_overRideLiftLimitError=1;
  }
  else if (GB.MB_request == GB_REQUEST_SET_HOMING_POS){
	  uint8_t success = writeHomingPositionToEeprom(GB.PWM_cnts);
	  if (success){
		  FDCAN_HomingDone(S.CAN_ID,HOMING_SETUP_DONE);
	  }else{
		  FDCAN_HomingDone(S.CAN_ID,HOMING_SETUP_FAILED);
	  }
	  GB.MB_request=GB_REQUEST_NONE;
  }
}


