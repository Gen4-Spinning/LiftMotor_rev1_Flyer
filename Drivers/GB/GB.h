/*
 * GB.h
 *
 *  Created on: Mar 30, 2023
 *      Author: harsha
 */

#ifndef GB_H_
#define GB_H_

#include "stdio.h"

//THESE CONSTANTS COME FROM THE TIM3 - SET as 40ns Per Clk count
#define GB_ENCODER_ERROR_COUNT 133
#define GB_ENCODER_ZERO_IN_TIM3CNTS 178
#define GB_ENCODER_MAX_IN_TIM3CNTS 47286
#define GB_ENCODER_SINGLE_ROTATION_IN_TIM3CNTS 47108
#define GB_ENCODER_ERROR_DUTY_CYCLE 0.3
#define GB_ENCODER_TIM3CNTS_PER_MM 118

#define GB_REQUEST_NONE 0
#define GB_REQUEST_STOP 1
#define GB_REQUEST_CONTINUOUS 2
#define GB_REQUEST_SINGLE_SHOT 3
#define GB_REQUEST_SET_HOMING_POS 4

#define HOMING_DONE 1
#define HOMING_SETUP_DONE 2
#define HOMING_SETUP_FAILED 3

//THEORY
//4 mm/sec is highest speed, ie 1 lead screw rotation per sec, which means the motor shaft before a 12.5 GB,
//is doing about 12.5 rotations/sec
//PWM from the encoder is at  @ 547Hz ie: 1.82 ms, so the largest delta angle we have btw neighbouring pulses is
//0.00182 *12.5*360 = 8.19 degrees. we know 360 degree is is 47108 cnts as per TIM3 clk settings(see GB.h).
//therefore 8 degrees = 1071 cnts. So max delta count is 1071. Anything more means a bad noisy reading.
//EXP = saw even at highest speed we had only 12-27 counts. so something wrong in above calcs
#define GB_THRESHOLD_NOISY 75 // 3x

typedef struct GB_struct {
	//RAW values from the GB_encoder;
	uint16_t rawPwmCnt;
	uint16_t PWM_cnts;
	uint16_t previousPWM_cnt;
	float PWM_dutyCycle;
	float absPosition;

	//encoder health.
	uint8_t firstReading;
	float deltaPwmCnt;
	uint16_t noisyRdngsPerSec;
	uint16_t msTimer;
	uint16_t maxNoiseRdngsSinceStart;

	char ErrorFlag;

	char MB_request;  // mainboard Request. only valid in IDLE state
	char MB_requestTimer;

} GB_TypeDef;

extern GB_TypeDef GB;

void init_GB(GB_TypeDef *gb);
uint8_t check_GB_Encoder_Health(void);
void send_GB_Request(void);


#endif /* GB_H_ */
