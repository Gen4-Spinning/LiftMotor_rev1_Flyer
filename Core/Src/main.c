/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Struct.h"
#include "Constants.h"
#include "ErrorShiftVals.h"

#include "sixSector.h"
#include "PID.h"

#include "GB.h"
#include "PosCntrl_CL.h"
#include "PosCntrl_OL.h"
#include "PosRamp.h"
#include "PosPts.h"
#include "LiftRamp.h"

#include "Eeprom.h"
#include "EepromFns.h"
#include "EepromSettings.h"

#include "FDCAN.h"
#include "CAN_Motor.h"

#include "AS5x47P.h"
#include "EncoderFns.h"
#include "EncoderCalibration.h"
#include "EncSpeed.h"
#include "EncPos.h"
#include "temperatureLUT.h"

#include "Console.h"

#include <stdio.h>
#include <math.h> // for fabs
#include "retarget.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac3;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

setup_typeDef setup;
runtimeVarsTypeDef R;
settingVar settingVarObj;
ErrorsTypeDef E;
StateTypeDef S;
TimerTypeDef T;
EncCalib_TypeDef encCalib;
EncSpeed_TypeDef encSpeed;
EncPos_TypeDef encPos;

//Structs to Control the motor.
LiftRunMgmtTypeDef LRM;
sixSector sixSectorObj;
sixSectorCntrl sixSectorCntrl_Obj;

//positioning Structs
GB_TypeDef GB;
LiftRampDuty liftRampDuty;
PosOL_TypeDef posOL;
PosCL_TypeDef posCL;
PosRamp posRamp;
PosPoints posPts;

//for testing strength of GB
Dbg_multiStroke Dbg_multiStrk;

PID_Typedef PIDpos;

//ADC variables here
uint16_t ADC1_buff[2]={0,2008},ADC2_buff[2];

//CAN variables here
FDCAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC3_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void configurationFromTerminal(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char UART_buffer[50];


uint8_t doCalibration =0;

//scanf variables
char scan_string[10], confirmation_char, configuration_mode_flg;
uint8_t start_string[3];
uint8_t AMS_manual_write_success = 0;
float ABI_mechAngle;
float SPI_mechAngle;

uint8_t dbg_posCL_start = 0;
uint8_t dbg_posCL_stop = 0;
uint8_t dbg_homing_start = 0;
uint8_t dbg_posOL_start = 0;
uint8_t dbg_posOL_stop = 0;

uint8_t manualWriteHomingPos = 0;
uint16_t homingPosition = 0;

uint8_t dbg = 0;

uint8_t enable_overRideLiftLimitError = 0;
uint8_t disable_overRideLiftLimitError = 0;
uint8_t overRideLiftError = 0;
uint8_t GB_outOfBounds = 0;
uint8_t setupGB = 1;

uint8_t motThermErrorFilter = 0;
uint8_t mosfetThermErrorFilter = 0;

uint8_t dbg_do_multiStroke_run;
uint8_t dbg_stop_multiStroke_run;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim->Instance==TIM1){ // PWM interrupt
		uint16_t encoderCNT = htim2.Instance->CNT;
		CalcSector_fromEncoder(&sixSectorObj,encoderCNT);
		//if Error Flag is 1, calibration State or not, we stop all the PWMs.
		if (E.errorFlag==0){
			if ((sixSectorCntrl_Obj.turnOn == 1 )&& (sixSectorCntrl_Obj.duty > 14)){
				if (sixSectorCntrl_Obj.direction == CW){
					sixSectorCommutateCW(&sixSectorObj,0,500);}
				if (sixSectorCntrl_Obj.direction == CCW){
					sixSectorCommutateCCW(&sixSectorObj,0,500);}
			}
			else{
				if (S.motorState != CALIBRATION_STATE){
					TurnOffAllChannels();}
			}
		} // closes ErrorFlag = 0;
		else{
		  TurnOffAllChannels();
		}
	}

	if (htim->Instance==TIM16){  // 40ms timer
		if (LRM.controlType == OPEN_LOOP){		//for moving one side independently,
			if (liftRampDuty.rampPhase != RAMP_WAIT){
				if (liftRampDuty.rampPhase!=RAMP_OVER){
					ExecLiftRampDuty(&liftRampDuty,&T);
					sixSectorSetDuty(&sixSectorCntrl_Obj,liftRampDuty.currentDuty);
					EncPos_CalculateMovement(&encPos,&encSpeed);
					posOL_CalcMoveDistance(&posOL,GB.absPosition);
					encPos_CalculateErrorWithGB_OL(&encPos, &posOL);
					posOL_CheckTargetReached(&posOL);
					if (posOL.targetReached){
						liftRampDuty.rampPhase = RAMP_OVER;
					}
				}
				else if (liftRampDuty.rampPhase == RAMP_OVER){
					StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
					ResetLiftRampDuty(&liftRampDuty);
					posOL_ClearMove(&posOL);
					if ((S.motorState == HOMING_STATE) && (S.recievedStopCommand == 0)){
						FDCAN_HomingDone(S.CAN_ID,HOMING_DONE);//send FD can over msg
					}
					if ((LRM.runType == DIAGNOSIS_RUN) && (S.recievedStopCommand == 0)){
						FDCAN_sendDiagDoneFrame();
					}
					//LRM.controlType = 0; // NO MORE OPEN LOOP or closed LOOP
					LRM.logReturn=NO_LOG;
					LRM.runType = NOT_RUNNING;
					S.motorState = IDLE_STATE;
					S.recievedStopCommand = 0;
				}
			}
			R.motor_state = S.motorState;
			R.targetPosition = posOL.GB_absEndPosition;
			R.presentPosition = posOL.GB_absCurrentPosition;
			R.liftDirection = posOL.moveDirection;
			R.appliedDuty = liftRampDuty.currentDuty;
			R.presentRPM = encSpeed.speedRPM;
		}

		else if (LRM.controlType == CLOSED_LOOP){ 			//ClosedLoop means Pos Control
			if (posRamp.rampPhase != POS_LOOP_IDLE){
				if (posRamp.rampPhase!=POS_LOOP_OVER){
					posRamp_Exec(&posRamp);
					posCL_CalcMoveDistance(&posCL,GB.absPosition);
					EncPos_CalculateMovement(&encPos,&encSpeed);
					ExecPID_PosLift(&PIDpos,&posRamp,&posCL);
					encPos_CalculateErrorWithGB_CL(&encPos, &posCL);
					if(PIDpos.errorF >= 5){  // to check this value
						E.errorFlag=E.liftPosTrackingError= 1;
						R.motorError |=1<<ERR_LTE_SHIFT;
					}
					if (fabs(encPos.error_with_GB) >= 5){
						E.errorFlag = E.liftSynchronicityError = 1;
						R.motorError |= 1 << ERR_LSF_SHIFT;
					}
					sixSectorSetDuty(&sixSectorCntrl_Obj,PIDpos.pwm);
				}else if (posRamp.rampPhase==POS_LOOP_OVER){
					StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
					posRamp_Reset(&posRamp);
					posCL_ClearMove(&posCL);//DO NOT zero RemainingDistance,or
					resetPID(&PIDpos);
					// if we re in local debug or Diag mode dont send stroke Over Command
					// if we re pausing also dont send any msg.
					// also dont send stroke over if you get a stop command from the motherboard.
					// also dont send a homing Over msg if you press stop while homing.
					// send a diag done msg if you finshed a diagnosis run with out a stop command
					if ((S.motorState == HOMING_STATE)&& (S.recievedStopCommand == 0)){
						FDCAN_HomingDone(S.CAN_ID,HOMING_DONE);
					}else if (S.motorState == RUN_STATE){
						if ((LRM.runType == NORMAL_RUN) && (S.recievedStopCommand == 0)){
							//sendStrokeOverMsg, but only send once RPM becomes zero.
							S.sendStrokeOver = 1;
						}else if ((LRM.runType == DIAGNOSIS_RUN) && (S.recievedStopCommand == 0)){
							FDCAN_sendDiagDoneFrame();
						}
					}
					if(Dbg_multiStrk.dbg_multiStroke_On == 1){
						Dbg_multiStrk.strokeOver = 1;
					}
					S.motorState = IDLE_STATE;
					//LRM.controlType = 0; control type is changed only when we get a diagnostic open loop msg,or run setup msg. otherwise remains in its last setting.
					//LRM.runType = NOT_RUNNING;
					LRM.logReturn=NO_LOG;
					S.recievedStopCommand = 0;
				}
			}
			R.motor_state = S.motorState ;
			R.appliedDuty = PIDpos.pwm;
			R.targetPosition = posCL.GB_absEndPosition;
			R.presentPosition = GB.absPosition;
			R.liftDirection = posCL.moveDirection;
			R.presentRPM = encSpeed.speedRPM;
			R.proportionalTerm = PIDpos.Kp_term;
			R.IntegralTerm = PIDpos.Ki_term;
			R.feedforwardTerm = PIDpos.feedForwardTerm;
			R.startOffsetTerm = PIDpos.startOffsetTerm;
		}

		if(sixSectorCntrl_Obj.turnOn == 1){
			if (LRM.logReturn == RUNTIME_LOG){
				FDCAN_liftRunDataFromMotor();
			}else if (LRM.logReturn == ANALYSIS_LOG){
				FDCAN_liftAnalysisDataFromMotor();
			}
		}

		// we Time a one sec counter over here;
		T.tim16_20msTimer++;
		if (T.tim16_20msTimer >= 50){
			T.tim16_oneSecTimer ++;
			T.tim16_20msTimer = 0;
		}
	}

	if (htim->Instance==TIM17){ //100 ms timer. Use this to check the encoder State
		if (GB.firstReading == 0){
			GB_outOfBounds= checkWithinLeadScrewLimits(&posPts,GB.PWM_cnts);
			if (overRideLiftError == 0){
				if (GB_outOfBounds == 1){
					TurnOffAllChannels();
					HAL_TIM_Base_Stop_IT(&htim17);
					E.errorFlag=E.liftOutOfBoundsError= 1;
					R.motorError|=1<<ERR_LOB_SHIFT;
					//whatever you were running switch it off!!!
					posRamp.rampPhase=POS_LOOP_OVER;
					liftRampDuty.rampPhase = RAMP_OVER;
				}
			}
		}
		//check here for encoderPos and GB pos difference.
	}

	/*E.motorEncoderError = AS5047_checkEncoderHealth();
		if (E.motorEncoderError == 1){
			TurnOffAllChannels();
			E.errorFlag=1;
			R.motorError|=E.motorEncoderError<<11;
			HAL_TIM_Base_Stop_IT(&htim17);
		}
	}*/

}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  if(hadc==&hadc1)
  {
	  R.busVoltageADC=ADC1_buff[0];
	  R.voltageVolts=(BUS_VOLTAGE_GAIN*ADC1_buff[0]);
	  R.MOTtemp=get_MOSFET_temperature(ADC1_buff[1]);//MOSFET LUT is used because beta value is almost the same, 0.5 deg difference

	  if(R.MOTtemp>130 && E.motorOvertemperature==0 )
	  {
		  E.motorOvertemperature=E.errorFlag=1;
		  R.motorError|=E.motorOvertemperature<<ERR_MOT_SHIFT;
	  }

	  if((ADC1_buff[1]<50 || ADC1_buff[1]>2300)){
		  // look for ten consecutive cycles
		  motThermErrorFilter ++;
		  if (motThermErrorFilter >= 10){
			  E.motorThermistorFault=E.errorFlag=1;
			  R.motorError|=E.motorThermistorFault<<ERR_MTF_SHIFT;
		  }
	  }else{
		  motThermErrorFilter = 0;
	  }
  }
  else
  {
	  R.busCurrentADC=ADC2_buff[0];
	  R.currentAmps=(0.9*R.prevcurrentAmps)+(0.1*BUS_CURRENT_GAIN*((float)ADC2_buff[0]));
	  R.prevcurrentAmps=R.currentAmps;
	  R.FETtemp=get_MOSFET_temperature(ADC2_buff[1]);

	  if(R.FETtemp>110 && E.fetOvertemperature==0 )
	  {
		  E.fetOvertemperature=E.errorFlag=1;
		  R.motorError|=1<<ERR_FOT_SHIFT;
	  }
	  if((ADC2_buff[1]<50 || ADC2_buff[1]>2300)){
		  mosfetThermErrorFilter ++;
		  if (mosfetThermErrorFilter >= 10){
			  E.fetThermistorFault=E.errorFlag=1;
			  R.motorError|=1<<ERR_FTF_SHIFT;
		  }
	  }else{
		  mosfetThermErrorFilter = 0;
	  }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
    	FDCAN_parseForMotor(S.CAN_ID);
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
}

//this runs at ~547Hz,
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		uint16_t PWMperiod =HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
		GB.rawPwmCnt = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);

		GB.deltaPwmCnt = GB.rawPwmCnt - GB.previousPWM_cnt;
		if (GB.firstReading == 1){
				GB.PWM_cnts = GB.rawPwmCnt;
				GB.firstReading = 0;
		}else{
			if (fabs(GB.deltaPwmCnt) > GB_THRESHOLD_NOISY){
				GB.noisyRdngsPerSec ++;	//dont update PWM counts
			}
			else{
				GB.PWM_cnts = GB.rawPwmCnt;
			}
		}
		//cant handle rollover. ie where pwm counts comes down to 178 and then goes to 47286,or the other
		//way round. 47286->178. Anyway we have limits stopping the lift at these points so we leave this for now?
		//Later we can put dead bands? Cant handle in the sense the deltaPwm count becomes greater than noisy count and
		//never recovers.
		GB.previousPWM_cnt = GB.PWM_cnts;

		if (PWMperiod != 0){
			GB.PWM_dutyCycle = (((float)GB.PWM_cnts * 100.0)/PWMperiod);
			GB.absPosition = (GB.PWM_cnts - posPts.homingPositionCnts)/(float)GB_ENCODER_TIM3CNTS_PER_MM;
		}

	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (S.motorState == IDLE_STATE){
		if((start_string[0]=='$')&&(start_string[1]=='$')&&(start_string[2]=='$')){
			configuration_mode_flg=1;
			start_string[0]=start_string[1]=start_string[2]=0;
		}
		else{
			HAL_UART_Receive_IT(&huart3, start_string, 3);
		}
	}
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc){
	TurnOffAllChannels();
	HAL_TIM_Base_Stop_IT(&htim4);
	HAL_TIM_Base_Stop_IT(&htim15);
	E.errorFlag=1;
	if (hadc == &hadc1){
		if(ADC1_buff[0]>2940){
			E.overvoltage=1;
			R.motorError|=1<<ERR_OV_SHIFT;
		}
		else{
			E.undervoltage=1;
			R.motorError|=1<<ERR_UV_SHIFT;
		}
	}
	if (hadc == &hadc2){ // TODO check in lift code
		E.overcurrent=1;
		R.motorError|=1<<ERR_OC_SHIFT;
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC3_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_TIM15_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  InitializeSetup_TypeDef(&setup);
  InitializeSettingsObj(&settingVarObj); // initialize the settings obj
  InitializeRunTime_TypeDef(&R);
  InitializeState_TypeDef(&S);
  InitializeEncoderCalib_TypeDef(&encCalib);
  InitializeTimer_TypeDef(&T);
  InitializeEncSpeed_TypeDef(&encSpeed);
  InitializePID_TypeDef(&PIDpos);
  posOL_Reset(&posOL);
  posCL_Reset(&posCL);
  posRamp_Reset(&posRamp);
  init_GB(&GB);
  initPosPts(&posPts);
  InitLiftRampDuty(&liftRampDuty);


  //Calibrate ADCs and start them
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
  HAL_Delay(50);
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
  HAL_Delay(400);

  // NOW start the Timers for PWM
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

  TurnOffAllChannels();

  //Read EEprom and check values.
  readSettingsFromEEPROM(&settingVarObj);//read values from EEPROM
  setup.eepromMotorValsGood = checkEEPROM_MotorSettings(&settingVarObj);
  setup.eepromPWMValsGood = checkEEPROM_PWMSettings(&settingVarObj);

  //Comment for Production
  //setup.eepromMotorValsGood = 0;
  if (setup.eepromMotorValsGood == 0){
	  AMS_manual_write_success = writeMotorSettingsToEEPROM_Manual(RIGHTLIFT_ADDRESS,-1,CCW);
	  //settingVarObj.AMS_offset_index = 1686;
	  settingVarObj.MOTID = RIGHTLIFT_ADDRESS;
	  settingVarObj.default_direction = CCW;
	  //force user to set the correct values before he continues.
	  // there are no 'default' settings
  }
  //setup.eepromMotorValsGood = 1;

  //setup.eepromPWMValsGood = 0;
  if (setup.eepromPWMValsGood == 0){
	  settingVarObj.Kp = 135;
	  settingVarObj.Ki = 1;
	  settingVarObj.ff_percent = 67;
	  settingVarObj.start_offset = 100;
	 //loadPWMDefaultSettings(&settingVarObj);
	  setup.defaults_eepromWriteFailed += writePWMSettingsToEEPROM(&settingVarObj);
  }
  //setup PID with the correct Settings.
  setupPID_LiftMotors(&PIDpos,135,1,67,100);//Kp,Ki,FF%,startoffset

  //Now setup the Encoder.Use whatever value you got from the eeprom, even if its bad.
  setup.encoderSetupOK = setupMotorEncoder_inABI_Mode(); //setup ABI mode without PWM
  setup.encoderZeroValueOK = updateEncoderZeroPosition(settingVarObj.AMS_offset_index);// set the Zero position
  setup.encoderMagErrorSetupOK = AS5047_EnableMagErrors();

  uint16_t startingCNT_val = getEncoderStartPosition(); // get the current angle and ..
  __HAL_TIM_SET_COUNTER(&htim2,startingCNT_val); // set it in the timer CNT register.
  HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL); //start the encoder

  //homing Position Setup
  manualWriteHomingPos = 0;
  //TODO CAN frame to make this zero.
  if (manualWriteHomingPos){
	  writeHomingPositionToEeprom(5938); // lift Left 5938
	  manualWriteHomingPos = 0;
  }

  homingPosition = readHomingPositionFromEeprom();
  setup.eepromHomingPositionGood = checkHomingPosition(homingPosition);
  if (setup.eepromHomingPositionGood == 1){
	  setupLeadScrewLimitsAndHoming(&posPts,homingPosition);//LiftRight -818  , Lift LEft - 980;
  }

  //setup.eepromMotorValsGood = 0;
  // Now that all the setup is then, we check the values and set an error Flag if there is an issue
  if ((setup.eepromMotorValsGood == 0 ) || (setup.eepromPWMValsGood == 0)){
	  S.motorSetupFailed = E.errorFlag = E.eepromBadValueError = 1;
	  R.motorError = 1 << ERR_EBV_SHIFT;
  }

  if (setup.eepromHomingPositionGood == 0){
	  S.motorSetupFailed = E.errorFlag = E.eerpomBadHomingPos = 1;
	  R.motorError = 1 << ERR_EBH_SHIFT;
  }
  if (setup.defaults_eepromWriteFailed == 1 ){
	  S.motorSetupFailed = E.errorFlag = E.eepromWriteError = 1;
	  R.motorError = 1 << ERR_EWE_SHIFT;
  }

  if ((setup.encoderSetupOK == 0 ) || (setup.encoderZeroValueOK == 0 ) || (setup.encoderMagErrorSetupOK == 0)){
	  S.motorSetupFailed = E.errorFlag = E.motorEncoderSetupError = 1;
	  R.motorError = 1 << ERR_MES_SHIFT;
  }

  // Initialize all the sixSector Structs
  sixSectorInit(&sixSectorObj);
  sixSectorCntrlInit(&sixSectorCntrl_Obj);
  updateSectorBoundaries(&sixSectorObj,CCW);

  //Start 2 Timers - one for speed calculation and one for ramp calculation
  HAL_TIM_Base_Start_IT(&htim7); // 1 ms interrupt for speed calculation, speed is averaged over 16 readings.
  HAL_TIM_Base_Start_IT(&htim16);  //20ms timer for ramp

  //SetUP CAN
  S.CAN_ID= settingVarObj.MOTID ;//settingVarObj.MOTID; // should come from the EEprom
  MX_FDCAN1_Init();
  FDCAN_TxInit(); // this has to be here.

  //For configuration through the UART
  RetargetInit(&huart3);
  HAL_UART_Receive_IT(&huart3, start_string, 3);

  //Start the ADCs(after CANID is set)
  HAL_TIM_Base_Start_IT(&htim4);//start timer for adc1 trigger
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_buff,2);
  HAL_TIM_Base_Start_IT(&htim15);//start timer for adc2 trigger
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_buff,2);

  //GB Encoder Setup
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2); // DIRECT and on RISING EDGE
  HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_1); // FALLING EDGE

  HAL_Delay(100); //we need to allow the start the 100ms timer used to check encoder Health
  GB.firstReading = 1; // give some time from when the encoder start to before we take the first reading
  HAL_Delay(100); //another delay to get good readings on the encoder. these delays are very important
  //because immedietly after we are checking for extreme limits. TODO: re look at this.

  HAL_TIM_Base_Start_IT(&htim17);

  S.motorState = IDLE_STATE; // always start from IDLE

  HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Wait till CAN PING is done.
	  //send all Error MSgs once here.
	  if ((E.errorFlag == 1) && (S.errorMsgSentOnce == 0)){
		  S.motorState = ERROR_STATE;
		  FDCAN_errorFromMotor();
		  LRM.controlType = 0; // stop control loops
		  S.errorMsgSentOnce = 1;
	  }


	  //move motorState to CONFIGState when doing Console Work. will prevent running CAN-start/pause/halt msgs
	  //Also only allow when your in IDLE STATE
	  if (S.motorState == IDLE_STATE){
		  if(configuration_mode_flg){
			  S.motorState = CONFIG_STATE;
			  configurationFromTerminal(); // BLOCKING
			  configuration_mode_flg=0;
			  HAL_UART_Receive_IT(&huart3, start_string, 3);
			  S.motorState = IDLE_STATE;
		  }

		  // only allow in IDLE_STATE
		  //TODO Improvement:in both directions
		  if (doCalibration == 1){
			 S.motorState = CALIBRATION_STATE;
			 uint8_t zeroed = updateEncoderZeroPosition(0);
			 uint8_t calibrationError = 0;
			 if (zeroed){
				 MX_TIM1_Init();
				 RunCalibration(); // BLOCKING
				 calibrationError = 0;
			 }else{
				 calibrationError = 1;
			 }
			 doCalibration = 0;
			 S.motorState = IDLE_STATE;
		  }

		  if (GB.MB_request != GB_REQUEST_NONE){
			  send_GB_Request();
		  }

	  }

 	  /*HAL_GPIO_TogglePin(GPIOC,LED1_Pin);
	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(GPIOC,LED2_Pin);
	  HAL_Delay(100);
	  HAL_GPIO_TogglePin(GPIOC,LED3_Pin);
	  HAL_Delay(100);*/

	  //AUTO RESET this?
	  if (enable_overRideLiftLimitError){
		  overRideLiftError = 1;
		  E.liftOutOfBoundsError = 0;
		  E.errorFlag=0;
		  R.motorError = 0;
		  HAL_TIM_Base_Start_IT(&htim17);
		  S.motorState=IDLE_STATE;
		  enable_overRideLiftLimitError = 0;
	  }
	  if (disable_overRideLiftLimitError){
		  overRideLiftError = 0;
		  disable_overRideLiftLimitError = 0;
	  }

	  if (dbg_do_multiStroke_run){
		  //user has to set the right distance and direction
		  //for the first stroke.
		  if (Dbg_multiStrk.currentLayer == 0){
			  Dbg_multiStrk.dbg_multiStroke_On = 1;
			  Dbg_multiStrk.currentLayer += 1;
			  Dbg_multiStrk.uart_ms_timer = 0;
			  dbg_posCL_start = 1;
		  }
		  else{
			  if(Dbg_multiStrk.strokeOver == 1){
			  //start a new Stroke
			  LRM.direction = !LRM.direction;
			  Dbg_multiStrk.currentLayer += 1;
			  Dbg_multiStrk.strokeOver = 0;
			  if (Dbg_multiStrk.currentLayer >= Dbg_multiStrk.targetLayers){
				  dbg_stop_multiStroke_run = 1;
			  }
			  dbg_posCL_start = 1;
			  }
		  }

		if (Dbg_multiStrk.uart_ms_timer >=250){
			//direction, current layer,GB-absPos,posCL.currentDist,posRamp-currentVelocity,encPos-velocity,encPos-distErrorwithGB
			//before the dot is total no of chars, not chars before the point.
			sprintf(UART_buffer,"%01d,%03d,%06.2f,%06.2f,%03.2f,%3.2f,%05.2f\r\n",LRM.direction,Dbg_multiStrk.currentLayer,GB.absPosition,posCL.currentMoveDist_mm,posRamp.currentMoveVelocity,encPos.strokeVelocity_mm_sec,encPos.error_with_GB);
			HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,37);
			Dbg_multiStrk.uart_ms_timer = 0;
		}
	  }

	  if (dbg_stop_multiStroke_run){
		  Dbg_multiStrk.dbg_multiStroke_On = 0;
		  Dbg_multiStrk.currentLayer = 0;
		  Dbg_multiStrk.uart_ms_timer = 0;
		  dbg_posCL_stop = 1;
		  dbg_do_multiStroke_run = 0;
		  dbg_stop_multiStroke_run = 0;
	  }


	  /* TODO To check if the ABI connections are correct, read both the SPI and the Enc counts and make sure they are very close
   	   to each other and increase in the same direction! */
	  //ABI_mechAngle = getEncoderAngleFromABI(&htim2);
	  //SPI_mechAngle = getEncoderAngleFromSPI(1);

	  if (dbg){
		  //FDCAN_HomingDone(S.CAN_ID);//send FD can over msg
		  AS5047_checkEncoderHealth();
		  dbg = 0;
	  }
	  //START CLOSED LOOP
	  if (dbg_posCL_start){
		LRM.runType = LOCAL_DEBUG;
		LRM.controlType = CLOSED_LOOP;
		posCL_SetupMove(&posCL,GB.absPosition,LRM.distance,LRM.direction,LRM.time);
		EncPos_ZeroMovement(&encPos);
		posRamp_SetupRampTimes(&posRamp,800,800);
		posRamp_SetupMove(&posRamp,LRM.distance,LRM.time,LRM.direction);
		// we need to start the six sector Obj, and then start the Ramp
		StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,LRM.direction);
		posRamp_Start(&posRamp);
		dbg_posCL_start = 0;
	  }
	  //STOP CLOSED LOOP
	  if (dbg_posCL_stop){
		  StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
		  posRamp_Stop(&posRamp);
		  dbg_posCL_stop =0;
	  }

	  if (dbg_homing_start){
		  LRM.runType = LOCAL_DEBUG;
		  calculateHomeMove(&posPts,GB.PWM_cnts);
		  if (posPts.alreadyAtHome_Flag == 0){
			  S.motorState = HOMING_STATE;
			  if (posPts.homingControlType == CLOSED_LOOP){
				  LRM.controlType = CLOSED_LOOP;
				  posCL_SetupMove(&posCL,GB.absPosition,posPts.homingDistance,posPts.homingDirection,posPts.homingTime);
				  posRamp_SetupRampTimes(&posRamp,800,800);
				  posRamp_SetupMove(&posRamp,posPts.homingDistance,posPts.homingTime,posPts.homingDirection);
				  EncPos_ZeroMovement(&encPos);
				  // we need to start the six sector Obj, and then start the Ramp
				  StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,posPts.homingDirection);
				  posRamp_Start(&posRamp);
			  }else{
				  LRM.controlType = OPEN_LOOP;
				  uint16_t homingDuty = 300;
				  posOL_SetupMove(&posOL,GB.absPosition,posPts.homingDistance,posPts.homingDirection,homingDuty);
				  SetupLiftRampDuty(&liftRampDuty,homingDuty,1000,1000,RUN_FOREVER,400,800);
				  StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,posPts.homingDirection); // lift Down is CW, lift UP is CCW
				  StartLiftRampDuty(&liftRampDuty);
			  }
		  }
		  dbg_homing_start = 0;
	  }


	  if (dbg_posOL_start){
		LRM.controlType = OPEN_LOOP;
		LRM.runType = LOCAL_DEBUG;
		posOL_SetupMove(&posOL,GB.absPosition,LRM.distance,LRM.direction,LRM.duty);
		EncPos_ZeroMovement(&encPos);
		SetupLiftRampDuty(&liftRampDuty,LRM.duty,2000,2000,RUN_FOREVER,400,800);
		StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,LRM.direction); // lift Down is CW, lift UP is CCW
		StartLiftRampDuty(&liftRampDuty);
		dbg_posOL_start = 0;
	  }

	  if (dbg_posOL_stop){
		  StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
		  StopLiftRampDuty(&liftRampDuty);
		  dbg_posOL_stop =0;
	  }


	  //MAIN CONTROL FUNCTIONS FROM CAN COMMANDS

	  // **********1. START AFTER A DIAGNOSTICS FRAME *******************
	  if ((S.CAN_MSG == START) && (LRM.runType == DIAGNOSIS_RUN)){
		  if (S.RM_state == RECEIVED_RAMP_SETTINGS){
			  if (S.motorState == IDLE_STATE){
				  S.RM_state = NO_RM_MSG;
				  S.CAN_MSG = NO_MSG; // we ve used the MSG ,now discard it.
				  S.motorState = RUN_STATE;
				  if (LRM.controlType == OPEN_LOOP){
					  posOL_SetupMove(&posOL,GB.absPosition,LRM.distance,LRM.direction,LRM.duty);
					  SetupLiftRampDuty(&liftRampDuty,LRM.duty,LRM.rampUpTime_ms,LRM.rampDownTime_ms,RUN_FOREVER,400,800);
					  EncPos_ZeroMovement(&encPos);
					  StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,LRM.direction); // lift Down is CW, lift UP is CCW
					  StartLiftRampDuty(&liftRampDuty);
				  }
				  if (LRM.controlType == CLOSED_LOOP){
					  posCL_SetupMove(&posCL,GB.absPosition,LRM.distance,LRM.direction,LRM.time);
					  EncPos_ZeroMovement(&encPos);
					  posRamp_SetupRampTimes(&posRamp,LRM.rampUpTime_ms,LRM.rampDownTime_ms);
					  //TODO handle a zero for ramp up and rampdown time
					  posRamp_SetupMove(&posRamp,LRM.distance,LRM.time,LRM.direction);
					  StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,LRM.direction);
					  posRamp_Start(&posRamp);
				  }
				  //need to reset the timer to allow for calculation of Time to take place
				  T.tim16_oneSecTimer = 0;
				  T.tim16_20msTimer = 0;
			  }else{
				  S.CAN_MSG = NO_MSG; // To prevent any repeated Starts
			  }
		  }else{
			  S.CAN_MSG = NO_MSG;
		  }
	  }

	  // **********2. START AFTER A FULL CYCLE SETUP FRAME *******************
	  if ((S.CAN_MSG == START) && (LRM.runType == NORMAL_RUN)){
	  		  if (S.RM_state == RECEIVED_RAMP_SETTINGS){
	  			  if (S.motorState == IDLE_STATE){
	  				  S.RM_state = NO_RM_MSG;
	  				  S.CAN_MSG = NO_MSG; // we ve used the MSG ,now discard it.
	  				  S.motorState = RUN_STATE;
					  posCL_SetupMove(&posCL,GB.absPosition,LRM.distance,LRM.direction,LRM.time);
					  //TODO handle a zero for ramp up and rampdown time
					  posRamp_SetupRampTimes(&posRamp,LRM.rampUpTime_ms,LRM.directionChangeRamp_ms);
					  posRamp_SetupMove(&posRamp,LRM.distance,LRM.time,LRM.direction);
					  EncPos_ZeroMovement(&encPos);
					  posCL_setPkVelocity(&posCL,posRamp.pkVelocity); //needed to recalculate Time at the end of a layer
					  StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,LRM.direction);
					  posRamp_Start(&posRamp);
					  //sprintf(UART_buffer,"\r\n Start-%03.02f,%03.02f,%01d,%03.02f",LRM.distance,LRM.time,LRM.direction,posRamp.pkVelocity);
					  //HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,34);

					  //need to reset the timer to allow for calculation of Time to take place
	  				  T.tim16_oneSecTimer = 0;
	  				  T.tim16_20msTimer = 0;
	  			  }else{
	  				  S.CAN_MSG = NO_MSG; // To prevent any repeated Starts
	  			  }
	  		  }else{
	  			S.CAN_MSG = NO_MSG;
	  		  }
	  	  }

	  //***********3. RESUME FRAME ****************************
	  if ((S.CAN_MSG == RESUME_AFTER_PAUSE) && (LRM.runType == NORMAL_RUN)){
		  S.CAN_MSG = NO_MSG;
		  if (LRM.controlType == CLOSED_LOOP){
			  S.motorState = RUN_STATE;
			  LRM.logReturn = RUNTIME_LOG;
			  float remainingTime = RecalculateTime_OnResume(&posCL,&LRM);
			  posCL_SetupMove(&posCL,GB.absPosition,posCL.remainingDistance_mm,LRM.direction,remainingTime);
			  posRamp_SetupRampTimes(&posRamp,LRM.rampUpTime_ms,LRM.directionChangeRamp_ms);
			  posRamp_SetupMove(&posRamp,posCL.remainingDistance_mm,remainingTime,LRM.direction);
			  EncPos_ZeroMovement(&encPos);
			  posCL_setPkVelocity(&posCL,posRamp.pkVelocity); //needed to recalculate Time at the end of a layer
			  StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,LRM.direction);
			  posRamp_Start(&posRamp);
			  //sprintf(UART_buffer,"\r\n Resume-%03.02f,%03.02f",posRamp.pkVelocity,posRamp.dV_RD);
			  //HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,23);
			  //need to reset the timer to allow for calculation of Time to take place
			  T.tim16_oneSecTimer = 0;
			  T.tim16_20msTimer = 0;
		  }//DO nothing for open Loop
	  }


	  //***********4. PAUSE FRAME ****************************
	  if ((S.CAN_MSG == RAMPDOWN_STOP) && (LRM.runType == NORMAL_RUN)){
		  S.CAN_MSG = NO_MSG;
		  if (LRM.controlType == CLOSED_LOOP){
			  posRamp_updateRDTime(&posRamp, LRM.rampDownTime_ms);
			  posRamp.rampPhase = VELOCITY_PAUSE;
			  S.motorState = PAUSE_STATE;
			  //sprintf(UART_buffer,"\r\n Pause-%03.02f,%03.02f",posRamp.pkVelocity,posRamp.dV_RD);
			  //HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,21);
		  }//DO nothing for open Loop
	  }

	  // **********4.A NEW LAYER FRAME  *******************
	  if ((S.CAN_MSG == NEW_LAYER) && (LRM.runType == NORMAL_RUN)){
		  if (S.motorState == IDLE_STATE){
			  S.RM_state = NO_RM_MSG;
			  S.CAN_MSG = NO_MSG; // we ve used the MSG ,now discard it.
			  S.motorState = RUN_STATE;
			  posCL_SetupMove(&posCL,GB.absPosition,LRM.distance,LRM.direction,LRM.time);
			  posRamp_SetupRampTimes(&posRamp,LRM.directionChangeRamp_ms,LRM.directionChangeRamp_ms);
			  posRamp_SetupMove(&posRamp,LRM.distance,LRM.time,LRM.direction);
			  posCL_setPkVelocity(&posCL,posRamp.pkVelocity);
			  EncPos_ZeroMovement(&encPos);
			  StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,LRM.direction);
			  posRamp_Start(&posRamp);
			  //sprintf(UART_buffer,"\r\n NewLayer-%03.02f,%03.02f,%01d,%03.02f",LRM.distance,LRM.time,LRM.direction,posRamp.pkVelocity);
			  //HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,34);
			  //need to reset the timer to allow for calculation of Time to take place
			  T.tim16_oneSecTimer = 0;
			  T.tim16_20msTimer = 0;
		  }else{
			  S.CAN_MSG = NO_MSG; // To prevent any repeated Starts
		  }
	  }

	  if (S.CAN_MSG == HOMING){
		  S.motorState = HOMING_STATE;
		  LRM.logReturn = RUNTIME_LOG;
		  calculateHomeMove(&posPts,GB.PWM_cnts);
		  if (posPts.alreadyAtHome_Flag == 0){
			  if (posPts.homingControlType == CLOSED_LOOP){
				  LRM.controlType = CLOSED_LOOP;
				  posCL_SetupMove(&posCL,GB.absPosition,posPts.homingDistance,posPts.homingDirection,posPts.homingTime);
				  posRamp_SetupRampTimes(&posRamp,1000,1000);
				  posRamp_SetupMove(&posRamp,posPts.homingDistance,posPts.homingTime,posPts.homingDirection);
				  EncPos_ZeroMovement(&encPos);
				  // we need to start the six sector Obj, and then start the Ramp
				  StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,posPts.homingDirection);
				  posRamp_Start(&posRamp);
				  //sprintf(UART_buffer,"\r\n Homing-%03.02f,%03.02f,%01d",posPts.homingDistance,posPts.homingTime,posPts.homingDirection);
				  //HAL_UART_Transmit_IT(&huart3,(uint8_t *)UART_buffer,25);
			  }else{
				  LRM.controlType = OPEN_LOOP;
				  uint16_t homingDuty = 300;
				  posOL_SetupMove(&posOL,GB.absPosition,posPts.homingDistance,posPts.homingDirection,homingDuty);
				  SetupLiftRampDuty(&liftRampDuty,homingDuty,1000,1000,RUN_FOREVER,400,800);
				  StartSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj,posPts.homingDirection); // lift Down is CW, lift UP is CCW
				  StartLiftRampDuty(&liftRampDuty);
			  }
		  }else{
			  FDCAN_HomingDone(S.CAN_ID,HOMING_DONE);
			  S.motorState = IDLE_STATE;
		  }
		  S.CAN_MSG = NO_MSG;
	  }

	  if (S.CAN_MSG == EMERGENCY_STOP){
		  if((S.motorState == RUN_STATE) || (S.motorState == HOMING_STATE)){
			  S.CAN_MSG = NO_MSG;
			  S.recievedStopCommand = 1;
			  S.motorState = IDLE_STATE;
			  if (LRM.controlType == OPEN_LOOP){
				  StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
				  StopLiftRampDuty(&liftRampDuty);
			  }
			  if (LRM.controlType == CLOSED_LOOP){
				  StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
				  posRamp_Stop(&posRamp);
			  }
			  //state becomes IDLE where the rpm becomes zero in the interrupt
		  }else{
			  S.CAN_MSG = NO_MSG;
		  }
	  }

	  if (S.sendStrokeOver){
		  if (encSpeed.zeroSpeed){
				FDCAN_StrokeOver(S.CAN_ID,&LRM);
				S.sendStrokeOver = 0;
		  }
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_11;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 198;
  AnalogWDGConfig.LowThreshold = 154;
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T15_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_4;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 46;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;
  if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV10;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 6;
  hfdcan1.Init.NominalTimeSeg1 = 23;
  hfdcan1.Init.NominalTimeSeg2 = 6;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.DataTimeSeg1 = 11;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  FDCAN_RxFilterInit();
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x70A06EFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIMEx_EnableDeadTimePreload(&htim1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 90;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 2048;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 64999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 29999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 99;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1499;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1499;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 99;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1499;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 3999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 1499;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 9999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 2000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DAC_DBG_Pin|DBG_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|FAULT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_INDEX_Pin */
  GPIO_InitStruct.Pin = ENC_INDEX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_INDEX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_DBG_Pin DBG_OUT_Pin */
  GPIO_InitStruct.Pin = DAC_DBG_Pin|DBG_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin FAULT_LED_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|FAULT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
