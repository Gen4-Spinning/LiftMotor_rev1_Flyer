/*
 * Console.c
 *
 *  Created on: Mar 16, 2023
 *      Author: harsha
 */


#include "Console.h"
#include "EepromFns.h"

uint8_t configureMotorCANID(void){
	char option1;
	char eepromWrite = 0;
	int decimalOption1 = 0;
	uint8_t firstTime = 1;
	uint8_t skipScanf = 0;
	while(1){
		if (firstTime == 1){
			printf("***MOTOR CAN-ID MENU***");
			printf("\r\n Enter new Motor CAN ID");
			printf("\r\n MOTORIDs CARDING: CYLINDER-2,BEATER-3,COILER-4,CAGE-5,CREEL-6");
			printf("\r\n MOTORIDs DF: FR-2,BR-3,CREEL-4");
			printf("\r\n FLYER-2,BOBBIN-3,LiftLeft-4,LiftRight-5,FR-6,BR-7");
			printf("\r\n Press b and enter to go back");
			firstTime = 0;
			skipScanf = 0;
		}

		if (skipScanf == 0){
			scanf("%c[^\n]",&option1);
		}

		//ascii 2 - 7 is decimal 50 - 55
		if (eepromWrite == 0){
			if (option1 =='b'){
			  printf("\r\n");
			  break;
			}

			if ((option1 <= 55 ) && ( option1 >= 50)){
				decimalOption1 = option1 - 48;
				printf("\r\n you entered %d", decimalOption1);
				printf("\r\n Write into Eeprom and enable? (y/n)");
				eepromWrite = 1;
				option1 = 0;
			}
			skipScanf = 0;
		}else{
			if (option1 == 'y'){
				uint8_t success  = writeMotorSettingsToEEPROM_Manual(decimalOption1,-1,-1);
				if (success){
					printf("\r\n Success! Written into Eeprom!");
					settingVarObj.MOTID = decimalOption1;
				}else{
					printf("\r\n Fail! Not Written into Eeprom!");
				}
				eepromWrite = 0;
				option1 = 'b'; // go back into the main Menu
				skipScanf = 1;
			}
			if (option1 == 'n'){
				printf("\r\n Exited without writing into Eeprom!");
				printf("\r\n");
				firstTime = 1;
				eepromWrite = 0;
				skipScanf = 1;
			}
		}

    }
    return 1;
}



uint8_t configureMotorDefaultDirection(void){
	char option1;
	char eepromWrite = 0;
	uint8_t decimalOption1 = 0;
	uint8_t firstTime = 1;
	uint8_t skipScanf = 0;
	while(1){
		if (firstTime == 1){
			printf("***MOTOR DEFAULT-DIRECTION MENU***");
			printf("\r\n Enter Motor Default Direction");
			printf("\r\n CW-0,CCW-1");
			printf("\r\n Press b and enter to go back");
			firstTime = 0;
			skipScanf = 0;
		}

		if (skipScanf == 0){
			scanf("%c[^\n]",&option1);
		}

		//ascii 0 - 1 is decimal 50 - 51
		if (eepromWrite == 0){
			if (option1 =='b'){
			  printf("\r\n");
			  break;
			}

			if ((option1 <= 49 ) && ( option1 >= 48)){
				decimalOption1 = option1 - 48;
				printf("\r\n you entered %d", decimalOption1);
				printf("\r\n Write into Eeprom and enable? (y/n)");
				eepromWrite = 1;
				option1 = 0;
			}
			skipScanf = 0;
		}else{
			if (option1 == 'y'){
				uint8_t success  = writeMotorSettingsToEEPROM_Manual(-1,-1,decimalOption1);
				if (success){
					printf("\r\n Success! Written into Eeprom!");
					settingVarObj.default_direction = decimalOption1;
				}else{
					printf("\r\n Fail! Not Written into Eeprom!");
				}
				eepromWrite = 0;
				option1 = 'b'; // go back into the main Menu
				skipScanf = 1;
			}
			if (option1 == 'n'){
				printf("\r\n Exited without writing into Eeprom!");
				printf("\r\n");
				firstTime = 1;
				eepromWrite = 0;
				skipScanf = 1;
			}
		}

    }
    return 1;
}


uint8_t configure_Encoder_Offset(void){
	int option1;
	char eepromWrite = 0;
	uint8_t firstTime = 1;
	uint8_t skipScanf = 0;
	int16_t decimalOption1 = 0;
	while(1){
		if (firstTime == 1){
			printf("***MOTOR ENCODER-OFFSET MENU***");
			printf("\r\n Enter Encoder Offset (0-16384)");
			printf("\r\n Enter -1 and enter to go back");
			firstTime = 0;
			skipScanf = 0;
			eepromWrite = 0;
		}

		if (skipScanf == 0){
			scanf("%d[^\n]",&option1);
		}

		if (eepromWrite == 0){
			if (option1 == -1){
			  printf("\r\n");
			  break;
			}

			if (option1 <= 16384 ){
				decimalOption1 = option1;
				printf("\r\n you entered %d", option1);
				printf("\r\n Write into Eeprom and enable? (enter 1 for yes/ 0 for no)");
				eepromWrite = 1;
				option1 = 0;
			}
			skipScanf = 0;
		}else{
			if (option1 == 1){
				uint8_t success  = writeMotorSettingsToEEPROM_Manual(-1,decimalOption1,-1);
				if (success){
					printf("\r\n Success! Written into Eeprom!");
					settingVarObj.AMS_offset_index = decimalOption1;
				}else{
					printf("\r\n Fail! Not Written into Eeprom!");
				}
				eepromWrite = 0;
				option1 = -1; // go back into the main Menu
				skipScanf = 1;
			}
			if (option1 == 0){
				printf("\r\n Exited without writing into Eeprom!");
				printf("\r\n");
				firstTime = 1;
				eepromWrite = 0;
				skipScanf = 1;
			}
		}

    }
    return 1;

}


uint8_t printSettings(void){
	printf("***MOTOR SETTINGS****");
	printf("\r\n MOTORIDs CARDING: CYLINDER-2,BEATER-3,COILER-4,CAGE-5,CREEL-6");
	printf("\r\n MOTORIDs DF: FR-2,BR-3,CREEL-4");
	printf("\r\n MOTORIDs FLYER : FLYER-2,BOBBIN-3,LiftLeft-4,LiftRight-5,FR-6,BR-7");
	printf("\r\n MOTOR CAN ID = %d",settingVarObj.MOTID);
	printf("\r\n");
	printf("\r\n MOTOR Encoder Offset = %d",settingVarObj.AMS_offset_index);
	printf("\r\n");
	printf("\r\n CW-0,CCW-1");
	printf("\r\n MOTOR Default Dir = %d",settingVarObj.default_direction);
	printf("\r\n");
	return 1;
}
/*
void runMotorCalibrationRoutine(void){
	printf("***MOTOR CALIBRATION ROUTINE****");
	printf("\r\n Start the Calibration Routine?(y/n)");
	scanf("%c[^\n]",&option1);
	if (option1 == 'y'){
		printf("\r\n running Calibration");
		currentStep = 0;
	}
}*/


void configurationFromTerminal(void)
{	  char option0;
	  char firstTime = 1;
	  while(1){
		  if (firstTime == 1){
			  printf("***MOTOR CONFIGURATION MENU***");
			  printf("\r\n 1. View Motor Settings");
			  printf("\r\n 2. Change Motor CAN ID");
			  printf("\r\n 3. Change Motor Encoder Offset");
			  printf("\r\n 4. Change Default Direction");
			  printf("\r\n 5. Run Encoder Calibration Routine");
			  printf("\r\n 6. Run Motor OpenLoop Tests");
			  printf("\r\n 7. Enable UART debug On");
			  printf("\r\n Enter a no btw 1-5 and enter to select an option");
			  printf("\r\n Press q and enter from this menu to exit \r\n");
			  firstTime = 0;
		  }

		  scanf("%c[^\n]",&option0);
		  if (option0 == 'q'){
			  printf("\r\n Bye!");
			  printf("\r\n ************");
			  break;
		  }

		  if (option0 == '1'){
			  firstTime = printSettings();
			  option0 = 0;
		  }
		  if (option0 == '2'){
			  firstTime = configureMotorCANID();
			  option0 = 0;
		  }
		  if (option0 == '3'){
			  firstTime = configure_Encoder_Offset();
			  option0 = 0;
		  }
		  if (option0 == '4'){
			  firstTime = configureMotorDefaultDirection();
			  option0 = 0;
		  }
		  /*if (option0 == '5'){

		  }*/
	  }

}
