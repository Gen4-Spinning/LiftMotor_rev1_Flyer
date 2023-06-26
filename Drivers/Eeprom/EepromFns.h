/*
 * EepromFns.h
 *
 *  Created on: 07-Mar-2023
 *      Author: Jonathan
 */

#ifndef EEPROM_EEPROMFNS_H_
#define EEPROM_EEPROMFNS_H_

#include "Struct.h"

void settingsInit(settingVar *stV);
void readSettingsFromEEPROM(settingVar *sV);
uint8_t writePWMSettingsToEEPROM(settingVar *stV);
uint8_t writeMotorSettingsToEEPROM(settingVar *stV);
uint8_t writeMotorSettingsToEEPROM_Manual(int8_t motorID, int16_t AMS_offset,int16_t default_direction);
void loadPWMDefaultSettings(settingVar *sV);
uint8_t checkEEPROM_PWMSettings(settingVar *sV);
uint8_t checkEEPROM_MotorSettings(settingVar *sV);
uint8_t writeHomingPositionToEeprom(uint16_t homingPosition);
uint16_t readHomingPositionFromEeprom(void);

#endif /* EEPROM_EEPROMFNS_H_ */
