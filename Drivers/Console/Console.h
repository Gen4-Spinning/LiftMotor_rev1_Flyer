/*
 * Console.h
 *
 *  Created on: Mar 16, 2023
 *      Author: harsha
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <stdio.h>
#include "Struct.h"


void configurationFromTerminal(void);
uint8_t configureMotorCANID(void);
uint8_t configure_Encoder_Offset(void);
uint8_t configureMotorDefaultDirection(void);
uint8_t printSettings(void);

#endif /* CONSOLE_H_ */
