/*
 * mMotor.h
 *
 *  Created on: Nov 1, 2013
 *      Author: Michael
 */

#ifndef MMOTOR_H_
#define MMOTOR_H_

#include "Tools/tPID.h"

typedef struct {
	tPIDStruct aPIDData;
	uint8_t aStopped;
	uint8_t aOverflowOld;
	uint16_t aCapt ;
	float aSpeed ;	
	void(*mMotor_mesure)(void);
} mMotorStruct;

void mMotor_mSetup();
void mMotor_mOpen();



#endif /* MMOTOR_H_ */
