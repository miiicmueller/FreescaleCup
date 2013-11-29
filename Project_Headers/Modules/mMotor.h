/*
 * mMotor.h
 *
 *  Created on: Nov 1, 2013
 *      Author: Michael
 */

#ifndef MMOTOR_H_
#define MMOTOR_H_

#include "Tools/tPID.h"

typedef struct
    {
	tPIDStruct aPIDData;
	uint8_t aStopped;
	uint8_t aOverflowOld;
	uint32_t aCapt;
	float aFreq ;
	void(*mMotor_mesure)(void);
    } mMotorStruct;

//exportation des variables
extern mMotorStruct mMotor1;
extern mMotorStruct mMotor2;

void mMotor_mSetup();
void mMotor_mOpen();

#endif /* MMOTOR_H_ */
