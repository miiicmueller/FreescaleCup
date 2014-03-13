/*
 * mMotor.h
 *
 *  Created on: Nov 1, 2013
 *      Author: Michael
 */

#ifndef MMOTOR_H_
#define MMOTOR_H_

#include "Tools/tRegulateurs.h"

#define FILTER_SIZE	7
#define F_COUNT		6000000.0

typedef struct
    {
	float aDifferential; //rapport a appliquer a la vitesse  domaine [0.66 ; 1]
	tRegulateurPIDStruct aPIDData;
	uint8_t aStopped;
	int8_t aDir; // Direction : 1 ou -1  
	uint16_t aOverflowOld;
	uint32_t aCapt;
	float aFreq;
	char aNumEchantillonsMot;
	void (*mMotor_mesure)(void);
    } mMotorStruct;

//exportation des variables
extern mMotorStruct mMotor1;
extern mMotorStruct mMotor2;

void mMotor_mSetup();
void mMotor_mOpen();

#endif /* MMOTOR_H_ */
