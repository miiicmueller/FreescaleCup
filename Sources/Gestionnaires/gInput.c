/*
 * gInput.c
 *
 *  Created on: Nov 1, 2013
 *      Author: cyrille.savy
 */

#include "TFC\TFC.h"
#include "Gestionnaires\gInput.h"
#include "Gestionnaires\gMbox.h"
#include "Modules\mMotor.h"

/* prototypes des fonctions statiques (propres au fichier) */

//-----------------------------------------------------------------------------
// fonctions publiques
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de donn�es de gInput
//------------------------------------------------------------------------
void gInput_Setup(void)
    {
    gInputInterStruct.gFreq[0] = (float) 0.0;
    gInputInterStruct.gFreq[1] = (float) 0.0;
    }

//------------------------------------------------------------------------
// acquisition de l'�tat des switchs et mise � disposition de leur �tat
// aux autres modules via la structure des donn�es de gInput
//------------------------------------------------------------------------
void gInput_Execute(void)
    {

    //Moteur 1 ;
    if (mMotor1.aNumEchantillonsMot >= (FILTER_SIZE - 1))
	{
	mMotor1.aCaptTab[0] = mMotor1.aCapt;
	}
    else
	{
	mMotor1.aCaptTab[(FILTER_SIZE - 1) - mMotor1.aNumEchantillonsMot] =
		mMotor1.aCapt;
	mMotor1.aNumEchantillonsMot++;
	gInputInterStruct.gFreq[0] = (F_COUNT) / mMotor1.aCapt;
	}

    //Moteur 2 ;
    if (mMotor2.aNumEchantillonsMot >= (FILTER_SIZE - 1))
	{
	mMotor2.aCaptTab[0] = mMotor2.aCapt;
	}
    else
	{
	mMotor2.aCaptTab[(FILTER_SIZE - 1) - mMotor2.aNumEchantillonsMot] =
		mMotor2.aCapt;
	mMotor2.aNumEchantillonsMot++;
	gInputInterStruct.gFreq[1] = (F_COUNT) / mMotor2.aCapt;
	}

    mMotor1.aPIDData.consigne = (int16_t) (gXbeeInterStruct.aMotorSpeedCons);
    mMotor2.aPIDData.consigne = (int16_t) (gXbeeInterStruct.aMotorSpeedCons);

    }
