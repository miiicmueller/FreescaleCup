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
#include "Modules/hal_dev_mma8451.h"

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

    //Moteur 1 et 2 ;
    static uint8_t posFiltre = 0;

    mMotor1.aCaptTab[posFiltre] = mMotor1.aCapt;
    mMotor2.aCaptTab[posFiltre] = mMotor2.aCapt;
    if (posFiltre < FILTER_SIZE - 1)
	{
	posFiltre++;
	}
    else
	{
	posFiltre = 0;
	}

    mMotor1.aPIDData.consigne = (int16_t) (gXbeeInterStruct.aMotorSpeedCons);
    mMotor2.aPIDData.consigne = (int16_t) (gXbeeInterStruct.aMotorSpeedCons);

    }

