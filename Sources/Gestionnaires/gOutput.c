/*
 * gOutput.c
 *
 *  Created on: Nov 1, 2013
 *      Author: cyrille.savy
 */

#include "Gestionnaires\gOutput.h"
#include "TFC\TFC.h"
#include "Gestionnaires\gMbox.h"

/* prototypes des fonctions statiques (propres au fichier) */

//-----------------------------------------------------------------------------
//fonctions publiques
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de données de gOutput
//------------------------------------------------------------------------
void gOutput_Setup(void)
    {
    TFC_SetServo(1, 0);

    TFC_SetMotorPWM(0, 0); //Make sure motors are off
    TFC_HBRIDGE_ENABLE;
    }

//------------------------------------------------------------------------
// mise à jour des commandes des moteurs et du servo de direction
//
//------------------------------------------------------------------------
void gOutput_Execute(void)
    {
    if ((TFC_Ticker[1] >= 20) && (gComputeInterStruct.isFinish == true))
	{
	TFC_Ticker[1] = 0;
	gComputeInterStruct.isFinish = false;

	TFC_SetServo(0, gComputeInterStruct.gCommandeServoDirection);

	TFC_SetMotorPWM(gComputeInterStruct.gCommandeMoteurDroit,
		gComputeInterStruct.gCommandeMoteurGauche); //consignes de vitesse
	}
    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------
