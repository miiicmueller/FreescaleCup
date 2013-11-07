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
// Initialisation de la structure de données de gInput
//------------------------------------------------------------------------
void gOutput_Setup(void)
    {
    TFC_SetServo(1, 0);

    TFC_SetMotorPWM(0, 0); //Make sure motors are off
    TFC_HBRIDGE_ENABLE;
    }

//------------------------------------------------------------------------
// acquisition de l'état des switchs et mise à disposition de leur état
// aux autres modules via la structure des données de gInput
//------------------------------------------------------------------------
void gOutput_Execute(void)
    {
    if ((TFC_Ticker[1] >= 20) && (gCompute.isFinish == true))
	{
	TFC_Ticker[1] = 0;
	gCompute.isFinish == false;

	TFC_SetServo(1, gCompute.gCommandeServoDirection);

	TFC_SetMotorPWM(gCompute.gCommandeMoteurDroit,
		gCompute.gCommandeMoteurGauche); //consignes de vitesse
	}
    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------
