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
    //consigne de direction
    TFC_SetServo(0,
	    gComputeInterStruct.gCommandeServoDirection +TFC_ReadPot(0));

    //consignes de vitesse
    TFC_SetMotorPWM(gComputeInterStruct.gCommandeMoteurDroit,
	    gComputeInterStruct.gCommandeMoteurGauche);

    //temps d'exposition et eclairage des leds pour la camera
    TFC_SetLineScanExposureTime(
	    (uint32_t) (((gXbeeInterStruct.aExpTime + 1.0) * 5000.0) + 1.0));
    mLeds_writeDyC(gXbeeInterStruct.aPWMLeds);

    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------
