/*
 * gCompute.c
 *
 *  Created on: Nov 1, 2013
 *      Author: cyrille.savy
 */

#include "TFC\TFC.h"
#include "Gestionnaires\gCompute.h"
#include "Gestionnaires\gMbox.h"
#include "Tools\tPID.h"
#include "Modules\mTrackline.h"

/* prototypes des fonctions statiques (propres au fichier) */
static tPIDStruct thePIDServo;
static int16_t theLinePosition;

//-----------------------------------------------------------------------------
//fonctions publiques
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de données de gCompute
//------------------------------------------------------------------------
void gCompute_Setup(void)
    {
    gComputeInterStruct.isFinish = false;

    gComputeInterStruct.gCommandeMoteurDroit = 0;
    gComputeInterStruct.gCommandeMoteurGauche = 0;
    gComputeInterStruct.gCommandeServoDirection = 0;

    thePIDServo.kp = 0.006; //val max pour kp servo = 1/64
    thePIDServo.ki = 0.00;
    thePIDServo.kd = 0.006;
    thePIDServo.consigne = 0; //ligne au milieu du champ de vision (étendue de -64 à 64)
    thePIDServo.erreurPrecedente = 0;
    thePIDServo.sommeErreurs = 0;
    }

//------------------------------------------------------------------------
// calcul des commandes à mettre pour les moteurs et le servo direction
//
//------------------------------------------------------------------------
void gCompute_Execute(void)
    {
    if ((TFC_Ticker[0] >= 20) && (LineScanImageReady == 1)
	    && (gComputeInterStruct.isFinish == false))
	{
	bool isLineFound;
	bool isStartStopFound;

	TFC_Ticker[0] = 0;
	LineScanImageReady = 0;
	gComputeInterStruct.isFinish = true;

	//recherche de la ligne
	int16_t LineAnalyze[128];
	for (uint16_t i = 0; i < 128; i++)
	    {
	    LineAnalyze[i] = LineScanImage0[i];
	    }

	mTrackLine_FindLine(LineAnalyze, 128, &theLinePosition, &isLineFound,
		&isStartStopFound);
	if (isLineFound)
	    {
	    TFC_BAT_LED0_ON;
	    tPID(&thePIDServo, (theLinePosition - 64));
	    }
	else
	    {
	    TFC_BAT_LED0_OFF;
	    }
	if (isStartStopFound)
	    {
	    TFC_BAT_LED1_ON;
	    }
	else
	    {
	    TFC_BAT_LED1_OFF;
	    }
	}

    gInputInterStruct.gPosCam1 = theLinePosition;
    gComputeInterStruct.gCommandeServoDirection = thePIDServo.commande;
    gComputeInterStruct.gCommandeMoteurGauche = 0;
    gComputeInterStruct.gCommandeMoteurDroit = 0;

    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------
