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
static uint16_t theLinePosition;

//-----------------------------------------------------------------------------
//fonctions publiques
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de données de gInput
//------------------------------------------------------------------------
void gCompute_Setup(void)
    {
    gCompute.gCommandeMoteurDroit = 0;
    gCompute.gCommandeMoteurGauche = 0;
    gCompute.gCommandeServoDirection = 0;

    thePIDServo.kp = 1;
    thePIDServo.ki = 0;
    thePIDServo.kd = 0;
    thePIDServo.consigne = 64; //ligne au milieu du champ de vision (128/2)
    thePIDServo.erreurPrecedente = 0;
    thePIDServo.sommeErreurs = 0;
    }

//------------------------------------------------------------------------
// acquisition de l'état des switchs et mise à disposition de leur état
// aux autres modules via la structure des données de gInput
//------------------------------------------------------------------------
void gCompute_Execute(void)
    {
    if (TFC_Ticker[0] > 100 && LineScanImageReady == 1)
	{
	TFC_Ticker[0] = 0;
	LineScanImageReady = 0;

	//recherche de la ligne
	int16_t LineAnalyze[128];
	uint16_t theLinePosition;
	for (uint16_t i = 0; i < 128; i++)
	    {
	    LineAnalyze[i] = LineScanImage0[i];
	    }

	if (mTrackLine_FindLine(LineAnalyze, 128, &theLinePosition))
	    {
	    TFC_BAT_LED0_ON;
	    }
	else
	    {
	    TFC_BAT_LED0_OFF;
	    }
	}

    tPID(&thePIDServo, theLinePosition);

    gCompute.gCommandeServoDirection = thePIDServo.commande;
    gCompute.gCommandeMoteurGauche = 0;
    gCompute.gCommandeMoteurDroit = 0.1;

    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------
