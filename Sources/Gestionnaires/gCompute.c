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
#include "Tools\Tools.h"
#include "Modules\mTrackline.h"

#define kTailleFiltre 16

/* prototypes des fonctions statiques (propres au fichier) */
static tPIDStruct thePIDServo;

//-----------------------------------------------------------------------------
//fonctions publiques
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de données de gCompute
//------------------------------------------------------------------------
void gCompute_Setup(void)
    {
    gComputeInterStruct.gCommandeMoteurDroit = 0.0;
    gComputeInterStruct.gCommandeMoteurGauche = 0.0;
    gComputeInterStruct.gCommandeServoDirection = 0;

    thePIDServo.kp = 0.006; //val max pour kp servo = 1/64
    thePIDServo.ki = 0.00;
    thePIDServo.kd = 0.006;
    thePIDServo.consigne = 0; //ligne au milieu du champ de vision (étendue de -64 à 64)
    thePIDServo.erreurPrecedente = 0;
    thePIDServo.sommeErreurs = 0;
    thePIDServo.coeffNormalisation = 1.0 / 64.0;
    }

//------------------------------------------------------------------------
// calcul des commandes à mettre pour les moteurs et le servo direction
//
//------------------------------------------------------------------------
void gCompute_Execute(void)
    {
    //TODO : séparer le code dans différentes fonctions afin d'améliorer la lisibilité

    //lecture des donnees provenant du monitoring
    if (gXbeeInterStruct.aPIDChangedServo)
	{
	thePIDServo.kp = gXbeeInterStruct.aGainPIDServo.gProprortionalGain;
	thePIDServo.ki = gXbeeInterStruct.aGainPIDServo.gIntegraleGain;
	thePIDServo.kd = gXbeeInterStruct.aGainPIDServo.gDerivativeGain;
	}

    //recherche de la ligne
    static int16_t theLinePosition = 0;
    bool isLineFound;
    bool isStartStopFound;
    int16_t LineAnalyze[128];
    for (uint16_t i = 0; i < 128; i++)
	{
	LineAnalyze[i] = LineScanImage0[i];
	}
    mTrackLine_FindLine(LineAnalyze, 128, &theLinePosition, &isLineFound,
	    &isStartStopFound);
    //---------------------------------------------------------------------------
    //si la ligne est trouvee
    if (isLineFound)
	{
	TFC_BAT_LED0_ON;

	//filtrage de la position de la ligne
	static uint8_t posFiltre = 0;
	static int16_t theLinePositionTab[kTailleFiltre];

	theLinePositionTab[posFiltre] = theLinePosition;
	if (posFiltre < kTailleFiltre - 1)
	    {
	    posFiltre++;
	    }
	else
	    {
	    posFiltre = 0;
	    }
	theLinePosition = tMean(theLinePositionTab, kTailleFiltre);
	}
    else
	{
	TFC_BAT_LED0_OFF;
	}
    // Mettre a jour le PID
    tPID(&thePIDServo, (theLinePosition - 64));

    //---------------------------------------------------------------------------
    //TODO : faire une machine d'états pour contrôler la course(départ, arrivée, virage, ligne droite) afin de donner une bonne consigne aux moteurs
    //si la ligne d'arrivee est trouvee
    static bool isInRace = false;
    static bool oldIsStartStopFound = false;
    if (isStartStopFound && !(oldIsStartStopFound))
	{
	if (isInRace == true)
	    {
	    isInRace = false;
	    }
	else
	    {
	    isInRace = true;
	    }
	}
    oldIsStartStopFound = isStartStopFound;
    if (isInRace)
	{
	TFC_BAT_LED1_ON;
	}
    else
	{
	TFC_BAT_LED1_OFF;
	}

    //TODO : ajouter le contrôle des moteurs (différentiel, filtrage, PID)

    //---------------------------------------------------------------------------
    gInputInterStruct.gPosCam1 = theLinePosition;
    gComputeInterStruct.gCommandeServoDirection = thePIDServo.commande;
    gComputeInterStruct.gCommandeMoteurGauche = 0;
    gComputeInterStruct.gCommandeMoteurDroit = gXbeeInterStruct.aMotorSpeedCons
	    / 100.0;

    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------
