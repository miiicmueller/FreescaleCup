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
#include "Modules/mMotor.h"

#define kTailleFiltre 16

/* prototypes des fonctions statiques (propres au fichier) */
static tPIDStruct thePIDServo;

//-----------------------------------------------------------------------------
//fonctions publiques
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de donn�es de gCompute
//------------------------------------------------------------------------
void gCompute_Setup(void)
    {
    gComputeInterStruct.gCommandeMoteurDroit = 0.0;
    gComputeInterStruct.gCommandeMoteurGauche = 0.0;
    gComputeInterStruct.gCommandeServoDirection = 0;

    thePIDServo.kp = 0.006; //val max pour kp servo = 1/64
    thePIDServo.ki = 0.00;
    thePIDServo.kd = 0.006;
    thePIDServo.consigne = 0; //ligne au milieu du champ de vision (�tendue de -64 � 64)
    thePIDServo.erreurPrecedente = 0;
    thePIDServo.sommeErreurs = 0;
    thePIDServo.coeffNormalisation = 1.0 / 64.0;
    }

//------------------------------------------------------------------------
// calcul des commandes � mettre pour les moteurs et le servo direction
//
//------------------------------------------------------------------------
void gCompute_Execute(void)
    {
    //TODO : s�parer le code dans diff�rentes fonctions afin d'am�liorer la lisibilit�

    uint32_t valExposure = 0;

    //lecture des donnees provenant du monitoring
    if (gXbeeInterStruct.aPIDChangedServo)
	{
	thePIDServo.kp = gXbeeInterStruct.aGainPIDServo.gProprortionalGain;
	thePIDServo.ki = gXbeeInterStruct.aGainPIDServo.gIntegraleGain;
	thePIDServo.kd = gXbeeInterStruct.aGainPIDServo.gDerivativeGain;
	}

    //lecture des donnees provenant du monitoring
    if (gXbeeInterStruct.aPIDChangedMotors)
	{
	//Motor1
	mMotor1.aPIDData.kp =
		gXbeeInterStruct.aGainPIDMotors.gProprortionalGain;
	mMotor1.aPIDData.ki = gXbeeInterStruct.aGainPIDMotors.gIntegraleGain;
	mMotor1.aPIDData.kd = gXbeeInterStruct.aGainPIDMotors.gDerivativeGain;

	//Motor 2
	mMotor2.aPIDData.kp =
		gXbeeInterStruct.aGainPIDMotors.gProprortionalGain;
	mMotor2.aPIDData.ki = gXbeeInterStruct.aGainPIDMotors.gIntegraleGain;
	mMotor2.aPIDData.kd = gXbeeInterStruct.aGainPIDMotors.gDerivativeGain;
	}

    valExposure =
	    (uint32_t) (((gXbeeInterStruct.aExpTime + 1.0) * 5000.0) + 1.0);

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
    //TODO : faire une machine d'�tats pour contr�ler la course(d�part, arriv�e, virage, ligne droite) afin de donner une bonne consigne aux moteurs
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

    //TODO : ajouter le contr�le des moteurs (diff�rentiel, filtrage, PID)

    //---------------------------------------------------------------------------
    //Appel des PID des moteurs
    // PID Moteur 1
    tPID(&mMotor1.aPIDData, (int16_t) (mMotor1.aFreq / 3.0)); // Frequence entre 0 et 100
    // PID Moteur 2
    tPID(&mMotor2.aPIDData, (int16_t) (mMotor2.aFreq / 3.0));

    gInputInterStruct.gPosCam1 = theLinePosition;
    gComputeInterStruct.gCommandeServoDirection = thePIDServo.commande;

    gComputeInterStruct.gCommandeMoteurGauche =
	    gXbeeInterStruct.aMotorSpeedCons; // mMotor1.aPIDData.commande;
    gComputeInterStruct.gCommandeMoteurDroit = gXbeeInterStruct.aMotorSpeedCons; // mMotor2.aPIDData.commande ;
//    gComputeInterStruct.gCommandeMoteurGauche = 0.8;
//    gComputeInterStruct.gCommandeMoteurDroit = 0.8;

    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------
