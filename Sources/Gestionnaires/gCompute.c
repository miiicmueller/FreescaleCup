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

#define kTailleFiltre 15
#define T_ERROR_MAX_BREAK 30
#define K_BREAK_FACTOR 0.6

/* prototypes des fonctions statiques (propres au fichier) */
static tPIDStruct thePIDServo;
//-----------------------------------------------------------------------------
// Compute differential
// param : aAngleServo --> Consigne du servoMoteur
//
// Description : 	Avec aAngleServo a 0.51, pour 1 tour de la roue exterieur
//					la roue interieur en fait 0.66.
//
//					Cette fonction calcul le differentiel à appliquer
//					aux moteurs selon aAngleServo.
//-----------------------------------------------------------------------------
static void compute_differential(const float aAngleServo);

float aFreqMesTabMot1[FILTER_SIZE] =
    {
    0.0
    };

float aFreqMesTabMot2[FILTER_SIZE] =
    {
    0.0
    };

//-----------------------------------------------------------------------------
//fonctions publiques
//-----------------------------------------------------------------------------
uint32_t median_filter_n(uint32_t *aTab, char aSize);

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
	static uint32_t theLinePositionTab[kTailleFiltre];

	theLinePositionTab[posFiltre] = theLinePosition;
	if (posFiltre < kTailleFiltre - 1)
	    {
	    posFiltre++;
	    }
	else
	    {
	    posFiltre = 0;
	    }

	theLinePosition = median_filter_n(theLinePositionTab, kTailleFiltre);
	//theLinePosition = tMean(theLinePositionTab, kTailleFiltre);
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
	//TFC_HBRIDGE_DISABLE;

	}

    //---------------------------------------------------------------------------
    //Filtrage des valeur du moteur
    //Moteur 1 ;
    uint32_t aCaptMedian = 0;
    aCaptMedian = median_filter_n(mMotor1.aCaptTab, FILTER_SIZE);
    if (aCaptMedian != 0)
	{
	gInputInterStruct.gFreq[0] = (float) (F_COUNT) / aCaptMedian;
	}
    else
	{
	gInputInterStruct.gFreq[0] = 0.0;
	}

    //Moteur 2 ;
    aCaptMedian = median_filter_n(mMotor2.aCaptTab, FILTER_SIZE);
    if (aCaptMedian != 0)
	{
	gInputInterStruct.gFreq[1] = (float) (F_COUNT) / aCaptMedian;
	}
    else
	{
	gInputInterStruct.gFreq[1] = 0.0;
	}

    compute_differential(gComputeInterStruct.gCommandeServoDirection);

    mMotor1.aPIDData.consigne *= mMotor1.aDifferential;
    mMotor2.aPIDData.consigne *= mMotor2.aDifferential;

    //---------------------------------------------------------------------------
    //Appel des PID des moteurs
    // PID Moteur 1
    tPID(&mMotor1.aPIDData, (int16_t) (gInputInterStruct.gFreq[0] / 3.0)); // Frequence entre 0 et 100
    // PID Moteur 2
    tPID(&mMotor2.aPIDData, (int16_t) (gInputInterStruct.gFreq[1] / 3.0));

    //---------------------------------------------------------------------------
    //mise à jour des sorties de gCompute
    gInputInterStruct.gPosCam1 = theLinePosition;
    gComputeInterStruct.gCommandeServoDirection = thePIDServo.commande;

    if (mMotor1.aPIDData.commande < 0.0)
	{
	mMotor1.aPIDData.commande = 0.0;
	}
    if (mMotor2.aPIDData.commande < 0.0)
	{
	mMotor2.aPIDData.commande = 0.0;
	}
    gComputeInterStruct.gCommandeMoteurGauche = mMotor1.aPIDData.commande;
    //gComputeInterStruct.gCommandeMoteurGauche; // mMotor1.aPIDData.commande;

    gComputeInterStruct.gCommandeMoteurDroit = mMotor2.aPIDData.commande;
    //gComputeInterStruct.gCommandeMoteurDroit; //mMotor2.aPIDData.commande;

    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------

uint32_t median_filter_n(uint32_t *aTab, char aSize)
    {
    uint32_t aCpyTab[20];
    signed char i = 0, j = 0;
    uint32_t aTemp;

//Copie du tableau
    for (i = 0; i < aSize; i++)
	{
	aCpyTab[i] = aTab[i];
	}

//On trie le tableau
    for (i = 0; i < aSize; i++)
	{
	for (j = 0; j < (aSize - 1); j++)
	    {
	    if (aCpyTab[j] > aCpyTab[j + 1])
		{
		// On swap les deux
		aTemp = aCpyTab[j + 1];
		aCpyTab[j + 1] = aCpyTab[j];
		aCpyTab[j] = aTemp;
		}
	    }
	}

//On prend la valeur du milieu
    return aCpyTab[(aSize - 1) / 2];
    }

static void compute_differential(const float aAngleServo)
    {

    float m = -0.6667; //-0.34 / 0.51;
    float speedScaleFactor = -0.5 * tAbs_float(aAngleServo) + 1.0;

    if (aAngleServo >= 0.0)
	{
	mMotor1.aDifferential = speedScaleFactor;
	mMotor2.aDifferential = (m * tAbs_float(aAngleServo) + 1.0)
		* speedScaleFactor;

	}
    else
	{
	mMotor1.aDifferential = (m * tAbs_float(aAngleServo) + 1.0)
		* speedScaleFactor;
	mMotor2.aDifferential = speedScaleFactor;
	}
    }

