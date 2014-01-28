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
#include "Tools/angle_cal.h"
#include "Modules\mTrackline.h"
#include "Modules/mMotor.h"
#include "parameters.h"

#define kTailleFiltre 11
#define T_ERROR_MAX_BREAK 30
#define K_BRAKE_FACTOR 0.4
#define K_BRAKE_FACTOR_DER 0.0

#define K_LOST_MAX_NUM 300  // 3s
/* prototypes des fonctions statiques (propres au fichier) */
static tPIDStruct thePIDServo;
static bool full_break = false;
static bool half_break = false;
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
static void compute_differential(const float aAngleServo,
	tPIDStruct* thePIDStruct);

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
    gComputeInterStruct.gCommandeServoDirection = 0.0;

    thePIDServo.kp = 0.3; //val max pour kp servo = 1/64
    thePIDServo.ki = 0;
    thePIDServo.kd = 0;
    thePIDServo.kp_init = thePIDServo.kp;
    thePIDServo.consigne = 0; //ligne au milieu du champ de vision (étendue de -64 à 64)
    thePIDServo.erreurPrecedente = 0;
    thePIDServo.sommeErreurs = 0;
    thePIDServo.coeffNormalisation = 1.0 / 64.0;
    thePIDServo.posFiltre = 0;
    thePIDServo.thePastError[0] = 0.0;
    thePIDServo.thePastError[1] = 0.0;
    thePIDServo.thePastError[2] = 0.0;

    gComputeInterStruct.aAccelPlanAngle[0] = 0;
    gComputeInterStruct.aAccelPlanAngle[1] = 0;
    gComputeInterStruct.aAccelPlanAngle[2] = 0;

    gComputeInterStruct.aAccelPlanMagn[0] = 0;
    gComputeInterStruct.aAccelPlanMagn[1] = 0;
    gComputeInterStruct.aAccelPlanMagn[2] = 0;

    }

//------------------------------------------------------------------------
// calcul des commandes à mettre pour les moteurs et le servo direction
//
//------------------------------------------------------------------------
void gCompute_Execute(void)
    {
    //---------------------------------------------------------------------------
    //lecture des donnees provenant du monitoring
//    if (gXbeeInterStruct.aPIDChangedServo)
//	{
//	thePIDServo.kp = gXbeeInterStruct.aGainPIDServo.gProprortionalGain;
//	thePIDServo.ki = gXbeeInterStruct.aGainPIDServo.gIntegraleGain;
//	thePIDServo.kd = gXbeeInterStruct.aGainPIDServo.gDerivativeGain;
//	thePIDServo.kp_init = thePIDServo.kp;
//	}

    //---------------------------------------------------------------------------
    //lecture des donnees provenant du monitoring
//    if (gXbeeInterStruct.aPIDChangedMotors)
//	{
//	//Motor1
//	mMotor1.aPIDData.kp =
//		gXbeeInterStruct.aGainPIDMotors.gProprortionalGain;
//	mMotor1.aPIDData.ki = gXbeeInterStruct.aGainPIDMotors.gIntegraleGain;
//	mMotor1.aPIDData.kd = gXbeeInterStruct.aGainPIDMotors.gDerivativeGain;
//
//	//Motor 2
//	mMotor2.aPIDData.kp =
//		gXbeeInterStruct.aGainPIDMotors.gProprortionalGain;
//	mMotor2.aPIDData.ki = gXbeeInterStruct.aGainPIDMotors.gIntegraleGain;
//	mMotor2.aPIDData.kd = gXbeeInterStruct.aGainPIDMotors.gDerivativeGain;
//	}

    //---------------------------------------------------------------------------
    //recherche de la ligne
    static int16_t theLinePosition = 0;
    static int16_t theLinePositionLostComp = 0;

    bool isLineFound = false;
    bool isStartStopFound = false;
    int16_t LineAnalyze[128];

    for (uint16_t i = 0; i < 128; i++)
	{
	LineAnalyze[i] = LineScanImage0[i];
	}
    mTrackLine_FindLine(LineAnalyze, 128, &theLinePosition, &isLineFound,
	    &isStartStopFound);
    // mTrackLine_Correlation(LineAnalyze, 128, &theLinePosition, &isLineFound,
//	    &isStartStopFound);

    //---------------------------------------------------------------------------

    //---------------------------------------------------------------------------
    //si la ligne est trouvee
    if (isLineFound)
	{
	TFC_BAT_LED0_ON;
	//On reset le compteur de perte de ligne
	theLinePositionLostComp = 0;

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

	//On enregistre l'ancienne valeur de la ligne
	theLinePosition = median_filter_n(theLinePositionTab, kTailleFiltre);
	}
    else
	{
	//On test si on est arrivé sur la bosse
	if (tAbs(theLinePosition - 64) > 6)
	    {
	    //Si on pert la ligne, on interpole
	    // ON sélectionne le sens de correction
	    if (theLinePosition > 64)
		{
		theLinePosition += 10;
		}
	    else
		{
		theLinePosition -= 10;
		}
	    theLinePositionLostComp++;
	    TFC_BAT_LED0_OFF;
	    }
	}
    thePIDServo.kp = thePIDServo.kp_init;

    // On diminue l'effet de P avec la vitesse
    // Consigne va de 0-1 => *2 => kp = 1/2 * kp si consigne max sinon kp=1/1 * kp si consigne =0.5 ;
    // Elle augmente si on va lentement
    // thePIDServo.kp /= (thePIDServo. * 0.05) + 1.0;

    // Mettre a jour le PID
    //if (tAbs((theLinePosition - 64)) < 20)
    //{
    //theLinePosition = 64;
    //}

    tPID(&thePIDServo, (theLinePosition - 64));
    //tPID_v2(&thePIDServo, (theLinePosition - 64));
    //---------------------------------------------------------------------------
    //si la ligne d'arrivee est trouvee
    static uint8_t isInRace = 0;
    static bool oldIsStartStopFound = false;
    if ((isStartStopFound && !(oldIsStartStopFound))
	    || ((theLinePositionLostComp >= K_LOST_MAX_NUM)))
	{
	if (isInRace > 0)
	    {
	    isInRace--;
	    }
	}
    oldIsStartStopFound = isStartStopFound;

    if (TFC_PUSH_BUTTON_0_PRESSED)
	{
	isInRace = 2;
	}

    if (isInRace > 0)
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

    compute_differential(gComputeInterStruct.gCommandeServoDirection,
	    &thePIDServo);

    if (isInRace > 0)
	{
	float aSpeedFact = 0.0;
	float aSpeedTotFactor = 0.0;

	mMotor1.aPIDData.consigne *= mMotor1.aDifferential;
	mMotor2.aPIDData.consigne *= mMotor2.aDifferential;

	if ((thePIDServo.thePastError[0] - thePIDServo.thePastError[1]) < 0.0)
	    {
	    aSpeedFact = (thePIDServo.thePastError[1]
		    - thePIDServo.thePastError[0]) * 6.5;
	    }

	aSpeedTotFactor =
		((K_SPEED_LOWEST - aSpeedFact) < 0.0) ?
			0.0 : (K_SPEED_LOWEST - aSpeedFact);
	aSpeedTotFactor = K_SPEED_LOWEST;

	if (mMotor1.aPIDData.consigne <= aSpeedTotFactor)
	    {
	    mMotor1.aPIDData.consigne = aSpeedTotFactor;
	    }

	if (mMotor2.aPIDData.consigne <= aSpeedTotFactor)
	    {
	    mMotor2.aPIDData.consigne = aSpeedTotFactor;
	    }

	}
    else
	{
	mMotor1.aPIDData.consigne = 0;
	mMotor2.aPIDData.consigne = 0;
	}

//---------------------------------------------------------------------------
//Appel des PID des moteurs
// PID Moteur 1
    tPID(&mMotor1.aPIDData, (int16_t) (gInputInterStruct.gFreq[0] / 3.0)); // Frequence entre 0 et 100
// PID Moteur 2
    tPID(&mMotor2.aPIDData, (int16_t) (gInputInterStruct.gFreq[1] / 3.0));

//---------------------------------------------------------------------------
//mise à jour des sorties de gCompute
    gInputInterStruct.gPosCam1 = theLinePosition;

    // Limitation des servo moteurs
    if (tAbs_float(thePIDServo.commande) > 0.8)
	{
	if (thePIDServo.commande < 0.0)
	    {
	    thePIDServo.commande = -0.8;
	    }
	else
	    {
	    thePIDServo.commande = 0.8;
	    }
	}
    gComputeInterStruct.gCommandeServoDirection = thePIDServo.commande;

// Test de freinage
    if ((mMotor1.aPIDData.commande < 0.0))
	{
	mMotor1.aPIDData.commande = 0.0;
	}
    if ((mMotor2.aPIDData.commande < 0.0))
	{
	mMotor2.aPIDData.commande = 0.0;
	}

    //Test si la fréquence du moteur est supérieur à vitesse min on autorise le freinage
    if ((gInputInterStruct.gFreq[0] / 3.0) > K_SPEED_LOWEST)
	{

	if (full_break == true)
	    {
	    mMotor1.aPIDData.commande = kFULL_BRAKE;
	    mMotor2.aPIDData.commande = kFULL_BRAKE;
	    }
	else if (half_break == true)
	    {
	    mMotor1.aPIDData.commande = kHALF_BRAKE;
	    mMotor2.aPIDData.commande = kHALF_BRAKE;
	    }
	half_break = false;
	full_break = false;
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

static void compute_differential(const float aAngleServo,
	tPIDStruct* thePIDStruct)
    {

    float m = -0.8;
    //Moyenne des erreurs
    float aMoyenneError = tAbs_float(
	    (1.0 / 3)
		    * (thePIDStruct->thePastError[0]
			    + thePIDStruct->thePastError[1]
			    + thePIDStruct->thePastError[2]));

    if (aMoyenneError < 0.3)
	{
	aMoyenneError = 0;
	full_break = false;
	half_break = false;
	TFC_BAT_LED2_OFF;
	TFC_BAT_LED3_OFF;
	}
    // Si on a perdu la ligne plus de ~30ms et que l'on braque à fond , on freine comme des porcs
    else if ((aMoyenneError > 0.25) && (tAbs_float(aAngleServo) > 0.35)
	    && ((gInputInterStruct.gFreq[0] / 3.0) > (K_SPEED_LOWEST - 15)))
	{
	TFC_BAT_LED2_ON;
	full_break = true;
	}
    // Sinon on freine un peu moins si on se trouve au bord et on corrige assez fort
    else if ((aMoyenneError > 0.20) && (tAbs_float(aAngleServo) > 0.15)
	    && ((gInputInterStruct.gFreq[0] / 3.0) > (K_SPEED_LOWEST - 15)))
	{
	TFC_BAT_LED3_ON;
	half_break = true;
	}
    else
	{
	TFC_BAT_LED2_OFF;
	TFC_BAT_LED3_OFF;
	full_break = false;
	half_break = false;
	}

    float speedScaleFactor = -K_BRAKE_FACTOR * aMoyenneError + 1.0;

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

