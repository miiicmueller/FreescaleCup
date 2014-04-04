/*
 * gCompute.c
 *
 *  Created on: Nov 1, 2013
 *      Author: cyrille.savy
 */

#include "TFC/TFC.h"
#include "Gestionnaires/gCompute.h"
#include "Gestionnaires/gMbox.h"
#include "Tools/tRegulateurs.h"
#include "Tools/Tools.h"
#include "Tools/angle_cal.h"
#include "Modules/mTrackline.h"
#include "Modules/mMotor.h"
#include "parameters.h"
#include "Tools/tDifferential.h"

#define kLENGTHLINESCAN (uint16_t)128
#define kMEDIANFILTERSIZE 7
#define T_ERROR_MAX_BREAK 30
#define K_BRAKE_FACTOR 0.4
#define K_BRAKE_FACTOR_DER 0.0

#define K_LOST_MAX_NUM 300  // 3s
/* prototypes des fonctions statiques (propres au fichier) */
static tRegulateurQuadStruct theRegServo;

float aFreqMesTabMot1[FILTER_SIZE] =
    {
    0.0
    };

float aFreqMesTabMot2[FILTER_SIZE] =
    {
    0.0
    };

//------------------------------------------------------------------------
// Initialisation de la structure de données de gCompute
//------------------------------------------------------------------------
void gCompute_Setup(void)
    {
    gComputeInterStruct.gCommandeMoteurDroit = 0.0;
    gComputeInterStruct.gCommandeMoteurGauche = 0.0;
    gComputeInterStruct.gCommandeServoDirection = 0.0;

    theRegServo.coefficient = (kREGQUAD_BRAQUAGEMAX )
	    / (kREGQUAD_ERREURMAX * kREGQUAD_ERREURMAX * kREGQUAD_ERREURMAX * kREGQUAD_ERREURMAX );

    theRegServo.consigne = 0;
    theRegServo.commande = 0;

#ifdef MONITORING_ENABLED
    //lecture des donnees provenant du monitoring
    //Motor1
    mMotor1.aPIDData.kp = gXbeeInterStruct.aGainPIDMotors.gProprortionalGain;
    mMotor1.aPIDData.ki = gXbeeInterStruct.aGainPIDMotors.gIntegraleGain;
    mMotor1.aPIDData.kd = gXbeeInterStruct.aGainPIDMotors.gDerivativeGain;

    //Motor 2
    mMotor2.aPIDData.kp = gXbeeInterStruct.aGainPIDMotors.gProprortionalGain;
    mMotor2.aPIDData.ki = gXbeeInterStruct.aGainPIDMotors.gIntegraleGain;
    mMotor2.aPIDData.kd = gXbeeInterStruct.aGainPIDMotors.gDerivativeGain;
#else
    //sans monitoring on fixe des constantes
    mMotor1.aPIDData.consigne = 0.4;
    mMotor1.aPIDData.erreurPrecedente = 0;
    mMotor1.aPIDData.kd = 0.015;
    mMotor1.aPIDData.kp = 1.0;
    mMotor1.aPIDData.ki = 0.32;
    mMotor1.aPIDData.coeffNormalisation = 0.01;
    mMotor1.aPIDData.sommeErreurs = 0;

    mMotor2.aPIDData.consigne = 0.4;
    mMotor2.aPIDData.erreurPrecedente = 0;
    mMotor2.aPIDData.kd = 0.015;
    mMotor2.aPIDData.kp = 1.0;
    mMotor2.aPIDData.ki = 0.32;
    mMotor2.aPIDData.coeffNormalisation = 0.01;
    mMotor2.aPIDData.sommeErreurs = 0;

    //temps d'exposition et PWM leds
    gXbeeInterStruct.aPWMLeds = kLEDSPWM;
    gXbeeInterStruct.aExpTime = kCAMEXPTIME;
#endif
    }

//------------------------------------------------------------------------
// calcul des commandes à mettre pour les moteurs et le servo direction
// intelligence de la voiture, regulation, anticipation, etc.
//------------------------------------------------------------------------
void gCompute_Execute(void)
    {
#ifdef MONITORING_ENABLED
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
#endif

    //___________________________________________________________________________
    //---------------------------------------------------------------------------
    //===========================================================================
    //structure pour la nouvelle version de gCompute
    //===========================================================================
    //---------------------------------------------------------------------------

    //---------------------------------------------------------------------------
    // 1 : analyse des lignes (camera proche : ligne et arrivee)
    //			      (camera lointaine : ligne)
    //---------------------------------------------------------------------------
    //Donnees des lignes
    static int16_t theLineNearPosition = 0;
    static bool isLineNearFound = false;
    static bool isStartStopNearFound = false;
    static int16_t LineNear[kLENGTHLINESCAN ];

    static int16_t theLineFarPosition = 0;
    static bool isLineFarFound = false;
    //bool isStartStopFarFound = false;
    static int16_t LineFar[kLENGTHLINESCAN ];

    static int16_t theLineMesure = 0;

    for (uint16_t i = 0; i < kLENGTHLINESCAN ; i++)
	{
	LineNear[i] = LineScanImage1[127 - i]; //<< 3; //une des deux cameras est montee la tete en bas, alors on l'inverse
	LineFar[i] = LineScanImage0[i]; //<< 3; //shift de 3 parce que la correlation travaille en q1.15 et que nos nombres sont sur 12bits (ADC)
	}

    TFC_BAT_LED2_ON;
//    mTrackLine_Correlation(LineNear, LineFar, kLENGTHLINESCAN, &theLineNearPosition, &theLineFarPosition,
//	    &isLineNearFound, &isLineFarFound, &isStartStopNearFound);
    mTrackLine_CorrelationFFT(LineNear, LineFar, kLENGTHLINESCAN, &theLineNearPosition, &theLineFarPosition,
	    &isLineNearFound, &isLineFarFound, &isStartStopNearFound);
    TFC_BAT_LED2_OFF;

//    mTrackLine_FindLine(LineNear, kLENGTHLINESCAN, &theLineNearPosition, &isLineNearFound, &isStartStopNearFound,
//	    theLineNearPosition);
//    mTrackLine_FindLine(LineFar, kLENGTHLINESCAN, &theLineFarPosition, &isLineFarFound, &isStartStopFarFound,
//	    theLineFarPosition);

    TFC_BAT_LED0_OFF;
    TFC_BAT_LED1_OFF;
    if (isLineNearFound)
	{
	TFC_BAT_LED0_ON;
	}
    if (isLineFarFound)
	{
	TFC_BAT_LED1_ON;
	}
    else
	{
	TFC_BAT_LED1_OFF;
	}

    //---------------------------------------------------------------------------
    // 2 : filtrage des positions des lignes et des vitesse des moteurs
    //---------------------------------------------------------------------------
    static uint8_t posFiltre = 0;
    static uint32_t theLineNearTab[kMEDIANFILTERSIZE];
    static uint32_t theLineFarTab[kMEDIANFILTERSIZE];
    static uint32_t theMotor1Tab[kMEDIANFILTERSIZE];
    static uint32_t theMotor2Tab[kMEDIANFILTERSIZE];
    static float theSpeedMotor1 = 0;
    static float theSpeedMotor2 = 0;

    theLineNearTab[posFiltre] = theLineNearPosition;
    theLineFarTab[posFiltre] = theLineFarPosition;
    theMotor1Tab[posFiltre] = mMotor1.aCapt;
    theMotor2Tab[posFiltre] = mMotor2.aCapt;

    if (posFiltre < kMEDIANFILTERSIZE - 1)
	{
	posFiltre++;
	}
    else
	{
	posFiltre = 0;
	}

    theLineNearPosition = median_filter_n(theLineNearTab, kMEDIANFILTERSIZE);
    theLineFarPosition = median_filter_n(theLineFarTab, kMEDIANFILTERSIZE);
    theSpeedMotor1 = median_filter_n(theMotor1Tab, kMEDIANFILTERSIZE);
    theSpeedMotor2 = median_filter_n(theMotor2Tab, kMEDIANFILTERSIZE);

    //transformation temps -> frequence
    if (theSpeedMotor1 != 0)
	{
	theSpeedMotor1 = (float) (F_COUNT) / theSpeedMotor1;
	theSpeedMotor1 *= mMotor1.aDir;
	}
    if (theSpeedMotor2 != 0)
	{
	theSpeedMotor2 = (float) (F_COUNT) / theSpeedMotor2;
	theSpeedMotor2 *= mMotor2.aDir;
	}

    //---------------------------------------------------------------------------
    // 3 : ligne d'arrivee trouvee (machine d'etat + temps) -> on s'arrete
    //---------------------------------------------------------------------------
    static uint8_t isInRace = 0;
    static bool oldIsStartStopNearFound = false;
    if (isStartStopNearFound && !(oldIsStartStopNearFound))
	{
	if (isInRace > 0)
	    {
	    isInRace--;
	    }
	}
    oldIsStartStopNearFound = isStartStopNearFound;

    if (TFC_PUSH_BUTTON_0_PRESSED)
	{
	isInRace = 2;
	}

    if (isInRace > 0)
	{
	TFC_BAT_LED3_ON;
	}
    else
	{
	TFC_BAT_LED3_OFF;
	}

    //---------------------------------------------------------------------------
    // 4 : SWITCH suivant les lignes trouvees (cf. structo papier)
    //	   calcul des consignes de direction et de vitesse
    //---------------------------------------------------------------------------
    //-----------------------------------------------------------------------
    // 4.1 : aucune ligne trouvee pendant plus de 2s -> on s'arrete
    //-----------------------------------------------------------------------
    static uint16_t perteLigne = 0;
    if ((isLineNearFound == false) && (isLineFarFound == false))
	{
	perteLigne++;

	if (perteLigne == (kTIMENOLINE / kGEST_CYCLETIME))
	    {
	    isInRace = 0;
	    }

	theLineMesure = theLineNearPosition;
	}

    //-----------------------------------------------------------------------
    // 4.2 : seule la ligne proche trouvee -> on regule sur cette ligne
    //	 en tenant compte de si on est en virage ou non
    //-----------------------------------------------------------------------
    else if ((isLineNearFound == true) && (isLineFarFound == false))
	{
	perteLigne = 0;
	theLineMesure = theLineNearPosition;
	}

    //-----------------------------------------------------------------------
    // 4.3 : seule la ligne lointaine est trouvee -> on reporte l'erreur sur la ligne proche
    //-----------------------------------------------------------------------
    else if ((isLineNearFound == false) && (isLineFarFound == true))
	{
	theLineMesure = theLineFarPosition; //TODO : a tester!!! pas sur...
	perteLigne = 0;
	}

    //-----------------------------------------------------------------------
    // 4.4 : les deux lignes sont trouvees -> on detecte les virages sur la 
    //	 ligne lointaine et on regule sur la ligne proche
    //-----------------------------------------------------------------------
    else
	{

	theRegServo.consigne = (int16_t) ((theLineFarPosition - (int16_t) (kLENGTHLINESCAN / 2))
		* (-kCONSIGNEPROCHECORRECTION ));
	theLineMesure = theLineNearPosition;

	perteLigne = 0;
	}

    //---------------------------------------------------------------------------
    // 5 : application des regulateurs
    //---------------------------------------------------------------------------
    //regulation camera proche
    theLineMesure -= (int16_t) (kLENGTHLINESCAN / 2);
    tRegQuadratic(&theRegServo, (theLineMesure));
    gComputeInterStruct.gCommandeServoDirection = theRegServo.commande;

    //on avance que si on est en course!!!!!
    if (isInRace == 0)
	{
	mMotor1.aPIDData.consigne = 0;
	mMotor2.aPIDData.consigne = 0;
	}

    // 5 : differentiel
    compute_differential(gComputeInterStruct.gCommandeServoDirection);
    mMotor1.aPIDData.consigne *= mMotor1.aDifferential;
    mMotor2.aPIDData.consigne *= mMotor2.aDifferential;

    // PIDs Moteurs
    tRegPID(&mMotor1.aPIDData, (int16_t) (theSpeedMotor1 / 3.0)); // Frequence entre 0 et 100
    tRegPID(&mMotor2.aPIDData, (int16_t) (theSpeedMotor2 / 3.0));
    gComputeInterStruct.gCommandeMoteurGauche = mMotor2.aPIDData.commande;
    gComputeInterStruct.gCommandeMoteurDroit = mMotor1.aPIDData.commande;
//---------------------------------------------------------------------------
//===========================================================================
// FIN
//===========================================================================
//---------------------------------------------------------------------------

    }
