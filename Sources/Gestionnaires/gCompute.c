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
    TFC_BAT_LED2_ON;
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
	LineNear[i] = LineScanImage1[127 - i]; //une des deux cameras est montee la tete en bas, alors on l'inverse
	LineFar[i] = LineScanImage0[i];
	}

//    mTrackLine_Correlation(LineNear, LineFar, kLENGTHLINESCAN, &theLineNearPosition, &theLineFarPosition,
//	    &isLineNearFound, &isLineFarFound, &isStartStopNearFound);
    mTrackLine_CorrelationFFT(LineNear, LineFar, kLENGTHLINESCAN,
	    &theLineNearPosition, &theLineFarPosition, &isLineNearFound,
	    &isLineFarFound, &isStartStopNearFound);

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
    TFC_BAT_LED2_OFF;
    //---------------------------------------------------------------------------
    //===========================================================================
    // FIN
    //===========================================================================
    //---------------------------------------------------------------------------

    /*
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
     aSpeedTotFactor = K_SPEED_LOWEST + 5;

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
     gComputeInterStruct.gCommandeMoteurDroit = mMotor2.aPIDData.commande;
     */
    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------
