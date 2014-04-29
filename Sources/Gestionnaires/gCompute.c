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
#define kMEDIANFILTERSIZE 9

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

    theRegServo.coefficient = (kREGQUAD_BRAQUAGEMAX ) / (kREGQUAD_ERREURMAX * kREGQUAD_ERREURMAX );

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
    mMotor1.aPIDData.kd = 0.06;
    mMotor1.aPIDData.kp = 0.8;
    mMotor1.aPIDData.ki = 0.09;
    mMotor1.aPIDData.coeffNormalisation = 0.01;
    mMotor1.aPIDData.sommeErreurs = 0;

    mMotor2.aPIDData.consigne = 0.4;
    mMotor2.aPIDData.erreurPrecedente = 0;
    mMotor2.aPIDData.kd = 0.06;
    mMotor2.aPIDData.kp = 0.8;
    mMotor2.aPIDData.ki = 0.09;
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
    static volatile int16_t* LineNear;
    static int16_t theLineNearPosition = 0;
    static bool isLineNearFound = false;
    static bool isStartStopNearFound = false;

    static volatile int16_t* LineFar;
    static int16_t theLineFarPosition = 0;
    static bool isLineFarFound = false;

    static int16_t theLineMesure = 0;

    LineNear = (int16_t*) LineScanImage1;
    LineFar = (int16_t*) LineScanImage0;

    //on recherche la ligne par corrélation
    tRescale(LineNear, 64, 3000, 0);
    tRescale(LineFar, 64, 3000, 0);
    mTrackLine_CorrelationFFT(LineNear, LineFar, &theLineNearPosition, &theLineFarPosition, &isLineNearFound,
	    &isLineFarFound, &isStartStopNearFound);

    if (isLineNearFound)
	{
	theLineNearPosition = kLENGTHLINESCAN - theLineNearPosition;
	} //une des deux cameras est montee la tete en bas, alors on l'inverse

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
    static int8_t isInRace = 0;
    static bool oldIsStartStopNearFound = false;
    static int16_t aDirConsigne = 0;
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
	theLineMesure = theLineFarPosition;
	perteLigne = 0;
	}

    //-----------------------------------------------------------------------
    // 4.4 : les deux lignes sont trouvees -> on detecte les virages sur la 
    //	 ligne lointaine et on regule sur la ligne proche
    //-----------------------------------------------------------------------
    else
	{
	aDirConsigne =
		(int16_t) ((theLineFarPosition - (int16_t) (kLENGTHLINESCAN / 2)) * (-kCONSIGNEPROCHECORRECTION ));
	theLineMesure = theLineNearPosition;

	if (tAbs(aDirConsigne) < kCONSIGNE_MIN_VIRAGE_1)
	    {
	    theRegServo.coefficient = ((kREGQUAD_BRAQUAGEMAX ) / (kREGQUAD_ERREURMAX * kREGQUAD_ERREURMAX )) * 0.20;
	    aDirConsigne *= 0.5;
	    }
	else if (tAbs(aDirConsigne) < kCONSIGNE_MIN_VIRAGE_2)
	    {
	    theRegServo.coefficient = ((kREGQUAD_BRAQUAGEMAX ) / (kREGQUAD_ERREURMAX * kREGQUAD_ERREURMAX )) * 0.50;
	    aDirConsigne *= 0.85;
	    }
	else if (tAbs(aDirConsigne) < kCONSIGNE_MIN_VIRAGE_3)
	    {
	    theRegServo.coefficient = ((kREGQUAD_BRAQUAGEMAX ) / (kREGQUAD_ERREURMAX * kREGQUAD_ERREURMAX )) * 0.80;
	    }
	else
	    {
	    theRegServo.coefficient = (kREGQUAD_BRAQUAGEMAX ) / (kREGQUAD_ERREURMAX * kREGQUAD_ERREURMAX );
	    }

	perteLigne = 0;
	}

    //-----------------------------------------------------------------------
    // 4.5 : filtrage de la consigne de direction
    //-----------------------------------------------------------------------
    theRegServo.consigne = (theRegServo.consigne * 4 + aDirConsigne);
    theRegServo.consigne /= 5;

    //---------------------------------------------------------------------------
    // 5 : freinage dans les virages
    //---------------------------------------------------------------------------
    static bool freinage = false;
    static bool oldFreinage = false;
    static int16_t aVitesseFreinage = 0;

    //on teste si on entre dans un virage
    if (tAbs(theRegServo.consigne) > kSEUIL_CONSIGNE_FREINAGE)
	{
	freinage = true;
	}
    else if (TFC_Ticker[2] >= kTEMPS_FREINAGE_2)
	{
	freinage = false;
	aVitesseFreinage = gInputInterStruct.vMax;
	TFC_BAT_LED2_OFF;
	}

    //on freine en fonction de la moyenne des dérivées (plus elle est grande, plus on freine)
    if (freinage && (!oldFreinage))
	{
	TFC_Ticker[2] = 0;
	aVitesseFreinage = kSPEED_MIN_1;
	TFC_BAT_LED2_ON;
	}
    else if ((TFC_Ticker[2] >= kTEMPS_FREINAGE_1) && ((freinage == true) && (oldFreinage == true)))
	{
	aVitesseFreinage = kSPEED_MIN_2;
	}

    oldFreinage = freinage;

    //on teste si la vitesse n'est pas supérieure à la vitesse max autorisée
    if (aVitesseFreinage < gInputInterStruct.vMax)
	{
	mMotor1.aPIDData.consigne = aVitesseFreinage;
	mMotor2.aPIDData.consigne = aVitesseFreinage;
	}
    else
	{
	mMotor1.aPIDData.consigne = gInputInterStruct.vMax;
	mMotor2.aPIDData.consigne = gInputInterStruct.vMax;
	}

    //---------------------------------------------------------------------------
    // 6 : application des regulateurs
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
    mMotor1.aPIDData.consigne = (int16_t) ((float) mMotor1.aPIDData.consigne * mMotor1.aDifferential);
    mMotor2.aPIDData.consigne = (int16_t) ((float) mMotor2.aPIDData.consigne * mMotor2.aDifferential);

    // PIDs Moteurs
    tRegPID(&mMotor1.aPIDData, (int16_t) (theSpeedMotor1 / 3.0));	    // Frequence entre 0 et 100
    tRegPID(&mMotor2.aPIDData, (int16_t) (theSpeedMotor2 / 3.0));
    gComputeInterStruct.gCommandeMoteurGauche = mMotor2.aPIDData.commande;
    gComputeInterStruct.gCommandeMoteurDroit = mMotor1.aPIDData.commande;

//---------------------------------------------------------------------------
//===========================================================================
// FIN
//===========================================================================
//---------------------------------------------------------------------------

    }
