/*
 * mMotor.c
 *
 *  Created on: Nov 1, 2013
 *      Author: Michael
 */

#include "TFC/TFC_Motor.h"
#include "Modules/mMotor.h"
#include "TFC/TFC.h"

#define FTM2_MOD_VALUE	(int)((float)(PERIPHERAL_BUS_CLOCK)/TFC_MOTOR_SWITCHING_FREQUENCY)

#define FTM2_CLOCK                                   	      (CORE_CLOCK/2)
#define FTM2_CLK_PRESCALE                                 	4  // Prescale Selector value - see comments in Status Control (SC) section for more details
#define FTM2_OVERFLOW_FREQUENCY 				10000		
#define N_OF							1
#define F_COUNT							6000000.0

#define FILTER_SIZE						5

/**
 * Instanciation des deux moteurs de propulsion
 */
mMotorStruct mMotor1;
mMotorStruct mMotor2;

float aFreqMesTabMot1[FILTER_SIZE] =
    {
    0.0
    };

char aNumEchantillonsMot1 = 0;
float aFreqMesTabMot2[FILTER_SIZE] =
    {
    0.0
    };

char aNumEchantillonsMot2 = 0;

float median_filter_n(float *aTab, char aSize);

/**
 * Permet de configurer les moteur
 * 	- Mesure de vitesse
 * 	- Paramètres de régulation
 */
void mMotor_mSetup()
    {

    // Configuration de l'input capture

    //Enable the Clock to the FTM0 Module
    //See Page 207 of f the KL25 Sub-Family Reference Manual, Rev. 3, September 2012
    SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;

    //The TPM Module has Clock.  Now set up the peripheral

    //Blow away the control registers to ensure that the counter is not running
    TPM2_SC = 0;
    TPM2_CONF = 0;

    //While the counter is disabled we can setup the prescaler

    TPM2_SC = TPM_SC_TOIE_MASK | TPM_SC_PS(FTM2_CLK_PRESCALE);

    //Setup the mod register to get the correct PWM Period

    TPM2_MOD = 0xFFFF; //(FTM2_CLOCK / (1 << FTM2_CLK_PRESCALE)) / FTM2_OVERFLOW_FREQUENCY;

    //Setup Channels 0 & 1 in input capture rising edge with interrupt
    TPM2_C0SC = TPM_CnSC_ELSB_MASK | TPM_CnSC_CHIE_MASK;
    TPM2_C1SC = TPM_CnSC_ELSB_MASK | TPM_CnSC_CHIE_MASK;

    //Enable the Counter

    enable_irq(INT_TPM2 - 16);

    //Enable the TPM COunter
    TPM2_SC |= TPM_SC_CMOD(1);

    //Enable the FTM functions on the the port
    PORTA_PCR1 = PORT_PCR_MUX(3);
    PORTA_PCR2 = PORT_PCR_MUX(3);

    //Config des moteurs
    TFC_InitMotorPWM();

    //Set the Default duty cycle to 50% duty cycle
    TFC_SetMotorPWM(0.0, 0.0);

    //initialaisation des moterus
    mMotor1.aCapt = 0;
    mMotor1.aStopped = 1;
    mMotor1.aPIDData.consigne = 0;
    mMotor1.aPIDData.erreurPrecedente = 0;
    mMotor1.aPIDData.kd = 0;
    mMotor1.aPIDData.kp = 1;
    mMotor1.aPIDData.ki = 0;
    mMotor1.aPIDData.sommeErreurs = 0;
    mMotor1.aOverflowOld = 0;

    mMotor2.aCapt = 0;
    mMotor2.aStopped = 1;
    mMotor2.aPIDData.consigne = 0;
    mMotor2.aPIDData.erreurPrecedente = 0;
    mMotor2.aPIDData.kd = 0;
    mMotor2.aPIDData.kp = 1;
    mMotor2.aPIDData.ki = 0;
    mMotor2.aPIDData.sommeErreurs = 0;
    mMotor2.aOverflowOld = 0;

    }

/**
 * N'est pas utilisé pour'instant
 */
void mMotor_mOpen()
    {
    //Rien pour l'instant
    }

/**
 * Permet d'appeler la fonction de regulation de la vitesse
 */
void mMotorCallPID()
    {
//    // PID Moteur 1
//    tPID(&mMotor1.aPIDData, mMotor1.aFreq);
//    // PID Moteur 2
//    tPID(&mMotor2.aPIDData, mMotor2.aFreq);
    }

/**
 * Permet de tester si un moteur est à l'arrêt
 * aMotorNum : Numéro du moteur 0 ou 1
 */
uint8_t mMotor_isStopped(uint8_t aMotorNum)
    {
    if (aMotorNum == 0)
	{
	return mMotor1.aStopped;
	}
    else
	{
	return mMotor2.aStopped;
	}
    }

/**
 * Interruptions des canaux
 */
void FTM2_IRQHandler()
    {
    static uint32_t mMotor1_oldCapt = 0;
    static uint32_t mMotor2_oldCapt = 0;

    // Overflow ?
    if ((TPM2_SC & TPM_SC_TOF_MASK))
	{
	if (mMotor1.aOverflowOld > N_OF)
	    {
	    mMotor1.aStopped = 1;
	    mMotor1_oldCapt = 0;
	    mMotor1.aCapt = 65535 * N_OF;
	    mMotor1.aFreq = 0.0;
	    //On vide le tableau
	    aNumEchantillonsMot1 = 0;
	    }
	else
	    {
	    mMotor1.aOverflowOld++;
	    }

	if (mMotor2.aOverflowOld > N_OF)
	    {
	    mMotor2.aStopped = 1;
	    mMotor2_oldCapt = 0;
	    mMotor2.aCapt = 65535 * N_OF;
	    mMotor2.aFreq = 0.0;
	    //On vide le tableau
	    aNumEchantillonsMot2 = 0;
	    }
	else
	    {
	    mMotor2.aOverflowOld++;
	    }

	//Clear du flag
	TPM2_SC |= TPM_SC_TOF_MASK;
	}

    //Test quel canal à interrrompu
    if (TPM2_C0SC & TPM_CnSC_CHF_MASK)
	{

	if (mMotor1.aStopped == 1)
	    {
	    mMotor1.aOverflowOld = 0;
	    }

	mMotor1.aCapt = (TPM2_C0V + (65535 * mMotor1.aOverflowOld))
		- mMotor1_oldCapt;

	mMotor1_oldCapt = TPM2_C0V;

	//On a avancé donc on oublie les overflow
	mMotor1.aOverflowOld = 0;
	mMotor1.aStopped = 0;

	mMotor1.aFreq = (F_COUNT) / mMotor1.aCapt;
	//Mise à jour de la vitesse
//	if (aNumEchantillonsMot1 >= (FILTER_SIZE - 1))
//	    {
//	    aFreqMesTabMot1[0] = (F_COUNT) / mMotor1.aCapt;
//	    mMotor1.aFreq = median_filter_n(aFreqMesTabMot1, FILTER_SIZE);
//	    }
//	else
//	    {
//	    mMotor1.aFreq = (F_COUNT) / mMotor1.aCapt;
//	    aFreqMesTabMot1[(FILTER_SIZE - 1) - aNumEchantillonsMot1] =
//		    mMotor1.aFreq;
//	    aNumEchantillonsMot1++;
//	    }

	//Clear du flag
	TPM2_C0SC |= TPM_CnSC_CHF_MASK;
	}
    if (TPM2_C1SC & TPM_CnSC_CHF_MASK)
	{
	if (mMotor2.aStopped == 1)
	    {
	    mMotor2.aOverflowOld = 0;
	    }

	mMotor2.aCapt = (TPM2_C1V + (65535 * mMotor2.aOverflowOld))
		- mMotor2_oldCapt;
	mMotor2_oldCapt = TPM2_C1V;

	//On a avancé donc on oublie l'overflow
	mMotor2.aOverflowOld = 0;
	mMotor2.aStopped = 0;

	mMotor2.aFreq = (F_COUNT) / mMotor2.aCapt;

//	//Mise à jour de la vitesse
//	if (aNumEchantillonsMot2 >= (FILTER_SIZE - 1))
//	    {
//	    aFreqMesTabMot2[0] = (F_COUNT) / mMotor2.aCapt;
//	    mMotor2.aFreq = median_filter_n(aFreqMesTabMot2, FILTER_SIZE);
//	    }
//	else
//	    {
//	    mMotor2.aFreq = (F_COUNT) / mMotor2.aCapt;
//	    aFreqMesTabMot2[(FILTER_SIZE - 1) - aNumEchantillonsMot2] =
//		    mMotor2.aFreq;
//	    aNumEchantillonsMot2++;
//	    }

	//Clear du flag
	TPM2_C1SC |= TPM_CnSC_CHF_MASK;
	}
    }

float median_filter_n(float *aTab, char aSize)
    {
    float aCpyTab[aSize];
    char i = 0, j = 0;
    float aTemp;

    //Copie du tableau
    for (i = 0; i < aSize; i++)
	{
	aCpyTab[i] = aTab[i];
	}

    //On décale l'indice du tableau pour la prochaine mesure
    for (i = (aSize); i > 0; i--)
	{
	aTab[i] = aTab[i - 1];
	}

    //On trie le tableau
    for (i = 0; i < aSize; i++)
	{
	for (j = 0; j < aSize; j++)
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

