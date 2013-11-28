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
#define FTM2_CLK_PRESCALE                                 	7  // Prescale Selector value - see comments in Status Control (SC) section for more details
#define FTM2_OVERFLOW_FREQUENCY 				10000		

// On a une consigne entre 0 -> 65535
// le plus court env 3.3 ms  = 303Hz => on aura capt = 33 
// Et au minimum capt = 65535 = 6.55 s.
// Donc 1 = vitesse max et 0 vitesse min
// ax + b 
// a*33 + b == 65535
// a*65535 +b == 0

//TODO Utiliser des entiers
#define CONVERSION_MES_CONS  		(int32_t)-1
#define CONVERSION_MES_CONS_OFF 	(int32_t)65535

/**
 * Instanciation des deux moteurs de propulsion
 */
mMotorStruct mMotor1;
mMotorStruct mMotor2;

/**
 * Permet de configurer les moteur
 * 	- Mesure de vitesse
 * 	- Paramètres de régulation
 */
void mMotor_mSetup()
    {
    //Config des moteurs
    TFC_InitMotorPWM();

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

    TPM2_MOD = 0xFFFF ;//(FTM2_CLOCK / (1 << FTM2_CLK_PRESCALE)) / FTM2_OVERFLOW_FREQUENCY;

    //Setup Channels 0 & 1 in input capture rising edge with interrupt
    TPM2_C0SC = TPM_CnSC_ELSA_MASK | TPM_CnSC_CHIE_MASK;
    TPM2_C1SC = TPM_CnSC_ELSA_MASK | TPM_CnSC_CHIE_MASK;

    //Enable the Counter

    //Set the Default duty cycle to 50% duty cycle
    TFC_SetMotorPWM(0.0, 0.0);

    enable_irq(INT_TPM2 - 16);

    //Enable the TPM COunter
    TPM2_SC |= TPM_SC_CMOD(1);

    //Enable the FTM functions on the the port
    PORTA_PCR1 = PORT_PCR_MUX(3);
    PORTA_PCR2 = PORT_PCR_MUX(3);

    //initialaisation des moterus
    mMotor1.aCapt = 0;
    mMotor1.aStopped = 1;
    mMotor1.aPIDData.consigne = 0;
    mMotor1.aPIDData.erreurPrecedente = 0;
    mMotor1.aPIDData.kd = 0;
    mMotor1.aPIDData.kp = 1;
    mMotor1.aPIDData.ki = 0;
    mMotor1.aPIDData.sommeErreurs = 0;
//    mMotor1.aPIDData.conversionMesureConsigne = CONVERSION_MES_CONS;
//    mMotor1.aPIDData.offsetMesureConsigne = CONVERSION_MES_CONS_OFF;
    mMotor1.aOverflowOld = 0;

    mMotor2.aCapt = 0;
    mMotor2.aStopped = 1;
    mMotor2.aPIDData.consigne = 0;
    mMotor2.aPIDData.erreurPrecedente = 0;
    mMotor2.aPIDData.kd = 0;
    mMotor2.aPIDData.kp = 1;
    mMotor2.aPIDData.ki = 0;
    mMotor2.aPIDData.sommeErreurs = 0;
//    mMotor2.aPIDData.conversionMesureConsigne = CONVERSION_MES_CONS;
//    mMotor2.aPIDData.offsetMesureConsigne = CONVERSION_MES_CONS_OFF;
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
    // PID Moteur 1
    tPID(&mMotor1.aPIDData, mMotor1.aCapt);
    // PID Moteur 2
    tPID(&mMotor2.aPIDData, mMotor2.aCapt);
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
    static uint16_t mMotor1_oldCapt = 0;
    static uint16_t mMotor2_oldCapt = 0;

    // Overflow ?
    if ((TPM2_SC & TPM_SC_TOF_MASK))
	{
	if (mMotor1.aOverflowOld >= 10)
	    {
	    mMotor1.aStopped = 1;
	    mMotor1_oldCapt = 0;
	    mMotor1.aCapt = 0;
	    }
	else
	    {
	    mMotor1.aOverflowOld++;
	    }

	if (mMotor2.aOverflowOld >= 10)
	    {
	    mMotor2.aStopped = 1;
	    mMotor2_oldCapt = 0;
	    mMotor1.aCapt = 0;
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
	if (TPM2_C0V > mMotor1_oldCapt)
	    {
	    mMotor1.aCapt = TPM2_C0V - mMotor1_oldCapt;
	    }
	else
	    {
	    mMotor1.aCapt = TPM2_C0V + (65535 - mMotor1_oldCapt);
	    }
	mMotor1_oldCapt = TPM2_C0V;

	//On a avancé donc on oublie l'overflow
	mMotor1.aOverflowOld = 0;
	mMotor1.aStopped = 0;

	//Clear du flag
	TPM2_C0SC |= TPM_CnSC_CHF_MASK;
	}
    if (TPM2_C1SC & TPM_CnSC_CHF_MASK)
	{
	if (TPM2_C1V > mMotor2_oldCapt)
	    {
	    mMotor2.aCapt = TPM2_C1V - mMotor2_oldCapt;
	    }
	else
	    {
	    mMotor2.aCapt = TPM2_C1V + (65535 - mMotor2_oldCapt);
	    }
	mMotor2_oldCapt = TPM2_C1V;

	//On a avancé donc on oublie l'overflow
	mMotor2.aOverflowOld = 0;
	mMotor2.aStopped = 0;

	//Clear du flag
	TPM2_C1SC |= TPM_CnSC_CHF_MASK;
	}
    }

