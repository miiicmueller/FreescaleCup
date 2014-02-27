/*
 * gXBEE.c
 *
 *  Created on: Nov 7, 2013
 *      Author: cyrille.savy
 */

#include "TFC/TFC.h"
#include "Gestionnaires/gMbox.h"
#include "Gestionnaires/gXBEE.h"
#include "Modules/mMotor.h"
#include "parameters.h"

#define END_OF_TRAME '\n'
#define CMD_MAX_SIZE 100

/* prototypes des fonctions statiques (propres au fichier) */

typedef enum
    {
    kSpeed,
    kServAngle,
    kPosCam,
    kPWM,
    kBattLed,
    kPIDMot,
    kPIDSer,
    kExpTime
    } aSendState;

static aSendState gStateSend = kPIDMot;
static char gCmdBuffer[100];
static char gLastCmdPointer = 0x00;

//-----------------------------------------------------------------------------
//fonctions publiques
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de données de gXBEE
//------------------------------------------------------------------------
void gXBEE_Setup(void)
    {
    // Config des gain des moteurs
    gXbeeInterStruct.aGainPIDMotors.gProprortionalGain = 1.5;
    gXbeeInterStruct.aGainPIDMotors.gIntegraleGain = 0.2;
    gXbeeInterStruct.aGainPIDMotors.gDerivativeGain = 0.045;

    // Config de la vitesse maximale des moteurs
    gXbeeInterStruct.aMotorSpeedCons = kSPEED_DUTY; //53.0

    // Config des gain du servo
    gXbeeInterStruct.aGainPIDServo.gProprortionalGain = 0.0;
    gXbeeInterStruct.aGainPIDServo.gIntegraleGain = 0.0;
    gXbeeInterStruct.aGainPIDServo.gDerivativeGain = 0.0;

    // Detection d'un changement de valeur
    gXbeeInterStruct.aPIDChangedServo = true;
    gXbeeInterStruct.aPIDChangedMotors = true;

    //Config de la position initiale
    gInputInterStruct.gPosCam1 = 0;
    gInputInterStruct.gPosCam2 = 0;

    //Niveau batterie
    gInputInterStruct.gBattLev = 100;

    //PWM des leds
    gXbeeInterStruct.aPWMLeds = 0;
    gXbeeInterStruct.aExpTime = 0.0;

    gComputeInterStruct.gCommandeMoteurDroit = 0.0;
    gComputeInterStruct.gCommandeMoteurGauche = 0.0;
    gComputeInterStruct.gCommandeServoDirection = 0.0;
    gComputeInterStruct.gConsigneMotor = 0;

    for (int i = 0; i < sizeof(gCmdBuffer); i++)
	{
	gCmdBuffer[i] = 0x00;
	}

    }

//------------------------------------------------------------------------
// communication avec le module xBEE
// envoi et lecture des trames
//------------------------------------------------------------------------
void gXBEE_Execute(void)
    {
    uint8_t aValTab_i[5];
    float aValTab_f[5];

    if (BytesInQueue(&XBEE_SERIAL_OUTGOING_QUEUE) == 0)
	{
	switch (gStateSend)
	    {

	case kPIDMot:
	    // Envoi des valeurs de gain
	    if (1) //(gXbeeInterStruct.aPIDChangedMotors)
		{
		gXbeeInterStruct.aPIDChangedMotors = false;

		aValTab_f[0] =
			gXbeeInterStruct.aGainPIDMotors.gProprortionalGain;
		aValTab_f[1] = gXbeeInterStruct.aGainPIDMotors.gIntegraleGain;
		aValTab_f[2] = gXbeeInterStruct.aGainPIDMotors.gDerivativeGain;

		//On envoie les gains
		send_val_float(REG_GAIN_SPEED, aValTab_f, 3);
		}
	    gStateSend = kPIDSer;
	    break;

	case kPIDSer:

	    if (1) // (gXbeeInterStruct.aPIDChangedServo)
		{
		gXbeeInterStruct.aPIDChangedServo = false;

		aValTab_f[0] =
			gXbeeInterStruct.aGainPIDServo.gProprortionalGain;
		aValTab_f[1] = gXbeeInterStruct.aGainPIDServo.gIntegraleGain;
		aValTab_f[2] = gXbeeInterStruct.aGainPIDServo.gDerivativeGain;

		//On envoie les gains
		send_val_float(REG_GAIN_DIR, aValTab_f, 3);
		}
	    gStateSend = kServAngle;
	    // gStateSend = kPIDMot;
	    break;

	case kServAngle:

	    // On envoie l'angle du servo
	    aValTab_f[0] = gComputeInterStruct.gCommandeServoDirection;
	    send_val_float(SERVO_ANGLE, aValTab_f, 1);
	    gStateSend = kSpeed;
	    break;

	case kSpeed:
	    // On envoie la vitesse des moteurs
	    aValTab_f[0] = ((float) gInputInterStruct.gFreq[0] / 3.0);
	    aValTab_f[1] = ((float) gInputInterStruct.gFreq[1] / 3.0);
	    aValTab_f[2] = ((float) gComputeInterStruct.gConsigneMotor);
	    send_val_float(SPEED_INFO, aValTab_f, 3);
	    gStateSend = kPosCam;
	    break;

	case kPosCam:
	    //On envoie la position de la ligne
	    aValTab_i[0] = gInputInterStruct.gPosCam1;
	    aValTab_i[1] = gInputInterStruct.gPosCam2;
	    send_val_int(CAM_POS_1_2, aValTab_i, 2);
	    gStateSend = kBattLed;
	    break;

	case kBattLed:
	    //Envoi la valeur de la batterie
	    aValTab_i[0] = gInputInterStruct.gBattLev;
	    send_val_int(BATT_LEV, aValTab_i, 1);
	    gStateSend = kPWM;
	    break;
	case kPWM:
	    //PWM des leds

	    aValTab_i[0] = ((gXbeeInterStruct.aPWMLeds + 1.0) / 2.0) * 100;

	    send_val_int(LED_PWM, aValTab_i, 1);
	    gStateSend = kExpTime;
	    break;
	case kExpTime:
	    //Temp d'exposition
	    aValTab_f[0] = gXbeeInterStruct.aExpTime;
	    send_val_float(EXPOSURE_T, aValTab_f, 1);
	    gStateSend = kPIDMot;
	    break;
	    }
	}

    //Lecture des bytes recus
    if (BytesInQueue(&XBEE_SERIAL_INCOMING_QUEUE) > 0)
	{
	bool endOfTrame = false;

	//On bufferize les bytes recu dans le buffer de commande, jusque à la détection du \n
	for (int i = gLastCmdPointer;
		BytesInQueue(&XBEE_SERIAL_INCOMING_QUEUE) > 0 && (!endOfTrame);
		i++)
	    {
	    ByteDequeue(&XBEE_SERIAL_INCOMING_QUEUE, &gCmdBuffer[i]);

	    // On enregistre la denière valeur de l'index
	    gLastCmdPointer = (i + 1);
	    if (gCmdBuffer[i] == END_OF_TRAME)
		{
		endOfTrame = true;
		gLastCmdPointer = 0x00;
		}
	    }

	//Test si on a recu une trame complete sinon on ne fait rien du tout
	if (endOfTrame == true)
	    {
	    commandAnalyser(gCmdBuffer);
	    }

	}
    }

void commandAnalyser(char *aCommandBuffer)
    {
    //On commence par tester la comamnde recue
    switch (aCommandBuffer[0])
	{

    //Gains pour la direction
    case REG_GAIN_DIR:
	//Ensuite on test le reste de la trame
	sscanf(aCommandBuffer, "G_%f_%f_%f\n",
		&gXbeeInterStruct.aGainPIDServo.gProprortionalGain,
		&gXbeeInterStruct.aGainPIDServo.gIntegraleGain,
		&gXbeeInterStruct.aGainPIDServo.gDerivativeGain);
	gXbeeInterStruct.aPIDChangedServo = true;
	break;

	//Gains pour la vitesse
    case REG_GAIN_SPEED:
	//Ensuite on test le reste de la trame
	sscanf(aCommandBuffer, "F_%f_%f_%f\n",
		&gXbeeInterStruct.aGainPIDMotors.gProprortionalGain,
		&gXbeeInterStruct.aGainPIDMotors.gIntegraleGain,
		&gXbeeInterStruct.aGainPIDMotors.gDerivativeGain);
	//On averti que l'on a changé les gains
	gXbeeInterStruct.aPIDChangedMotors = true;

	break;
    case MOTOR_SPEED:
	sscanf(aCommandBuffer, "J_%f\n", &gXbeeInterStruct.aMotorSpeedCons);
	gXbeeInterStruct.aMotorSpeedCons = ((gXbeeInterStruct.aMotorSpeedCons)); // ?? 

	break;

    case EXPOSURE_T:
	sscanf(aCommandBuffer, "H_%f\n", &gXbeeInterStruct.aExpTime);
	break;
    case LED_PWM:
	sscanf(aCommandBuffer, "I_%f\n", &gXbeeInterStruct.aPWMLeds);
	gXbeeInterStruct.aPWMLeds = ((gXbeeInterStruct.aPWMLeds / 100.0) * 2.0)
		- 1.0;
	break;
    default:
	break;
	// Commande non-connue
	}

    //Clean buffer
    for (int i = 0; i < sizeof(gCmdBuffer); i++)
	{
	gCmdBuffer[i] = 0x00;
	}

    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------
void send_val_int(char aType, uint8_t aValTab[], uint8_t aSize)
    {
    char aBufferToSend[20] =
	{
	0
	};
    char aBufferParam[10] =
	{
	0
	};

// On ecrit le caractère de début
    sprintf(aBufferToSend, "%c", aType);

// On écrit les valeurs des paramètres
    for (int i = 0; i < aSize; i++)
	{
	sprintf(aBufferParam, "_%d", aValTab[i]);
	strcat(aBufferToSend, aBufferParam);
	}

//Fin de trame
    strcat(aBufferToSend, "\n");

    TERMINAL_PRINTF(aBufferToSend);
    }

void send_val_float(char aType, float aValTab[], uint8_t aSize)
    {
    char aBufferToSend[30] =
	{
	0
	};
    char aBufferParam[30] =
	{
	0
	};

// On ecrit le caractère de début
    sprintf(aBufferToSend, "%c", aType);

// On écrit les valeurs des paramètres
    for (int i = 0; i < aSize; i++)
	{
	float temp = aValTab[i];
	sprintf(aBufferParam, "_%.3f", temp);
	strcat(aBufferToSend, aBufferParam);
	}

//Fin de trame
    strcat(aBufferToSend, "\n");

    TERMINAL_PRINTF(aBufferToSend);
    }
