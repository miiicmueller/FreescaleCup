/*
 * gXBEE.h
 *
 *  Created on: Nov 7, 2013
 *      Author: cyrille.savy
 */

#ifndef GXBEE_H_
#define GXBEE_H_

#include "TFC\TFC.h"

//-----------------------------------------------------------------------------
// D�claration des constantes
//-----------------------------------------------------------------------------
#define CAM_POS_1_2 	'A' // posLn1,posLn2 [0-128]
#define ACCEL   	'B' // X,Y,Z
#define SPEED_INFO 	'C' // lSpeed,lRight,consigne1 [0-65535]
#define SERVO_ANGLE	'D' // [-1.0,1.0]
#define BATT_LEV	'E' // [0-100] %
#define REG_GAIN_SPEED 	'F' // floats
#define REG_GAIN_DIR 	'G' // floats
#define EXPOSURE_T 	'H' // [-1.0,1.0]
#define LED_PWM 	'I' // [0-100] %

//-----------------------------------------------------------------------------
// D�claration des types
//-----------------------------------------------------------------------------
typedef struct
    {
	float gProprortionalGain;
	float gIntegraleGain;
	float gDerivativeGain;
    }PIDGainStruct;
//-----------------------------------------------------------------------------
// D�claration des variables globales
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// D�claration des prototypes de fonctions
//-----------------------------------------------------------------------------

/**
 * Envoi une trame au Labview
 * 
 * aType : type de trame (Accel,Pos ligne, etc..)
 * aValTab : Tableau qui contient les valeurs
 * aSize   : Taille du tableau
 * retour : rien
 */
void send_cmd(char aType,uint8_t aValTab[],uint8_t aSize);
void send_val_float(char aType, float aValTab[], uint8_t aSize);


//------------------------------------------------------------------------
// Initialisation de la structure de donn�es de gXBEE
//------------------------------------------------------------------------
void gXBEE_Setup(void);

//------------------------------------------------------------------------
// communication avec le module xBEE
// envoi et lecture des trames
//------------------------------------------------------------------------
void gXBEE_Execute(void);

#endif /* GXBEE_H_ */
