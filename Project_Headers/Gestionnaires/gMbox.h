/*
 * gMbox.h
 *
 *  Created on: Nov 1, 2013
 *      Author: cyrille.savy
 */

#ifndef GMBOX_H_
#define GMBOX_H_

#include "TFC\TFC.h"
#include "Gestionnaires/gXBEE.h"

//-----------------------------------------------------------------------------
// D�claration des constantes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// D�claration des types
//-----------------------------------------------------------------------------
// Cr�ation du type de la structure contenant les Inputs
typedef struct
    {
	uint8_t gPosCam1; //position de la ligne sur l'image de la cam�ra 1
	uint8_t gPosCam2; //position de la ligne sur l'image de la cam�ra 2
	uint8_t gAccel[3]; // 0 = X, 1=Y, 2=Z : acc�l�rom�tre
	uint8_t gBattLev;
	uint16_t gSpeed[2]; // 0 : vitesse moteur gauche, 1 : vitesse moteur droit
    } InpInterStruct;

// Cr�ation du type de la structure contenant les outputs
typedef struct
    {
	float gCommandeMoteurDroit;
	float gCommandeMoteurGauche;
	float gCommandeServoDirection;
	uint16_t gConsigneMotor;
	uint8_t gPWMLeds;
	float gExpTime;
	bool isFinish;
    } ComputeInterStruct;

// Cr�ation du type de la structure contenant les outputs
typedef struct
    {
	PIDGainStruct aGainPIDMotors;
	PIDGainStruct aGainPIDServo;
	bool aPIDChangedServo;
	bool aPIDChangedMotors;
    } xBeeInterStruct;

//-----------------------------------------------------------------------------
// D�claration des variables globales
//-----------------------------------------------------------------------------
// Importation de la variable du type contenant les Inputs (elle est dans gMBox.c)
extern InpInterStruct gInputInterStruct;

// Importation de la variable du type contenant les �tats des erreurs (elle est dans gMBox.c)
extern ComputeInterStruct gComputeInterStruct;

// Importation de la variable du type qui contient les param�tres � envoyer
extern xBeeInterStruct gXbeeInterStruct;

//-----------------------------------------------------------------------------
// D�claration des prototypes de fonctions
//-----------------------------------------------------------------------------

#endif /* GMBOX_H_ */
