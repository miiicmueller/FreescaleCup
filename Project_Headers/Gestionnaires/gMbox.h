/*
 * gMbox.h
 *
 *  Created on: Nov 1, 2013
 *      Author: cyrille.savy
 */

#ifndef GMBOX_H_
#define GMBOX_H_

#include "TFC\TFC.h"

//-----------------------------------------------------------------------------
// Déclaration des constantes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Déclaration des types
//-----------------------------------------------------------------------------
// Création du type de la structure contenant les Inputs
typedef struct
    {
    } InputInterStruct;

// Création du type de la structure contenant les outputs
typedef struct
    {
	float gCommandeMoteurDroit;
	float gCommandeMoteurGauche;
	float gCommandeServoDirection;
	bool isFinish;
    } ComputeInterStruct;

//-----------------------------------------------------------------------------
// Déclaration des variables globales
//-----------------------------------------------------------------------------
// Importation de la variable du type contenant les Inputs (elle est dans gMBox.c)
extern InputInterStruct gInput;

// Importation de la variable du type contenant les États des erreurs (elle est dans gMBox.c)
extern ComputeInterStruct gCompute;

//-----------------------------------------------------------------------------
// Déclaration des prototypes de fonctions
//-----------------------------------------------------------------------------

#endif /* GMBOX_H_ */

