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
// D�claration des constantes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// D�claration des types
//-----------------------------------------------------------------------------
// Cr�ation du type de la structure contenant les Inputs
typedef struct
    {
    } InputInterStruct;

// Cr�ation du type de la structure contenant les outputs
typedef struct
    {
	float gCommandeMoteurDroit;
	float gCommandeMoteurGauche;
	float gCommandeServoDirection;
    } ComputeInterStruct;

//-----------------------------------------------------------------------------
// D�claration des variables globales
//-----------------------------------------------------------------------------
// Importation de la variable du type contenant les Inputs (elle est dans gMBox.c)
extern InputInterStruct gInput;

// Importation de la variable du type contenant les �tats des erreurs (elle est dans gMBox.c)
extern ComputeInterStruct gCompute;

//-----------------------------------------------------------------------------
// D�claration des prototypes de fonctions
//-----------------------------------------------------------------------------

#endif /* GMBOX_H_ */

