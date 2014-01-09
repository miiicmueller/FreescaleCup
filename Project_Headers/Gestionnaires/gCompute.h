/*
 * gCompute.h
 *
 *  Created on: Nov 1, 2013
 *      Author: cyrille.savy
 */

#ifndef GCOMPUTE_H_
#define GCOMPUTE_H_

#include "TFC\TFC.h"

//-----------------------------------------------------------------------------
// D�claration des constantes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// D�claration des types
//-----------------------------------------------------------------------------
  
//-----------------------------------------------------------------------------
// D�claration des variables globales
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// D�claration des prototypes de fonctions
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de donn�es de gInput
//------------------------------------------------------------------------
void gCompute_Setup ( void );
//------------------------------------------------------------------------
// acquisition de l'�tat des switchs et mise � disposition de leur �tat
// aux autres modules via la structure des donn�es de gInput
//------------------------------------------------------------------------
void gCompute_Execute ( void );

//-----------------------------------------------------------------------------
// Fonctions privees
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Compute differential
// param : aAngleServo --> Consigne du servoMoteur
//
// Description : 	Avec aAngleServo a 0.51, pour 1 tour de la roue exterieur
//					la roue interieur en fait 0.66.
//
//					Cette fonction calcul le differentiel � appliquer
//					aux moteurs selon aAngleServo.
//-----------------------------------------------------------------------------
static void compute_differential(const float aAngleServo);

#endif /* GCOMPUTE_H_ */
