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
// Déclaration des constantes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Déclaration des types
//-----------------------------------------------------------------------------
  
//-----------------------------------------------------------------------------
// Déclaration des variables globales
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Déclaration des prototypes de fonctions
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de données de gInput
//------------------------------------------------------------------------
void gCompute_Setup ( void );
//------------------------------------------------------------------------
// acquisition de l'état des switchs et mise à disposition de leur état
// aux autres modules via la structure des données de gInput
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
//					Cette fonction calcul le differentiel à appliquer
//					aux moteurs selon aAngleServo.
//-----------------------------------------------------------------------------
static void compute_differential(const float aAngleServo);

#endif /* GCOMPUTE_H_ */
