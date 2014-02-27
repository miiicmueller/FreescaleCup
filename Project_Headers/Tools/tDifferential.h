/*
 * tDifferential.h
 *
 *  Created on: Feb 27, 2014
 *      Author: cyrille.savy
 */

#ifndef TDIFFERENTIAL_H_
#define TDIFFERENTIAL_H_

#include "Tools/tPID.h"

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
void compute_differential(const float aAngleServo, tPIDStruct* thePIDStruct);

#endif /* TDIFFERENTIAL_H_ */
