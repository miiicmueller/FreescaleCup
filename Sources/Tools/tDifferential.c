/*
 * tDifferential.c
 *
 *  Created on: Feb 27, 2014
 *      Author: cyrille.savy
 */

#include "Tools/tDifferential.h"
#include "parameters.h"
#include "Tools/Tools.h"
#include "Modules/mMotor.h"
#include "Gestionnaires/gMbox.h"

void compute_differential(const float aAngleServo)
    {
    float m = -0.82;

    if (aAngleServo < 0.0)
	{
	mMotor2.aDifferential = (m * tAbs_float(aAngleServo) + 1.0);
	mMotor1.aDifferential = 1;
	}
    else
	{
	mMotor1.aDifferential = (m * tAbs_float(aAngleServo) + 1.0);
	mMotor2.aDifferential = 1;
	}
    }
