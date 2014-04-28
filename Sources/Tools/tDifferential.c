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
    if (aAngleServo < 0.0)
	{
	mMotor2.aDifferential = (kCOEFF_DIFFERENTIAL * tAbs_float(aAngleServo) + 1.0);
	mMotor1.aDifferential = 1;
	}
    else
	{
	mMotor1.aDifferential = (kCOEFF_DIFFERENTIAL * tAbs_float(aAngleServo) + 1.0);
	mMotor2.aDifferential = 1;
	}
    }
