/*
 * tDifferential.c
 *
 *  Created on: Feb 27, 2014
 *      Author: cyrille.savy
 */

#include "Tools/tDifferential.h"
#include "parameters.h"
#include "Tools/Tools.h"
#include "Gestionnaires/gMbox.h"

void compute_differential(const float aAngleServo, tPIDStruct* thePIDStruct)
    {

    float m = -0.8;
    //Moyenne des erreurs
    float aMoyenneError = tAbs_float(
	    (1.0 / 3)
		    * (thePIDStruct->thePastError[0]
			    + thePIDStruct->thePastError[1]
			    + thePIDStruct->thePastError[2]));

    if (aMoyenneError < 0.3)
	{
	aMoyenneError = 0;
	//full_break = false;
	//half_break = false;
	TFC_BAT_LED2_OFF;
	TFC_BAT_LED3_OFF;
	}
    // Si on a perdu la ligne plus de ~30ms et que l'on braque à fond , on freine comme des porcs
    else if ((aMoyenneError > 0.25) && (tAbs_float(aAngleServo) > 0.35)
	    && ((gInputInterStruct.gFreq[0] / 3.0) > (K_SPEED_LOWEST - 15)))
	{
	TFC_BAT_LED2_ON;
	//full_break = true;
	}
    // Sinon on freine un peu moins si on se trouve au bord et on corrige assez fort
    else if ((aMoyenneError > 0.20) && (tAbs_float(aAngleServo) > 0.15)
	    && ((gInputInterStruct.gFreq[0] / 3.0) > (K_SPEED_LOWEST - 15)))
	{
	TFC_BAT_LED3_ON;
	//half_break = true;
	}
    else
	{
	TFC_BAT_LED2_OFF;
	TFC_BAT_LED3_OFF;
	//full_break = false;
	//half_break = false;
	}

    //float speedScaleFactor = -K_BRAKE_FACTOR * aMoyenneError + 1.0;

    if (aAngleServo >= 0.0)
	{
	//mMotor1.aDifferential = speedScaleFactor;
	//mMotor2.aDifferential = (m * tAbs_float(aAngleServo) + 1.0)
	//*speedScaleFactor;

	}
    else
	{
	//mMotor1.aDifferential = (m * tAbs_float(aAngleServo) + 1.0)
	//*speedScaleFactor;
	//mMotor2.aDifferential = speedScaleFactor;
	}
    }
