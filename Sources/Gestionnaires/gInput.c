/*
 * gInput.c
 *
 *  Created on: Nov 1, 2013
 *      Author: cyrille.savy
 */

#include "TFC\TFC.h"
#include "Gestionnaires\gInput.h"
#include "Gestionnaires\gMbox.h"
#include "Modules\mMotor.h"

/* prototypes des fonctions statiques (propres au fichier) */

//-----------------------------------------------------------------------------
// fonctions publiques
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de données de gInput
//------------------------------------------------------------------------
void gInput_Setup(void)
    {
    }

//------------------------------------------------------------------------
// acquisition de l'état des switchs et mise à disposition de leur état
// aux autres modules via la structure des données de gInput
//------------------------------------------------------------------------
void gInput_Execute(void)
    {
    gInputInterStruct.gFreq[0] = mMotor1.aFreq;
    gInputInterStruct.gFreq[1] = mMotor2.aFreq;
    }
