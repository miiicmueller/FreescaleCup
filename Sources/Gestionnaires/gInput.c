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
// Initialisation de la structure de donn�es de gInput
//------------------------------------------------------------------------
void gInput_Setup(void)
    {
    }

//------------------------------------------------------------------------
// acquisition de l'�tat des switchs et mise � disposition de leur �tat
// aux autres modules via la structure des donn�es de gInput
//------------------------------------------------------------------------
void gInput_Execute(void)
    {
    gInputInterStruct.gSpeed[0] = mMotor1.aCapt;
    gInputInterStruct.gSpeed[1] = mMotor2.aCapt;
    }
