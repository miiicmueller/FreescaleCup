/*
 * gOutput.c
 *
 *  Created on: Nov 1, 2013
 *      Author: cyrille.savy
 */

#include "Gestionnaires\gOutput.h"
#include "TFC\TFC.h"
#include "Gestionnaires\gMbox.h"

/* prototypes des fonctions statiques (propres au fichier) */

//-----------------------------------------------------------------------------
//fonctions publiques
//-----------------------------------------------------------------------------
//------------------------------------------------------------------------
// Initialisation de la structure de donn�es de gInput
//------------------------------------------------------------------------
void gOutput_Setup(void)
    {
    TFC_SetServo(1, 0);

    TFC_SetMotorPWM(0, 0); //Make sure motors are off
    TFC_HBRIDGE_DISABLE;
    }

//------------------------------------------------------------------------
// acquisition de l'�tat des switchs et mise � disposition de leur �tat
// aux autres modules via la structure des donn�es de gInput
//------------------------------------------------------------------------
void gOutput_Execute(void)
    {

    }

//-----------------------------------------------------------------------------
//fonctions statiques
//-----------------------------------------------------------------------------
