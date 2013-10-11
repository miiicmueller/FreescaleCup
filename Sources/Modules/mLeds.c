/* 
 ------------------------------------------------------------ 
 Copyright 2003-200x Haute Ecole ARC Ingénierie,  
 Switzerland. All rights reserved 
 ------------------------------------------------------------ 
 Nom du fichier : mLeds.c
 Auteur et Date : Michael Mueller
 
 Description dans le fichier mLeds.h 
 ------------------------------------------------------------ 
 */

#include "TFC\TFC.h"
#include "Modules/mLeds.h"
#include "derivative.h"

//--------------------------------------------------------- 
// Configuration du système pour piloter les LEDs d'éclairage
// retour   : rien    
//--------------------------------------------------------- 

void mLeds_Setup() {

	//Setup Channels 4
	TPM0_C4SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;

	//Sortie PWM TMP0 CH4
	PORTE_PCR31 = PORT_PCR_MUX(3) | PORT_PCR_DSE_MASK;

	//On arrête l'envoi 
	mLeds_writeDyC(0.0);
}

//--------------------------------------------------------- 
// Activation  des LEDs d'éclairage
// retour   : rien    
//--------------------------------------------------------- 

void mLeds_Open() {

}

//--------------------------------------------------------- 
// Desactivation  des LEDs d'éclairage
// retour   : rien    
//--------------------------------------------------------- 

void mLeds_Close() {
	mLeds_writeDyC(0.0);
}

//--------------------------------------------------------- 
// Activation  des LEDs d'éclairage
// aDyC : Duty cycle de 0-100 en %
// retour   : rien    
//--------------------------------------------------------- 
void mLeds_writeDyC(float aDyC) {

	if (aDyC > 1.0)
		aDyC = 1.0;
	else if (aDyC < -1.0)
		aDyC = -1.0;
	
	
	TPM0_C4V = (uint16_t) ((float)TPM0_MOD * (float)((aDyC + 1.0)/2.0));
}
