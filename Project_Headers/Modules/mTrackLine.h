//*****************************************************************************
//Nom du fichier : mTrackLine.h
//Auteur et Date : SAVY Cyrille 25.10.2013
//But : fonctions permettant de trouver la ligne sur la piste
//*****************************************************************************

#ifndef TTRACKLINE_H_
#define TTRACKLINE_H_

//fichiers utilises
#include "TFC\TFC.h"

//definitions de types et structures
typedef struct
    {
	uint16_t location;
	uint16_t length;
    } mTrackLineObject;

//--------------------------------------------------------
// trouve la position de la ligne sur l'image de la camera
// parametre de retour	: bool : true si la ligne est presente, false sinon
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
bool mFindLine(int16_t* tab, uint16_t size);

#endif /* TTRACKLINE_H_ */
