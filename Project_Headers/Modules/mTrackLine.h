//*****************************************************************************
//Nom du fichier : mTrackLine.h
//Auteur et Date : SAVY Cyrille 25.10.2013
//But : fonctions permettant de trouver la ligne sur la piste
//*****************************************************************************

#ifndef TTRACKLINE_H_
#define TTRACKLINE_H_

//fichiers utilises
#include "TFC\TFC.h"

//--------------------------------------------------------
// trouve la position de la ligne sur l'image de la camera
// parametre de retour	: bool : true si la ligne est presente, false sinon
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
void mTrackLine_FindLine(int16_t* tab, uint16_t size, int16_t* thePosition, bool* isLineFound, bool* isStartStopFound,
	int16_t oldLinePosition);

//--------------------------------------------------------
// trouve la position de la ligne sur l'image de la camera
// parametre de retour	: bool : true si la ligne est presente, false sinon
// 			: int16_t : thePosition = position de la ligne
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
void mTrackLine_Correlation(int16_t* tabNear, int16_t* tabFar, uint16_t size, int16_t* thePositionNear,
	int16_t* thePositionFar, bool* isLineNearFound, bool* isLineFarFound, bool* isStartStopFound);

//--------------------------------------------------------
// trouve la position de la ligne sur l'image de la camera
// parametre de retour	: bool : true si la ligne est presente, false sinon
// 			: int16_t : thePosition = position de la ligne
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
void mTrackLine_CorrelationFFT(int16_t* tabNear, int16_t* tabFar, int16_t* thePositionNear, int16_t* thePositionFar,
	bool* isLineNearFound, bool* isLineFarFound, bool* isStartStopFound);

#endif /* TTRACKLINE_H_ */
