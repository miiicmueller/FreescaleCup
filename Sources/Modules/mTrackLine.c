//*****************************************************************************
//Nom du fichier : mTrackLine.c
//Auteur et Date : SAVY Cyrille 25.10.2013
//But : fonctions permettant de trouver la ligne sur la piste
//*****************************************************************************

//fichiers utilises
#include "Modules/mTrackLine.h"
#include "Tools/Tools.h"

//definitions de constantes
#define kScaleTab	100	//echelle a laquelle on veut mettre le tableau
#define kThresholdTab	50	//seuil a partir duquel un flanc est considere comme significatif
#define kLengthLineMin	25	//longueur min de la ligne a trouver (en pixels)
#define kLengthLineMax	50	//longueur max de la ligne a trouver (en pixels)
// prototypes des fonctions statiques au module
static bool mTrackLine_FindObject(int16_t* tab, uint16_t size,
	uint16_t objectNumber, mTrackLineObject* theObject);

//definition des fonctions
//--------------------------------------------------------
// trouve la position de la ligne sur l'image de la camera
// parametre de retour	: bool : true si la ligne est presente, false sinon
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
bool mTrackLine_FindLine(int16_t* tab, uint16_t size, uint16_t* thePosition)
    {
    mTrackLineObject theLine;
    uint16_t i = 0;
    bool isLineFound = false;
    bool isFrameEnd = false;

    tDerivative(tab, size);
    tSuppressDC(tab, size);
    tRescale(tab, size, kScaleTab);
    tThreshold(tab, size, kThresholdTab);

    while (!isLineFound && !isFrameEnd)
	{
	if (mTrackLine_FindObject(tab, size, i, &theLine))
	    {
	    if ((theLine.length > kLengthLineMin)
		    && (theLine.length < kLengthLineMax))
		{
		isLineFound = true;
		}
	    }
	else
	    {
	    isFrameEnd = true;
	    }
	}

    *thePosition = theLine.location;

    return isLineFound;
    }

//--------------------------------------------------------
// trouve un objet present dans le champ de vision
// parametre de retour	: bool : true si l'objet a ete trouve, false sinon
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//		theObject: l'objet trouve
//--------------------------------------------------------
static bool mTrackLine_FindObject(int16_t* tab, uint16_t size,
	uint16_t objectNumber, mTrackLineObject* theObject)
    {
    bool isObjectFound = false;

    return isObjectFound;
    }
