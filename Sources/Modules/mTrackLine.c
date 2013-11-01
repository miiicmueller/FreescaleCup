//*****************************************************************************
//Nom du fichier : mTrackLine.c
//Auteur et Date : SAVY Cyrille 25.10.2013
//But : fonctions permettant de trouver la ligne sur la piste
//*****************************************************************************

//fichiers utilises
#include "Modules\mTrackLine.h"
#include "Tools\Tools.h"

//definitions de constantes
#define kScaleTab	100	//echelle a laquelle on veut mettre le tableau
#define kThresholdTab	40	//seuil a partir duquel un flanc est considere comme significatif
#define kLengthLineMin	25	//longueur min de la ligne a trouver (en pixels)
#define kLengthLineMax	50	//longueur max de la ligne a trouver (en pixels)
//definitions de types et structures
typedef struct
    {
	uint16_t location;
	uint16_t length;
    } mTrackLineObject;

// prototypes des fonctions statiques au module
static bool mTrackLine_FindDarkObject(int16_t* tab, uint16_t size,
	uint16_t objectNumber, mTrackLineObject* theObject, bool restart);

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
    uint16_t i = 1;
    bool isLineFound = false;
    bool isFrameEnd = false;
    bool isDebut = true;

    tDerivative(tab, size);
    tSuppressDC(tab, size);
    tRescale(tab, size, kScaleTab);
    tThreshold(tab, size, kThresholdTab);

    while (!isLineFound && !isFrameEnd)
	{
	if (mTrackLine_FindDarkObject(tab, size, i, &theLine, isDebut))
	    {
	    isDebut = false;
	    if ((theLine.length > kLengthLineMin)
		    && (theLine.length < kLengthLineMax))
		{
		isLineFound = true;
		}
	    else
		{
		i++;
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
// parametre de retour	: bool 	: true si l'objet a ete trouve, false sinon
// parametres : tab		: adresse du tableau a traiter
//		size		: longueur du tableau
//		objectNumber	: le n-ieme objet a trouver dans le champ de vision
//		theObject	: l'objet trouve
//		restart		: indique si la recherche reprend depuis le debut du tableau (true),
//				  ou si elle reprend a partir du dernier objet trouve (false)
//--------------------------------------------------------
static bool mTrackLine_FindDarkObject(int16_t* tab, uint16_t size,
	uint16_t objectNumber, mTrackLineObject* theObject, bool restart)
    {
    static uint16_t i = 1;

    uint16_t risingEdge;
    uint16_t fallingEdge;
    bool isObjectFound;

    uint16_t currentObject = 0;

    if (restart)
	{
	i = 1;
	}

    while ((currentObject < objectNumber) && i < size)
	{
	risingEdge = 0;
	fallingEdge = 0;
	isObjectFound = false;

	// dark object : un flanc descendant suivi d'un flanc montant
	while ((i < size) && (!isObjectFound))
	    {
	    //on cherche un flanc descendant
	    if (((tab[i - 1] == 0) || (tab[i - 1] == 1)) && (tab[i] == -1))
		{
		fallingEdge = i;
		}
	    //suivi d'un flanc montant
	    if (((tab[i - 1] == 0) || (tab[i - 1] == -1)) && (tab[i] == 1))
		{
		risingEdge = i;
		}

	    if (risingEdge > fallingEdge)
		{
		isObjectFound = true;
		currentObject++;
		}
	    i++;
	    }
	}

    theObject->length = risingEdge - fallingEdge;
    theObject->location = fallingEdge + (theObject->length / 2);

    return isObjectFound;
    }
