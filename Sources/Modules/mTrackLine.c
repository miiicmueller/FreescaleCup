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
#define kSizeEdgesTab	10	//nombre de flancs maximum que l'on peut trouver
//TODO : mesurer les quatre valeurs suivantes
#define kLengthLineMin	15	//longueur min de la ligne a trouver (en pixels)
#define kLengthLineMax	30	//longueur max de la ligne a trouver (en pixels)
#define kLengthStartStopMin	15	//longueur min d'un morceau de la ligne de départ/arrivée a trouver (en pixels)
#define kLengthStartStopMax	30	//longueur max d'un morceau de la ligne de départ/arrivée a trouver (en pixels)
//--

//definitions de types et structures
typedef enum
    {
    dark = 0,
    bright = 1
    } mTrackLineObjectType;

typedef enum
    {
    falling = 0,
    rising = 1
    } mTrackLineEdgeEnum;

typedef struct
    {
	uint16_t location;
	uint16_t length;
	mTrackLineObjectType theType;
    } mTrackLineObject;

typedef struct
    {
	mTrackLineEdgeEnum edgeType;
	uint16_t location;
    } mTrackLineEdges;

// prototypes des fonctions statiques au module
//static bool mTrackLine_FindDarkObject(int16_t* tab, uint16_t size,
//	uint16_t objectNumber, mTrackLineObject* theObject, bool restart);

static uint16_t mTrackLine_FindEdges(int16_t* tab, uint16_t sizeTab,
	mTrackLineEdges* theEdgesTab);

//definition des fonctions
//--------------------------------------------------------
// trouve la position de la ligne sur l'image de la camera
// parametre de retour	: bool : true si la ligne est presente, false sinon
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
//bool mTrackLine_FindLine(int16_t* tab, uint16_t size, int16_t* thePosition)
//    {
//    mTrackLineObject theLine;
//    uint16_t i = 1;
//    bool isLineFound = false;
//    bool isFrameEnd = false;
//    bool isDebut = true;
//
//    tDerivative(tab, size);
//    tSuppressDC(tab, size);
//    tRescale(tab, size, kScaleTab);
//    tThreshold(tab, size, kThresholdTab);
//
//    while (!isLineFound && !isFrameEnd)
//	{
//	if (mTrackLine_FindDarkObject(tab, size, i, &theLine, isDebut))
//	    {
//	    isDebut = false;
//	    if ((theLine.length > kLengthLineMin)
//		    && (theLine.length < kLengthLineMax))
//		{
//		isLineFound = true;
//		}
//	    else
//		{
//		i++;
//		}
//	    }
//	else
//	    {
//	    isFrameEnd = true;
//	    }
//	}
//
//    *thePosition = theLine.location;
//
//    return isLineFound;
//    }

//definition des fonctions
//--------------------------------------------------------
// trouve la position de la ligne sur l'image de la camera
// parametre de retour	: bool : true si la ligne est presente, false sinon
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
void mTrackLine_FindLine(int16_t* tab, uint16_t size, int16_t* thePosition,
	bool* isLineFound, bool* isStartStopFound)
    {
    mTrackLineEdges theEdges[kSizeEdgesTab];
    uint16_t nbOfEdges;

    *isLineFound = false;
    *isStartStopFound = false;

    tDerivative(tab, size);
    tSuppressDC(tab, size);
    tRescale(tab, size, kScaleTab);
    tThreshold(tab, size, kThresholdTab);
    nbOfEdges = mTrackLine_FindEdges(tab, size, &theEdges);

    //pattern pour la ligne centrale
    //on doit avoir au moins deux flancs
    if (nbOfEdges >= 2)
	{
	for (uint16_t i = 0; i < (nbOfEdges - 1); i++)
	    {
	    if ((theEdges[i].edgeType == falling)
		    && (theEdges[i + 1].edgeType == rising))
		{
		if (((theEdges[i + 1].location - theEdges[i].location)
			> kLengthLineMin)
			&& ((theEdges[i + 1].location - theEdges[i].location)
				< kLengthLineMax))
		    {
		    *isLineFound = true;
		    *thePosition = (theEdges[i + 1].location
			    + theEdges[i].location) / 2;
		    }
		}
	    }
	}

    //pattern pour la ligne de départ/arrivée
    //on doit avoir au moins quatre flancs
    if (nbOfEdges >= 4)
	{
	for (uint16_t i = 0; i < (nbOfEdges - 3); i++)
	    if ((theEdges[i].edgeType == rising)
		    && (theEdges[i + 1].edgeType == falling)
		    && (theEdges[i + 2].edgeType == rising)
		    && (theEdges[i + 3].edgeType == falling))
		{
		if (((theEdges[i + 1].location - theEdges[i].location)
			> kLengthStartStopMin)
			&& ((theEdges[i + 1].location - theEdges[i].location)
				< kLengthStartStopMax)
			&& ((theEdges[i + 2].location - theEdges[i + 1].location)
				> kLengthLineMin)
			&& ((theEdges[i + 2].location - theEdges[i + 1].location)
				< kLengthLineMax)
			&& ((theEdges[i + 3].location - theEdges[i + 2].location)
				> kLengthStartStopMin)
			&& ((theEdges[i + 3].location - theEdges[i + 2].location)
				< kLengthStartStopMax))
		    {
		    *isStartStopFound = true;
		    }
		}
	}

    return;
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
//static bool mTrackLine_FindDarkObject(int16_t* tab, uint16_t size,
//	uint16_t objectNumber, mTrackLineObject* theObject, bool restart)
//    {
//    static uint16_t i = 1;
//
//    uint16_t risingEdge;
//    uint16_t fallingEdge;
//    bool isObjectFound;
//
//    uint16_t currentObject = 0;
//
//    if (restart)
//	{
//	i = 1;
//	}
//
//    while ((currentObject < objectNumber) && i < size)
//	{
//	risingEdge = 0;
//	fallingEdge = 0;
//	isObjectFound = false;
//
//	// dark object : un flanc descendant suivi d'un flanc montant
//	while ((i < size) && (!isObjectFound))
//	    {
//	    //on cherche un flanc descendant
//	    if (((tab[i - 1] == 0) || (tab[i - 1] == 1)) && (tab[i] == -1))
//		{
//		fallingEdge = i;
//		}
//	    //suivi d'un flanc montant
//	    if (((tab[i - 1] == 0) || (tab[i - 1] == -1)) && (tab[i] == 1))
//		{
//		risingEdge = i;
//		}
//
//	    if ((fallingEdge != 0) && (risingEdge != 0))
//		{
//		if (risingEdge > fallingEdge)
//		    {
//		    isObjectFound = true;
//		    currentObject++;
//		    }
//		}
//
//	    i++;
//	    }
//	}
//
//    theObject->length = risingEdge - fallingEdge;
//    theObject->location = fallingEdge + (theObject->length / 2);
//
//    return isObjectFound;
//    }

//--------------------------------------------------------
// trouve tous les flanc présents dans le champ de vision
// parametre de retour	: uint16_t : nombre d'objets edges qui ont été trouvés
// parametres : tab		: adresse du tableau a traiter
//		sizeTab		: longueur du tableau
//		theEdgesTab	: adresse du tableau qui contient les objets Edges
//--------------------------------------------------------
static uint16_t mTrackLine_FindEdges(int16_t* tab, uint16_t sizeTab,
	mTrackLineEdges* theEdgesTab)
    {
    uint16_t indexTab = 0;
    uint16_t indexEdgesTab = 0;

    // dark object : un flanc descendant suivi d'un flanc montant
    while ((indexTab < sizeTab) && (indexEdgesTab < kSizeEdgesTab))
	{
	//on cherche un flanc descendant
	if (((tab[indexTab - 1] == 0) || (tab[indexTab - 1] == 1))
		&& (tab[indexTab] == -1))
	    {
	    theEdgesTab[indexEdgesTab].edgeType = falling;
	    theEdgesTab[indexEdgesTab].location = indexTab;
	    indexEdgesTab++;
	    }
	//on cherche un flanc montant
	if (((tab[indexTab - 1] == 0) || (tab[indexTab - 1] == -1))
		&& (tab[indexTab] == 1))
	    {
	    theEdgesTab[indexEdgesTab].edgeType = rising;
	    theEdgesTab[indexEdgesTab].location = indexTab;
	    indexEdgesTab++;
	    }
	indexTab++;
	}

    return indexEdgesTab;
    }
