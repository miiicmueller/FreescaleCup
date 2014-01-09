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
#define kEcartMin	3*kThresholdTab	//ecart minimal entre le min et le max de la derivee pour que l'on considere que quelquechose se trouve dans le champ de vision
#define kSizeEdgesTab	10	//nombre de flancs maximum que l'on peut trouver
#define kLengthLineMin	4	//longueur min de la ligne a trouver (en pixels)
#define kLengthLineMax	20	//longueur max de la ligne a trouver (en pixels)
#define kLengthStartStopMin	10	//longueur min d'un morceau de la ligne de départ/arrivée a trouver (en pixels)
//Attention aux contours, la largeur augmente
#define kLengthStartStopMax	40	//longueur max d'un morceau de la ligne de départ/arrivée a trouver (en pixels)
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

static uint16_t mTrackLine_FindEdges(int16_t* tab, uint16_t sizeTab,
	mTrackLineEdges* theEdgesTab);

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
    tRescale(tab, size, kScaleTab, kEcartMin); //on fait le rescale seulement si un ecart minimal est atteint
    tThreshold(tab, size, kThresholdTab);
    nbOfEdges = mTrackLine_FindEdges(tab, size, theEdges);

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

    //pattern pour la ligne de depart/arrivee
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
// trouve tous les flanc présents dans le champ de vision
// parametre de retour	: uint16_t : nombre d'objets edges qui ont ete trouves
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
