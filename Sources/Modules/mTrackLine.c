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
#define kLengthLineMin	5	//longueur min de la ligne a trouver (en pixels)
#define kLengthLineMax	30	//longueur max de la ligne a trouver (en pixels)
#define kLengthStartStopMin	3	//longueur min d'un morceau de la ligne de d�part/arriv�e a trouver (en pixels)
//Attention aux contours, la largeur augmente
#define kLengthStartStopMax	30	//longueur max d'un morceau de la ligne de d�part/arriv�e a trouver (en pixels)
//--

#define kINIT_VAL 304
#define kLINE_SCAN_SIZE 128

#define kLINE_PATTERN_SIZE 35
#define kLINE_PATTERN {1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1, 1, 1, 1,-4, -4, -4, -4, -4, -4, -4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1, 1, 1, 1} 

#define kFINISH_PATTERN_SIZE 35
#define kFINISH_PATTERN {-6, -6, -6, -6, -6, -6, -6, 9, 9, 9, 9, 9,9, 9, -6, -6, -6, -6, -6, -6, -6, 9, 9, 9, 9, 9,9, 9, -6, -6, -6, -6, -6, -6, -6} 

#define kSEUIL_LIGNE 		25000
#define kSEUIL_LIGNE_ARR 	180000

#define kOFFSET_LIGNE 		kLINE_PATTERN_SIZE/2
#define kOFFSET_LIGNE_ARR 	kFINISH_PATTERN_SIZE/2

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

    if (nbOfEdges < 9)
	{

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
			    && ((theEdges[i + 2].location
				    - theEdges[i + 1].location) > kLengthLineMin)
			    && ((theEdges[i + 2].location
				    - theEdges[i + 1].location) < kLengthLineMax)
			    && ((theEdges[i + 3].location
				    - theEdges[i + 2].location)
				    > kLengthStartStopMin)
			    && ((theEdges[i + 3].location
				    - theEdges[i + 2].location)
				    < kLengthStartStopMax))
			{
			*isStartStopFound = true;
			}
		    }
	    }
	}

    return;
    }

//--------------------------------------------------------
// trouve la position de la ligne sur l'image de la camera et d�tecte si 
// pr�sence de la ligne d'arriv�e.
// parametre de retour	: bool : true si la ligne est presente, false sinon
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//
// 	Ici nous utilisons le principe de la correlation pour retrouver
//	l'endroit qui ressemble le plus � notre pattern. (Convolution)
//
//
//--------------------------------------------------------
void mTrackLine_Correlation(int16_t* tab, uint16_t size, int16_t* thePosition,
	bool* isLineFound, bool* isStartStopFound)
    {
    // Variable n�cessaire � la convolution
    int32_t aConvultionResultTab[kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE]; // ATTENTION : kLINE_PATTERN_SIZE >= kFINISH_PATTERN_SIZE, sinon il y aura des probl�me
    int32_t aConvultionResultTab_Finish[kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE];

    int32_t aLinePattern[kLINE_PATTERN_SIZE] = kLINE_PATTERN
    ;
    int32_t aFinishPattern[kFINISH_PATTERN_SIZE] = kFINISH_PATTERN
    ;
    //Variable de convolution
    uint8_t n = 0;
    uint8_t k = 0;

    //Convolution recherche de la ligne
    //Attention il faut g�rer le d�passement du c�t� droit
    for (n = 0; n < (kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE); n++)
	{

	aConvultionResultTab[n] = 0;
	aConvultionResultTab_Finish[n] = 0;

	for (k = 0; k < kLINE_PATTERN_SIZE; k++)
	    {
	    int32_t aElem_n_k = 0;

	    //Test des condition initiale
	    if ((n - k) > (kLINE_SCAN_SIZE - 1)) //D�passement
		{
		//Alors on renvoie la derni�re valeur du tableau
		aElem_n_k = tab[kLINE_SCAN_SIZE - 1];
		}
	    else if ((n - k) < 0)
		{
		aElem_n_k = kINIT_VAL;
		}
	    else
		{
		aElem_n_k = tab[n - k];
		}

	    // y[n] = Somme[h[k]*x[n*k]]
	    aConvultionResultTab[n] += (int32_t) (aLinePattern[k] * aElem_n_k);
	    aConvultionResultTab_Finish[n] += (int32_t) (aFinishPattern[k]
		    * aElem_n_k);
	    }
	}

    //On parcourt le tableau resultant de la convolution et on recherche un max
    uint8_t aMaxIndex = tMax_32(aConvultionResultTab, kLINE_SCAN_SIZE);

    //On test si ca passe le seuil
    if (aConvultionResultTab[aMaxIndex] >= kSEUIL_LIGNE)
	{
	*isLineFound = true;
	*thePosition = aMaxIndex - kOFFSET_LIGNE;
	}
    else
	{
	*isLineFound = false;
	//On ne change pas la position
	}

    //On parcourt le tableau resultant de la convolution et on recherche un max
    aMaxIndex = tMax_32(aConvultionResultTab_Finish, kLINE_SCAN_SIZE);

    //On test si ca passe le seuil
    if (aConvultionResultTab_Finish[aMaxIndex] >= kSEUIL_LIGNE_ARR)
	{
	*isStartStopFound = true;
	//*thePosition = aMaxIndex - kOFFSET_LIGNE_ARR;
	}
    else
	{
	*isStartStopFound = false;
	//On ne change pas la position
	}

    }

//--------------------------------------------------------
// trouve tous les flanc pr�sents dans le champ de vision
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
