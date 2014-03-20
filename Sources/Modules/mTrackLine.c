//*****************************************************************************
//Nom du fichier : mTrackLine.c
//Auteur et Date : SAVY Cyrille 25.10.2013
//But : fonctions permettant de trouver la ligne sur la piste
//*****************************************************************************

//fichiers utilises
#include "Modules\mTrackLine.h"
#include "Tools\Tools.h"

//definitions de constantes pour le calcul par DERIVEE
#define kScaleTab	100	//echelle a laquelle on veut mettre le tableau
#define kThresholdTab	35	//seuil a partir duquel un flanc est considere comme significatif
#define kEcartMin	3*kThresholdTab	//ecart minimal entre le min et le max de la derivee pour que l'on considere que quelquechose se trouve dans le champ de vision
#define kSizeEdgesTab	15	//nombre de flancs maximum que l'on peut trouver
#define kLengthLineMin	3	//longueur min de la ligne a trouver (en pixels)
#define kLengthLineMax	30	//longueur max de la ligne a trouver (en pixels)
#define kLengthStartStopMin	3	//longueur min d'un morceau de la ligne de départ/arrivée a trouver (en pixels)
//Attention aux contours, la largeur augmente
#define kLengthStartStopMax	30	//longueur max d'un morceau de la ligne de départ/arrivée a trouver (en pixels)
//--

//definitions de constantes pour le calcul par CORRELATION
#define kINIT_VAL 304
#define kLINE_SCAN_SIZE 128

#define kLINE_PATTERN_SIZE 35
#define kLINE_PATTERN {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -4, -4, -4, -4, -4, -4, -4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} 

#define kFINISH_PATTERN_SIZE 35
#define kFINISH_PATTERN {-6, -6, -6, -6, -6, -6, -6, 9, 9, 9, 9, 9, 9, 9, -6, -6, -6, -6, -6, -6, -6, 9, 9, 9, 9, 9, 9, 9, -6, -6, -6, -6, -6, -6, -6} 

#define kSEUIL_LIGNE 		8000
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

static uint16_t mTrackLine_FindEdges(int16_t* tab, uint16_t sizeTab, mTrackLineEdges* theEdgesTab);

//definition des fonctions
//--------------------------------------------------------
// trouve la position de la ligne sur l'image de la camera
// parametre de retour	: void
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//		thePosition	: position de la ligne (en pixels), de 0 a 127
//		isLineFound	: true si la ligne est presente, false sinon
//		isStartStopFound: true si la ligne est presente, false sinon
//		oldLinePosition	: ancienne position de la ligne (utilise si l'algorithme detecte plusieurs lignes)
//--------------------------------------------------------
void mTrackLine_FindLine(int16_t* tab, uint16_t size, int16_t* thePosition, bool* isLineFound, bool* isStartStopFound,
	int16_t oldLinePosition)
    {
    mTrackLineEdges theEdges[kSizeEdgesTab];
    uint16_t nbOfEdges;

    uint8_t nbOfLines = 0;
    uint8_t linesLocationTab[10];
    uint8_t linesLengthTab[10];

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
	    for (uint8_t i = 0; i < (nbOfEdges - 1); i++)
		{
		if ((theEdges[i].edgeType == falling) && (theEdges[i + 1].edgeType == rising))
		    {
		    if (((theEdges[i + 1].location - theEdges[i].location) > kLengthLineMin)
			    && ((theEdges[i + 1].location - theEdges[i].location) < kLengthLineMax))
			{
//			*isLineFound = true;
//			*thePosition = (theEdges[i + 1].location
//				+ theEdges[i].location) / 2;
			linesLocationTab[nbOfLines] = (theEdges[i + 1].location + theEdges[i].location) / 2;
			linesLengthTab[nbOfLines] = theEdges[i + 1].location - theEdges[i].location;
			nbOfLines++;
			}
		    }
		}
	    }

	//on cherche la meilleure des lignes potentielles trouvees
	//la plus proche de l'ancienne ligne (un grand saut ne doit pas arriver normalement)
	uint8_t ecartLigneCourante;
	uint8_t ecartLigneOptimale = 128;
	for (uint8_t i = 0; i < nbOfLines; i++)
	    {
	    ecartLigneCourante = tAbs(oldLinePosition - linesLocationTab[i]);
	    if (ecartLigneCourante < ecartLigneOptimale)
		{
		*isLineFound = true;
		*thePosition = linesLocationTab[i];
		ecartLigneOptimale = ecartLigneCourante;
		}
	    }

	if (*isLineFound == false)
	    {
	    *isLineFound = false;
	    }

	//pattern pour la ligne de depart/arrivee
	//on doit avoir au moins quatre flancs
	if (nbOfEdges >= 4)
	    {
	    for (uint16_t i = 0; i < (nbOfEdges - 3); i++)
		if ((theEdges[i].edgeType == rising) && (theEdges[i + 1].edgeType == falling)
			&& (theEdges[i + 2].edgeType == rising) && (theEdges[i + 3].edgeType == falling))
		    {
		    if (((theEdges[i + 1].location - theEdges[i].location) > kLengthStartStopMin)
			    && ((theEdges[i + 1].location - theEdges[i].location) < kLengthStartStopMax)
			    && ((theEdges[i + 2].location - theEdges[i + 1].location) > kLengthLineMin)
			    && ((theEdges[i + 2].location - theEdges[i + 1].location) < kLengthLineMax)
			    && ((theEdges[i + 3].location - theEdges[i + 2].location) > kLengthStartStopMin)
			    && ((theEdges[i + 3].location - theEdges[i + 2].location) < kLengthStartStopMax))
			{
			*isStartStopFound = true;
			}
		    }
	    }
	}

    return;
    }

//--------------------------------------------------------
// trouve la position de la ligne sur l'image de la camera et détecte si 
// présence de la ligne d'arrivée.
// parametre de retour	: bool : true si la ligne est presente, false sinon
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//
// 	Ici nous utilisons le principe de la correlation pour retrouver
//	l'endroit qui ressemble le plus à notre pattern. (Convolution)
//
//
//--------------------------------------------------------
void mTrackLine_Correlation(int16_t* tabNear, int16_t* tabFar, uint16_t size, int16_t* thePositionNear,
	int16_t* thePositionFar, bool* isLineNearFound, bool* isLineFarFound, bool* isStartStopFound)
    {
    // Variable nécessaire à la convolution
    int32_t aLineNearResult[kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE]; // ATTENTION : kLINE_PATTERN_SIZE >= kFINISH_PATTERN_SIZE, sinon il y aura des problemes
    int32_t aLineFarResult[kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE]; // ATTENTION : kLINE_PATTERN_SIZE >= kFINISH_PATTERN_SIZE, sinon il y aura des problemes
    int32_t aLineFinishResult[kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE];

    int32_t aLinePattern[kLINE_PATTERN_SIZE] = kLINE_PATTERN
    ;
    int32_t aFinishPattern[kFINISH_PATTERN_SIZE] = kFINISH_PATTERN
    ;

    //Variable de convolution
    uint8_t n = 0;
    uint8_t k = 0;

    tRescale(tabNear, size, 1000, 0); //on fait le rescale seulement si un ecart minimal est atteint
    tRescale(tabFar, size, 1000, 0); //on fait le rescale seulement si un ecart minimal est atteint

    //Convolution recherche de la ligne
    //Attention il faut gérer le dépassement du cote droit
    for (n = 0; n < (kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE); n++)
	{

	aLineNearResult[n] = 0;
	aLineFarResult[n] = 0;
	aLineFinishResult[n] = 0;

	for (k = 0; k < kLINE_PATTERN_SIZE; k++)
	    {
	    int32_t aElem_n_k_Near = 0;
	    int32_t aElem_n_k_Far = 0;

	    //Test des condition initiale
	    if ((n - k) > (kLINE_SCAN_SIZE - 1)) //Dépassement
		{
		//Alors on renvoie la dernière valeur du tableau
		aElem_n_k_Near = tabNear[kLINE_SCAN_SIZE - 1];
		aElem_n_k_Far = tabFar[kLINE_SCAN_SIZE - 1];
		}
	    else if ((n - k) < 0)
		{
		aElem_n_k_Near = kINIT_VAL;
		aElem_n_k_Far = kINIT_VAL;
		}
	    else
		{
		aElem_n_k_Near = tabNear[n - k];
		aElem_n_k_Far = tabFar[n - k];
		}

	    // y[n] = Somme[h[k]*x[n*k]]
	    aLineNearResult[n] += (int32_t) (aLinePattern[k] * aElem_n_k_Near);
	    aLineFarResult[n] += (int32_t) (aLinePattern[k] * aElem_n_k_Far);
	    aLineFinishResult[n] += (int32_t) (aFinishPattern[k] * aElem_n_k_Near);
	    }
	}

    //On parcourt le tableau resultant de la convolution et on recherche un max
    uint8_t aMaxIndex = tMax_32(aLineNearResult, kLINE_SCAN_SIZE);
    //On teste si ca passe le seuil
    if (aLineNearResult[aMaxIndex] >= kSEUIL_LIGNE)
	{
	*isLineNearFound = true;
	*thePositionNear = aMaxIndex - kOFFSET_LIGNE;
	}
    else
	{
	*isLineNearFound = false;
	//On ne change pas la position
	}

    //On parcourt le tableau resultant de la convolution et on recherche un max
    aMaxIndex = tMax_32(aLineFarResult, kLINE_SCAN_SIZE);
    //On teste si ca passe le seuil
    if (aLineFarResult[aMaxIndex] >= kSEUIL_LIGNE)
	{
	*isLineFarFound = true;
	*thePositionFar = aMaxIndex - kOFFSET_LIGNE;
	}
    else
	{
	*isLineFarFound = false;
	//On ne change pas la position
	}

    //On parcourt le tableau resultant de la convolution et on recherche un max
    aMaxIndex = tMax_32(aLineFinishResult, kLINE_SCAN_SIZE);
    //On test si ca passe le seuil
    if (aLineFinishResult[aMaxIndex] >= kSEUIL_LIGNE_ARR)
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
// trouve tous les flanc présents dans le champ de vision
// parametre de retour	: uint16_t : nombre d'objets edges qui ont ete trouves
// parametres : tab		: adresse du tableau a traiter
//		sizeTab		: longueur du tableau
//		theEdgesTab	: adresse du tableau qui contient les objets Edges
//--------------------------------------------------------
static uint16_t mTrackLine_FindEdges(int16_t* tab, uint16_t sizeTab, mTrackLineEdges* theEdgesTab)
    {
    uint16_t indexTab = 0;
    uint16_t indexEdgesTab = 0;

// dark object : un flanc descendant suivi d'un flanc montant
    while ((indexTab < sizeTab) && (indexEdgesTab < kSizeEdgesTab))
	{
	//on cherche un flanc descendant
	if (((tab[indexTab - 1] == 0) || (tab[indexTab - 1] == 1)) && (tab[indexTab] == -1))
	    {
	    theEdgesTab[indexEdgesTab].edgeType = falling;
	    theEdgesTab[indexEdgesTab].location = indexTab;
	    indexEdgesTab++;
	    }
	//on cherche un flanc montant
	if (((tab[indexTab - 1] == 0) || (tab[indexTab - 1] == -1)) && (tab[indexTab] == 1))
	    {
	    theEdgesTab[indexEdgesTab].edgeType = rising;
	    theEdgesTab[indexEdgesTab].location = indexTab;
	    indexEdgesTab++;
	    }
	indexTab++;
	}

    return indexEdgesTab;
    }
