//*****************************************************************************
//Nom du fichier : Tools.c
//Auteur et Date : SAVY Cyrille 25.10.2013
//But : petites fonctions d'analyse pour le signal mesure par les camera
//*****************************************************************************

#include "Tools/Tools.h"

//--------------------------------------------------------
//calcule la derivee d'un tableau (x' = dx/dy)
// parametre de retour	: void
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
void tDerivative(int16_t* tab, uint16_t size)
    {
    for (int16_t i = 0; i < (size - 1); i++)
	{
	tab[i] = tab[i + 1] - tab[i];
	}
    tab[size - 1] = tab[size - 2];
    }

//--------------------------------------------------------
// supprime la composante continue du signal
// parametre de retour	: void
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
void tSuppressDC(int16_t* tab, uint16_t size)
    {
    int16_t theMean = tMean(tab, size);

    for (uint16_t i = 0; i < size; i++)
	{
	tab[i] -= theMean;
	}
    }

//--------------------------------------------------------
// etend le signal afin que sa valeur max vale theScale
// parametre de retour	: void
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//		theScale: valeur max (absolue) que le tableau doit prendre
//		aEcartMin: ecart minimal pour avoir le droit de faire le rescale
//--------------------------------------------------------
void tRescale(int16_t* tab, uint16_t size, uint16_t theScale, uint16_t aEcartMin)
    {
    float theFactor;
    int16_t oldMax = tab[tMax(tab, size)];
    int16_t oldMin = tab[tMin(tab, size)];

    //on le fait seulement si l'ecart min est garanti
    if ((oldMax - oldMin) > aEcartMin)
	{
	oldMax = tAbs(oldMax);
	oldMin = tAbs(oldMin);

	if (oldMax > oldMin)
	    {
	    theFactor = (float) theScale / (float) oldMax;
	    }
	else
	    {
	    theFactor = (float) theScale / (float) oldMin;
	    }

	for (uint16_t i = 0; i < size; i++)
	    {
	    tab[i] = (int16_t) ((float) tab[i] * theFactor);
	    }
	}
    }

//--------------------------------------------------------
// etablit un seuil : si la valeur du tableau lui est superieur il prendra 1, sinon 0
// parametre de retour	: void
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//		Threshold: valeur du seuil
//--------------------------------------------------------
void tThreshold(int16_t* tab, uint16_t size, uint16_t Threshold)
    {
    for (uint16_t i = 0; i < size; i++)
	{
	if (tAbs(tab[i]) < Threshold)
	    {
	    tab[i] = 0;
	    }
	else
	    {
	    if (tab[i] < 0)
		{
		tab[i] = -1;
		}
	    else
		{
		tab[i] = 1;
		}
	    }
	}
    }

//--------------------------------------------------------
//renvoie l'indice de la valeur min du signal
// parametre de retour	: indice de la valeur min
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
uint8_t tMin(int16_t* tab, uint16_t size)
    {
    uint8_t indexMin = 0;

    for (uint8_t i = 1; i < size; i++)
	{
	if (tab[i] < tab[indexMin])
	    {
	    indexMin = i;
	    }
	}
    return indexMin;
    }

//--------------------------------------------------------
//renvoie l'indice de la valeur min du signal
// parametre de retour	: indice de la valeur min
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
uint8_t tMin_q15(q15_t* tab, uint16_t size)
    {
    uint8_t indexMin = 0;

    for (uint8_t i = 1; i < size; i++)
	{
	if (tab[i] < tab[indexMin])
	    {
	    indexMin = i;
	    }
	}
    return indexMin;
    }

//--------------------------------------------------------
//renvoie l'indice de la valeur max du signal
// parametre de retour	: indice de la valeur max
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
uint8_t tMax(int16_t* tab, uint16_t size)
    {
    uint8_t indexMax = 0;

    for (uint8_t i = 1; i < size; i++)
	{
	if (tab[i] > tab[indexMax])
	    {
	    indexMax = i;
	    }
	}
    return indexMax;
    }

//--------------------------------------------------------
//renvoie l'indice de la valeur max du signal
// parametre de retour	: indice de la valeur max
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
uint8_t tMax_32(int32_t* tab, uint16_t size)
    {
    uint8_t indexMax = 0;

    for (uint8_t i = 1; i < size; i++)
	{
	if (tab[i] > tab[indexMax])
	    {
	    indexMax = i;
	    }
	}
    return indexMax;
    }

//--------------------------------------------------------
//renvoie l'indice de la valeur max du signal
// parametre de retour	: indice de la valeur max
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
uint8_t tMax_f(float* tab, uint16_t size)
    {
    uint8_t indexMax = 0;

    for (uint8_t i = 1; i < size; i++)
	{
	if (tab[i] > tab[indexMax])
	    {
	    indexMax = i;
	    }
	}
    return indexMax;
    }

//--------------------------------------------------------
//renvoie l'indice de la valeur max du signal
// parametre de retour	: indice de la valeur max
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
uint8_t tMax_q15(q15_t* tab, uint16_t size)
    {
    uint8_t indexMax = 0;

    for (uint8_t i = 1; i < size; i++)
	{
	if (tab[i] > tab[indexMax])
	    {
	    indexMax = i;
	    }
	}
    return indexMax;
    }

//--------------------------------------------------------
//renvoie l'indice de la valeur max du signal
// parametre de retour	: indice de la valeur max
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
void tMaxMin_3Tab(q15_t* tab1, uint8_t* tabIndex1, q15_t* tab2, uint8_t* tabIndex2, q15_t* tab3, uint8_t* tabIndex3,
	uint16_t size)
    {
    *tabIndex1 = 0;
    *tabIndex2 = 0;
    *tabIndex3 = 0;

    for (uint8_t i = 1; i < size; i++)
	{
	if (tab1[i] > tab1[*tabIndex1])
	    {
	    *tabIndex1 = i;
	    }
	if (tab2[i] > tab2[*tabIndex2])
	    {
	    *tabIndex2 = i;
	    }
	//min pour la ligne d'arrivée
	if (tab3[i] < tab3[*tabIndex3])
	    {
	    *tabIndex3 = i;
	    }
	}
    return;
    }

//--------------------------------------------------------
// renvoie la valeur de la moyenne du tableau
// parametre de retour	: valeur moyenne
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
int16_t tMean(int16_t* tab, uint16_t size)
    {
    int16_t sum = 0;

    for (uint16_t i = 0; i < size; i++)
	{
	sum += tab[i];
	}
    return (int16_t) (sum / size);
    }

//--------------------------------------------------------
// renvoie la valeur absolue du parametre d'entree
// parametre de retour	: valeur absolue de x
// parametres : x	: valeur d'entree
//--------------------------------------------------------
int16_t tAbs(int16_t x)
    {
    if (x < 0)
	{
	x = (-x);
	}

    return x;
    }

float tAbs_float(float x)
    {
    if (x < 0.0)
	{
	x = (-x);
	}

    return x;
    }

//--------------------------------------------------------
// renvoie le signe de x (-1 ou 1)
// parametre de retour	: -1 ou 1
// parametres : x	: valeur d'entree
//--------------------------------------------------------
int16_t tSign(int16_t x)
    {
    if (x < 0)
	{
	return (-1);
	}
    else
	{
	return 1;
	}
    }

//--------------------------------------------------------
// renvoie la valeur mediane du tableau passe en entree
// parametre de retour	: valeur mediane
// parametres : *aTab	: tableau a filtrer
// 	      : aSize	: longueur du tableau
//--------------------------------------------------------
uint32_t median_filter_n(uint32_t *aTab, char aSize)
    {
    uint32_t aCpyTab[20];
    signed char i = 0, j = 0;
    uint32_t aTemp;

    //Copie du tableau
    for (i = 0; i < aSize; i++)
	{
	aCpyTab[i] = aTab[i];
	}

    //On trie le tableau
    for (i = 0; i < aSize; i++)
	{
	for (j = 0; j < (aSize - 1); j++)
	    {
	    if (aCpyTab[j] > aCpyTab[j + 1])
		{
		// On swap les deux
		aTemp = aCpyTab[j + 1];
		aCpyTab[j + 1] = aCpyTab[j];
		aCpyTab[j] = aTemp;
		}
	    }
	}

    //On prend la valeur du milieu
    return aCpyTab[(aSize - 1) / 2];
    }
