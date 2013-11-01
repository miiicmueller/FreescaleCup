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
//--------------------------------------------------------
void tRescale(int16_t* tab, uint16_t size, uint16_t theScale)
    {
    float theFactor;
    uint16_t oldMax = tAbs(tab[tMax(tab, size)]);
    uint16_t oldMin = tAbs(tab[tMin(tab, size)]);

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
uint16_t tMin(int16_t* tab, uint16_t size)
    {
    uint16_t indexMin = 0;

    for (uint16_t i = 1; i < size; i++)
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
int8_t tMax(int16_t* tab, uint16_t size)
    {
    uint16_t indexMax = 0;

    for (uint16_t i = 1; i < size; i++)
	{
	if (tab[i] > tab[indexMax])
	    {
	    indexMax = i;
	    }
	}
    return indexMax;
    }

//--------------------------------------------------------
// renvoie la valeur de la moyenne du tableau
// parametre de retour	: valeur moyenne
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
int16_t tMean(int16_t* tab, uint16_t size)
    {
    uint16_t sum = 0;

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
