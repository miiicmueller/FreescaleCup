//*****************************************************************************
//Nom du fichier : Tools.c
//Auteur et Date : SAVY Cyrille 25.10.2013
//But : petites fonctions d'analyse pour le signal mesure par les camera
//*****************************************************************************

//void tDerivative(int16_t* tab, uint16_t size);
//void tSuppressDC(int16_t* tab, uint16_t size);
//void tRescale(int16_t* tab, uint16_t size, uint16_t theScale);
//void tThreshold(int16_t* tab, uint16_t size, uint16_t Threshold);
//uint16_t tMin(int16_t* tab, uint16_t size);
//int8_t tMax(int16_t* tab, uint16_t size);
//int16_t tMean(int16_t* tab, uint16_t size);
//void tAbs(int16_t x);

#ifndef TOOLS_H_
#define TOOLS_H_

//fichiers utilises
#include "TFC\TFC.h"
#include "arm_math.h"

//--------------------------------------------------------
//calcule la derivee d'un tableau (x' = dx/dy)
// parametre de retour	: void
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
void tDerivative(int16_t* tab, uint16_t size);

//--------------------------------------------------------
// supprime la composante continue du signal
// parametre de retour	: void
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
void tSuppressDC(int16_t* tab, uint16_t size);

//--------------------------------------------------------
// etend le signal afin que sa valeur max vale theScale
// parametre de retour	: void
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//		theScale: valeur max (absolue) que le tableau doit prendre
//		aEcartMin: ecart minimal pour avoir le droit de faire le rescale
//--------------------------------------------------------
void tRescale(int16_t* tab, uint16_t size, uint16_t theScale, uint16_t aEcartMin);

//--------------------------------------------------------
// etablit un seuil : si la valeur du tableau lui est superieur il prendra 1, sinon 0
// parametre de retour	: void
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//		Threshold: valeur du seuil
//--------------------------------------------------------
void tThreshold(int16_t* tab, uint16_t size, uint16_t Threshold);

//--------------------------------------------------------
//renvoie l'indice de la valeur min du signal
// parametre de retour	: indice de la valeur min
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
uint8_t tMin(int16_t* tab, uint16_t size);
uint8_t tMin_q15(q15_t* tab, uint16_t size);

//--------------------------------------------------------
//renvoie l'indice de la valeur max du signal
// parametre de retour	: indice de la valeur max
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
uint8_t tMax(int16_t* tab, uint16_t size);
uint8_t tMax_q15(q15_t* tab, uint16_t size);
uint8_t tMax_f(float* tab, uint16_t size);
uint8_t tMax_32(int32_t* tab, uint16_t size);
void tMaxMin_3Tab(q15_t* tab1, uint8_t* tabIndex1, q15_t* tab2, uint8_t* tabIndex2, uint16_t size);

//--------------------------------------------------------
// renvoie la valeur de la moyenne du tableau
// parametre de retour	: valeur moyenne
// parametres : tab	: adresse du tableau a traiter
//		size	: longueur du tableau
//--------------------------------------------------------
int16_t tMean(int16_t* tab, uint16_t size);

//--------------------------------------------------------
// renvoie la valeur absolue du parametre d'entree
// parametre de retour	: valeur absolue de x
// parametres : x	: valeur d'entree
//--------------------------------------------------------
int16_t tAbs(int16_t x);
float tAbs_float(float x);

//--------------------------------------------------------
// renvoie le signe de x (-1 ou 1)
// parametre de retour	: -1 ou 1
// parametres : x	: valeur d'entree
//--------------------------------------------------------
int16_t tSign(int16_t x);

//--------------------------------------------------------
// renvoie la valeur mediane du tableau passe en entree
// parametre de retour	: valeur m�diane
// parametres : *aTab	: tableau a filtrer
// 	      : aSize	: longueur du tableau
//--------------------------------------------------------
uint32_t median_filter_n(uint32_t *aTab, char aSize);

#endif /* TOOLS_H_ */
