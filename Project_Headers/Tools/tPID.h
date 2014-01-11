//*****************************************************************************
//Nom du fichier : tPID.h
//Auteur et Date : SAVY Cyrille 31.10.2013
//But : fonction implementant un regulateur PID, avec la structure permettant
//	de stocker les donnees du regulateur
//*****************************************************************************

#ifndef TPID_H_
#define TPID_H_

#include "TFC/TFC.h"
#define WINDOWPID_SIZE 10

//fichiers utilises
typedef struct
    {
	float kp; //facteur proportionnel
	float ki; //facteur integral
	float kd; //facteur differentiel
	float sommeErreurs; //integrale des erreurs
	int16_t consigne; //valeur cible de la grandeur mesuree
	float erreurPrecedente; //derniere erreur
	float commande; //commande a appliquer a l'actionneur
	float coeffNormalisation; //coefficient pour normaliser la sortie du PID
	float theIntegratorError[WINDOWPID_SIZE];
	uint8_t posFiltre;
    } tPIDStruct;

//--------------------------------------------------------
// regulateur PID
// parametre de retour	: int16_t : la valeur de la commande a assigner
// parametres : thePIDStruct	  : structure contenant toutes les 
//				    informations necessaires au regulateur (cf. tPID.h)
//		theMesure	  : valeur mesuree
//--------------------------------------------------------
void tPID(tPIDStruct* thePIDStruct, int16_t theMesure);

#endif /* TPID_H_ */

