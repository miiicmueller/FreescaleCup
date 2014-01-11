//*****************************************************************************
//Nom du fichier : tPID.c
//Auteur et Date : SAVY Cyrille 31.10.2013
//But : fonction implementant un regulateur PID, avec la structure permettant
//	de stocker les donnees du regulateur
//*****************************************************************************

//fichiers utilises
#include "Tools/tPID.h"
#define LimiteIntegrale 2

#define WINDOW_SIZE  10 // ~100ms 
//--------------------------------------------------------
// regulateur PID
// parametre de retour	: int16_t : la valeur de la commande a assigner
// parametres : thePIDStruct	  : structure contenant toutes les 
//				    informations necessaires au regulateur (cf. tPID.h)
//--------------------------------------------------------
void tPID(tPIDStruct* thePIDStruct, int16_t theMesure)
    {
    //calcul de la nouvelle erreur
    //la mesure doit etre convertie dans la grandeur de la consigne
    float nouvelleErreur = (float) ((thePIDStruct->consigne) - theMesure)
	    * thePIDStruct->coeffNormalisation;

    // Fen�tre glissante pour rendre le r�gulateur I plus stable
    static uint8_t posFiltre = 0;
    static float theIntergratorError[WINDOW_SIZE];

    theIntergratorError[posFiltre] = nouvelleErreur;
    if (posFiltre < WINDOW_SIZE - 1)
	{
	posFiltre++;
	}
    else
	{
	posFiltre = 0;
	}

    //integration des erreurs successives
    thePIDStruct->sommeErreurs = 0;
    for (int i = 0; i < WINDOW_SIZE; i++)
	{
	// On somme les "WINDOW_SIZE" derni�res valeurs
	thePIDStruct->sommeErreurs += theIntergratorError[i];
	}

    /* if (thePIDStruct->sommeErreurs > LimiteIntegrale)
     {
     thePIDStruct->sommeErreurs = LimiteIntegrale;
     }
     else if (thePIDStruct->sommeErreurs < -LimiteIntegrale)
     {
     thePIDStruct->sommeErreurs = -LimiteIntegrale;
     }*/

    //nouvelle consigne avec un regulateur PID
    thePIDStruct->commande = (thePIDStruct->kp * nouvelleErreur); //partie proportionelle
    thePIDStruct->commande += (thePIDStruct->ki * thePIDStruct->sommeErreurs); //partie int�grale
    thePIDStruct->commande += (thePIDStruct->kd
	    * (nouvelleErreur - thePIDStruct->erreurPrecedente)); //partie d�riv�e

    //mettre a jour l'erreur precedente pour la partie derivee
    thePIDStruct->erreurPrecedente = nouvelleErreur;
    }

void tPID_v2(tPIDStruct* thePIDStruct, int16_t theMesure)
    {
    //calcul de la nouvelle erreur
    //la mesure doit etre convertie dans la grandeur de la consigne
    float nouvelleErreur = (float) ((thePIDStruct->consigne) - theMesure)
	    * thePIDStruct->coeffNormalisation;
    float a = ;
    float b = ;
    float c = ;
        
    
    // Algorithme calcul� : u[k] = u[k-1] + a *  e[k] + b* e[k-1] + c* e[k-2]
    
    thePIDStruct->commande = thePIDStruct->commande + 

    }
