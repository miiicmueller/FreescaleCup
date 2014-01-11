//*****************************************************************************
//Nom du fichier : tPID.c
//Auteur et Date : SAVY Cyrille 31.10.2013
//But : fonction implementant un regulateur PID, avec la structure permettant
//	de stocker les donnees du regulateur
//*****************************************************************************

//fichiers utilises
#include "Tools/tPID.h"

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

    // Fenêtre glissante pour rendre le régulateur I plus stable
    thePIDStruct->theIntegratorError[thePIDStruct->posFiltre] = nouvelleErreur;
    if (thePIDStruct->posFiltre < WINDOWPID_SIZE - 1)
	{
	thePIDStruct->posFiltre++;
	}
    else
	{
	thePIDStruct->posFiltre = 0;
	}

    //integration des erreurs successives
    thePIDStruct->sommeErreurs = 0;
    for (int i = 0; i < WINDOWPID_SIZE; i++)
	{
	// On somme les "WINDOW_SIZE" dernières valeurs
	thePIDStruct->sommeErreurs += thePIDStruct->theIntegratorError[i];
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
    thePIDStruct->commande += (thePIDStruct->ki * thePIDStruct->sommeErreurs); //partie intégrale
    thePIDStruct->commande += (thePIDStruct->kd
	    * (nouvelleErreur - thePIDStruct->erreurPrecedente)); //partie dérivée

    //mettre a jour l'erreur precedente pour la partie derivee
    thePIDStruct->erreurPrecedente = nouvelleErreur;
    }

/**
 * Nouvel algorithme de calcul PID. Calculé mathématiquement !
 */
void tPID_v2(tPIDStruct* thePIDStruct, int16_t theMesure)
    {
    //calcul de la nouvelle erreur
    //la mesure doit etre convertie dans la grandeur de la consigne
    thePIDStruct->thePastError[0] = (float) ((thePIDStruct->consigne)
	    - theMesure) * thePIDStruct->coeffNormalisation;

    float a = thePIDStruct->kp * (1.0 + thePIDStruct->kd);
    float b = thePIDStruct->kp
	    * ((1.0 / thePIDStruct->ki) - 1.0 - 2.0 * thePIDStruct->kd);
    float c = thePIDStruct->kd;

    // Algorithme calculé : u[k] = u[k-1] + a *  e[k] + b* e[k-1] + c* e[k-2]

    thePIDStruct->commande = thePIDStruct->commande
	    + a * thePIDStruct->thePastError[0]
	    + b * thePIDStruct->thePastError[1]
	    + c * thePIDStruct->thePastError[2];

    //Décalage du tableau d'erreur pour la prochaine boucle
    thePIDStruct->thePastError[2] = thePIDStruct->thePastError[1];
    thePIDStruct->thePastError[1] = thePIDStruct->thePastError[0];

    }
