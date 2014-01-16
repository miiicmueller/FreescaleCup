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

    thePIDStruct->thePastError[0] = nouvelleErreur;

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

    //Décalage du tableau d'erreur pour la prochaine boucle
    thePIDStruct->thePastError[2] = thePIDStruct->thePastError[1];
    thePIDStruct->thePastError[1] = thePIDStruct->thePastError[0];
    }

/**
 * Nouvel algorithme de calcul PID. Calculé mathématiquement !
 */
void tPID_v2(tPIDStruct* thePIDStruct, int16_t theMesure)
    {
    static float a = (float) 0.0;
    static float b = (float) 0.0;
    static float c = (float) 0.0;

    //calcul de la nouvelle erreur
    //la mesure doit etre convertie dans la grandeur de la consigne
    thePIDStruct->thePastError[0] = (float) ((thePIDStruct->consigne)
	    - theMesure) * thePIDStruct->coeffNormalisation;

    //On augmente le gain kp si la ligne se trouve au bord de la caméra
    if (tAbs_float(thePIDStruct->thePastError[0]) > 0.8)
	{
	thePIDStruct->kp *= 2.0;
	}
    //On diminue la correction si l'erreur est petite
    if (tAbs_float(thePIDStruct->thePastError[0]) < 0.2)
	{
	//thePIDStruct->kp *= 0.5;
	}

    a = (float) thePIDStruct->kp * (1.0 + thePIDStruct->kd);
    b = (float) thePIDStruct->kp
	    * ((0.002 / thePIDStruct->ki) - 1.0 - 2.0 * thePIDStruct->kd);
    c = (float) thePIDStruct->kd;

    // Algorithme calculé : u[k] = u[k-1] + a *  e[k] + b* e[k-1] + c* e[k-2]

    thePIDStruct->commande = thePIDStruct->commande
	    + a * thePIDStruct->thePastError[0]
	    + b * thePIDStruct->thePastError[1]
	    + c * thePIDStruct->thePastError[2];

    if (thePIDStruct->commande >= 1.0)
	{
	thePIDStruct->commande = 1.0;
	}
    else if (thePIDStruct->commande <= -1.0)
	{
	thePIDStruct->commande = -1.0;
	}

    //Décalage du tableau d'erreur pour la prochaine boucle
    thePIDStruct->thePastError[2] = thePIDStruct->thePastError[1];
    thePIDStruct->thePastError[1] = thePIDStruct->thePastError[0];

    }
