//*****************************************************************************
//Nom du fichier : tPID.c
//Auteur et Date : SAVY Cyrille 31.10.2013
//But : fonction implementant un regulateur PID, avec la structure permettant
//	de stocker les donnees du regulateur
//*****************************************************************************

//fichiers utilises
#include "parameters.h"
#include "Tools/tRegulateurs.h"
#include "Tools/Tools.h"

#define kSEUIL_PENTE 8
#define kREGQUAD_ERREURMAX_2 175
#define kREGQUAD_ERREURMAX_3 73

#define kPENTE_1 kREGQUAD_BRAQUAGEMAX/kREGQUAD_ERREURMAX_2
#define kPENTE_2 (kREGQUAD_BRAQUAGEMAX-(kPENTE_1*kSEUIL_PENTE))/(kREGQUAD_ERREURMAX_3-kSEUIL_PENTE)

//--------------------------------------------------------
// regulateur PID
// parametre de retour	: int16_t : la valeur de la commande a assigner
// parametres : thePIDStruct	  : structure contenant toutes les 
//				    informations necessaires au regulateur (cf. tPID.h)
//--------------------------------------------------------
void tRegPID(tRegulateurPIDStruct* thePIDStruct, int16_t theMesure)
    {
    //calcul de la nouvelle erreur
    //la mesure doit etre convertie dans la grandeur de la consigne
    float nouvelleErreur = (float) ((thePIDStruct->consigne) - theMesure) * thePIDStruct->coeffNormalisation;

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

    //nouvelle consigne avec un regulateur PID
    thePIDStruct->commande = (thePIDStruct->kp * nouvelleErreur); //partie proportionelle
    thePIDStruct->commande += (thePIDStruct->ki * thePIDStruct->sommeErreurs); //partie intégrale
    thePIDStruct->commande += (thePIDStruct->kd * (nouvelleErreur - thePIDStruct->erreurPrecedente)); //partie dérivée

    //mettre a jour l'erreur precedente pour la partie derivee
    thePIDStruct->erreurPrecedente = nouvelleErreur;
    }

//--------------------------------------------------------
// regulateur proportionnel avec un tableau de ponderation exponentiel
// parametre de retour	: float : la valeur de la commande a assigner (-1 à 1)
// parametres : theExpStruct	  : structure contenant toutes les 
//				    informations necessaires au regulateur (cf. tPID.h)
// 	      : theMesure	  : derniere mesure
//--------------------------------------------------------
void tRegQuadratic(tRegulateurQuadStruct* theExpStruct, int16_t theMesure, bool isLineFarFound)
    {
    int16_t erreur = theExpStruct->consigne - theMesure;
    theExpStruct->commande = theExpStruct->coefficient * ((float) (erreur * erreur)) * tSign(erreur);
//    if (tAbs_float(theExpStruct->commande) > kREGQUAD_BRAQUAGEMAX)
//	{
//	if (theExpStruct->commande < 0)
//	    {
//	    theExpStruct->commande = kREGQUAD_BRAQUAGEMAX * (-1.0);
//	    }
//	else
//	    {
//	    theExpStruct->commande = kREGQUAD_BRAQUAGEMAX;
//	    }
//	}
//    if (tAbs(erreur) < kSEUIL_PENTE)
//	{
    theExpStruct->commande = erreur * kPENTE_1;
//	}

    if ((!isLineFarFound) && (tAbs(erreur) > kSEUIL_PENTE))
	{
	theExpStruct->commande = (erreur - tSign(erreur) * kSEUIL_PENTE) * kPENTE_2
		+ tSign(erreur) * (kPENTE_1 * kSEUIL_PENTE);
	}

    }
