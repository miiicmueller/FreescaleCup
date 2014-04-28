/*
 * parameters.h
 *
 *  Created on: Jan 28, 2014
 *      Author: cyrille.savy
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

//parametres d'indication de compilation
//#define MONITORING_ENABLED

//temps de cycle d'execution des gestionnaires
#define kGEST_CYCLETIME 2 //en ms
//

//parametres concernant la vitesse
#define kSPEED_MAX 100.0
#define kSPEED_MIN_1 30.0
#define kSPEED_MIN_2 57.0

//parametres du regulateur de direction
#define kCONSIGNEPROCHECORRECTION (float)1.1 //facteur de correction appliqué à la consigne de la ligne proche par rapport à l'erreur de la ligne lointaine
#define kREGQUAD_BRAQUAGEMAX (float)0.60 //valeur a laquelle les roues sont tournees a fond
#define kREGQUAD_ERREURMAX (float)64.0 //erreur a laquelle le braquage est max
#define kCONSIGNE_MIN_VIRAGE_1 10
#define kCONSIGNE_MIN_VIRAGE_2 20
#define kCONSIGNE_MIN_VIRAGE_3 30
//
//parametres concernant le freinage
#define kNMOY_DERIVEE 3
#define kBRAKE_COEFF -((float)kSPEED_MAX-(float)kSPEED_MIN)/(kREGQUAD_ERREURMAX/(kNMOY_DERIVEE+2))
#define kSEUIL_DERIVEE_VIRAGE 3.5
#define kTEMPS_FREINAGE_1 60
#define kTEMPS_FREINAGE_2 100
#define kSEUIL_CONSIGNE_FREINAGE 20

//parametre du differentiel
#define kCOEFF_DIFFERENTIAL (float)-0.75

//
//parametres de l'eclairement des LEDs et du temps d'exposition des cameras
#define kLEDSPWM (float)0.8 //de -1 a 1 ------- (0% a 100%) ----- 0 : 50% de duty cycle
#define kCAMEXPTIME (float)-0.6 //de -1 a 1 ---- (0ms a 10ms) ---- 0 : temps d'exposition de 5ms
//
//parametre du temps de perte des deux lignes avant de s'arreter
#define kTIMENOLINE 2000 //en ms
//
#endif /* PARAMETERS_H_ */
