//*****************************************************************************
//Nom du fichier : mTrackLine.c
//Auteur et Date : SAVY Cyrille 25.10.2013
//But : fonctions permettant de trouver la ligne sur la piste
//*****************************************************************************

//fichiers utilises
#include "Modules\mTrackLine.h"
#include "Tools\Tools.h"
#include "arm_math.h"

//definitions de constantes pour le calcul par CORRELATION
//#define kLINE_SCAN_SIZE 128
//#define kLINE_PATTERN_SIZE 33
//#define kSTART_STOP_MINS_GAP 22
#define kLINE_SCAN_SIZE 64
#define kLINE_PATTERN_SIZE 17
#define kSTART_STOP_MINS_GAP 11

#define kSEUIL_LIGNEFAR		1500
#define kSEUIL_LIGNENEAR	2250
#define kSEUIL_LIGNE_ARR 	1500

#define kOFFSET_LIGNE 		kLINE_PATTERN_SIZE/2

static const q15_t kLINE_PATTERN_Q15_NEAR[] =
    {
	    5120,
	    5120,
	    5120,
	    5120,
	    5120,
	    5120,
	    ~(12288) | 0x8000,
	    ~(12288) | 0x8000,
	    ~(12288) | 0x8000,
	    ~(12288) | 0x8000,
	    ~(12288) | 0x8000,
	    5120,
	    5120,
	    5120,
	    5120,
	    5120,
	    5120
    };

static const q15_t kLINE_PATTERN_Q15_FAR[] =
    {
	    3072,
	    3072,
	    3072,
	    3072,
	    3072,
	    3072,
	    3072,
	    ~(14336) | 0x8000,
	    ~(14336) | 0x8000,
	    ~(14336) | 0x8000,
	    3072,
	    3072,
	    3072,
	    3072,
	    3072,
	    3072,
	    3072,
    };

void mTrackLine_CorrelationFFT(int16_t* tabNear, int16_t* tabFar, int16_t* thePositionNear, int16_t* thePositionFar,
	bool* isLineNearFound, bool* isLineFarFound, bool* isStartStopFound)
    {
    /* ----------------------------------------------------------------------
     * Declare I/O buffers
     * ------------------------------------------------------------------- */
    //pour la convolution
    const q15_t* aImgKernelTabNear = kLINE_PATTERN_Q15_NEAR;
    const q15_t* aImgKernelTabFar = kLINE_PATTERN_Q15_FAR;

    q15_t aOutConvTabNear[kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE - 1]; /* Convolution output */
    q15_t aOutConvTabFar[kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE - 1]; /* Convolution output */
    q15_t aPscratch1[(kLINE_SCAN_SIZE + 2 * kLINE_PATTERN_SIZE) - 2];
    q15_t aPscratch2[kLINE_PATTERN_SIZE];

    //pour la recherche du max
    uint8_t aMaxIndexNear;
    uint8_t aMaxIndexFar;
    uint8_t aMaxIndexStop2 = 0;
    uint8_t aMaxIndexStop3 = 0;

    //on fait la convolution
    arm_conv_fast_opt_q15(tabNear, kLINE_SCAN_SIZE, aImgKernelTabNear, kLINE_PATTERN_SIZE, aOutConvTabNear, aPscratch1,
	    aPscratch2);
    arm_conv_fast_opt_q15(tabFar, kLINE_SCAN_SIZE, aImgKernelTabFar, kLINE_PATTERN_SIZE, aOutConvTabFar, aPscratch1,
	    aPscratch2);

//On parcourt le tableau resultant de la convolution et on recherche un max
    tMaxMin_3Tab((aOutConvTabNear + kOFFSET_LIGNE + 1), &aMaxIndexNear, (aOutConvTabFar + kOFFSET_LIGNE + 1),
	    &aMaxIndexFar, kLINE_SCAN_SIZE);
    aMaxIndexNear += kOFFSET_LIGNE + 1;
    aMaxIndexFar += kOFFSET_LIGNE + 1;

    //cherche deux maxs pour la ligne d'arrivée
    if (aMaxIndexNear >= kLINE_PATTERN_SIZE)
	{
	aMaxIndexStop3 = tMax_q15(&aOutConvTabNear[aMaxIndexNear - ((kSTART_STOP_MINS_GAP / 2) + kSTART_STOP_MINS_GAP)],
		kSTART_STOP_MINS_GAP);
	aMaxIndexStop3 += aMaxIndexNear - ((kSTART_STOP_MINS_GAP / 2) + kSTART_STOP_MINS_GAP);
	}
    if (aMaxIndexNear <= (kLINE_SCAN_SIZE - 2))
	{
	aMaxIndexStop2 = tMax_q15(&aOutConvTabNear[aMaxIndexNear + (kSTART_STOP_MINS_GAP / 2)], kSTART_STOP_MINS_GAP);
	aMaxIndexStop2 += aMaxIndexNear + (kSTART_STOP_MINS_GAP / 2);
	}

    //On teste si ca passe le seuil
    *isLineNearFound = false;
    if (aOutConvTabNear[aMaxIndexNear] >= kSEUIL_LIGNENEAR)
	{
	*isLineNearFound = true;
	*thePositionNear = (aMaxIndexNear - kOFFSET_LIGNE) * 2;
	}
    //On teste si ca passe le seuil
    *isLineFarFound = false;
    if (aOutConvTabFar[aMaxIndexFar] >= kSEUIL_LIGNEFAR)
	{
	*isLineFarFound = true;
	*thePositionFar = (aMaxIndexFar - kOFFSET_LIGNE) * 2;
	}
    //On teste si ca passe le seuil
    *isStartStopFound = false;
    if (aOutConvTabNear[aMaxIndexNear] >= kSEUIL_LIGNE_ARR)
	{
	if ((aMaxIndexStop2 > 0) && (aMaxIndexStop2 < 71) && (aOutConvTabNear[aMaxIndexStop2] >= kSEUIL_LIGNE_ARR)
		&& (aMaxIndexStop3 > 0) && (aMaxIndexStop3 < 71)
		&& (aOutConvTabNear[aMaxIndexStop3] >= kSEUIL_LIGNE_ARR))
	    {
	    *isStartStopFound = true;
	    }
	}

    return;
    }
