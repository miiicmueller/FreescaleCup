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
#define kLINE_SCAN_SIZE 128
#define kLINE_PATTERN_SIZE 33

static const q15_t kLINE_PATTERN_Q15_FAR[] =
    {
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    ~(17920) | 0x8000,
	    ~(17920) | 0x8000,
	    ~(17920) | 0x8000,
	    ~(17920) | 0x8000,
	    ~(17920) | 0x8000,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200,
	    3200
    };
static const q15_t kLINE_PATTERN_Q15_NEAR[] =
    {
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    ~(8192) | 0x8000,
	    ~(8192) | 0x8000,
	    ~(8192) | 0x8000,
	    ~(8192) | 0x8000,
	    ~(8192) | 0x8000,
	    ~(8192) | 0x8000,
	    ~(8192) | 0x8000,
	    ~(8192) | 0x8000,
	    ~(8192) | 0x8000,
	    ~(8192) | 0x8000,
	    ~(8192) | 0x8000,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096,
	    4096
    };

static const q15_t kLINE_PATTERN_Q15_FINISH[] =
    {
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    9728,
	    9728,
	    9728,
	    9728,
	    9728,
	    9728,
	    9728,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    9728,
	    9728,
	    9728,
	    9728,
	    9728,
	    9728,
	    9728,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000,
	    ~(7168) | 0x8000
    };

#define kSEUIL_LIGNEFAR		3800
#define kSEUIL_LIGNENEAR	10000
#define kSEUIL_LIGNE_ARR 	15000

#define kOFFSET_LIGNE 		kLINE_PATTERN_SIZE/2

void mTrackLine_CorrelationFFT(int16_t* tabNear, int16_t* tabFar, int16_t* thePositionNear, int16_t* thePositionFar,
	bool* isLineNearFound, bool* isLineFarFound, bool* isStartStopFound)
    {
    /* ----------------------------------------------------------------------
     * Declare I/O buffers
     * ------------------------------------------------------------------- */
    //pour la convolution
    const q15_t* aImgKernelTabNear = kLINE_PATTERN_Q15_NEAR;
    const q15_t* aImgKernelTabFar = kLINE_PATTERN_Q15_FAR;
    const q15_t* aImgKernelTabStop = kLINE_PATTERN_Q15_FINISH;

    q15_t aOutConvTabNear[kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE - 1]; /* Convolution output */
    q15_t aOutConvTabFar[kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE - 1]; /* Convolution output */
    q15_t aOutConvTabStop[kLINE_SCAN_SIZE + kLINE_PATTERN_SIZE - 1]; /* Convolution output */
    q15_t aPscratch1[(kLINE_SCAN_SIZE + 2 * kLINE_PATTERN_SIZE) - 2];
    q15_t aPscratch2[kLINE_PATTERN_SIZE];

    //pour la recherche du max
    uint8_t aMaxIndexNear;
    uint8_t aMaxIndexFar;
    uint8_t aMaxIndexStop;

    //on fait la convolution
    arm_conv_fast_opt_q15(tabNear, kLINE_SCAN_SIZE, aImgKernelTabNear, kLINE_PATTERN_SIZE, aOutConvTabNear, aPscratch1,
	    aPscratch2);
    arm_conv_fast_opt_q15(tabFar, kLINE_SCAN_SIZE, aImgKernelTabFar, kLINE_PATTERN_SIZE, aOutConvTabFar, aPscratch1,
	    aPscratch2);
    arm_conv_fast_opt_q15(tabNear, kLINE_SCAN_SIZE, aImgKernelTabStop, kLINE_PATTERN_SIZE, aOutConvTabStop, aPscratch1,
	    aPscratch2);

    //On parcourt le tableau resultant de la convolution et on recherche un max
    tMax_3Tab((aOutConvTabNear + kOFFSET_LIGNE), &aMaxIndexNear, (aOutConvTabFar + kOFFSET_LIGNE), &aMaxIndexFar,
	    (aOutConvTabStop + kOFFSET_LIGNE), &aMaxIndexStop, kLINE_SCAN_SIZE);

    //On teste si ca passe le seuil
    if (aOutConvTabNear[aMaxIndexNear + kOFFSET_LIGNE] >= kSEUIL_LIGNENEAR)
	{
	*isLineNearFound = true;
	*thePositionNear = aMaxIndexNear;
	}
    else
	{
	*isLineNearFound = false;
	}
    //On teste si ca passe le seuil
    if (aOutConvTabFar[aMaxIndexFar + kOFFSET_LIGNE] >= kSEUIL_LIGNEFAR)
	{
	*isLineFarFound = true;
	*thePositionFar = aMaxIndexFar;
	}
    else
	{
	*isLineFarFound = false;
	}
//    //On teste si ca passe le seuil
//    if (aOutConvTabStop[aMaxIndexStop + kOFFSET_LIGNE] >= kSEUIL_LIGNE_ARR)
//	{
//	*isStartStopFound = true;
//	}
//    else
//	{
    *isStartStopFound = false;
//	}

    return;
    }
