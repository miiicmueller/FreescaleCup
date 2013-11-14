#include "TFC\TFC.h"
#include "Modules/mMotor.h"

void TFC_Init()
    {
    TFC_InitClock();
    TFC_InitSysTick();
    TFC_InitGPIO();
    TFC_InitServos();
    mMotor_mSetup();
    TFC_InitADCs();
    TFC_InitLineScanCamera();
    TFC_InitTerminal();
    TFC_InitUARTs();
    TFC_HBRIDGE_DISABLE;
    TFC_SetMotorPWM(0, 0);

    //Leds
    mLeds_Setup();

    //Gestionnaires
    gXBEE_Setup();

    }

void TFC_Task()
    {
#if defined(TERMINAL_USE_SDA_SERIAL)
    if ((TFC_Ticker[0] >= 60))
	{
	TFC_UART_Process();
	}
#endif

    TFC_ProcessTerminal();
    }
