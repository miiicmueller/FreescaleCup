#include "derivative.h" /* include peripheral declarations */
#include "TFC\TFC.h"
#include "Modules\mLeds.h"
#include "Modules\mTrackLine.h"
#include "Gestionnaires\gMbox.h"
#include "Gestionnaires\gInput.h"
#include "Gestionnaires\gCompute.h"
#include "Gestionnaires\gOutput.h"

int main(void)
    {
    uint32_t t, i = 0;
    bool initAutoMode = true;

    TFC_Init();

    for (;;)
	{
	//TFC_Task must be called in your main loop.  This keeps certain processing happy (I.E. Serial port queue check)
	TFC_Task();

	//This Demo program will look at the middle 2 switch to select one of 4 demo modes.
	//Let's look at the middle 2 switches
	switch ((TFC_GetDIP_Switch() >> 1) & 0x03)
	    {
	default:
	case 0:
	    if (initAutoMode)
		{
		gInput_Setup();
		gCompute_Setup();
		gOutput_Setup();
		}
	    else
		{
		//Notre magnifique programme
		gInput_Execute();
		gCompute_Execute();
		gOutput_Execute();
		}

	    break;

	case 1:

	    //Demo mode 1 will just move the servos with the on-board potentiometers
	    if (TFC_Ticker[0] >= 20)
		{
		TFC_Ticker[0] = 0; //reset the Ticker
		//Every 20 mSeconds, update the Servos
		TFC_SetServo(0, TFC_ReadPot(0));
		TFC_SetServo(1, TFC_ReadPot(1));
		}
	    //Let's put a pattern on the LEDs
	    if (TFC_Ticker[1] >= 125)
		{
		TFC_Ticker[1] = 0;
		t++;
		if (t > 4)
		    {
		    t = 0;
		    }
		TFC_SetBatteryLED_Level(t);
		}

	    TFC_SetMotorPWM(0, 0); //Make sure motors are off
	    TFC_HBRIDGE_DISABLE;

	    break;

	case 2:

	    //Demo Mode 2 will use the Pots to make the motors move
	    TFC_HBRIDGE_ENABLE;
	    TFC_SetMotorPWM(TFC_ReadPot(0), TFC_ReadPot(1));

	    //Let's put a pattern on the LEDs
	    if (TFC_Ticker[1] >= 125)
		{
		TFC_Ticker[1] = 0;
		t++;
		if (t > 4)
		    {
		    t = 0;
		    }
		TFC_SetBatteryLED_Level(t);
		}
	    break;

	case 3:

	    //Demo Mode 3 will be in Freescale Garage Mode.  It will beam data from the Camera to the 
	    //Labview Application

	    if (TFC_Ticker[1] > 100)
		{
		TFC_Ticker[1] = 0;
		// pour pouvoir ajuster le temps d'exposition du capteur CCD de 0 � 10ms (valeur par d�faut : 5ms)
		static uint32_t oldvalExposure = 5000;
		uint32_t valExposure = (uint32_t) (((TFC_ReadPot(0) + 1.0)
			* 5000.0) + 1.0);
		if (valExposure != oldvalExposure)
		    {
		    TFC_SetLineScanExposureTime(valExposure);
		    }
		oldvalExposure = valExposure;
		}

	    if (TFC_Ticker[0] > 100 && LineScanImageReady == 1)
		{
		// pour pouvoir ajuster la luminosite des LEDs
		mLeds_writeDyC(TFC_ReadPot(1));

		TFC_Ticker[0] = 0;
		LineScanImageReady = 0;

		//recherche de la ligne
		int16_t LineAnalyze[128];
		uint16_t positionLine;
		for (uint16_t i = 0; i < 128; i++)
		    {
		    LineAnalyze[i] = LineScanImage0[i];
		    }

		if (mTrackLine_FindLine(LineAnalyze, 128, &positionLine))
		    {
		    TFC_BAT_LED0_ON;
		    }
		else
		    {
		    TFC_BAT_LED0_OFF;
		    }

		TERMINAL_PRINTF("\r\n");
		TERMINAL_PRINTF("L:");

		for (i = 0; i < 128; i++)
		    {
		    TERMINAL_PRINTF("%d,", LineScanImage0[i]);
		    }

		for (i = 0; i < 128; i++)
		    {
		    TERMINAL_PRINTF("%d", LineAnalyze[i]);
		    if (i == 127)
			TERMINAL_PRINTF("\r\n", LineAnalyze[i]);
		    else
			TERMINAL_PRINTF(",", LineAnalyze[i]);
		    }
		}

	    TFC_SetMotorPWM(0, 0); //Make sure motors are off
	    TFC_HBRIDGE_DISABLE;

	    break;
	    }
	}

    return 0;
    }
