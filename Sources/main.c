#include "derivative.h" /* include peripheral declarations */
#include "TFC\TFC.h"
#include "Modules\mLeds.h"
#include "Modules\mTrackLine.h"
#include "Modules/mMotor.h"
#include "Modules/hal_dev_mma8451.h"
#include "Gestionnaires\gMbox.h"
#include "Gestionnaires\gInput.h"
#include "Gestionnaires\gCompute.h"
#include "Gestionnaires\gOutput.h"
#include "Gestionnaires/gXBEE.h"
#include "Tools/angle_cal.h"

#define FREEDOM 

int main(void)
    {
    uint32_t t = 0, i = 0;
    bool autoMode = false; //flag indiquant si le mode automatique est en cours

    TFC_Init();

    PORTA_PCR4 = PORT_PCR_MUX(1);

    for (;;)
	{
	//TFC_Task must be called in your main loop.  This keeps certain processing happy (I.E. Serial port queue check)
	TFC_Task();

	//This Demo program will look at the middle 2 switch to select one of 4 demo modes.
	//Let's look at the middle 2 switches
	switch ((TFC_GetDIP_Switch()) & 0x03)
	    {
	default:
	case 0:
	    if (!autoMode)
		{
		gInput_Setup();
		gCompute_Setup();
		gOutput_Setup();
		gXBEE_Setup();
		autoMode = true;
		}
	    else
		{
		//Notre magnifique programme
		if ((TFC_Ticker[0] >= 10) && (LineScanImageReady == 1))
		    {
		   
		    TFC_Ticker[0] = 0;
		    LineScanImageReady = 0;

		    gInput_Execute();
		    gCompute_Execute();
		    gOutput_Execute();
	
		    }
		if (TFC_Ticker[1] >= 40)
		    {
		    TFC_Ticker[1] = 0;
		    gXBEE_Execute();
		    }
		}

	    break;

	case 1:

	    //Demo mode 1 will just move the servos with the on-board potentiometers
	    if (TFC_Ticker[0] >= 10)
		{
		TFC_Ticker[0] = 0; //reset the Ticker
		//Every 20 mSeconds, update the Servos
		float aCommand = TFC_ReadPot(0);
		TFC_SetServo(0, aCommand);

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
	    autoMode = false;
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
	    autoMode = false;
	    break;

	case 3:

	    //Demo Mode 3 will be in Freescale Garage Mode.  It will beam data from the Camera to the 
	    //Labview Application

	    if (TFC_Ticker[1] > 100)
		{
		TFC_Ticker[1] = 0;
		// pour pouvoir ajuster le temps d'exposition du capteur CCD de 0 à 10ms (valeur par défaut : 5ms)
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
		bool isLineFound;
		bool isStartStopFound;
		// pour pouvoir ajuster la luminosite des LEDs
		mLeds_writeDyC(TFC_ReadPot(1));

		TFC_Ticker[0] = 0;
		LineScanImageReady = 0;

		//recherche de la ligne
		int16_t LineAnalyze[128];
		int16_t positionLine;
		for (uint16_t i = 0; i < 128; i++)
		    {
		    LineAnalyze[i] = LineScanImage0[i];
		    }

		mTrackLine_FindLine(LineAnalyze, 128, &positionLine,
			&isLineFound, &isStartStopFound);
		if (isLineFound)
		    {
		    TFC_BAT_LED0_ON;
		    }
		else
		    {
		    TFC_BAT_LED0_OFF;
		    }
		if (isStartStopFound)
		    {
		    TFC_BAT_LED1_ON;
		    }
		else
		    {
		    TFC_BAT_LED1_OFF;
		    }

		TERMINAL_PRINTF("\r\n");
		TERMINAL_PRINTF("L:");

		TFC_Ticker[2] = 0;
		for (i = 0; i < 128; i++)
		    {
		    TERMINAL_PRINTF("%d", i);
		    //LineScanImage0[i]>>3);
		    // >>3 pour avoir de 0 à 512 (4096 / 8)
		    if (i == 127)
			TERMINAL_PRINTF("\n");
		    else
			TERMINAL_PRINTF(",");
		    while (TFC_Ticker[2] < 3000)
			;
		    TFC_Ticker[2] = 0; //pour éviter de saturer le xBee
		    }

//		for (i = 0; i < 128; i++)
//		    {
//		    TERMINAL_PRINTF("%d", LineAnalyze[i]);
//		    if (i == 127)
//			TERMINAL_PRINTF("\r\n");
//		    else
//			TERMINAL_PRINTF(",");
//		    }
		}

	    TFC_SetMotorPWM(0, 0); //Make sure motors are off
	    TFC_HBRIDGE_DISABLE;
	    autoMode = false;
	    break;
	    }
	}

    return 0;
    }

void accel_read(void)
    {
    if ((hal_dev_mma8451_read_reg(0x00) & 0xf) != 0)
	{
	gInputInterStruct.gAccelXYZ[0] = hal_dev_mma8451_read_reg(0x01) << 8;
	gInputInterStruct.gAccelXYZ[0] |= hal_dev_mma8451_read_reg(0x02);
	gInputInterStruct.gAccelXYZ[0] >>= 2;

	gInputInterStruct.gAccelXYZ[1] = hal_dev_mma8451_read_reg(0x03) << 8;
	gInputInterStruct.gAccelXYZ[1] |= hal_dev_mma8451_read_reg(0x04);
	gInputInterStruct.gAccelXYZ[1] >>= 2;

	gInputInterStruct.gAccelXYZ[2] = hal_dev_mma8451_read_reg(0x05) << 8;
	gInputInterStruct.gAccelXYZ[2] |= hal_dev_mma8451_read_reg(0x06);
	gInputInterStruct.gAccelXYZ[2] >>= 2;

	gInputInterStruct.gAccelResXYZ[0] = hal_dev_mma8451_read_reg(0x01) << 8;
	gInputInterStruct.gAccelResXYZ[0] |= hal_dev_mma8451_read_reg(0x02);
	gInputInterStruct.gAccelResXYZ[0] >>= 8;

	gInputInterStruct.gAccelResXYZ[1] = hal_dev_mma8451_read_reg(0x03) << 8;
	gInputInterStruct.gAccelResXYZ[1] |= hal_dev_mma8451_read_reg(0x04);
	gInputInterStruct.gAccelResXYZ[1] >>= 8;

	gInputInterStruct.gAccelResXYZ[2] = hal_dev_mma8451_read_reg(0x05) << 8;
	gInputInterStruct.gAccelResXYZ[2] |= hal_dev_mma8451_read_reg(0x06);
	gInputInterStruct.gAccelResXYZ[2] >>= 8;

	}
    }

void NMI_Handler()
    {
    __asm("bkpt");
    TFC_HBRIDGE_DISABLE;
    }
void HardFault_Handler()
    {
    __asm("bkpt");
    TFC_HBRIDGE_DISABLE;
    }
void SVC_Handler()
    {
    __asm("bkpt");
    TFC_HBRIDGE_DISABLE;
    }
