// @author: Javier Villaverde Ramallo
// @Version: April 2018
// Based on PMW3360DM-T2QU Datasheet Version 1.50 | 26 Sep 2016
// Source: https://d3s5r33r268y59.cloudfront.net/datasheets/9604/2017-05-07-18-19-11/PMS0058-PMW3360DM-T2QU-DS-R1.50-26092016._20161202173741.pdf

#ifndef _PMW3360DM_T2QU_
#define _PMW3360DM_T2QU_

#include <Arduino.h>
#include <SPI.h>
#include "PMW3360DM_srom_0x04.h"

//Be sure to add the SROM file into this sketch via "Sketch->Add File"
extern const unsigned short firmware_length;
extern const unsigned char  firmware_data[];

#define DEBUG_SERIAL

// Registers
#define REG_Product_ID  				0x00
#define REG_Revision_ID 				0x01
#define REG_Motion  					0x02
#define REG_Delta_X_L 					0x03
#define REG_Delta_X_H 					0x04
#define REG_Delta_Y_L 					0x05
#define REG_Delta_Y_H 					0x06
#define REG_SQUAL 						0x07
#define REG_Raw_Data_Sum  				0x08
#define REG_Maximum_Raw_data  			0x09
#define REG_Minimum_Raw_data  			0x0A
#define REG_Shutter_Lower 				0x0B
#define REG_Shutter_Upper 				0x0C
#define REG_Control 					0x0D
#define REG_Config1 					0x0F
#define REG_Config2 					0x10
#define REG_Angle_Tune  				0x11
#define REG_Frame_Capture 				0x12
#define REG_SROM_Enable 				0x13
#define REG_Run_Downshift 				0x14
#define REG_Rest1_Rate_Lower  			0x15
#define REG_Rest1_Rate_Upper  			0x16
#define REG_Rest1_Downshift 			0x17
#define REG_Rest2_Rate_Lower  			0x18
#define REG_Rest2_Rate_Upper  			0x19
#define REG_Rest2_Downshift 			0x1A
#define REG_Rest3_Rate_Lower  			0x1B
#define REG_Rest3_Rate_Upper  			0x1C
#define REG_Observation 				0x24
#define REG_Data_Out_Lower  			0x25
#define REG_Data_Out_Upper  			0x26
#define REG_Raw_Data_Dump 				0x29
#define REG_SROM_ID 					0x2A
#define REG_Min_SQ_Run  				0x2B
#define REG_Raw_Data_Threshold  		0x2C
#define REG_Config5 					0x2F
#define REG_Power_Up_Reset  			0x3A
#define REG_Shutdown  					0x3B
#define REG_Inverse_Product_ID  		0x3F
#define REG_LiftCutoff_Tune3  			0x41
#define REG_Angle_Snap 	 				0x42
#define REG_LiftCutoff_Tune1  			0x4A
#define REG_Motion_Burst  				0x50
#define REG_LiftCutoff_Tune_Timeout 	0x58
#define REG_LiftCutoff_Tune_Min_Length  0x5A
#define REG_SROM_Load_Burst 			0x62
#define REG_Lift_Config 				0x63
#define REG_Raw_Data_Burst  			0x64
#define REG_LiftCutoff_Tune2  			0x65

class PMW3360DM_T2QU
{
public:

	PMW3360DM_T2QU ();
	bool begin (int ncsPin_ = 10);

	bool powerUp 		(void);
	void reset 			(int nRESET_PIN);
	void shutdown 		(void);
	bool SROM_download  (void);
	uint16_t SROM_CRC 	(void);
	uint8_t  SROM_ID 	(void);

	void motionBurstRead  (void);
	bool updatePointer    (void);
	void displayRegisters (void);

	int16_t convTwosComp (int16_t msbits, int16_t lsbits);

	uint8_t spi_read8  (uint8_t reg_addr) const;
	void    spi_write8 (uint8_t reg_addr, uint8_t data) const;

	int x () const;
	int y () const;
	void printMotionBurstData  () const;
	void printDefaultRegisters () const;
	

	enum MotionBurst 
	{
		Motion = 0, 
		Observation, 
        Delta_X_L, 
        Delta_X_H, 
        Delta_Y_L, 
        Delta_Y_H, 
        SQUAL, 
        Raw_Data_Sum, 
        Maximum_Raw_Data, 
        Minimum_Raw_Data, 
        Shutter_Upper, 
        Shutter_Lower
    };

    uint8_t motionBurstData[12];

	int ncsPin;
};

#endif // PMW3360DM_T2QU