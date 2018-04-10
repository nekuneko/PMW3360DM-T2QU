// @author: Javier Villaverde Ramallo
// @Version: April 2018
// Based on PMW3360DM-T2QU Datasheet Version 1.50 | 26 Sep 2016
// Source: https://d3s5r33r268y59.cloudfront.net/datasheets/9604/2017-05-07-18-19-11/PMS0058-PMW3360DM-T2QU-DS-R1.50-26092016._20161202173741.pdf


#include "PMW3360DM_T2QU.h"

uint8_t PMW3360DM_T2QU::spi_read8 (uint8_t reg_addr) const
{
	uint8_t data = 0x00;

	digitalWrite(ncsPin, LOW); 		// com begin
	SPI.transfer(reg_addr & 0x7f); 	// send address of the register with MSBit = 0  to indicate it's a read
	delayMicroseconds(160); 		// tSRAD (=160us)
	data = SPI.transfer(0);			// read data
	delayMicroseconds(1); 			// tSCLK-NCS (=120ns) for read operation
	digitalWrite(ncsPin, HIGH); 	// com end
	delayMicroseconds(20); 			// tSRW/tSRR (=20us) minus tSCLK-NCS

	return data;
}

void PMW3360DM_T2QU::spi_write8 (uint8_t reg_addr, uint8_t data) const
{
	digitalWrite(ncsPin, LOW);		// com begin
	SPI.transfer(reg_addr | 0x80);	// send address of the register with MSBit = 1 to indicate it's a write
	SPI.transfer(data); 			// send data
	delayMicroseconds(35);			// tSCLK-NCS (=35us) for write operation
	digitalWrite(ncsPin, HIGH);		// com end
	delayMicroseconds(120);			// tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but it looks like a safe lower bound
}

PMW3360DM_T2QU::PMW3360DM_T2QU () {}

bool PMW3360DM_T2QU::begin (int ncsPin_)
{
	bool success = false;

	this->ncsPin = ncsPin_;
	pinMode(ncsPin, OUTPUT);
	digitalWrite(ncsPin, HIGH);

	SPI.begin();
	SPI.setDataMode(SPI_MODE3);
	SPI.setBitOrder(MSBFIRST);
	//SPI.setClockDivider(4);

	// Shutdown Mouse
	shutdown();

	// Perform startup & upoad the firmware
	success = powerUp();

	#ifdef DEBUG_SERIAL
	if (success)
		Serial.println("Optical Chip Initialized.");
	else
		Serial.println("Fail to download firmware.");
	#endif

	return success;
}

bool PMW3360DM_T2QU::updatePointer (void)
{
	bool success = false;

	// Motion register (0x02) allows the user to determine if motion has ocurred since last time it was read.
	// The procedure to read the motion registers (Delta_X_L, Delta_X_H, Delta_Y_L and Delta_Y_H) is as follows:
	// 1. Write any value to the Motion register
	// ok I done before, I think it means to configure register
	// 2. Read the Motion register. This will freeze the Delta_X_L, Delta_X_H, Delta_Y_L and Delta_Y_H register values.
	motionBurstData[Motion] = spi_read8(REG_Motion);
	// 3. If the MOT bit is set, Delta_X_L, Delta_X_H, Delta_Y_L and Delta_Y_H should be read in the given sequence
	//    to get the accumulated motion. Note: if these registers are not read before the motion register is read for the
	//    second time, the data in Delta_X_L, Delta_X_H, Delta_Y_L and Delta_Y_H will be lost.
	if (motionBurstData[Motion] >> 7) // MOT bit is [7], 1 is true
	{
		motionBurstData[Delta_X_L] = spi_read8(REG_Delta_X_L);
		motionBurstData[Delta_X_H] = spi_read8(REG_Delta_X_H);
		motionBurstData[Delta_Y_L] = spi_read8(REG_Delta_Y_L);
		motionBurstData[Delta_Y_H] = spi_read8(REG_Delta_Y_H);
		success = true;
	}
	// 4. To read a new set of motion data (Delta_X_L, Delta_X_H, Delta_Y_L and Delta_Y_H), repeat from Step 2.
	// 5. If any other register was read i.e. any other regiter besides Motion, Delta_X_L, Delta_X_H, Delta_Y_L and Delta_Y_H,
	//    then, to read a new set of motion data, repeat from Step 1 instead.

	return success;
}


void PMW3360DM_T2QU::displayRegisters (void)
{
	int oreg[7] = {REG_Product_ID, REG_Inverse_Product_ID, REG_SROM_ID, REG_Motion};
	char* oregname[] = {"Product_ID", "Inverse_Product_ID", "SROM_ID", "Motion"};
	uint8_t regVal;

	digitalWrite(ncsPin, LOW);		//com begin
	Serial.println("---");
	for (int i=0; i<=3; ++i)
	{
		SPI.transfer(oreg[i]);
		delay(1);
		regVal = SPI.transfer(0);
		Serial.print(String(oregname[i]) + " (0x" + String(oreg[i]) + "): 0x"); Serial.println(regVal, HEX);
		Serial.print("Binary: "); Serial.println(regVal, BIN);
		Serial.println("---");
		delay(1);
	}

	digitalWrite(ncsPin, HIGH);		// com end
}

int16_t PMW3360DM_T2QU::convTwosComp (int16_t msbits, int16_t lsbits)
{
	return (int16_t) ((msbits << 8) | lsbits);
}

int PMW3360DM_T2QU::x (void) const
{
//	Serial.println("Delta_X_L: " + String(Delta_X_L));
//	Serial.println("Delta_X_H: " + String(Delta_X_H));

	uint16_t val_x = motionBurstData[Delta_X_H] << 8;
	val_x |= motionBurstData[Delta_X_L];

	return (int16_t) val_x;
}

int PMW3360DM_T2QU::y (void) const 
{	
//	Serial.println("Delta_Y_L: " + String(Delta_Y_L));
//	Serial.println("Delta_Y_H: " + String(Delta_Y_H));

	uint16_t val_y = motionBurstData[Delta_Y_H] << 8;
	val_y |= motionBurstData[Delta_Y_L];
	return (int16_t) val_y;
}

void PMW3360DM_T2QU::motionBurstRead (void)
{
	// Burst Mode Operation is a special serial port operation mode which can may used to reduce the serial transaction time
	// for three predefined operations: motion read and SROM download and frame capture.
	// The speed improvement is achieved by continuous data clocking to or from multiple registers without the need
	// to specify the register address, and by not requiring the normal delay period between data bytes.

	// Motion read. Reading the Motion_Burst register activates this mode. 
	// PMW3360DM-T2QU chip will respond with the following motion burst report in order. Motion burst report:
	// BYTE[00] = Motion 			(0x02)
	// BYTE[01] = Observation 		(0x24)
	// BYTE[02] = Delta_X_L 		(0x03)
	// BYTE[03] = Delta_X_H 		(0x04)
	// BYTE[04] = Delta_Y_L 		(0x05)
	// BYTE[05] = Delta_Y_H 		(0x06)
	// BYTE[06] = SQUAL 			(0x07)
	// BYTE[07] = Raw_Data_Sum		(0x08)
	// BYTE[08] = Maximum_Raw_Data 	(0x09)
	// BYTE[09] = Minimum_Raw_Data 	(0x0A)
	// BYTE[10] = Shutter_Upper		(0x0B)
	// BYTE[11] = Shutter_Lower 	(0x0C)

	// After sending the register address, the microcontroller must wait fot tSRAD_MOTBR (=35us), and then begin reading data.
	// All data bits can be read with no delay between bytes by driving SCLK at the normal rate.
	// The data are latched into the output buffer after the last address bit is received.
	// After the burst transmission is complete, the microcontroller must raiser the NCS line for at least tBEXIT (=500ns) to terminate burst mode.
	// The serial port is not available for use until it is reset with NCS, even for a second burst transmission.

	// Procedure to start motion burst:
	// 1. Write any value to Motion_Burst register  (0x50).
	spi_write8(REG_Motion_Burst, 0x50);	// send any data for high-speed access of up to 12 registers bytes.
	// 2. Lower NCS
	digitalWrite(ncsPin, LOW);			// com begin
	// 3. Send Motion_Burst address (0x50).
	SPI.transfer(0x50);		 			// REVISAR -- | 0x80?
	// 4. Wait for tSRAD_MOTBR (=35us)
	delayMicroseconds(35);				// tSCLK-NCS (=35us) for write operation
	// 5.1 Start reading SPI Data continuously up to 12 bytes. 
	for (int i=0; i<12; ++i)
		motionBurstData[i] = SPI.transfer(0);
	// 5.2 Motion burst may be terminated by pulling NCS high for at least tBEXIT (=500ns)
	digitalWrite(ncsPin, HIGH);			// com end
	delayMicroseconds(1);				// 500ns ~= 1us
	// 6. To read new motion burst data, repeat from step 2.
	// 7. If a non-burst register read operation was executed; then, to read new burst data, start from step 1 instead.
	// Note: Motion burst data can be read from the Motion_Burst registers even in run o rest mode.
	// Flip 180º, default 0x00, 0º
	spi_write8(REG_Control, 0x60); //0110 0000b
}

void PMW3360DM_T2QU::printMotionBurstData () const
{
	Serial.println("--- Motion Burst Data ---");
	Serial.print("Motion:           0x"); Serial.println(motionBurstData[Motion], 			HEX);
	Serial.print("Observation:      0x"); Serial.println(motionBurstData[Observation], 		HEX);
	Serial.print("Delta_X_L:        0x"); Serial.println(motionBurstData[Delta_X_L], 		HEX);
	Serial.print("Delta_X_H:        0x"); Serial.println(motionBurstData[Delta_X_H], 		HEX);
	Serial.print("Delta_Y_L:        0x"); Serial.println(motionBurstData[Delta_Y_L], 		HEX);
	Serial.print("Delta_Y_H:        0x"); Serial.println(motionBurstData[Delta_Y_H], 		HEX);
	Serial.print("SQUAL:            0x"); Serial.println(motionBurstData[SQUAL], 			HEX);
	Serial.print("Raw_Data_Sum:     0x"); Serial.println(motionBurstData[Raw_Data_Sum], 	HEX);
	Serial.print("Maximum_Raw_Data: 0x"); Serial.println(motionBurstData[Maximum_Raw_Data], HEX);
	Serial.print("Minimum_Raw_Data: 0x"); Serial.println(motionBurstData[Minimum_Raw_Data], HEX);
	Serial.print("Shutter_Upper:    0x"); Serial.println(motionBurstData[Shutter_Upper], 	HEX);
	Serial.print("Shutter_Lower:    0x"); Serial.println(motionBurstData[Shutter_Lower], 	HEX);
	Serial.println();
}

void PMW3360DM_T2QU::printDefaultRegisters ()  const
{
	uint8_t regValue = 0x00;

	Serial.println("--- Default Values Registers ---");
	Serial.print("Product_ID:                 0x"); Serial.print(spi_read8(REG_Product_ID),                 HEX); Serial.println(" default is 0x42");
	Serial.print("Revision_ID:                0x"); Serial.print(spi_read8(REG_Revision_ID),                HEX); Serial.println(" default is 0x01");
	Serial.print("Motion:                     0x"); Serial.print(spi_read8(REG_Motion),                     HEX); Serial.println(" default is 0x20");
	Serial.print("Shutter_Lower:              0x"); Serial.print(spi_read8(REG_Shutter_Lower),              HEX); Serial.println(" default is 0x12");
	Serial.print("Shutter_Upper:              0x"); Serial.print(spi_read8(REG_Shutter_Upper),              HEX); Serial.println(" default is 0x00");
	Serial.print("Control:                    0x");	Serial.print(spi_read8(REG_Control),                    HEX); Serial.println(" default is 0x02");
	Serial.print("Config1:                    0x"); Serial.print(spi_read8(REG_Config1),                    HEX); Serial.println(" default is 0x31");
	Serial.print("Config2:                    0x"); Serial.print(spi_read8(REG_Config2),                    HEX); Serial.println(" default is 0x20");
	Serial.print("Run_Downshift:              0x"); Serial.print(spi_read8(REG_Run_Downshift),              HEX); Serial.println(" default is 0x32");
	Serial.print("Rest1_Downshift:            0x"); Serial.print(spi_read8(REG_Rest1_Downshift),            HEX); Serial.println(" default is 0x1f");
	Serial.print("Rest2_Downshift:            0x"); Serial.print(spi_read8(REG_Rest2_Downshift),            HEX); Serial.println(" default is 0xbc");
	Serial.print("Rest3_Rate_Lower:           0x"); Serial.print(spi_read8(REG_Rest3_Rate_Lower),           HEX); Serial.println(" default is 0xf3");
	Serial.print("Rest3_Rate_Upper:           0x"); Serial.print(spi_read8(REG_Rest3_Rate_Upper),           HEX); Serial.println(" default is 0x01");
	Serial.print("Min_SQ_Run:                 0x"); Serial.print(spi_read8(REG_Min_SQ_Run),                 HEX); Serial.println(" default is 0x10");
	Serial.print("Raw_Data_Threshold:         0x"); Serial.print(spi_read8(REG_Raw_Data_Threshold),         HEX); Serial.println(" default is 0x0a");
	Serial.print("Config5:                    0x"); Serial.print(spi_read8(REG_Config5),                    HEX); Serial.println(" default is 0x31");
	Serial.print("Inverse_Product_ID:         0x"); Serial.print(spi_read8(REG_Inverse_Product_ID),         HEX); Serial.println(" default is 0xbd");
	Serial.print("LiftCutoff_Tune_Timeout:    0x"); Serial.print(spi_read8(REG_LiftCutoff_Tune_Timeout),    HEX); Serial.println(" default is 0x27");
	Serial.print("LiftCutoff_Tune_Min_Length: 0x"); Serial.print(spi_read8(REG_LiftCutoff_Tune_Min_Length), HEX); Serial.println(" default is 0x09");
	Serial.print("Lift_Config:                0x"); Serial.print(spi_read8(REG_Lift_Config),                HEX); Serial.println(" default is 0x02");
	Serial.println();
}

bool PMW3360DM_T2QU::powerUp (void)
{
	bool success = false;

	// Although the chip PMW3360DM performs an internal power up selft reset, it is still recommend that the 
	// Power_Up_Reset register is written every time power is applied. The appropiate sequence is as follows:

	// 1. Apply power to VDD and VDDIO in any order, with a delay of no more than 100 ms in between each supply.
	//    Ensure all supplies are stable.
	// delay(100);
	// 2. Drive NCS high, and then low to reset the SPI port
	digitalWrite(ncsPin, HIGH);
	digitalWrite(ncsPin, LOW);
	// 3. Write 0x5a to Power_Up_Reset register (address 0x3a), or alternatively toggle the NRESET pin. 
	//    Note ADNS-9800:  Reset is required after recovering from shutdown mode and restore normal operation after Frame Capture.
	spi_write8(REG_Power_Up_Reset, 0x5a);	// write 0x5a to this regiser to reset the chip. Al settings will be revert to default values.
	// 4. Wait for at least 50ms time.
	delay(50);
	// 5. Read from registers 0x02, 0x03, 0x04, 0x05, 0x06 (or read these same 5 bytes from burst motion register)
	//    one time regardless of the motion pin state.
	spi_read8(REG_Motion);		// 0x02
	spi_read8(REG_Delta_X_L);	// 0x03
	spi_read8(REG_Delta_X_H);	// 0x04
	spi_read8(REG_Delta_Y_L);	// 0x05
	spi_read8(REG_Delta_Y_H);	// 0x06
	// 6. Perform SROM download
	success = SROM_download();
	// 7. Load configuration for other registers.
	/*
	spi_write8(REG_Shutter_Lower,              0x12);
	spi_write8(REG_Shutter_Upper,              0x00);
	spi_write8(REG_Raw_Data_Threshold,         0x0a);
	spi_write8(REG_Config5,                    0x31);
	spi_write8(REG_LiftCutoff_Tune_Timeout,    0x27);
	spi_write8(REG_LiftCutoff_Tune_Min_Length, 0x09);
	spi_write8(REG_Lift_Config,                0x02);
	*/

	return success;
}

void PMW3360DM_T2QU::reset (int nRESET_PIN)
{
	// The NRESET pin can be used to perform a full chip  reset. When asserted, it performs the same reset function
	// as the Power_Up_Register. The NRESET pin needs to be asserted (held to logic 0) for at least 100 ns.

	// Note: NRESET pin has a built in weak pull up circuit. 
	// During active low reset phase, it can draw a static current of up to 600uA.

	pinMode(nRESET_PIN, OUTPUT);
	digitalWrite(nRESET_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(nRESET_PIN, HIGH);
}

void PMW3360DM_T2QU::shutdown (void)
{
	// PMW3360DM_T2QU can be set in Shutdown mode by writing to Shutdown register.
	// The SPI port shuild not be accesed when Shutdown mode is asserted, except the power-up command (writing 0x5a to register 0x3a).
	// Other ICs (Integrated Circuits) on the same SPI bus can be accessed, as long as the chip's NCS pin is not asserted.
	// The SROM download is required when wake up from Shutdown mode.

	// Write 0xB6 to set the chip to shutdown mode. 
	spi_write8(REG_Shutdown, 0xB6);

	// To de-assert Shutdown mode: Perfomr a "powerUp" sequence. 
	// There are long wake-up times form shutdown. 
	// Thee features should not be used for power management during normal mouse motion.
}

uint16_t PMW3360DM_T2QU::SROM_CRC ()
{
	// SROM CRC test can be performed to check if SROM download was successful. 
	// Navigation is halted and the SPI port should not be used during this SROM CRC test.
	// Registers will be reset to default value after completion of CRC test.
	// SROM CRC read procedure is as below:
	// 1. Write 0x15 to SROM_Enable register (0x13).
	spi_write8(REG_SROM_Enable, 0x15);
	// 2. Wait for at least 10ms.
	delay(10);
	// 3. Read register Data_Out_Upper (0x26) and register Data_Out_Lower (0x25)
	// Data in these registers come from the SROM CRC test. The data can be read out in any order.
	// Default value is 0x00 for each one.
	uint8_t dataOutH = spi_read8(REG_Data_Out_Upper);
	uint8_t dataOutL = spi_read8(REG_Data_Out_Lower);

	return ((dataOutH << 8) | dataOutL);
}

uint8_t PMW3360DM_T2QU::SROM_ID ()
{
	// Contains the revision of the downloaded Shadow ROM (SROM) firmware. 
	// If the firmware has been successfully downloaded and the chip is operating out of SROM,
	// this register will contain the SROM firmware revision; otherwise it will contain 0x00.
	return spi_read8(REG_SROM_ID);
}

bool PMW3360DM_T2QU::SROM_download ()
{
	// This function is used to load the supplied firmware file contents into PMW3360DM-T2QU after chip power up sequence.
	// The firmware file is an ASCII text file. The SROM download success may be verified in two ways. 
	// One execution from SROM space begins, the SROM_ID register will report the firmware version.
	// At any time, a self-test may be executed which performs a CRC on the SROM contents and reports the results in a register.
	// Take note that the self-test does disrupt tracking performance and also reset registers to default value.

	uint8_t regValue = 0x00;

	#ifdef DEBUG_SERIAL
	Serial.print("Uploading firmware... ");
	#endif 

	// SROM download procedure:
	// 1. Perform the Power-Up sequence (steps 1 to 8???)
	// done by precondition.
	// 2. Write 0 to Rest_En bit (5) of Config2 register (0x10) to disable Rest mode.
	regValue = spi_read8(REG_Config2);			// bits 7, 6, 4, 3, 2 are reserved and 1 is logic '0', if not write simply 0x20.
	spi_write8(REG_Config2, regValue & 0xdf); 	// regValue & 1101 1111b, Normal Operation without REST mode.
	// 3. Write 0x1d to SROM_Enable register (0x13) for initializing
	spi_write8(REG_SROM_Enable, 0x1d);	
	// 4. Wait for 10 ms
	delay(10);
	// 5. Write 0x18 to SROM_Enable register (0x13) again to start SROM Download.
	spi_write8(REG_SROM_Enable, 0x18); 
	// 6. Write SROM file into SROM_Load_Burst register (0x62), first data must start with SROM_Load_Burst address.
	//    All the SROM data must be downloaded before SROM starts running.
	digitalWrite(ncsPin, LOW); 					// com begin
	SPI.transfer(REG_SROM_Load_Burst | 0x80); 	// write burst destination address
	delayMicroseconds(35);						// tSCLK-NCS (=35us) for write operation

	// send all bytes of the firmware
	unsigned char c;
	for (int i=0; i<firmware_length; ++i)
	{
		c = (unsigned char) pgm_read_byte(firmware_data + i); // firmware_data is global & extern
		SPI.transfer(c);
		delayMicroseconds(35);
	}
	digitalWrite(ncsPin, HIGH);		// com end
	delayMicroseconds(120);			// tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but it looks like a safe lower bound

	// 7. Read the SROM_ID register (0x2A) to verify the ID before any other register reads or writes.
	regValue = spi_read8(REG_SROM_ID);
	//Serial.print("REG_SROM_ID: 0x"); Serial.println(regValue);
	if (regValue == 0x00)
		return false;
	// 8. Write 0x00 to Config2 register (0x10) for wired mouse or 0x20 for wireless mouse design.
	// Wired: 
		// [7:6] 	Reserved
		// [5] 		Normal Operation without REST mode.
		// [4:3] 	Reserved
		// [2]		Normal CPI setting affects both delta X and Y.
		// [1]		Reserved
		// [0] 		Always '0'	
	spi_write8(REG_Config2, 0x00); 	//0x00
	// Wireless: 
		// [7:6] 	Reserved
		// [5] 		REST mode enabled.
		// [4:3] 	Reserved
		// [2]		Normal CPI setting affects both delta X and Y.
		// [1]		Reserved
		// [0] 		Always '0'	
	// spi_write8(REG_Config2, 0x20); 	


	#ifdef DEBUG_SERIAL
	Serial.println("done.");
	#endif

	return true;
}
