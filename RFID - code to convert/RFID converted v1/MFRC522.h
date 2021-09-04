/**
 * MFRC522.h - Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
 * Based on code Dr.Leong   ( WWW.B2CQSHOP.COM )
 * Created by Miguel Balboa (circuitito.com), Jan, 2012.
 * Rewritten by Søren Thing Andersen (access.thing.dk), fall of 2013 (Translation to English, refactored, comments, anti collision, cascade levels.)
 * Extended by Tom Clement with functionality to write to sector 0 of UID changeable Mifare cards.
 * Released into the public domain.
 * 
 * Please read this file for an overview and then MFRC522.cpp for comments on the specific functions.
 * Search for "mf-rc522" on ebay.com to purchase the MF-RC522 board. 
 * 
 * There are three hardware components involved:
 * 1) The micro controller: An Arduino
 * 2) The PCD (short for Proximity Coupling Device): NXP MFRC522 Contactless Reader IC
 * 3) The PICC (short for Proximity Integrated Circuit Card): A card or tag using the ISO 14443A interface, eg Mifare or NTAG203.
 * 
 * The microcontroller and card reader uses SPI for communication.
 * The protocol is described in the MFRC522 datasheet: http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * 
 * The card reader and the tags communicate using a 13.56MHz electromagnetic field.
 * The protocol is defined in ISO/IEC 14443-3 Identification cards -- Contactless integrated circuit cards -- Proximity cards -- Part 3: Initialization and anticollision".
 * A free version of the final draft can be found at http://wg8.de/wg8n1496_17n3613_Ballot_FCD14443-3.pdf
 * Details are found in chapter 6, Type A – Initialization and anticollision.
 * 
 * If only the PICC UID is wanted, the above documents has all the needed information.
 * To read and write from MIFARE PICCs, the MIFARE protocol is used after the PICC has been selected.
 * The MIFARE Classic chips and protocol is described in the datasheets:
 *		1K:   http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf
 * 		4K:   http://datasheet.octopart.com/MF1S7035DA4,118-NXP-Semiconductors-datasheet-11046188.pdf
 * 		Mini: http://www.idcardmarket.com/download/mifare_S20_datasheet.pdf
 * The MIFARE Ultralight chip and protocol is described in the datasheets:
 *		Ultralight:   http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf
 * 		Ultralight C: http://www.nxp.com/documents/short_data_sheet/MF0ICU2_SDS.pdf
 * 
 * MIFARE Classic 1K (MF1S503x):
 * 		Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
 * 		The blocks are numbered 0-63.
 * 		Block 3 in each sector is the Sector Trailer. See http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf sections 8.6 and 8.7:
 * 				Bytes 0-5:   Key A
 * 				Bytes 6-8:   Access Bits
 * 				Bytes 9:     User data
 * 				Bytes 10-15: Key B (or user data)
 * 		Block 0 is read-only manufacturer data.
 * 		To access a block, an authentication using a key from the block's sector must be performed first.
 * 		Example: To read from block 10, first authenticate using a key from sector 3 (blocks 8-11).
 * 		All keys are set to FFFFFFFFFFFFh at chip delivery.
 * 		Warning: Please read section 8.7 "Memory Access". It includes this text: if the PICC detects a format violation the whole sector is irreversibly blocked.
 *		To use a block in "value block" mode (for Increment/Decrement operations) you need to change the sector trailer. Use PICC_SetAccessBits() to calculate the bit patterns.
 * MIFARE Classic 4K (MF1S703x):
 * 		Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
 * 		The blocks are numbered 0-255.
 * 		The last block in each sector is the Sector Trailer like above.
 * MIFARE Classic Mini (MF1 IC S20):
 * 		Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
 * 		The blocks are numbered 0-19.
 * 		The last block in each sector is the Sector Trailer like above.
 * 
 * MIFARE Ultralight (MF0ICU1):
 * 		Has 16 pages of 4 bytes = 64 bytes.
 * 		Pages 0 + 1 is used for the 7-byte UID.
 * 		Page 2 contains the last check digit for the UID, one byte manufacturer internal data, and the lock bytes (see http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf section 8.5.2)
 * 		Page 3 is OTP, One Time Programmable bits. Once set to 1 they cannot revert to 0.
 * 		Pages 4-15 are read/write unless blocked by the lock bytes in page 2. 
 * MIFARE Ultralight C (MF0ICU2):
 * 		Has 48 pages of 4 bytes = 192 bytes.
 * 		Pages 0 + 1 is used for the 7-byte UID.
 * 		Page 2 contains the last check digit for the UID, one byte manufacturer internal data, and the lock bytes (see http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf section 8.5.2)
 * 		Page 3 is OTP, One Time Programmable bits. Once set to 1 they cannot revert to 0.
 * 		Pages 4-39 are read/write unless blocked by the lock bytes in page 2. 
 * 		Page 40 Lock bytes
 * 		Page 41 16 bit one way counter
 * 		Pages 42-43 Authentication configuration
 * 		Pages 44-47 Authentication key 
 */
#ifndef MFRC522_h
#define MFRC522_h


#ifndef F_CPU		//To prevent defining F_CPU multiple times
//Defining internal RC oscillator clock frequency as 8MHz
#define F_CPU 8000000UL
#endif		//F_CPU



#include "main_timer.h"
#include "lcd_i2c.h"
#include<stdlib.h>

#include "spilib.h"

//Pin and static functions for selecting the MFRC522 SPI SS (active low)
#define MFRC522_SPI_CS_DDR			DDRB
#define MFRC522_SPI_CS_PORT			PORTB
#define MFRC522_SPI_CS				PB4

#define MFRC522_SETUP_SPI_CS()		(MFRC522_SPI_CS_DDR |= (1 << MFRC522_SPI_CS))

#define MFRC522_ENABLE_CS()			(MFRC522_SPI_CS_PORT &= ~(1 << MFRC522_SPI_CS))
#define MFRC522_DISABLE_CS()		(MFRC522_SPI_CS_PORT |= (1 << MFRC522_SPI_CS))

//Pin and static functions for hard resetting the MFRC522 using the reset pin (active high)
#define MFRC522_RST_DDR				DDRB
#define MFRC522_RST_PORT			PORTB
#define MFRC522_RST_PIN				PINB
#define MFRC522_RST					PB3

#define MFRC522_SET_RST_IN()		(MFRC522_RST_DDR &= ~(1 << MFRC522_RST))
#define MFRC522_SET_RST_OUT()		(MFRC522_RST_DDR |= (1 << MFRC522_RST))

#define MFRC522_READ_RST()			(MFRC522_RST_PIN & (1 << MFRC522_RST))
#define MFRC522_RST_HIGH()			(MFRC522_RST_PORT |= (1 << MFRC522_RST))


//Function to get the sizeof a byte pointer
#define byteCount(b)		(sizeof(b) / sizeof(b[0]))


#include <avr/pgmspace.h>

// Firmware data for self-test
// Reference values based on firmware version
// Hint: if needed, you can remove unused self-test data to save flash memory
//
// Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August 2005; 16.1 self-test
const uint8_t MFRC522_firmware_referenceV0_0[] PROGMEM = {
	0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19,
	0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
	0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
	0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
	0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
	0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
	0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
	0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D
};
// Version 1.0 (0x91)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const uint8_t MFRC522_firmware_referenceV1_0[] PROGMEM = {
	0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
	0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
	0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
	0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
	0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
	0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
	0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
	0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
};
// Version 2.0 (0x92)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const uint8_t MFRC522_firmware_referenceV2_0[] PROGMEM = {
	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
	0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
	0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
	0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
	0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
};
// Clone
// Fudan Semiconductor FM17522 (0x88)
const uint8_t FM17522_firmware_reference[] PROGMEM = {
	0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18,
	0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
	0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7,
	0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
	0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7,
	0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
	0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB,
	0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62
};




// Size of the MFRC522 FIFO
static uint8_t MFRC522_FIFO_SIZE = 64;		// The FIFO is 64 bytes.



// MFRC522 registers. Described in chapter 9 of the datasheet.
// When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
typedef enum {
	// Page 0: Command and status
	//								  0x00			// reserved for future use
	MFRC522_CommandReg				= 0x01 << 1,	// starts and stops command execution
	MFRC522_ComIEnReg				= 0x02 << 1,	// enable and disable interrupt request control bits
	MFRC522_DivIEnReg				= 0x03 << 1,	// enable and disable interrupt request control bits
	MFRC522_ComIrqReg				= 0x04 << 1,	// interrupt request bits
	MFRC522_DivIrqReg				= 0x05 << 1,	// interrupt request bits
	MFRC522_ErrorReg				= 0x06 << 1,	// error bits showing the error status of the last command executed 
	MFRC522_Status1Reg				= 0x07 << 1,	// communication status bits
	MFRC522_Status2Reg				= 0x08 << 1,	// receiver and transmitter status bits
	MFRC522_FIFODataReg				= 0x09 << 1,	// input and output of 64 byte FIFO buffer
	MFRC522_FIFOLevelReg			= 0x0A << 1,	// number of bytes stored in the FIFO buffer
	MFRC522_WaterLevelReg			= 0x0B << 1,	// level for FIFO underflow and overflow warning
	MFRC522_ControlReg				= 0x0C << 1,	// miscellaneous control registers
	MFRC522_BitFramingReg			= 0x0D << 1,	// adjustments for bit-oriented frames
	MFRC522_CollReg					= 0x0E << 1,	// bit position of the first bit-collision detected on the RF interface
	//								  0x0F			// reserved for future use
	
	// Page 1: Command
	// 								  0x10			// reserved for future use
	MFRC522_ModeReg					= 0x11 << 1,	// defines general modes for transmitting and receiving 
	MFRC522_TxModeReg				= 0x12 << 1,	// defines transmission data rate and framing
	MFRC522_RxModeReg				= 0x13 << 1,	// defines reception data rate and framing
	MFRC522_TxControlReg			= 0x14 << 1,	// controls the logical behavior of the antenna driver pins TX1 and TX2
	MFRC522_TxASKReg				= 0x15 << 1,	// controls the setting of the transmission modulation
	MFRC522_TxSelReg				= 0x16 << 1,	// selects the internal sources for the antenna driver
	MFRC522_RxSelReg				= 0x17 << 1,	// selects internal receiver settings
	MFRC522_RxThresholdReg			= 0x18 << 1,	// selects thresholds for the bit decoder
	MFRC522_DemodReg				= 0x19 << 1,	// defines demodulator settings
	// 								  0x1A			// reserved for future use
	// 								  0x1B			// reserved for future use
	MFRC522_MfTxReg					= 0x1C << 1,	// controls some MIFARE communication transmit parameters
	MFRC522_MfRxReg					= 0x1D << 1,	// controls some MIFARE communication receive parameters
	// 								  0x1E			// reserved for future use
	MFRC522_SerialSpeedReg			= 0x1F << 1,	// selects the speed of the serial UART interface
	
	// Page 2: Configuration
	// 								  0x20			// reserved for future use
	MFRC522_CRCResultRegH			= 0x21 << 1,	// shows the MSB and LSB values of the CRC calculation
	MFRC522_CRCResultRegL			= 0x22 << 1,
	// 								  0x23			// reserved for future use
	MFRC522_ModWidthReg				= 0x24 << 1,	// controls the ModWidth setting?
	// 								  0x25			// reserved for future use
	MFRC522_RFCfgReg				= 0x26 << 1,	// configures the receiver gain
	MFRC522_GsNReg					= 0x27 << 1,	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
	MFRC522_CWGsPReg				= 0x28 << 1,	// defines the conductance of the p-driver output during periods of no modulation
	MFRC522_ModGsPReg				= 0x29 << 1,	// defines the conductance of the p-driver output during periods of modulation
	MFRC522_TModeReg				= 0x2A << 1,	// defines settings for the internal timer
	MFRC522_TPrescalerReg			= 0x2B << 1,	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
	MFRC522_TReloadRegH				= 0x2C << 1,	// defines the 16-bit timer reload value
	MFRC522_TReloadRegL				= 0x2D << 1,
	MFRC522_TCounterValueRegH		= 0x2E << 1,	// shows the 16-bit timer value
	MFRC522_TCounterValueRegL		= 0x2F << 1,
	
	// Page 3: Test Registers
	// 								  0x30			// reserved for future use
	MFRC522_TestSel1Reg				= 0x31 << 1,	// general test signal configuration
	MFRC522_TestSel2Reg				= 0x32 << 1,	// general test signal configuration
	MFRC522_TestPinEnReg			= 0x33 << 1,	// enables pin output driver on pins D1 to D7
	MFRC522_TestPinValueReg			= 0x34 << 1,	// defines the values for D1 to D7 when it is used as an I/O bus
	MFRC522_TestBusReg				= 0x35 << 1,	// shows the status of the internal test bus
	MFRC522_AutoTestReg				= 0x36 << 1,	// controls the digital self-test
	MFRC522_VersionReg				= 0x37 << 1,	// shows the software version
	MFRC522_AnalogTestReg			= 0x38 << 1,	// controls the pins AUX1 and AUX2
	MFRC522_TestDAC1Reg				= 0x39 << 1,	// defines the test value for TestDAC1
	MFRC522_TestDAC2Reg				= 0x3A << 1,	// defines the test value for TestDAC2
	MFRC522_TestADCReg				= 0x3B << 1		// shows the value of ADC I and Q channels
	// 								  0x3C			// reserved for production tests
	// 								  0x3D			// reserved for production tests
	// 								  0x3E			// reserved for production tests
	// 								  0x3F			// reserved for production tests
} MFRC522_PCD_Register;

// MFRC522 commands. Described in chapter 10 of the datasheet.
typedef enum {
	MFRC522_PCD_Idle				= 0x00,		// no action, cancels current command execution
	MFRC522_PCD_Mem					= 0x01,		// stores 25 bytes into the internal buffer
	MFRC522_PCD_GenerateRandomID	= 0x02,		// generates a 10-byte random ID number
	MFRC522_PCD_CalcCRC				= 0x03,		// activates the CRC coprocessor or performs a self-test
	MFRC522_PCD_Transmit			= 0x04,		// transmits data from the FIFO buffer
	MFRC522_PCD_NoCmdChange			= 0x07,		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
	MFRC522_PCD_Receive				= 0x08,		// activates the receiver circuits
	MFRC522_PCD_Transceive 			= 0x0C,		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
	MFRC522_PCD_MFAuthent 			= 0x0E,		// performs the MIFARE standard authentication as a reader
	MFRC522_PCD_SoftReset			= 0x0F		// resets the MFRC522
} MFRC522_PCD_Command;

// MFRC522 RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the PCD).
// Described in 9.3.3.6 / table 98 of the datasheet at http://www.nxp.com/documents/data_sheet/MFRC522.pdf
typedef enum {
	MFRC522_RxGain_18dB				= 0x00 << 4,	// 000b - 18 dB, minimum
	MFRC522_RxGain_23dB				= 0x01 << 4,	// 001b - 23 dB
	MFRC522_RxGain_18dB_2			= 0x02 << 4,	// 010b - 18 dB, it seems 010b is a duplicate for 000b
	MFRC522_RxGain_23dB_2			= 0x03 << 4,	// 011b - 23 dB, it seems 011b is a duplicate for 001b
	MFRC522_RxGain_33dB				= 0x04 << 4,	// 100b - 33 dB, average, and typical default
	MFRC522_RxGain_38dB				= 0x05 << 4,	// 101b - 38 dB
	MFRC522_RxGain_43dB				= 0x06 << 4,	// 110b - 43 dB
	MFRC522_RxGain_48dB				= 0x07 << 4,	// 111b - 48 dB, maximum
	MFRC522_RxGain_min				= 0x00 << 4,	// 000b - 18 dB, minimum, convenience for RxGain_18dB
	MFRC522_RxGain_avg				= 0x04 << 4,	// 100b - 33 dB, average, convenience for RxGain_33dB
	MFRC522_RxGain_max				= 0x07 << 4		// 111b - 48 dB, maximum, convenience for RxGain_48dB
} MFRC522_PCD_RxGain;

// Commands sent to the PICC.
typedef enum {
	// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
	MFRC522_PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
	MFRC522_PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
	MFRC522_PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
	MFRC522_PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
	MFRC522_PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
	MFRC522_PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
	MFRC522_PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
	MFRC522_PICC_CMD_RATS           = 0xE0,     // Request command for Answer To Reset.
	// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
	// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
	// The read/write commands can also be used for MIFARE Ultralight.
	MFRC522_PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A
	MFRC522_PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
	MFRC522_PICC_CMD_MF_READ		= 0x30,		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
	MFRC522_PICC_CMD_MF_WRITE		= 0xA0,		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
	MFRC522_PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
	MFRC522_PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
	MFRC522_PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
	MFRC522_PICC_CMD_MF_TRANSFER	= 0xB0,		// Writes the contents of the internal data register to a block.
	// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
	// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
	MFRC522_PICC_CMD_UL_WRITE		= 0xA2		// Writes one 4 byte page to the PICC.
} MFRC522_PICC_Command;

// MIFARE constants that does not fit anywhere else
typedef enum {
	MFRC522_MF_ACK					= 0xA,		// The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
	MFRC522_MF_KEY_SIZE				= 6			// A Mifare Crypto1 key is 6 bytes.
} MIFARE_Misc;

// PICC types we can detect. Remember to update PICC_GetTypeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
typedef enum {
	MFRC522_PICC_TYPE_UNKNOWN		,
	MFRC522_PICC_TYPE_ISO_14443_4	,	// PICC compliant with ISO/IEC 14443-4 
	MFRC522_PICC_TYPE_ISO_18092		, 	// PICC compliant with ISO/IEC 18092 (NFC)
	MFRC522_PICC_TYPE_MIFARE_MINI	,	// MIFARE Classic protocol, 320 bytes
	MFRC522_PICC_TYPE_MIFARE_1K		,	// MIFARE Classic protocol, 1KB
	MFRC522_PICC_TYPE_MIFARE_4K		,	// MIFARE Classic protocol, 4KB
	MFRC522_PICC_TYPE_MIFARE_UL		,	// MIFARE Ultralight or Ultralight C
	MFRC522_PICC_TYPE_MIFARE_PLUS	,	// MIFARE Plus
	MFRC522_PICC_TYPE_MIFARE_DESFIRE,	// MIFARE DESFire
	MFRC522_PICC_TYPE_TNP3XXX		,	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
	MFRC522_PICC_TYPE_NOT_COMPLETE	= 0xff	// SAK indicates UID is not complete.
} MFRC522_PICC_Type;

// Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
typedef enum {
	MFRC522_STATUS_OK				,	// Success
	MFRC522_STATUS_ERROR			,	// Error in communication
	MFRC522_STATUS_COLLISION		,	// Collission detected
	MFRC522_STATUS_TIMEOUT			,	// Timeout in communication.
	MFRC522_STATUS_NO_ROOM			,	// A buffer is not big enough.
	MFRC522_STATUS_INTERNAL_ERROR	,	// Internal error in the code. Should not happen ;-)
	MFRC522_STATUS_INVALID			,	// Invalid argument.
	MFRC522_STATUS_CRC_WRONG		,	// The CRC_A does not match
	MFRC522_STATUS_MIFARE_NACK		= 0xff	// A MIFARE PICC responded with NAK.
} MFRC522_StatusCode;

// A struct used for passing the UID of a PICC.
typedef struct {
	uint8_t		size;			// Number of bytes in the UID. 4, 7 or 10.
	uint8_t		uidByte[10];
	uint8_t		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
} Uid;

// A struct used for passing a MIFARE Crypto1 key
typedef struct {
	uint8_t		keyByte[MFRC522_MF_KEY_SIZE];
} MIFARE_Key;

// Member variables
Uid uid;	// Used by PICC_ReadCardSerial().
//SPI Communication
SPI_Settings MFRC522_spi_settings;




/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////
void MFRC522_PCD_WriteRegister(MFRC522_PCD_Register reg, uint8_t value);
void MFRC522_PCD_WriteRegister_Many(MFRC522_PCD_Register reg, uint8_t count, uint8_t *values);
uint8_t MFRC522_PCD_ReadRegister(MFRC522_PCD_Register reg);
void MFRC522_PCD_ReadRegister_Many(MFRC522_PCD_Register reg, uint8_t count, uint8_t *values, uint8_t rxAlign);
void MFRC522_PCD_SetRegisterBitMask(MFRC522_PCD_Register reg, uint8_t mask);
void MFRC522_PCD_ClearRegisterBitMask(MFRC522_PCD_Register reg, uint8_t mask);
MFRC522_StatusCode MFRC522_PCD_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result);

/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////
void MFRC522_PCD_Init();
void MFRC522_PCD_Reset();
void MFRC522_PCD_AntennaOn();
void MFRC522_PCD_AntennaOff();
uint8_t MFRC522_PCD_GetAntennaGain();
void MFRC522_PCD_SetAntennaGain(uint8_t mask);
uint8_t MFRC522_PCD_PerformSelfTest();

/////////////////////////////////////////////////////////////////////////////////////
// Power control functions
/////////////////////////////////////////////////////////////////////////////////////
void MFRC522_PCD_SoftPowerDown();
void MFRC522_PCD_SoftPowerUp();

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////
MFRC522_StatusCode MFRC522_PCD_TransceiveData(uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, uint8_t checkCRC);
MFRC522_StatusCode MFRC522_PCD_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, uint8_t checkCRC);
MFRC522_StatusCode MFRC522_PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize);
MFRC522_StatusCode MFRC522_PICC_WakeupA(uint8_t *bufferATQA, uint8_t *bufferSize);
MFRC522_StatusCode MFRC522_PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize);
MFRC522_StatusCode MFRC522_PICC_Select(Uid *uid, uint8_t validBits);
MFRC522_StatusCode MFRC522_PICC_HaltA();

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////
MFRC522_StatusCode MFRC522_PCD_Authenticate(uint8_t command, uint8_t blockAddr, MIFARE_Key *key, Uid *uid);
void MFRC522_PCD_StopCrypto1();
MFRC522_StatusCode MFRC522_MIFARE_Read(uint8_t blockAddr, uint8_t *buffer, uint8_t *bufferSize);
MFRC522_StatusCode MFRC522_MIFARE_Write(uint8_t blockAddr, uint8_t *buffer, uint8_t bufferSize);
MFRC522_StatusCode MFRC522_MIFARE_Ultralight_Write(uint8_t page, uint8_t *buffer, uint8_t bufferSize);
MFRC522_StatusCode MFRC522_MIFARE_Decrement(uint8_t blockAddr, int32_t delta);
MFRC522_StatusCode MFRC522_MIFARE_Increment(uint8_t blockAddr, int32_t delta);
MFRC522_StatusCode MFRC522_MIFARE_Restore(uint8_t blockAddr);
MFRC522_StatusCode MFRC522_MIFARE_Transfer(uint8_t blockAddr);
MFRC522_StatusCode MFRC522_MIFARE_GetValue(uint8_t blockAddr, int32_t *value);
MFRC522_StatusCode MFRC522_MIFARE_SetValue(uint8_t blockAddr, int32_t value);
MFRC522_StatusCode MFRC522_PCD_MIFARE_Transceive(uint8_t *sendData, uint8_t sendLen, uint8_t acceptTimeout);
MFRC522_StatusCode MFRC522_PCD_NTAG216_AUTH(uint8_t *passWord, uint8_t pACK[]);

/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////
MFRC522_StatusCode PCD_MIFARE_Transceive(uint8_t *sendData, uint8_t sendLen, uint8_t acceptTimeout);
static MFRC522_PICC_Type PICC_GetType(uint8_t sak);

// Support functions for debuging - should be given a char* of length 15
static void MFRC522_GetStatusCodeName(MFRC522_StatusCode code, char *str);
static void MFRC522_PICC_GetTypeName(MFRC522_PICC_Type type, char *str);

// Support functions for debuging
void MFRC522_PCD_DumpVersionToOutput();
void MFRC522_PICC_DumpToOutput(Uid *uid);
void MFRC522_PICC_DumpDetailsToOutput(Uid *uid);
void MFRC522_PICC_DumpMifareClassicToOutput(Uid *uid, MFRC522_PICC_Type piccType, MIFARE_Key *key);
void MFRC522_PICC_DumpMifareClassicSectorToOutput(Uid *uid, MIFARE_Key *key, uint8_t sector);

// Advanced functions for MIFARE
void MIFARE_SetAccessBits(uint8_t *accessBitBuffer, uint8_t g0, uint8_t g1, uint8_t g2, uint8_t g3);

/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////
uint8_t MFRC522_PICC_IsNewCardPresent();
uint8_t MFRC522_PICC_ReadCardSerial();


// Functions for communicating with MIFARE PICCs
MFRC522_StatusCode MFRC522_MIFARE_TwoStepHelper(uint8_t command, uint8_t blockAddr, int32_t data);




void output_heading(char *str){
	lcd_clear();
	lcd_first_line();
	lcd_print(str);
	lcd_second_line();
}
void output_hex (uint8_t outputByte, uint8_t hex_prefix){
	if (hex_prefix)
		lcd_print("0x");
	char tmp[10];
    itoa((int)outputByte, tmp, 16);
	if (outputByte < 0x10)
		lcd_print("0");
    lcd_print(tmp);
}
void output_number (uint8_t outputByte){
	char tmp[10];
    itoa((int)outputByte, tmp, 10);
    lcd_print(tmp);
}
void output_hold_delay(){
	//1.5 seconds delay for the screen to be read
	uint64_t cont = main_timer_now() + 46875UL;
	while (main_timer_now() <= cont);
}


#endif		//MFRC522_h
