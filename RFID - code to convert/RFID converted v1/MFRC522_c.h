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


/////////////////////////////////////////////////////////////////////////////////////
// Functions for setting up the Arduino
/////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522_PCD_WriteRegister(	MFRC522_PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
									uint8_t value			///< The value to write.
								) {
	spi_master_init(MFRC522_spi_settings);	// Set the settings to work with SPI bus
	MFRC522_ENABLE_CS();		// Select slave
	spi_transceiver(reg);						// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	spi_transceiver(value);
	MFRC522_DISABLE_CS();		// Release slave again
	spi_un_init(); // Stop using the SPI bus
} // End PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522_PCD_WriteRegister_Many(	MFRC522_PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
									uint8_t count,			///< The number of bytes to write to the register
									uint8_t *values		///< The values to write. Byte array.
								) {
	spi_master_init(MFRC522_spi_settings);	// Set the settings to work with SPI bus
	MFRC522_ENABLE_CS();		// Select slave
	spi_transceiver(reg);						// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	uint8_t index;
	for (index = 0; index < count; index++) {
		spi_transceiver(values[index]);
	}
	MFRC522_DISABLE_CS();		// Release slave again
	spi_un_init(); // Stop using the SPI bus
} // End PCD_WriteRegister_Many()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t MFRC522_PCD_ReadRegister(	MFRC522_PCD_Register reg	///< The register to read from. One of the PCD_Register enums.
								) {
	uint8_t value;
	spi_master_init(MFRC522_spi_settings);	// Set the settings to work with SPI bus
	MFRC522_ENABLE_CS();			// Select slave
	spi_transceiver(0x80 | reg);					// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	value = spi_transceiver(0);					// Read the value back. Send 0 to stop reading.
	MFRC522_DISABLE_CS();			// Release slave again
	spi_un_init(); // Stop using the SPI bus
	return value;
} // End PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void MFRC522_PCD_ReadRegister_Many(	MFRC522_PCD_Register reg,	///< The register to read from. One of the PCD_Register enums.
								uint8_t count,			///< The number of bytes to read
								uint8_t *values,		///< Byte array to store the values in.
								uint8_t rxAlign		///< Only bit positions rxAlign..7 in values[0] are updated.
								) {
	if (count == 0) {
		return;
	}
	//Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
	uint8_t address = 0x80 | reg;				// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	uint8_t index = 0;							// Index in values array.
	spi_master_init(MFRC522_spi_settings);	// Set the settings to work with SPI bus
	MFRC522_ENABLE_CS();		// Select slave
	count--;								// One read is performed outside of the loop
	spi_transceiver(address);					// Tell MFRC522 which address we want to read
	if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		uint8_t mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
		uint8_t value = spi_transceiver(address);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}
	while (index < count) {
		values[index] = spi_transceiver(address);	// Read value and tell that we want to read the same address again.
		index++;
	}
	values[index] = spi_transceiver(0);			// Read the final byte. Send 0 to stop reading.
	MFRC522_DISABLE_CS();			// Release slave again
	spi_un_init(); // Stop using the SPI bus
} // End PCD_ReadRegister_Many()

/**
 * Sets the bits given in mask in register reg.
 */
void MFRC522_PCD_SetRegisterBitMask(	MFRC522_PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
										uint8_t mask			///< The bits to set.
									) { 
	uint8_t tmp;
	tmp = MFRC522_PCD_ReadRegister(reg);
	MFRC522_PCD_WriteRegister(reg, tmp | mask);			// set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void MFRC522_PCD_ClearRegisterBitMask(	MFRC522_PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
										uint8_t mask			///< The bits to clear.
									  ) {
	uint8_t tmp;
	tmp = MFRC522_PCD_ReadRegister(reg);
	MFRC522_PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_PCD_CalculateCRC(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
												uint8_t length,	///< In: The number of bytes to transfer.
												uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 ) {
	MFRC522_PCD_WriteRegister(MFRC522_CommandReg, MFRC522_PCD_Idle);		// Stop any active command.
	MFRC522_PCD_WriteRegister(MFRC522_DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	MFRC522_PCD_WriteRegister(MFRC522_FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	MFRC522_PCD_WriteRegister_Many(MFRC522_FIFODataReg, length, data);	// Write data to the FIFO
	MFRC522_PCD_WriteRegister(MFRC522_CommandReg, MFRC522_PCD_CalcCRC);		// Start the calculation
	
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs @ 16MHz.
	// TODO check/modify for other architectures than Arduino Uno 16bit

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us @ 16MHz.
	uint16_t i;
	for (i = (5000 * (F_CPU / 16000000UL)); i > 0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		uint8_t n = MFRC522_PCD_ReadRegister(MFRC522_DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			MFRC522_PCD_WriteRegister(MFRC522_CommandReg, MFRC522_PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = MFRC522_PCD_ReadRegister(MFRC522_CRCResultRegL);
			result[1] = MFRC522_PCD_ReadRegister(MFRC522_CRCResultRegH);
			return MFRC522_STATUS_OK;
		}
	}
	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return MFRC522_STATUS_TIMEOUT;
} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void MFRC522_PCD_Init() {
	uint8_t hardReset = 0;

	// Set the chipSelectPin as digital output, do not select the slave yet
	MFRC522_SETUP_SPI_CS();
	MFRC522_DISABLE_CS();
	MFRC522_spi_settings.prescaler_divider = 4;
	MFRC522_spi_settings.MSB_first = 1;
	MFRC522_spi_settings.mode = 0;
	
	// First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
	MFRC522_SET_RST_IN();

	if (MFRC522_READ_RST() == 0) {	// The MFRC522 chip is in power down mode.
		MFRC522_SET_RST_OUT();		// Now set the resetPowerDownPin as digital output.
		MFRC522_RST_HIGH();		// Exit power down mode. This triggers a hard reset.
		// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: delay for 50.24ms.
		main_timer_clear();
		while(main_timer_now() < 1570);
		hardReset = 1;
	}

	if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
		MFRC522_PCD_Reset();
	}
	
	// Reset baud rates
	MFRC522_PCD_WriteRegister(MFRC522_TxModeReg, 0x00);
	MFRC522_PCD_WriteRegister(MFRC522_RxModeReg, 0x00);
	// Reset ModWidthReg

	MFRC522_PCD_WriteRegister(MFRC522_ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	MFRC522_PCD_WriteRegister(MFRC522_TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	MFRC522_PCD_WriteRegister(MFRC522_TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	MFRC522_PCD_WriteRegister(MFRC522_TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	MFRC522_PCD_WriteRegister(MFRC522_TReloadRegL, 0xE8);
	
	MFRC522_PCD_WriteRegister(MFRC522_TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	MFRC522_PCD_WriteRegister(MFRC522_ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	MFRC522_PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void MFRC522_PCD_Reset() {
	MFRC522_PCD_WriteRegister(MFRC522_CommandReg, MFRC522_PCD_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	uint8_t count = 0;
	uint64_t t;
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		t = main_timer_now();
		while((main_timer_now() - t) < 1570);	//50.24ms delay
	} while ((MFRC522_PCD_ReadRegister(MFRC522_CommandReg) & (1 << 4)) && (++count) < 3);
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void MFRC522_PCD_AntennaOn() {
	uint8_t value = MFRC522_PCD_ReadRegister(MFRC522_TxControlReg);
	if ((value & 0x03) != 0x03) {
		MFRC522_PCD_WriteRegister(MFRC522_TxControlReg, value | 0x03);
	}
} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void MFRC522_PCD_AntennaOff() {
	MFRC522_PCD_ClearRegisterBitMask(MFRC522_TxControlReg, 0x03);
} // End PCD_AntennaOff()

/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b=0x70 as RCFfgReg may use reserved bits.
 * 
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
uint8_t MFRC522_PCD_GetAntennaGain() {
	return MFRC522_PCD_ReadRegister(MFRC522_RFCfgReg) & (0x70);
} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b=0x70 as RCFfgReg may use reserved bits.
 */
void MFRC522_PCD_SetAntennaGain(uint8_t mask) {
	if (MFRC522_PCD_GetAntennaGain() != mask) {						// only bother if there is a change
		MFRC522_PCD_ClearRegisterBitMask(MFRC522_RFCfgReg, 0x70);		// clear needed to allow 000 pattern
		MFRC522_PCD_SetRegisterBitMask(MFRC522_RFCfgReg, mask & 0x70);	// only set RxGain[2:0] bits
	}
} // End PCD_SetAntennaGain()

/**
 * Performs a self-test of the MFRC522
 * See 16.1.1 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * 
 * @return Whether or not the test passed. Or false if no firmware reference is available.
 */
uint8_t MFRC522_PCD_PerformSelfTest() {
	// This follows directly the steps outlined in 16.1.1
	// 1. Perform a soft reset.
	MFRC522_PCD_Reset();
	
	// 2. Clear the internal buffer by writing 25 bytes of 00h
	uint8_t ZEROES[25] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						  0x00, 0x00, 0x00, 0x00, 0x00
						};
	MFRC522_PCD_WriteRegister(MFRC522_FIFOLevelReg, 0x80);		// flush the FIFO buffer
	MFRC522_PCD_WriteRegister_Many(MFRC522_FIFODataReg, 25, ZEROES);	// write 25 bytes of 00h to FIFO
	MFRC522_PCD_WriteRegister(MFRC522_CommandReg, MFRC522_PCD_Mem);		// transfer to internal buffer
	
	// 3. Enable self-test
	MFRC522_PCD_WriteRegister(MFRC522_AutoTestReg, 0x09);
	
	// 4. Write 00h to FIFO buffer
	MFRC522_PCD_WriteRegister(MFRC522_FIFODataReg, 0x00);
	
	// 5. Start self-test by issuing the CalcCRC command
	MFRC522_PCD_WriteRegister(MFRC522_CommandReg, MFRC522_PCD_CalcCRC);
	
	// 6. Wait for self-test to complete
	uint8_t n, i;
	for (i = 0; i < 0xFF; i++) {
		// The datasheet does not specify exact completion condition except
		// that FIFO buffer should contain 64 bytes.
		// While selftest is initiated by CalcCRC command
		// it behaves differently from normal CRC computation,
		// so one can't reliably use DivIrqReg to check for completion.
		// It is reported that some devices does not trigger CRCIRq flag
		// during selftest.
		n = MFRC522_PCD_ReadRegister(MFRC522_FIFOLevelReg);
		if (n >= 64) {
			break;
		}
	}
	MFRC522_PCD_WriteRegister(MFRC522_CommandReg, MFRC522_PCD_Idle);		// Stop calculating CRC for new content in the FIFO.
	
	// 7. Read out resulting 64 bytes from the FIFO buffer.
	uint8_t result[64];
	MFRC522_PCD_ReadRegister_Many(MFRC522_FIFODataReg, 64, result, 0);
	
	// Auto self-test done
	// Reset AutoTestReg register to be 0 again. Required for normal operation.
	MFRC522_PCD_WriteRegister(MFRC522_AutoTestReg, 0x00);
	
	// Determine firmware version (see section 9.3.4.8 in spec)
	uint8_t version = MFRC522_PCD_ReadRegister(MFRC522_VersionReg);
	
	// Pick the appropriate reference values
	const uint8_t *reference;
	switch (version) {
		case 0x88:	// Fudan Semiconductor FM17522 clone
			reference = FM17522_firmware_reference;
			break;
		case 0x90:	// Version 0.0
			reference = MFRC522_firmware_referenceV0_0;
			break;
		case 0x91:	// Version 1.0
			reference = MFRC522_firmware_referenceV1_0;
			break;
		case 0x92:	// Version 2.0
			reference = MFRC522_firmware_referenceV2_0;
			break;
		default:	// Unknown version
			return 0; // abort test
	}
	
	// Verify that the results match up to our expectations
	for (i = 0; i < 64; i++) {
		if (result[i] != pgm_read_byte(&(reference[i]))) {
			return 0;
		}
	}
	
	// Test passed; all is good.
	return 1;
} // End PCD_PerformSelfTest()

/////////////////////////////////////////////////////////////////////////////////////
// Power control
/////////////////////////////////////////////////////////////////////////////////////

//IMPORTANT NOTE!!!!
//Calling any other function that uses CommandReg will disable soft power down mode !!!
//For more details about power control, refer to the datasheet - page 33 (8.6)

void MFRC522_PCD_SoftPowerDown(){//Note : Only soft power down mode is available throught software
	uint8_t val = MFRC522_PCD_ReadRegister(MFRC522_CommandReg); // Read state of the command register 
	val |= (1<<4);// set PowerDown bit ( bit 4 ) to 1 
	MFRC522_PCD_WriteRegister(MFRC522_CommandReg, val);//write new value to the command register
}

void MFRC522_PCD_SoftPowerUp(){
	uint8_t val = MFRC522_PCD_ReadRegister(MFRC522_CommandReg); // Read state of the command register 
	val &= ~(1<<4);// set PowerDown bit ( bit 4 ) to 0 
	MFRC522_PCD_WriteRegister(MFRC522_CommandReg, val);//write new value to the command register
	// wait until PowerDown bit is cleared (this indicates end of wake up procedure) 
	const uint64_t timeout = main_timer_now() + 15625UL;// create timer for 500ms timeout (just in case) 
	
	while(main_timer_now() <= timeout){ // set timeout to 500 ms 
		val = MFRC522_PCD_ReadRegister(MFRC522_CommandReg);// Read state of the command register
		if(!(val & (1<<4))){ // if powerdown bit is 0 
			break;// wake up procedure is finished 
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
													uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
													uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
													uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
													uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													uint8_t checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return MFRC522_PCD_CommunicateWithPICC(MFRC522_PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_PCD_CommunicateWithPICC(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
														uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
														uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
														uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
														uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
														uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														uint8_t checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	MFRC522_PCD_WriteRegister(MFRC522_CommandReg, MFRC522_PCD_Idle);			// Stop any active command.
	MFRC522_PCD_WriteRegister(MFRC522_ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	MFRC522_PCD_WriteRegister(MFRC522_FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
	MFRC522_PCD_WriteRegister_Many(MFRC522_FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	MFRC522_PCD_WriteRegister(MFRC522_BitFramingReg, bitFraming);		// Bit adjustments
	MFRC522_PCD_WriteRegister(MFRC522_CommandReg, command);				// Execute the command
	if (command == MFRC522_PCD_Transceive) {
		MFRC522_PCD_SetRegisterBitMask(MFRC522_BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	
	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86μs @ 16MHz.
	// TODO check/modify for other architectures than Arduino Uno 16bit
	uint16_t i;
	for (i = (2000 * (F_CPU / 16000000UL)); i > 0; i--) {
		uint8_t n = MFRC522_PCD_ReadRegister(MFRC522_ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return MFRC522_STATUS_TIMEOUT;
		}
	}
	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if (i == 0) {
		return MFRC522_STATUS_TIMEOUT;
	}
	
	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = MFRC522_PCD_ReadRegister(MFRC522_ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return MFRC522_STATUS_ERROR;
	}
  
	uint8_t temp_validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		uint8_t n = MFRC522_PCD_ReadRegister(MFRC522_FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return MFRC522_STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		MFRC522_PCD_ReadRegister_Many(MFRC522_FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		temp_validBits = MFRC522_PCD_ReadRegister(MFRC522_ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = temp_validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return MFRC522_STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && temp_validBits == 4) {
			return MFRC522_STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || temp_validBits != 0) {
			return MFRC522_STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		MFRC522_StatusCode status = MFRC522_PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != MFRC522_STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return MFRC522_STATUS_CRC_WRONG;
		}
	}
	
	return MFRC522_STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_PICC_RequestA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return MFRC522_PICC_REQA_or_WUPA(MFRC522_PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_PICC_WakeupA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return MFRC522_PICC_REQA_or_WUPA(MFRC522_PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
MFRC522_StatusCode MFRC522_PICC_REQA_or_WUPA(	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
												uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
												uint8_t *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											) {
	uint8_t validBits;
	MFRC522_StatusCode status;
	
	if (bufferATQA == 0 || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return MFRC522_STATUS_NO_ROOM;
	}
	MFRC522_PCD_ClearRegisterBitMask(MFRC522_CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = MFRC522_PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, 0);
	if (status != MFRC522_STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return MFRC522_STATUS_ERROR;
	}
	return MFRC522_STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
											uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										 ) {
	uint8_t uidComplete;		//bool
	uint8_t selectDone;			//bool
	uint8_t useCascadeTag;		//bool
	uint8_t cascadeLevel = 1;
	MFRC522_StatusCode result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	uint8_t *responseBuffer;
	uint8_t responseLength;
	
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return MFRC522_STATUS_INVALID;
	}
	
	// Prepare MFRC522
	MFRC522_PCD_ClearRegisterBitMask(MFRC522_CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = 0;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = MFRC522_PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = MFRC522_PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = MFRC522_PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = 0;						// Never used in CL3.
				break;
			
			default:
				return MFRC522_STATUS_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = MFRC522_PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = 0;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = MFRC522_PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != MFRC522_STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= byteCount(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			MFRC522_PCD_WriteRegister(MFRC522_BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = MFRC522_PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, 0);
			if (result == MFRC522_STATUS_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg = MFRC522_PCD_ReadRegister(MFRC522_CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return MFRC522_STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return MFRC522_STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != MFRC522_STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = 1; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == MFRC522_PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == MFRC522_PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return MFRC522_STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = MFRC522_PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != MFRC522_STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return MFRC522_STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = 1;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return MFRC522_STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
MFRC522_StatusCode MFRC522_PICC_HaltA() {
	MFRC522_StatusCode result;
	uint8_t buffer[4];
	
	// Build command buffer
	buffer[0] = MFRC522_PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = MFRC522_PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	
	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HALTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = MFRC522_PCD_TransceiveData(buffer, byteCount(buffer), 0, 0, 0, 0, 0);
	if (result == MFRC522_STATUS_TIMEOUT) {
		return MFRC522_STATUS_OK;
	}
	if (result == MFRC522_STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return MFRC522_STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 * 
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
MFRC522_StatusCode MFRC522_PCD_Authenticate(uint8_t command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
											uint8_t blockAddr, 	///< The block number. See numbering in the comments in the .h file.
											MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
											Uid *uid			///< Pointer to Uid struct. The first 4 bytes of the UID is used.
											) {
	uint8_t waitIRq = 0x10;		// IdleIRq
	
	// Build command buffer
	uint8_t sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	uint8_t i;
	for (i = 0; i < MFRC522_MF_KEY_SIZE; i++) {	// 6 key bytes
		sendData[2+i] = key->keyByte[i];
	}
	// Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
	// section 3.2.5 "MIFARE Classic Authentication".
	// The only missed case is the MF1Sxxxx shortcut activation,
	// but it requires cascade tag (CT) byte, that is not part of uid.
	for (i = 0; i < 4; i++) {				// The last 4 bytes of the UID
		sendData[8+i] = uid->uidByte[i+uid->size-4];
	}
	
	// Start the authentication.
	return MFRC522_PCD_CommunicateWithPICC(MFRC522_PCD_MFAuthent, waitIRq, &sendData[0], byteCount(sendData), 0, 0, 0, 0, 0);
} // End PCD_Authenticate()

/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void MFRC522_PCD_StopCrypto1() {
	// Clear MFCrypto1On bit
	MFRC522_PCD_ClearRegisterBitMask(MFRC522_Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 * 
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_MIFARE_Read(	uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
											uint8_t *buffer,		///< The buffer to store the data in
											uint8_t *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
										) {
	MFRC522_StatusCode result;
	
	// Sanity check
	if (buffer == 0 || *bufferSize < 18) {
		return MFRC522_STATUS_NO_ROOM;
	}
	
	// Build command buffer
	buffer[0] = MFRC522_PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = MFRC522_PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	
	// Transmit the buffer and receive the response, validate CRC_A.
	return MFRC522_PCD_TransceiveData(buffer, 4, buffer, bufferSize, 0, 0, 1);
} // End MIFARE_Read()

/**
 * Writes 16 bytes to the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
 * Even though 16 bytes are transferred to the Ultralight PICC, only the least significant 4 bytes (bytes 0 to 3)
 * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
 * * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_MIFARE_Write(	uint8_t blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
											uint8_t *buffer,	///< The 16 bytes to write to the PICC
											uint8_t bufferSize	///< Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
										) {
	MFRC522_StatusCode result;
	
	// Sanity check
	if (buffer == 0 || bufferSize < 16) {
		return MFRC522_STATUS_INVALID;
	}
	
	// Mifare Classic protocol requires two communications to perform a write.
	// Step 1: Tell the PICC we want to write to block blockAddr.
	uint8_t cmdBuffer[2];
	cmdBuffer[0] = MFRC522_PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	result = MFRC522_PCD_MIFARE_Transceive(cmdBuffer, 2, 0); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	
	// Step 2: Transfer the data
	result = MFRC522_PCD_MIFARE_Transceive(buffer, bufferSize, 0); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	
	return MFRC522_STATUS_OK;
} // End MIFARE_Write()

/**
 * Writes a 4 byte page to the active MIFARE Ultralight PICC.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_MIFARE_Ultralight_Write(	uint8_t page, 		///< The page (2-15) to write to.
														uint8_t *buffer,	///< The 4 bytes to write to the PICC
														uint8_t bufferSize	///< Buffer size, must be at least 4 bytes. Exactly 4 bytes are written.
													) {
	MFRC522_StatusCode result;
	
	// Sanity check
	if (buffer == 0 || bufferSize < 4) {
		return MFRC522_STATUS_INVALID;
	}
	
	// Build commmand buffer
	uint8_t cmdBuffer[6];
	cmdBuffer[0] = MFRC522_PICC_CMD_UL_WRITE;
	cmdBuffer[1] = page;
	//memcpy(&cmdBuffer[2], buffer, 4);
	uint8_t i;
	for (i = 0; i < 4; i++){
		cmdBuffer[2+i] = buffer[i];
	}
	
	// Perform the write
	result = MFRC522_PCD_MIFARE_Transceive(cmdBuffer, 6, 0); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	return MFRC522_STATUS_OK;
} // End MIFARE_Ultralight_Write()

/**
 * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_MIFARE_Decrement(	uint8_t blockAddr, ///< The block (0-0xff) number.
												int32_t delta		///< This number is subtracted from the value of block blockAddr.
											) {
	return MFRC522_MIFARE_TwoStepHelper(MFRC522_PICC_CMD_MF_DECREMENT, blockAddr, delta);
} // End MIFARE_Decrement()

/**
 * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_MIFARE_Increment(	uint8_t blockAddr, ///< The block (0-0xff) number.
												int32_t delta		///< This number is added to the value of block blockAddr.
											) {
	return MFRC522_MIFARE_TwoStepHelper(MFRC522_PICC_CMD_MF_INCREMENT, blockAddr, delta);
} // End MIFARE_Increment()

/**
 * MIFARE Restore copies the value of the addressed block into a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_MIFARE_Restore(	uint8_t blockAddr ///< The block (0-0xff) number.
											) {
	// The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
	// Doing only a single step does not work, so I chose to transfer 0L in step two.
	return MFRC522_MIFARE_TwoStepHelper(MFRC522_PICC_CMD_MF_RESTORE, blockAddr, 0L);
} // End MIFARE_Restore()

/**
 * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_MIFARE_TwoStepHelper(	uint8_t command,	///< The command to use
													uint8_t blockAddr,	///< The block (0-0xff) number.
													int32_t data		///< The data to transfer in step 2
													) {
	MFRC522_StatusCode result;
	uint8_t cmdBuffer[2]; // We only need room for 2 bytes.
	
	// Step 1: Tell the PICC the command and block address
	cmdBuffer[0] = command;
	cmdBuffer[1] = blockAddr;
	result = MFRC522_PCD_MIFARE_Transceive(	cmdBuffer, 2, 0); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	
	// Step 2: Transfer the data
	result = MFRC522_PCD_MIFARE_Transceive(	(uint8_t *)&data, 4, 1); // Adds CRC_A and accept timeout as success.
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	
	return MFRC522_STATUS_OK;
} // End MIFARE_TwoStepHelper()

/**
 * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_MIFARE_Transfer(	uint8_t blockAddr ///< The block (0-0xff) number.
											) {
	MFRC522_StatusCode result;
	uint8_t cmdBuffer[2]; // We only need room for 2 bytes.
	
	// Tell the PICC we want to transfer the result into block blockAddr.
	cmdBuffer[0] = MFRC522_PICC_CMD_MF_TRANSFER;
	cmdBuffer[1] = blockAddr;
	result = MFRC522_PCD_MIFARE_Transceive(	cmdBuffer, 2, 0); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	return MFRC522_STATUS_OK;
} // End MIFARE_Transfer()

/**
 * Helper routine to read the current value from a Value Block.
 * 
 * Only for MIFARE Classic and only for blocks in "value block" mode, that
 * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
 * the block must be authenticated before calling this function. 
 * 
 * @param[in]   blockAddr   The block (0x00-0xff) number.
 * @param[out]  value       Current value of the Value Block.
 * @return STATUS_OK on success, STATUS_??? otherwise.
  */
MFRC522_StatusCode MFRC522_MIFARE_GetValue(uint8_t blockAddr, int32_t *value) {
	MFRC522_StatusCode status;
	uint8_t buffer[18];
	uint8_t size = byteCount(buffer);
	
	// Read the block
	status = MFRC522_MIFARE_Read(blockAddr, buffer, &size);
	if (status == MFRC522_STATUS_OK) {
		// Extract the value
		*value = ((int32_t)(buffer[3])<<24) | ((int32_t)(buffer[2])<<16) | ((int32_t)(buffer[1])<<8) | ((int32_t)(buffer[0]));
	}
	return status;
} // End MIFARE_GetValue()

/**
 * Helper routine to write a specific value into a Value Block.
 * 
 * Only for MIFARE Classic and only for blocks in "value block" mode, that
 * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
 * the block must be authenticated before calling this function. 
 * 
 * @param[in]   blockAddr   The block (0x00-0xff) number.
 * @param[in]   value       New value of the Value Block.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_MIFARE_SetValue(uint8_t blockAddr, int32_t value) {
	uint8_t buffer[18];
	
	// Translate the int32_t into 4 bytes; repeated 2x in value block
	buffer[0] = buffer[ 8] = (value & 0xFF);
	buffer[1] = buffer[ 9] = (value & 0xFF00) >> 8;
	buffer[2] = buffer[10] = (value & 0xFF0000) >> 16;
	buffer[3] = buffer[11] = (value & 0xFF000000) >> 24;
	// Inverse 4 bytes also found in value block
	buffer[4] = ~buffer[0];
	buffer[5] = ~buffer[1];
	buffer[6] = ~buffer[2];
	buffer[7] = ~buffer[3];
	// Address 2x with inverse address 2x
	buffer[12] = buffer[14] = blockAddr;
	buffer[13] = buffer[15] = ~blockAddr;
	
	// Write the whole data block
	return MFRC522_MIFARE_Write(blockAddr, buffer, 16);
} // End MIFARE_SetValue()

/**
 * Authenticate with a NTAG216.
 * 
 * Only for NTAG216. First implemented by Gargantuanman.
 * 
 * @param[in]   passWord   password.
 * @param[in]   pACK       result success???.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_PCD_NTAG216_AUTH(uint8_t* passWord, uint8_t pACK[]) //Authenticate with 32bit password
{
	// TODO: Fix cmdBuffer length and rxlength. They really should match.
	//       (Better still, rxlength should not even be necessary.)

	MFRC522_StatusCode result;
	uint8_t cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.
	
	cmdBuffer[0] = 0x1B; //Comando de autentificacion
	
	uint8_t i;
	for (i = 0; i < 4; i++)
		cmdBuffer[i+1] = passWord[i];
	
	result = MFRC522_PCD_CalculateCRC(cmdBuffer, 5, &cmdBuffer[5]);
	
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	
	// Transceive the data, store the reply in cmdBuffer[]
	uint8_t waitIRq		= 0x30;	// RxIRq and IdleIRq
	//	byte cmdBufferSize	= byteCount(cmdBuffer);
	uint8_t validBits		= 0;
	uint8_t rxlength		= 5;
	result = MFRC522_PCD_CommunicateWithPICC(MFRC522_PCD_Transceive, waitIRq, cmdBuffer, 7, cmdBuffer, &rxlength, &validBits, 0, 0);
	
	pACK[0] = cmdBuffer[0];
	pACK[1] = cmdBuffer[1];
	
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	
	return MFRC522_STATUS_OK;
} // End PCD_NTAG216_AUTH()


/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
MFRC522_StatusCode MFRC522_PCD_MIFARE_Transceive(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
													uint8_t sendLen,		///< Number of bytes in sendData.
													uint8_t acceptTimeout	///< True => A timeout is also success
												) {
	MFRC522_StatusCode result;
	uint8_t cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.
	
	// Sanity check
	if (sendData == 0 || sendLen > 16) {
		return MFRC522_STATUS_INVALID;
	}
	
	// Copy sendData[] to cmdBuffer[] and add CRC_A
	//memcpy(cmdBuffer, sendData, sendLen);
	uint8_t i;
	for (i = 0; i < sendLen; i++){
		cmdBuffer[i] = sendData[i];
	}
	result = MFRC522_PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
	if (result != MFRC522_STATUS_OK) { 
		return result;
	}
	sendLen += 2;
	
	// Transceive the data, store the reply in cmdBuffer[]
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	uint8_t cmdBufferSize = byteCount(cmdBuffer);
	uint8_t validBits = 0;
	result = MFRC522_PCD_CommunicateWithPICC(MFRC522_PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits, 0, 0);
	if (acceptTimeout && result == MFRC522_STATUS_TIMEOUT) {
		return MFRC522_STATUS_OK;
	}
	if (result != MFRC522_STATUS_OK) {
		return result;
	}
	// The PICC must reply with a 4 bit ACK
	if (cmdBufferSize != 1 || validBits != 4) {
		return MFRC522_STATUS_ERROR;
	}
	if (cmdBuffer[0] != MFRC522_MF_ACK) {
		return MFRC522_STATUS_MIFARE_NACK;
	}
	return MFRC522_STATUS_OK;
} // End PCD_MIFARE_Transceive()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 * 
 * @return PICC_Type
 */
static MFRC522_PICC_Type MFRC522_PICC_GetType(uint8_t sak		///< The SAK byte returned from PICC_Select().
										) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf 
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return MFRC522_PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return MFRC522_PICC_TYPE_MIFARE_MINI;
		case 0x08:	return MFRC522_PICC_TYPE_MIFARE_1K;
		case 0x18:	return MFRC522_PICC_TYPE_MIFARE_4K;
		case 0x00:	return MFRC522_PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return MFRC522_PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return MFRC522_PICC_TYPE_TNP3XXX;
		case 0x20:	return MFRC522_PICC_TYPE_ISO_14443_4;
		case 0x40:	return MFRC522_PICC_TYPE_ISO_18092;
		default:	return MFRC522_PICC_TYPE_UNKNOWN;
	}
} // End PICC_GetType()

/**
 * Saves the status code name to a char* passed in as an argument.
 * 
 * @return void
 */
static void MFRC522_GetStatusCodeName(MFRC522_StatusCode code,	///< One of the StatusCode enums.
										char * str		//String to hold the status code name
										) {
	switch (code){
		case MFRC522_STATUS_OK:
			str = "OK";
			break;
		case MFRC522_STATUS_ERROR:
			str = "ERROR";
			break;
		case MFRC522_STATUS_COLLISION:
			str = "COLLISION";
			break;
		case MFRC522_STATUS_TIMEOUT:
			str = "TIMEOUT";
			break;
		case MFRC522_STATUS_NO_ROOM:
			str = "NO_ROOM";
			break;
		case MFRC522_STATUS_INTERNAL_ERROR:
			str = "INTERNAL_ERROR";
			break;
		case MFRC522_STATUS_INVALID:
			str = "INVALID";
			break;
		case MFRC522_STATUS_CRC_WRONG:
			str = "CRC_WRONG";
			break;
		case MFRC522_STATUS_MIFARE_NACK:
			str = "MIFARE_NACK";
			break;
	}
} // End GetStatusCodeName()

/**
 * Saves the PICC type name to a char* passed in as an argument.
 * 
 * @return void
 */
static void MFRC522_PICC_GetTypeName(MFRC522_PICC_Type piccType,	///< One of the PICC_Type enums.
										char * str		//String to hold the PICC type name
													) {
	switch (piccType){
		case MFRC522_PICC_TYPE_UNKNOWN:
			str = "UNKNOWN";
			break;
		case MFRC522_PICC_TYPE_ISO_14443_4:
			str = "ISO_14443_4";
			break;
		case MFRC522_PICC_TYPE_ISO_18092:
			str = "ISO_18092";
			break;
		case MFRC522_PICC_TYPE_MIFARE_MINI:
			str = "MIFARE_MINI";
			break;
		case MFRC522_PICC_TYPE_MIFARE_1K:
			str = "MIFARE_1K";
			break;
		case MFRC522_PICC_TYPE_MIFARE_4K:
			str = "MIFARE_4K";
			break;
		case MFRC522_PICC_TYPE_MIFARE_UL:
			str = "MIFARE_UL";
			break;
		case MFRC522_PICC_TYPE_MIFARE_PLUS:
			str = "MIFARE_PLUS";
			break;
		case MFRC522_PICC_TYPE_MIFARE_DESFIRE:
			str = "MIFARE_DESFIRE";
			break;
		case MFRC522_PICC_TYPE_TNP3XXX:
			str = "TNP3XXX";
			break;
		case MFRC522_PICC_TYPE_NOT_COMPLETE:
			str = "NOT_COMPLETE";
			break;
	}
} // End PICC_GetTypeName()

/**
 * Dumps debug info about the connected PCD to LCD.
 * Shows all known firmware versions
 */
void MFRC522_PCD_DumpVersionToOutput() {
	// Get the MFRC522 firmware version
	uint8_t v = MFRC522_PCD_ReadRegister(MFRC522_VersionReg);
	output_heading("Firmware Version");
	output_hex(v, 1);
	// Lookup which version
	switch(v) {
		case 0x88: lcd_print(" = (clone)");  break;
		case 0x90: lcd_print(" = v0.0");     break;
		case 0x91: lcd_print(" = v1.0");     break;
		case 0x92: lcd_print(" = v2.0");     break;
		default:   lcd_print(" = (unknown)");
	}
	output_hold_delay();
	// When 0x00 or 0xFF is returned, communication probably failed
	if ((v == 0x00) || (v == 0xFF)){
		output_heading("WARNING:");
		lcd_print("COMM FAILURE");
		output_hold_delay();
	}
} // End PCD_DumpVersionToOutput()

/**
 * Dumps debug info about the selected PICC to LCD.
 * On success the PICC is halted after dumping the data.
 * For MIFARE Classic the factory default key of 0xFFFFFFFFFFFF is tried.  
 *
 * @DEPRECATED Kept for bakward compatibility
 */
void MFRC522_PICC_DumpToOutput(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
								) {
	MIFARE_Key key;
	
	// Dump UID, SAK and Type
	MFRC522_PICC_DumpDetailsToOutput(uid);
	
	// Dump contents
	MFRC522_PICC_Type piccType = MFRC522_PICC_GetType(uid->sak);
	uint8_t i;
	switch (piccType) {
		case MFRC522_PICC_TYPE_MIFARE_MINI:
		case MFRC522_PICC_TYPE_MIFARE_1K:
		case MFRC522_PICC_TYPE_MIFARE_4K:
			// All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
			for (i = 0; i < 6; i++) {
				key.keyByte[i] = 0xFF;
			}
			MFRC522_PICC_DumpMifareClassicToOutput(uid, piccType, &key);
			break;
			
		case MFRC522_PICC_TYPE_MIFARE_UL:
		case MFRC522_PICC_TYPE_ISO_14443_4:
		case MFRC522_PICC_TYPE_MIFARE_DESFIRE:
		case MFRC522_PICC_TYPE_ISO_18092:
		case MFRC522_PICC_TYPE_MIFARE_PLUS:
		case MFRC522_PICC_TYPE_TNP3XXX:
			output_heading("ContentDmp: This");
			lcd_print("PICC no support");
			output_hold_delay();
			break;
			
		case MFRC522_PICC_TYPE_UNKNOWN:
		case MFRC522_PICC_TYPE_NOT_COMPLETE:
		default:
			break; // No memory dump here
	}
	
	MFRC522_PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
} // End PICC_DumpToSerial()

/**
 * Dumps card info (UID,SAK,Type) about the selected PICC to LCD.
 *
 * @DEPRECATED kept for backward compatibility
 */
void MFRC522_PICC_DumpDetailsToOutput(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
									) {
	// UID
	output_heading("Card UID:");
	uint8_t i;
	for (i = 0; i < uid->size; i++) {
		output_hex(uid->uidByte[i], 1);
	}
	output_hold_delay();
	
	// SAK
	output_heading("Card SAK:");
	output_hex(uid->sak, 1);
	output_hold_delay();
	
	// (suggested) PICC type
	char type [15];
	MFRC522_PICC_Type piccType = MFRC522_PICC_GetType(uid->sak);
	output_heading("PICC type:");
	MFRC522_PICC_GetTypeName(piccType, type);
	lcd_print(type);
	output_hold_delay();
} // End PICC_DumpDetailsToSerial()

/**
 * Dumps memory contents of a MIFARE Classic PICC.
 * On success the PICC is halted after dumping the data.
 */
void MFRC522_PICC_DumpMifareClassicToOutput(	Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
												MFRC522_PICC_Type piccType,	///< One of the PICC_Type enums.
												MIFARE_Key *key		///< Key A used for all sectors.
											) {
	uint8_t no_of_sectors = 0;
	switch (piccType) {
		case MFRC522_PICC_TYPE_MIFARE_MINI:
			// Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
			no_of_sectors = 5;
			break;
			
		case MFRC522_PICC_TYPE_MIFARE_1K:
			// Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
			no_of_sectors = 16;
			break;
			
		case MFRC522_PICC_TYPE_MIFARE_4K:
			// Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
			no_of_sectors = 40;
			break;
			
		default: // Should not happen. Ignore.
			break;
	}
	
	// Dump sectors, highest address first.
	if (no_of_sectors) {
		output_heading("Sector Read");
		output_hold_delay();
		for (int8_t i = no_of_sectors - 1; i >= 0; i--) {
			MFRC522_PICC_DumpMifareClassicSectorToOutput(uid, key, i);
		}
	}
	MFRC522_PICC_HaltA(); // Halt the PICC before stopping the encrypted session.
	MFRC522_PCD_StopCrypto1();
} // End PICC_DumpMifareClassicToSerial()

/**
 * Dumps memory contents of a sector of a MIFARE Classic PICC.
 * Uses PCD_Authenticate(), MIFARE_Read() and PCD_StopCrypto1.
 * Always uses PICC_CMD_MF_AUTH_KEY_A because only Key A can always read the sector trailer access bits.
 */
void MFRC522_PICC_DumpMifareClassicSectorToOutput(Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
													MIFARE_Key *key,	///< Key A for the sector.
													uint8_t sector			///< The sector to dump, 0..39.
													) {
	MFRC522_StatusCode status;
	uint8_t firstBlock;		// Address of lowest address to dump actually last block dumped)
	uint8_t no_of_blocks;		// Number of blocks in sector
	uint8_t isSectorTrailer;	// Set to true while handling the "last" (ie highest address) in the sector.
	
	// The access bits are stored in a peculiar fashion.
	// There are four groups:
	//		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
	//		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
	//		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
	//		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
	// Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
	// The four CX bits are stored together in a nible cx and an inverted nible cx_.
	uint8_t c1, c2, c3;		// Nibbles
	uint8_t c1_, c2_, c3_;		// Inverted nibbles
	uint8_t invertedError;		// True if one of the inverted nibbles did not match
	uint8_t g[4];				// Access bits for each of the four groups.
	uint8_t group;				// 0-3 - active group for access bits
	uint8_t firstInGroup;		// True for the first block dumped in the group
	
	// Determine position and size of sector.
	if (sector < 32) { // Sectors 0..31 has 4 blocks each
		no_of_blocks = 4;
		firstBlock = sector * no_of_blocks;
	}
	else if (sector < 40) { // Sectors 32-39 has 16 blocks each
		no_of_blocks = 16;
		firstBlock = 128 + (sector - 32) * no_of_blocks;
	}
	else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
		return;
	}
		
	// Dump blocks, highest address first.
	uint8_t byteCount;
	uint8_t buffer[18];
	uint8_t blockAddr;
	isSectorTrailer = 1;
	invertedError = 0;	// Avoid "unused variable" warning.
	int8_t blockOffset;
	for (blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) {
		blockAddr = firstBlock + blockOffset;
		// Sector number - only on first line
		if (isSectorTrailer) {
			output_heading("Sector:");
			output_number(sector);
			output_hold_delay();
		}
		// Establish encrypted communications before reading the first block
		if (isSectorTrailer) {
			status = MFRC522_PCD_Authenticate(MFRC522_PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
			if (status != MFRC522_STATUS_OK) {
				output_heading("PCD_AUTH failure");
				char tmp[15];
				MFRC522_GetStatusCodeName(status, tmp);
				lcd_print(tmp);
				output_hold_delay();
				return;
			}
		}
		// Read block
		byteCount = byteCount(buffer);
		status = MFRC522_MIFARE_Read(blockAddr, buffer, &byteCount);
		if (status != MFRC522_STATUS_OK) {
			output_heading("Read failure");
			char tmp[15];
			MFRC522_GetStatusCodeName(status, tmp);
			lcd_print(tmp);
			output_hold_delay();
			continue;
		}
		// Dump data
		uint8_t index;
		lcd_clear();
		lcd_first_line();
		for (index = 0; index < 16; index++) {
			if(index == 8)
				lcd_second_line();
			output_hex(buffer[index], 0);
		}
		output_hold_delay();
		// Parse sector trailer data
		if (isSectorTrailer) {
			c1  = buffer[7] >> 4;
			c2  = buffer[8] & 0xF;
			c3  = buffer[8] >> 4;
			c1_ = buffer[6] & 0xF;
			c2_ = buffer[6] >> 4;
			c3_ = buffer[7] & 0xF;
			invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
			g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
			g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
			g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
			g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
			isSectorTrailer = 0;
		}
		
		// Which access group is this block in?
		if (no_of_blocks == 4) {
			group = blockOffset;
			firstInGroup = 1;
		}
		else {
			group = blockOffset / 5;
			firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
		}
		
		if (firstInGroup) {
			// Print access bits
			lcd_clear();
			lcd_first_line();
			lcd_print("ACC [ ");
			output_number((g[group] >> 2) & 1);
			lcd_second_line();
			output_number((g[group] >> 1) & 1);
			lcd_print(" ");
			output_number((g[group] >> 0) & 1);
			lcd_print(" ] ");
			if (invertedError) {
				lcd_print("E");
			}
			output_hold_delay();
		}
		
		if (group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
			int32_t value = ((int32_t)(buffer[3])<<24) | ((int32_t)(buffer[2])<<16) | ((int32_t)(buffer[1])<<8) | ((int32_t)(buffer[0]));
			lcd_clear();
			lcd_first_line();
			lcd_print(" Value=0x");
			output_hex(value, 0);
			lcd_second_line();
			lcd_print(" Adr=0x");
			output_hex(buffer[12], 0);
			output_hold_delay();
		}
	}
	
	return;
} // End PICC_DumpMifareClassicSectorToSerial()

/**
 * Calculates the bit pattern needed for the specified access bits. In the [C1 C2 C3] tuples C1 is MSB (=4) and C3 is LSB (=1).
 */
void MIFARE_SetAccessBits(	uint8_t *accessBitBuffer,	///< Pointer to byte 6, 7 and 8 in the sector trailer. Bytes [0..2] will be set.
									uint8_t g0,				///< Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
									uint8_t g1,				///< Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
									uint8_t g2,				///< Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
									uint8_t g3					///< Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
								) {
	uint8_t c1 = ((g3 & 4) << 1) | ((g2 & 4) << 0) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
	uint8_t c2 = ((g3 & 2) << 2) | ((g2 & 2) << 1) | ((g1 & 2) << 0) | ((g0 & 2) >> 1);
	uint8_t c3 = ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | ((g0 & 1) << 0);
	
	accessBitBuffer[0] = (~c2 & 0xF) << 4 | (~c1 & 0xF);
	accessBitBuffer[1] =          c1 << 4 | (~c3 & 0xF);
	accessBitBuffer[2] =          c3 << 4 | c2;
} // End MIFARE_SetAccessBits()

/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
uint8_t MFRC522_PICC_IsNewCardPresent() {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = byteCount(bufferATQA);

	// Reset baud rates
	MFRC522_PCD_WriteRegister(MFRC522_TxModeReg, 0x00);
	MFRC522_PCD_WriteRegister(MFRC522_RxModeReg, 0x00);
	// Reset ModWidthReg
	MFRC522_PCD_WriteRegister(MFRC522_ModWidthReg, 0x26);

	MFRC522_StatusCode result = MFRC522_PICC_RequestA(bufferATQA, &bufferSize);
	return (result == MFRC522_STATUS_OK || result == MFRC522_STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
uint8_t MFRC522_PICC_ReadCardSerial() {
	MFRC522_StatusCode result = MFRC522_PICC_Select(&uid, 0);
	return (result == MFRC522_STATUS_OK);
} // End 


#endif		//MFRC522_h
