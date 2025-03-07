//
//                   MSP430G2553
//                 -----------------
//            /|\ |             P2.3|->   ADC !RST (GPIO) [Active Low]
//             |  |                 |
//             ---|RST          P1.5|->   !CS (GPIO) [Active Low]
//                |                 |
//       SCL   -->|P1.6         P2.5|<-   !DRDY (GPIO) [Active Low]
//                |                 |
//       SDA  <-->|P1.7         P2.4|->   START/SYNC (GPIO)
//                |                 |
// I2CAddr_0   -->|P2.0         P1.2|->   Data Out (UCA0SIMO)
//                |                 |
// I2CAddr_1   -->|P2.1         P1.1|<-   Data In (UCA0SOMI)
//                |                 |
// I2CAddr_2   -->|P2.2         P1.4|->   Serial Clock Out (UCA0CLK)
//                |   P2.6   P1.0   |
//                 -----------------
//                      |      |
//                      |      ---->SYNC
//                      V
//                      LED Heartbeat
//********************************************************************************************

#ifndef __TIC_H__

#ifndef __LINUX__
#include <msp430.h>
#else
#define BIT0 (1<<0)
#define BIT1 (1<<1)
#define BIT2 (1<<2)
#define BIT3 (1<<3)
#define BIT4 (1<<4)
#define BIT5 (1<<5)
#define BIT6 (1<<6)
#define BIT7 (1<<7)

/* Other Definitions  */
#define WDTPW 1
#define WDTHOLD 2
#define GIE 1

#endif

#include <stdint.h>

//******************************************************************************
// Definitions and Commands ****************************************************
//******************************************************************************

#define DUMMY     0x00 //Does this need to be 0xFF instead?

#define BAUDRATE_0  0x41  //UCBRA0 byte 0 (baudrate = SMCLK/(UCBRA0+UCBRA1) ) At 8 MHZ this is 9600 baud
#define BAUDRATE_1  0x06  //UCBRA1 byte 1
// sleep times in milliseconds.  CHANGE IF YOU CHANGE THE SA<PLE RATES! 

#define FIRST_CONVERSION_SLEEP 110
#define CONVERSION_SLEEP 51
#define SFOCAL_SLEEP 500
#define RESET_SLEEP 100

#define CONVERSION_DELAY    1000000    //Delay needed after conversion starts or has to restart following changes to DATARATE and INPMUX registers
#define SFOCAL_DELAY        4800000    //Delay needed after self offset calibration
#define RESET_DELAY         1000000    //Delay needed after ADC is reset

#define CS_OUT    P1OUT  //P1.5 CS Pin config
#define CS_DIR    P1DIR
#define CS_PIN    BIT5

#define I2CA_0  BIT0        //I2C Address bits
#define I2CA_1  BIT1
#define I2CA_2  BIT2

#define NOP                 0x00 //No operation
#define START_CMD           0x09 //Start conversion
#define STOP_CMD            0x0B //Stop conversion
#define RDATA_CMD           0x13 //Read data
#define SFOCAL_CMD          0x19 //Self offset calibration

#define CMD_LENGTH          1 //Length of single byte SPI commands

#define RDATA_TX_LENGTH     2 //This is a workaround so that the MSP430 is Tx'ing while
                              //the ADC is sending the previous byte in the data-holding register
#define RDATA_RX_LENGTH     3 //Don't forget this affects the burnout detect code too. Check to make sure
                              //It doesn't get screwed up if this is set to 4 or 5 to accomodate
                              //Status or CRC bytes

#define RREG_TX_LENGTH      3 //Number of bytes transmitted in RREG command
#define RREG_RX_LENGTH      1 //Number of bytes expected to be received from ADC after RREG command (this assumes that the second byte is 0x00)
#define RREG_RX_LENGTH_ALL  10//Number of bytes expected to be received from ADC after RREG command (this assumes that the second byte is 0x0A

#define WREG_TX_LENGTH      3

#define STATUS_WORD_LENGTH  4    // Length of output data status word

#define MAX_BUFFER_SIZE     10   // Maximum size of SPI TX and RX buffers

#define FS_1                0xFF // Full-scale ADC measurement bits 1-3
#define FS_2                0xFF
#define FS_3                0x7F

//******I2C Slave definitions******
//#define SLAVE_ADDR  0x09 //********still need to base this on address pins ********

#define CCMM_RESET_CMD      BIT0    //Bit 0 high requires MSP430 to run reset command
#define CCMM_SFOCAL_CMD     BIT1    //Bit 1 high requires MSP430 to run self offset calibration
#define CCMM_BURNOUT_CMD        BIT2    //Bit 2 high requires MSP430 to run burnout detection

#define CMD_LENGTH          1       //Length of I2C commands sent by CCMM to MSP430

#define CCMM_RX_BUFFER_SIZE 2       //Size of buffer used to store data received from CCMM. Not used
#define CCMM_TX_BUFFER_SIZE 40      //Size of data buffer transmitted to CCMM. Assumes 9 4-bit integers plus 4 byte bitfield


/*Function Declarations*/
void initClockTo8MHz(void);
void initGPIO(void);
void initSPI(void);
void initI2C(void);
/* Write to ADC register(s)
 *
 * cmd: sequence of bytes to execute WREG command. See commands above.
 * tx_count: number of bytes to be transmitted. If one register is being read, this will be
 *           three bytes. Note that the number of bytes that the ADC expects to receive
 *           is determined by the second byte of cmd*/
void WREG(uint8_t *cmd, uint8_t tx_count,uint8_t rx_count);

/*Requests data from the ADC. The ADC will send whatever is in the conversion buffer,
 * which is measured from whatever + and - inputs are defined by the input mux.
 * By default, the ADC should only send 3 bytes unless the status and crc bytes are enabled
 * by setting their respective bits high in the SYS register, in which case it could send 4 or 5
 * bytes. This function only expects 3 bytes, so data will be truncated if the status byte is turned on.*/
uint8_t BW_RDATA(uint8_t * buf);

/*Read Register(s) from ADC
 * cmd: sequence of bytes to execute RREG command. See commands above.
 * tx_count: number of bytes in the command. Reading 1 register needs 3 bytes.
 * rx_count: number of registers that will be read. Note that this is defined
 *           for the ADC by the second byte of cmd. rx_count is the number of
 *           bytes read back by the MSP430*/
void  BW_RREG(uint8_t *cmd, int tx_count);

/*Read and write Register(s) from ADC
 * cmd_buf: sequence of bytes to execute RREG command. See commands above.
 * rcv_buf: receive buffertx_count: number of bytes in the command. Reading 1 register needs 3 bytes.
 * tx_count: bytes to transmit.  must be one more than the number read.
 * rx_count: number of registers that will be read. Note that this is defined
 *           for the ADC by the second byte of cmd. rx_count is the number of
 *           bytes read back by the MSP430*/
void  BW_RWREG(const uint8_t *cmd_buf, uint8_t * rcv_buf, int tx_count, int rx_count);


/*Copy an array into another array, starting with the lowest-value index
 * source:  array that is being copied
 * dest:    destination of copied array
 * count:   how many bytes are to be copied from source to dest */
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count); //Copy source array to dest array

/*Copies and array into another array, starting at the lowest index of the source and a chosen
 *  index of the destination.
 *  dest: destination of copied array
 *  source: array that is being copied
 *  count: how many bytes are to be copied from source to dest
 *  startIndex: index of destination array at which the source array will start being copied.
 */
void CopytoDataBuffer(uint8_t *dest, uint8_t *source, uint8_t count, uint8_t startIndex);

/* Places byte val into the SPI UCA0TX buffer*/
void SendUCA0Data(uint8_t val); //Add val to UCA0TXBUF

/*Performs self offset calibration, burnout detect, and reads registers 0x00 to 0x09 from the ADC*/
void PowerOnSetup();

/*Sends self offset calibration command to ADC and gives a delay for the procedure to finish*/
void SelfCalibration();

/*Sends command to turn on burnout current sources and measures channels 0 through 4. If a channel measurement
 * is EQUAL to the full-scale values defined above, then the corresponding bits 0 through 4 of Byte 1 in the
 * output status word will be set high. Turns off burnout current sources when finished.
 */
void BurnoutDetect();

/*  Sends commands to set PGA gain to 32 V/V, turn on global-chop and turn off burnout current sources
 * For each channel 1 through 5, analog and digital supplies, and on-chip temperature sensor:
 *  +Set input mux to appropriate channel or SYS register to internal measurement
 *  +Read three bytes of data and copy into corresponding channel data buffer on MSP430
 */
void ReadADCData();

/*Hold Reset pin low and then bring high after appropriate delay. Delay to allow startup
 * Send command to start data conversion
 * Setup ADC basic parameters (turn on global chop is the only difference from power on default)
 */
void ADCReset();

/*Sends a few commands to make sure the SPI interface is working. Only use for debugging*/
void SPI_Debug();

/*Checks if measured voltage is EQUAL to full scale values defined above*/
uint8_t BurnoutCheck(uint8_t *data);


/*Respond to received command byte from CCMM depending on which command bits are high:
 * Bit 0: Execute reset procedure, set byte 1 bit 5 of the status word high
 * Bit 1: Execute self offset calibration, set byte 1 bit 6 of the status word high
 * Bit 2: Execute burnout detect procedure, set byte 1 bit 7 of the status word high
 *
 */
void I2C_Slave_ProcessCMD(uint8_t cmd);

/* Publish Data from individual channel data-holding registers to the
 * I2C Transmit Buffer to be read by the Master. Data is not processed.
*/
void Publish_Data();


#endif 
