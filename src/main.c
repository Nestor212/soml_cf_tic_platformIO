/*******************************************************************************
Copyright 2021
Steward Observatory Engineering & Technical Services, University of Arizona

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
*******************************************************************************/

/**
@brief SOML Casting Furnace Chassis Control Microcontroller Module firmware
*/

#include "tic.h"

/* Command Definitions */

#define SendUCA0Data(val) { while (!(IFG2 & UCA0TXIFG)); UCA0TXBUF = val; }

/* Command Buffers */
uint8_t SLAVE_ADDR;  // Global variable to store I2C slave address

uint8_t cmd_rst_ctr = 0;  // increments when reset command received
uint8_t cmd_cal_ctr = 0;  // increments when cal command received
uint8_t cmd_bot_ctr = 0;  // increments when burnout test command received
uint8_t ack_data = 0;     // command ack data, for i2c
uint8_t data_idx = 0;

const uint8_t start_cmd[CMD_LENGTH] = {0x09};    // Command byte sent to the ADC to start conversion
uint8_t sfocal_cmd[CMD_LENGTH] = {0x19};         // Command byte sent to ADC to perform self offset calibration

uint32_t DataArray[10];  // array of 32 bit buffers
enum {
    ch0 = 0,
    ch1,
    ch2,
    ch3,
    ch4,
    ch5,
    AVDD,
    DVDD,
    TEMP,
    STATUS
};

uint8_t *statusBytes = (uint8_t *) &(DataArray[STATUS]);  // Status bytes

const uint8_t ADC_Setup[WREG_TX_LENGTH] = {0x44, 0x00, 0x86};  // Sets ADC DATARATE register
const uint8_t ADC_Sysmon_Setup[WREG_TX_LENGTH] = {0x49, 0x00, 0x11};  // Startup values
const uint8_t ADC_Reference_Setup[WREG_TX_LENGTH] = {0x45, 0x00, 0x10};  // Startup values, external reference
const uint8_t Dis_PGA[WREG_TX_LENGTH] = {0x43, 0x00, 0x00};  // Disable PGA and set gain to 1
const uint8_t En_PGA_32[WREG_TX_LENGTH] = {0x43, 0x00, 0x0D};  // Enable PGA and set gain to 32V/V
const uint8_t En_PGA_4[WREG_TX_LENGTH] = {0x43, 0x00, 0x0A};  // Enable PGA and set gain to 4V/V
const uint8_t En_PGA_2[WREG_TX_LENGTH] = {0x43, 0x00, 0x09};  // Enable PGA and set gain to 2V/V
const uint8_t Chop_Mode_Off[WREG_TX_LENGTH] = {0x44, 0x00, 0x06};  // Disable global chop
const uint8_t Burnout_On[WREG_TX_LENGTH] = {0x49, 0x00, 0xD1};  // Turn on burnout current sources (1uA)
const uint8_t Burnout_Off[WREG_TX_LENGTH] = {0x49, 0x00, 0x11};  // Turn off burnout current sources
const uint8_t Channel_0[WREG_TX_LENGTH] = {0x42, 0x00, 0x01};  // Set input mux to channel 0
const uint8_t Channel_1[WREG_TX_LENGTH] = {0x42, 0x00, 0x23};  // Set input mux to channel 1
const uint8_t Channel_2[WREG_TX_LENGTH] = {0x42, 0x00, 0x45};  // Set input mux to channel 2
const uint8_t Channel_3[WREG_TX_LENGTH] = {0x42, 0x00, 0x67};  // Set input mux to channel 3
const uint8_t Channel_4[WREG_TX_LENGTH] = {0x42, 0x00, 0x89};  // Set input mux to channel 4
const uint8_t Channel_5[WREG_TX_LENGTH] = {0x42, 0x00, 0xAB};  // Set input mux to channel 5
const uint8_t V_analog_mon[WREG_TX_LENGTH] = {0x49, 0x00, 0x71};  // Set analog supply voltage monitor
const uint8_t V_digital_mon[WREG_TX_LENGTH] = {0x49, 0x00, 0x91};  // Set digital supply voltage monitor
const uint8_t Chip_temperature[WREG_TX_LENGTH] = {0x49, 0x00, 0x51};  // Set On-Chip temperature monitor

//******I2C Command******

/* Variables to handle Rx and Tx buffers */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};  // Buffer to receive data from SPI

/* Function Definitions */
uint32_t get_thermocouple_reading(void) {
  return 0x037718;  // Corresponds to 100C with 25C thermistor.
}

// for gain = 2
uint32_t get_thermistor_reading(void) {
  return 0x4F383F;
}

uint32_t get_avdd_reading(void) {
  return 0x400000;
}

uint32_t get_dvdd_reading(void) {
  return 6000000;  // 0x2A3D70;
}

uint32_t get_temperature_reading(void) {
  return 0x506b1a;  // 0x1a6b50;
}

void get_test_data(void) {
  DataArray[ch0] = get_thermocouple_reading();
  DataArray[ch1] = get_thermocouple_reading();
  DataArray[ch2] = get_thermocouple_reading();
  DataArray[ch3] = get_thermocouple_reading();
  DataArray[ch4] = get_thermocouple_reading();
  DataArray[ch5] = get_thermistor_reading();
  DataArray[AVDD] = get_avdd_reading();
  DataArray[DVDD] = get_dvdd_reading();
  DataArray[TEMP] = get_temperature_reading();
}

#ifdef __LINUX__

void BW_RDATA(uint8_t *buf) {;}
void BW_RREG(uint8_t *buf, int rx_count) {;}
void BW_RWREG(uint8_t *cmd_buf, uint8_t *rcv_buf, int tx_count, int rx_count) {;}
void BW_WREG(uint8_t *cmd, uint8_t tx_count) {;}
void setTimer(int milliseconds) {;}
void LP_sleep(int milliseconds) {;}
void ADCReset() {;}

void _BIS_SR(uint8_t foo) {;}
void _BIC_SR(uint8_t foo) {;}

uint8_t WDTCTL;
uint8_t P2OUT;

#else

uint8_t BW_RDATA(uint8_t *buf) {
  uint8_t status;
  CS_OUT &= ~(CS_PIN);  // assert chip select
  status = UCA0RXBUF;  // discard any cruft...
  SendUCA0Data(RDATA_CMD);
  while (!(IFG2 & UCA0RXIFG));  // USCI_A0 RX buffer ready?
  status = UCA0RXBUF;
  SendUCA0Data(RDATA_CMD);
  while (!(IFG2 & UCA0RXIFG));  // USCI_A0 RX buffer ready?
  *buf++ = UCA0RXBUF;
  SendUCA0Data(DUMMY);
  while (!(IFG2 & UCA0RXIFG));  // USCI_A0 RX buffer ready?
  *buf++ = UCA0RXBUF;
  SendUCA0Data(DUMMY);
  while (!(IFG2 & UCA0RXIFG));  // USCI_A0 RX buffer ready?
  *buf++ = UCA0RXBUF;
  SendUCA0Data(DUMMY);
  while (!(IFG2 & UCA0RXIFG));  // USCI_A0 RX buffer ready?
  *buf++ = UCA0RXBUF;
  CS_OUT |= CS_PIN;  // deassert chip select
  return status;
}

void BW_RWREG(const uint8_t *cmd_buf, uint8_t *rcv_buf, int tx_count, int rx_count) {
  unsigned int i;
  uint8_t junk = UCA0RXBUF;  // drain buffer
  CS_OUT &= ~(CS_PIN);  // assert chip select
  for (i = tx_count; i > 0; i--) {
    SendUCA0Data(*cmd_buf++);
    while (!(IFG2 & UCA0RXIFG));  // USCI_A0 RX buffer ready?
    if (rx_count-- > 0) {
      *rcv_buf++ = UCA0RXBUF;
    }
  }
  CS_OUT |= CS_PIN;  // deassert chip select
}

void BW_WREG(const uint8_t *cmd, uint8_t tx_count) {
  unsigned int i;
  CS_OUT &= ~(CS_PIN);  // assert chip select
  for (i = tx_count; i > 0; i--) {
    SendUCA0Data(*cmd++);
  }
  // wait for the transfer to complete
  while ((UCA0STAT & UCBUSY));
  CS_OUT |= CS_PIN;  // deassert chip select
}

/*
 * This bit of cruft is for debugging the ADC.  Could be used for other
 * stuff as well if we had other SPI devices.
 */
uint8_t readAllCmdBuf[32] = {0};
uint8_t readAllRecvBuf[32] = {0};

#define DONT_CARE 2

void readAllRegs() {
  const int numReg = 9;  // this gets you 10 registers...
  readAllCmdBuf[0] = 0x20;
  readAllCmdBuf[1] = numReg;
  // need +1 + DONT_CARE because the first 2 bytes are junk,
  // and we get one more than numReg registers!  Thanks, TI!
  BW_RWREG(readAllCmdBuf, readAllRecvBuf, numReg + 1 + DONT_CARE, numReg + 1 + DONT_CARE);
}

/* Initialize timer configuration */
/* NOTE limit of 255 milliseconds!  */

void setTimer(int milliseconds) {
  TACTL = 0;
  TAR = 0;
  if (milliseconds > 255)
    milliseconds = 255;
  CCR0 = milliseconds << 8;  // close to 250!
  TACTL = TASSEL_2 + ID_3 + MC_1;
  CCTL0 = CCIE;  // CCR0 interrupt enabled
}

void LP_sleep(int milliseconds) {
  setTimer(milliseconds);
  __bis_SR_register(LPM0_bits + GIE);
}

void ADCReset() {
  P2OUT &= ~BIT3;  // With SPI signals initialized,
  LP_sleep(RESET_SLEEP);  // Half this?
  P2OUT |= BIT3;  // reset slave
  LP_sleep(RESET_SLEEP);  // Wait for ADC to initialize (4096*tclk = 1ms)
  BW_WREG(ADC_Setup, WREG_TX_LENGTH);  // Setup ADC basic parameters
  LP_sleep(FIRST_CONVERSION_SLEEP);
  BW_WREG(ADC_Sysmon_Setup, WREG_TX_LENGTH);  // Setup ADC basic parameters
  LP_sleep(FIRST_CONVERSION_SLEEP);
  BW_WREG(ADC_Reference_Setup, WREG_TX_LENGTH);  // Setup ADC basic parameters
  LP_sleep(FIRST_CONVERSION_SLEEP);
  BW_WREG(start_cmd, 1);  // Start conversion
  LP_sleep(FIRST_CONVERSION_SLEEP);
}

#endif

void copyLongs(uint32_t *source, uint32_t *dest, int count) {
  while (count--) {
    *dest++ = *source++;
  }
}

void PowerOnSetup() {
  SelfCalibration();  // ADC self offset calibration
  BurnoutDetect();  // Run Burnout Detect
  readAllRegs();
}

void SelfCalibration() {
  BW_WREG(sfocal_cmd, 1);  // ADC self offset calibration
  /* Limited to 255 milliseconds */
  LP_sleep(SFOCAL_SLEEP / 2);  // Need delay to let conversion restart
  LP_sleep(SFOCAL_SLEEP / 2);  // Need delay to let conversion restart
}

void ChannelBurnoutTest(const uint8_t *channel, int bitIndex) {
  int burnedOut;
  BW_WREG(channel, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Need delay to let conversion restart
  BW_RDATA(ReceiveBuffer);
  burnedOut = BurnoutCheck(ReceiveBuffer);
  if (burnedOut == 1) {
    statusBytes[0] |= (1 << bitIndex);  // set burnout flag
  } else {
    statusBytes[0] &= ~(1 << bitIndex);  // clear burnout flag
  }
}

void BurnoutDetect() {
  BW_WREG(En_PGA_32, WREG_TX_LENGTH);  // Enable PGA and set gain to 32V/V
  BW_WREG(Chop_Mode_Off, WREG_TX_LENGTH);
  BW_WREG(Burnout_On, WREG_TX_LENGTH);

  ChannelBurnoutTest(Channel_0, 0);
  ChannelBurnoutTest(Channel_1, 1);
  ChannelBurnoutTest(Channel_2, 2);
  ChannelBurnoutTest(Channel_3, 3);
  ChannelBurnoutTest(Channel_4, 4);

  BW_WREG(Burnout_Off, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Necessary?
}

void ReadADCData() {
  data_idx = 0;
  BW_WREG(En_PGA_32, WREG_TX_LENGTH);  // Enable PGA and set gain to 32V/V
  BW_WREG(ADC_Setup, WREG_TX_LENGTH);
  BW_WREG(Burnout_Off, WREG_TX_LENGTH);

  BW_WREG(Channel_0, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Need delay to let conversion restart
  BW_RDATA((uint8_t*)&DataArray[ch0]);

  data_idx++;  // 1
  BW_WREG(Channel_1, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Need delay to let conversion restart
  BW_RDATA((uint8_t*)&DataArray[ch1]);

  data_idx++;  // 2
  BW_WREG(Channel_2, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Need delay to let conversion restart
  BW_RDATA((uint8_t*)&DataArray[ch2]);

  data_idx++;  // 3
  BW_WREG(Channel_3, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Need delay to let conversion restart
  BW_RDATA((uint8_t*)&DataArray[ch3]);

  data_idx++;  // 4
  BW_WREG(Channel_4, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Need delay to let conversion restart
  BW_RDATA((uint8_t*)&DataArray[ch4]);

  data_idx++;  // 5
  BW_WREG(Dis_PGA, WREG_TX_LENGTH);  // Disable PGA and set gain to 1V/V
  BW_WREG(Channel_5, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Need delay to let conversion restart
  BW_RDATA((uint8_t*)&DataArray[ch5]);

  data_idx++;  // 6
  BW_WREG(Chop_Mode_Off, WREG_TX_LENGTH);
  BW_WREG(V_analog_mon, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Need delay to let conversion restart
  BW_RDATA((uint8_t*)&DataArray[AVDD]);

  data_idx++;  // 7
  BW_WREG(V_digital_mon, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Need delay to let conversion restart
  BW_RDATA((uint8_t*)&DataArray[DVDD]);

  data_idx++;  // 8
  BW_WREG(En_PGA_4, WREG_TX_LENGTH);  // PGA gain is capped at 4v/v to remain within absolute maximum electrical characteristics
  BW_WREG(Chip_temperature, WREG_TX_LENGTH);
  LP_sleep(FIRST_CONVERSION_SLEEP);  // Need delay to let conversion restart
  BW_RDATA((uint8_t*)&DataArray[TEMP]);
  BW_WREG(Burnout_Off, WREG_TX_LENGTH);  // Stop reading voltage and temperature monitors (same as burnout off)

  data_idx++;  // 9
}

uint8_t BurnoutCheck(uint8_t *data) {
  uint8_t isBurnedOut = 0;
  if ((data[0] == FS_3) && (data[1] == FS_2) && (data[2] == FS_1)) {  // switch to bitwise & AND ?
    isBurnedOut = 1;
  }
  return isBurnedOut;
}

void SPI_Debug() {
  BurnoutDetect();
  ReadADCData();
  readAllRegs();
}

volatile uint8_t wdt_counter = 0;

void init_watchdog()
{
    // Configure ACLK to use the internal VLO (approximately 12 kHz)
    BCSCTL3 |= LFXT1S_2;  // Select VLO as ACLK source

    // Enable Watchdog Timer in interval mode /32768
    // VLO ~12kHz, divide by 32768 to get approximately 2.73 second intervals
    // Watchdog Control Register = WG Password + WD Clear count + Interval Timer Mode + ACLK as clock source (set above to 12khz)
    WDTCTL = WDTPW + WDTCNTCL + WDTTMSEL + WDTSSEL;

    // Enable Watchdog Timer interrupt
    IE1 |= WDTIE;
}

void pet_watchdog()
{
    // Reset the watchdog timer counter
    WDTCTL = WDTPW + WDTCNTCL + WDTTMSEL + WDTSSEL;
    wdt_counter = 0;
}

// Watchdog Timer interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = WDT_VECTOR
__interrupt void watchdog_timer_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(WDT_VECTOR))) watchdog_timer_ISR(void)
#else
#error Compiler not supported!
#endif
{
    if (++wdt_counter >= 4)  // Approx. 10.92 seconds (2.73s * 4 ~ 10.92 seconds)
    {
        // 10.92 seconds have passed, trigger a reset
        WDTCTL = WDTPW | WDTCNTCL;  // Reset the watchdog to trigger a device reset
        wdt_counter = 0;
    }
}

//******************************************************************************
// Main ************************************************************************
//******************************************************************************
int main(void)
{
  uint8_t last_cmd_rst_ctr; // last reset command handled
  uint8_t last_cmd_cal_ctr; // last calibrate command handled
  uint8_t last_cmd_bot_ctr; // last burnout test command handled

  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer


  initClockTo8MHz();                       //Initialize Sub-Main Clock (SMCLK) to 8MHz
  initGPIO();                              //Initialize GPIO
  initSPI();                               //Initialize SPI
  initI2C();                               //Initialize I2C

  //Perform startup tasks
  ADCReset();                               //Reset ADC
  PowerOnSetup();                           //Run Power-on procedure
  _BIS_SR(GIE);

  init_watchdog();

  // main loop.  go read the ADC, or do other commanded tasks, when asked.
  for(;;)
  {
    ReadADCData();
    statusBytes[3]++;
    data_idx++; // 10, out of range = safe for all data

    pet_watchdog();

    if(cmd_rst_ctr != last_cmd_rst_ctr)
    {
      ADCReset();
      last_cmd_rst_ctr = cmd_rst_ctr;
    }
    if(cmd_cal_ctr != last_cmd_cal_ctr)
    {
      SelfCalibration();
      last_cmd_cal_ctr = cmd_cal_ctr;
    }
    if(cmd_bot_ctr != last_cmd_bot_ctr)
    {
      BurnoutDetect();
      last_cmd_bot_ctr = cmd_bot_ctr;
    }

    // LP_sleep(250);LP_sleep(250);LP_sleep(250);LP_sleep(250);
    // LP_sleep(250);LP_sleep(250);LP_sleep(250);LP_sleep(250);
    // LP_sleep(250);LP_sleep(250);LP_sleep(250);LP_sleep(250);
    // LP_sleep(250);LP_sleep(250);LP_sleep(250);LP_sleep(250);
    // LP_sleep(250);LP_sleep(250);LP_sleep(250);LP_sleep(250);
    // LP_sleep(250);LP_sleep(250);LP_sleep(250);LP_sleep(250);
    // LP_sleep(250);LP_sleep(250);LP_sleep(250);LP_sleep(250);
    // LP_sleep(250);LP_sleep(250);LP_sleep(250);LP_sleep(250);
    // LP_sleep(250);LP_sleep(250);LP_sleep(250);LP_sleep(250);
    // LP_sleep(250);LP_sleep(250);LP_sleep(250);LP_sleep(250);


    P2OUT ^= BIT6;                                //Toggle LED Heartbeat
  }
  // should never get here...
}

//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************

// 40-byte buffer to contain data and status bytes to be sent to CCMM
uint32_t i2cDataBuffer[10];
uint8_t *i2cTransmitBuffer = (uint8_t *)&i2cDataBuffer;
uint8_t i2cTransmitIndex = 0;

/********Function Definitions********/

#ifdef __LINUX__

// Process RPI I2C reading stuff here

#else
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIA0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(USCIAB0RX_VECTOR))) USCIA0RX_ISR(void)
#else
#error Compiler not supported!
#endif
{
  /* I2C Interrupt for Start, Restart, Nack, Stop */
  if (UCB0STAT & UCSTPIFG) {  // Stop or NACK Interrupt
    UCB0STAT &= ~(UCSTTIFG + UCSTPIFG + UCNACKIFG);  // Clear START/STOP/NACK Flags
    // set the counters back to normal
    i2cTransmitIndex = 0;
  }
  if (UCB0STAT & UCSTTIFG) {
    UCB0STAT &= ~(UCSTTIFG);  // Clear START Flags
  }
}

//******************************************************************************
// I2C Interrupt For Received and Transmitted Data *****************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR(void)
#else
#error Compiler not supported!
#endif
{
  if (IFG2 & UCB0RXIFG) // Receive Data Interrupt
  {
    uint8_t cmd_reg = UCB0RXBUF;
    if (cmd_reg & CCMM_RESET_CMD)
    {
      cmd_rst_ctr++;
    }
    if (cmd_reg & CCMM_SFOCAL_CMD)
    {
      cmd_cal_ctr++;
    }
    if (cmd_reg & CCMM_BURNOUT_CMD)
    {
      cmd_bot_ctr++;
    }
    ack_data |= cmd_reg << 5;  // copy command bits to ack bits
  }
  else if (IFG2 & UCB0TXIFG) // Transmit Data Interrupt
  {
    if (i2cTransmitIndex == 0)
    {
      uint8_t idx;
      for (idx = 0; idx < 10; idx++)
      {
        if (data_idx != idx)
        {
          i2cDataBuffer[idx] = DataArray[idx];
        }
      }
      i2cTransmitBuffer[39] |= ack_data;  // set cmd ack bits
      ack_data = 0;                       // clear ack data so it's only sent once
    }

    if (i2cTransmitIndex < 40)
    {
      UCB0TXBUF = i2cTransmitBuffer[i2cTransmitIndex++];
    }

  }
}

// Timer A0 interrupt service routine
// Used in the LP_sleep routine

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(TIMER0_A0_VECTOR))) Timer_A(void)
#else
#error Compiler not supported!
#endif
{
  __bic_SR_register_on_exit(LPM0_bits + GIE);  // wake up CPU
}

#endif
