

//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

#include "tic.h"

extern uint8_t SLAVE_ADDR;
#ifdef __LINUX__

void initClockTo8MHz(){ ; }
void initGPIO() { ; }
void initSPI() { ; }
void initI2C() { ; }
#else

/*Calibrate Clock to 8MHz*/
void initClockTo8MHz()
{
    if (CALBC1_8MHZ==0xFF)                  // If calibration constant erased
    {
        while(1);                               // Trap CPU
    }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_8MHZ;                    // Set DCO
    DCOCTL = CALDCO_8MHZ;

    /* Set up prescale of 4 on smclk */
    BCSCTL2 |= BIT2;

}


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
//                |                 |
//                |   P2.6   P1.0   |
//                 -----------------
//                      |      ^
//                      |      +----SYNC
//                      V
//                      LED Heartbeat

/*Initialize Reset, DRDY, START/SYNC, SPI and I2C Pins*/
void initGPIO()
{
   //LED pin setup
  P2SEL = 0;    // default is 0xC0, due to these being oscillator pins
  P2OUT = 0x00;                             // P2 setup for debug LED
  P2DIR |= BIT6;                            // LED Heartbeat output

  //ADC Reset, DRDY and START/SYNC (Active Low), LED
  P2DIR |= BIT3 + BIT4 + BIT6;                       //P2.3, P2.4 Output, LED output
  P2OUT |= BIT3;                              //P2.3 High --> ADC !RST
  P2OUT &= ~BIT4;                             //P2.4 Low  --> START/SYNC
  P2DIR &= ~(I2CA_0 + I2CA_1 + I2CA_2 + BIT5);//P2.0, 2.1, 2.2, 2.5 Input --> I2C Pins, !DRDY

  P1DIR |= BIT0 ; // SYNC input
  P1OUT = 0x00; ; // Set for pulldown
  P1REN |= BIT0 ; // Enable pullup/down

  //SPI Pins
  P1SEL = BIT1 + BIT2 + BIT4;               //Assign SPI pins to USCI_A0
  P1SEL2 = BIT1 + BIT2 + BIT4;

  //I2C Pins
  P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
  P1SEL2|= BIT6 + BIT7;

  /* Port 3 pins */
  P3DIR = 0xFF;
  P3OUT = 0xFF;
}

/*Initialize SPI configuration*/
void initSPI()
{
  UCA0CTL1 |= UCSWRST;                             // Enable SW reset
  //Clock Polarity: The inactive state is low
  //MSB First, 8-bit, Master, 3-pin mode, Synchronous
  UCA0CTL0 |= UCMSB + UCMST + UCSYNC;
  UCA0CTL0 &= ~(UCCKPL);

  UCA0CTL1 |= UCSSEL_2;                     // SMCLK selected for SCLK

  UCA0BR1 = 0x00;                           // 2 MHz assuming SMCLK = 2MHz
  UCA0BR0 = 0x00 ;

  UCA0MCTL = 0;                             // No modulation
  UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine


  CS_DIR |= CS_PIN;                         //CS pin set Output
  CS_OUT |= CS_PIN;                         //CS pin set High
}

/*Initialize I2C configuration*/
void initI2C()
{
    uint8_t pins = (~P2IN & (I2CA_0 + I2CA_1 + I2CA_2));// Slave Address determined by slave address pins
    SLAVE_ADDR = pins + 0x08;// Must start at 8 for Arduino compatibility!
    UCB0CTL1 |= UCSWRST;                             // Enable SW reset
    UCB0CTL0 = UCMODE_3 + UCSYNC;                    // I2C Slave, synchronous mode
    UCB0I2COA |= SLAVE_ADDR ;                        // Own Address
    UCB0CTL1 &= ~UCSWRST;                            // Clear SW reset, resume operation
    UCB0I2CIE |= UCSTPIE + UCSTTIE;                  // Enable STT and STP interrupt
    IE2 |= UCB0RXIE;                                 // Enable RX interrupt
    IE2 |= UCB0TXIE;                                 // Enable TX interrupt?
}

#endif
