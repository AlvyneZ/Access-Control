/*
    short library to enable spi in atmega32a

*/
#ifndef SPIMOD
#define SPIMOD


#define DDR_SPI  DDRB
#define DD_MOSI  DDB5
#define DD_MISO  DDB6
#define DD_SCK   DDB7
#define DD_SS    DDB4
#define SPI_PORT PORTB
#define SPI_PIN  PINB
#define SPI_MOSI PB5
#define SPI_MIS0 PB6
#define SPI_SCK  PB7


#include<avr/io.h>


#define ENABLE_SPI() (SPI_PORT &= (~(1<<SPI_SS)))
#define DISABLE_SPI() (SPI_PORT |= (1<<SPI_SS))


typedef struct {
    uint8_t prescaler_divider;
    uint8_t MSB_first;
    uint8_t mode;
} SPI_Settings;


void spi_master_init(SPI_Settings spi_st){
    DDR_SPI |= (1<<DD_MOSI) | (1<<DD_SCK);
    uint8_t ctrl_reg = SPCR;
    ctrl_reg = (1<<SPE)|(1<<MSTR);
    //Disabling SPI interrupts
    ctrl_reg &= ~(1<<SPIE);
    /*
        prescaler_divider is to enable different prescalers...
    */
    switch(spi_st.prescaler_divider){
        case 2:     //SPI2X = 1, SPR = 00
            SPSR |= (1 << SPI2X);
            ctrl_reg &= ~((1 << SPR1) | (1 << SPR0));
            break;
        case 4:     //SPI2X = 0, SPR = 00
            SPSR &= ~(1 << SPI2X);
            ctrl_reg &= ~((1 << SPR1) | (1 << SPR0));
            break;
        case 8:     //SPI2X = 1, SPR = 01
            SPSR |= (1 << SPI2X);
            ctrl_reg &= ~(1 << SPR1);
            ctrl_reg |= (1 << SPR0);
            break;
        case 16:     //SPI2X = 0, SPR = 01
            SPSR &= ~(1 << SPI2X);
            ctrl_reg &= ~(1 << SPR1);
            ctrl_reg |= (1 << SPR0);
            break;
        case 32:     //SPI2X = 1, SPR = 10
            SPSR |= (1 << SPI2X);
            ctrl_reg |= (1 << SPR1);
            ctrl_reg &= ~(1 << SPR0);
            break;
        case 64:     //SPI2X = 0, SPR = 10
            SPSR &= ~(1 << SPI2X);
            ctrl_reg |= (1 << SPR1);
            ctrl_reg &= ~(1 << SPR0);
            break;
        case 128:     //SPI2X = 0, SPR = 11
            SPSR &= ~(1 << SPI2X);
            ctrl_reg |= (1 << SPR1) | (1 << SPR0);
            break;
    }
    /*
        MSB_first is to set the Data Order, LSB first or MSB_first...
    */
    if (spi_st.MSB_first == 0)
        ctrl_reg |= (1 << DORD);
    else
        ctrl_reg &= ~(1 << DORD);
    /*
        mode is to set the CPOL and CPHA...
    */
    if ((spi_st.mode >= 0) && (spi_st.mode < 4)){
        //Clearing CPHA and CPOL
        ctrl_reg &= ~(3 << CPHA);
        //CPOL -> bit 3 of SPCR; CPHA -> bit 2 of SPCR
        //CPOL = bit 1 of mode; CPHA = bit 0 of mode
        ctrl_reg |= (spi_st.mode << CPHA);
    }
    //Setting the Control register to the editted buffer value
    SPCR = ctrl_reg;
}

void spi_slave_init(void){
    //set MISO  to be output and all the rest input
    DDR_SPI = (1<<DD_MISO);
    // Enable SPI
    SPCR = (1 << SPE);
}

char spi_transceiver(char data){
    SPDR = data;
    while(!(SPSR & (1 << SPIF)));
    return SPDR;
}

void spi_un_init(){
    SPCR &= ~(1 << SPE);
}



#endif      //SPIMOD
