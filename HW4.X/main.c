#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_12 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_16 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_4 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 00000001 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATAbits.LATA0 

//prototype functions
void setVoltage(char channel, char voltage);
void SPI_1_init();
unsigned char SPI_1_io(unsigned char write);

//does there need to be an interrupt here?

int main(void){
    //do some stuff here
}

void setVoltage(char channel, char voltage){ //8 bits = 1024
    unsigned char word1, word2;
    if(channel == 0){
        //use channel A
        //VrefA = HI (3.3volts))
        //do something here with VoutA
    }
    else{
        //use channel B
        //VrefB = HI (3.3 volts)
        //do something with VoutB
        
        
    }
    CS = 0;
    spi_io(word1);
    spi_io(word2);
    
}

void SPI_1_init(void){
    //initialize all the pins to what we want them to be
    
    //set A0 to be SS1
    TRISAbits.TRISA0 = 0; //make it an output
    RPA0Rbits.RPA0R = 0b0011; 
    
    //set A1 to be SDO1
    TRISAbits.TRISA1 = 0; //make it an output
    RPA1Rbits.RPA1R = 0b0011;
    
    //set B14 as SCLK
    //need to do anything here?
    
    CS = 1;
    //setup the rest of SPI, taken from sample code
    SPI1CON = 0;              // turn off the spi module and reset it
    SPI1BUF;                  // clear the rx buffer by reading from it
    SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    SPI1CONbits.ON = 1;       // turn on spi 4
}

unsigned char spi_io(unsigned char write) {
    //this code was taken from SPI_example.c
  SPI1BUF = write;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}