#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include <math.h>

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
unsigned char sine_gen(int ii);
unsigned char sawtooth_gen(int ii);

//does there need to be an interrupt here?

int main(void){
    _builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    /*// do your TRIS and LAT commands here
    //make pushbutton pin an input pin. Pin number: 11
    TRISBbits.TRISB4 = 1; //1 for input
    
    //make LED pin an output pin that is initially high. Pin number: 12
    TRISAbits.TRISA4 = 0; //0 for output
    LATAbits.LATA4 = 1; //1 for initially high*/

    __builtin_enable_interrupts();

    while(1) {
	    // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
		  // remember the core timer runs at half the CPU speed
        _CP0_SET_COUNT(0);
        
        while(_CP0_GET_COUNT() < 24000){
        }
        

        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT() < 24000){
            ; //do nothing
        }
        
        //read pushbutton; if pushed, wait for it to be unpushed before cont'ing
        while(!PORTBbits.RB4){
            ;
        }
    }
    return 0;
}
}

void setVoltage(char channel, char voltage){ //8 bits = 1024
    unsigned char word1, word2;
   
    word1 = 0b00110000;
    word2 = 0b00000000;
    
    word1 += channel<<7; //stick the channel selection bit where it should be
    word1 += voltage>>4; //stick the first 4 bits of "voltage" into word1
    word2 += voltage<<4; //stick the second 4 bits of "voltage" into word2
    
    CS = 0;
    spi_io(word1);
    spi_io(word2);
    CS = 1;    
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

unsigned char sine_gen(int ii){
    return (unsigned char)(255*sin((double)(ii)/200));
}

unsigned char sawtooth_gen(int ii){
    //not sure what to put here
}