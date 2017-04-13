#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include <math.h>
#include "i2c_master_noint.h"

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

//prototype functions
void initExpander(void);
void setExpander(char pin, char level);
char getExpander(void); 

//define slave address
#define SLAVE_ADDR 0b0100000 //0100 is device address, 000 is A0-A2, all grounded

int main(void){
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    initExpander();

    __builtin_enable_interrupts();  

    while(1) {
	            
        if(getExpander()==0b10000000){ //query the expander state to see if pin
            //G7 has gone HI, indicating a pushbutton switch is pushed
            setExpander(0,0b1); //set pin GP0 high
        }
        else{
            setExpander(0,0b0); //set pin GP0 low
        }
    }
    return 0;
}

void initExpander(void){
    //turn off analog input in PIC32 
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0; 
    
    //setup I2C communication
    i2c_master_setup(); //this is currently written to use 100kHz baud
       
    i2c_master_start(); //make start bit
    
    i2c_master_send(SLAVE_ADDR < 1 | 0); //write to MCP23008
    
    i2c_master_send(0x00); //write to register IODIR
    
    i2c_master_send(0b11110000); //configure pins GP0-3 as outputs, 4-7 inputs
    
    i2c_master_stop();
    
    i2c_master_start();
    
    i2c_master_send(SLAVE_ADDR < 1 | 0); 
    
    i2c_master_send(0x09); //write to register GPIO
    
    i2c_master_send(0b0000); //make pins 0-3 initially low
    
    i2c_master_stop();
}

void setExpander(char pin, char level){ //"level" should be HI or LO (1 or 0)
    i2c_master_start(); //send the start bit
    
    i2c_master_send(SLAVE_ADDR < 1 | 0); //send it to the chip with address
    //"slave_addr", left shifted by 1, or'd with 0 to indicate a write
    
    i2c_master_send(0x09); //write to GPIO register
    
    i2c_master_send(level << pin); 
    
    i2c_master_stop(); 
}

char getExpander(void){
    i2c_master_start(); 
    
    i2c_master_send(SLAVE_ADDR < 1 | 0); 
    
    i2c_master_send(0x09); //read from GPIO
    
    i2c_master_restart(); 
    
    i2c_master_send(SLAVE_ADDR < 1 | 1); //reading now
    
    char r = i2c_master_recv(); //save value returned
    
    i2c_master_ack(1); //acknowledge and shut up
    
    i2c_master_stop(); 
    
    return r; 
} 

//explicitly dumping in the i2c_ functions from the other c file

// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be callled in the correct order as per the I2C protocol
// Change I2C1 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

void i2c_master_setup(void) {
  I2C2BRG = 233;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // look up PGD for your PIC32
                                    //for PIC32MX250F128B, PGD = 104 ns
                                    //Fsck = desired baud (use 100 kHz); 100 kHz = 1e-5 sec = 1e4 ns
                                    //Pbclk = 48 MHz = 2.08333333333e-8 sec = 20.8333333333333 ns
                                    //Therefore, I2C2BRG = 233 
                                    //Fsck in Hz, Tpgd in sec, Fpb in Hz
  I2C2CONbits.ON = 1;               // turn on the I2C2 module
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;           // send a restart 
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
    // ("I2C2 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}
