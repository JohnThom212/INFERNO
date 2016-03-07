/****** ASEN 4028 SP Software **************************************************
 * File:        SP_AltBoard.c
 * Author:      Devon Campbell
 * Modified:    2/28/2016
 *
 * Created on February 1, 2016, 11:33 AM
 * 
 * Description
 * This program will collect temperature data from peripherals on 
 *  the PIC18F67K22, timestamp that data, store it in EEPROM, and transmit it 
 *  over a connected XBee using USART.
 * 
 * 
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/

/*******************************************************************************
 * Include directories and PIC pragmas
 ******************************************************************************/
#include <p18f67k22.h>
#include <stdio.h>
#include <stdlib.h>
#include <usart.h>
#include "LCDroutines.h"
#include <delays.h>
#include <string.h>
#include <math.h>

#pragma config FOSC = INTIO1, FCMEN = OFF, IESO = OFF
#pragma config BOREN = OFF, WDTEN = OFF, XINST = OFF
#pragma config RTCOSC = INTOSCREF, INTOSCSEL = LOW

//ERASE = 1: Erase EEPROM on start up
//ERASE = 0: Keep old memory in EEPROM
unsigned char ERASE = 1;

/*******************************************************************************
 * EEPROM Definitions
 ******************************************************************************/
unsigned char ReadEEPROM = 0b00000011;  //READ
unsigned char WriteEEPROM = 0b00000010; //WRITE
unsigned char WriteDISABLE = 0b00000100;//WRDI
unsigned char WriteENABLE = 0b00000110; //WREN
unsigned char ReadSTATUS = 0b00000101;  //RDSR
unsigned char WriteSTATUS = 0b00000001; //WRSR
unsigned char EraseMEMORY = 0b11000111; //ERME
unsigned char WrStENABLE = 0b01010000;  //EWSR
unsigned char AutoWRITE = 0b10101101;   //ADH

/*******************************************************************************
 * Global variables
 ******************************************************************************/
unsigned char Hold[10];                 //Used as a buffer for incoming USART data
unsigned char ERRORFlag = 0;            //Used to indicate that a packet was bad
unsigned char USARTFlag = 0;            //Indicates new byte received from USART
unsigned char SENDFlag = 0;             //Indicates chunk of data ready for transmission
unsigned char SECFlag = 0;              //Flag for running main loop once a second
unsigned char FIRST = 0;                //Used to indicate that this is the first
                                        // time we are transmitting
unsigned char FINISHED = 0;             //Used to indicate that there are no more error
const char Go[] = "GO";                 //Tell PIC when to start transmission
unsigned char OKGOFlag = 0;             //If GO is received, set this flag forever
#pragma udata TSTORE = 0x0200
unsigned char TRAM[30];                 //Data memory location for storing temperature
#pragma udata                           // data that has just been collected
#pragma udata TTRANS = 0x0250
unsigned char TTRAN[30];                //Data memory location for storing temperature 
#pragma udata                           // data that was just pulled from EEPROM
unsigned int T_Store = 0;               //Used for counting number of seconds
                                        // that has passed since last storage
unsigned int TimeStamp = 0;             //Used to keep track of number of seconds
                                        // since start for timestamping
unsigned int Total = 0;                 //Keeps track of number of chunks saved to
                                        // EEPROM before "GO" is received
const int TMRL = 0x69;                  //These two bytes load Timer0 for the 
const int TMRH = 0x67;                  // correct value to wait one second
unsigned short long ADDR = 0;           //EERPOM location for data
unsigned char ADTen;
unsigned char ADOne;
unsigned char HowMuch = 0;
unsigned short long TempSAVE = 0;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);                     //Function to initialize hardware and interrupts
void HiPriISR(void);                    //High priority interrupt
void LoPriISR(void);                    //Low priority interrupt
void TMR0Handler(void);                 //Handles Timer0 interrupts
void TMR1Handler(void);
void ADhandler(void);                   //Interrupt handler for ADC
void USARThandler(void);                //Interrupt handler for USART receive
void CheckUSART(void);                  //Checks if new byte was received
void SendBYTE(char STUFF);              //Sends single byte out through USART
void InitUSART(void);                   //Initialize the USART
void InitSPI1(void);                    //Initialize the SPI
void InitIO(void);                      //Initialize all I/O
void Store(void);                       //Store latest data to memory
void ADConverter(void);                 //Stores the result of the A/D conversion
void SendDATA(unsigned char * mem);     //Sends chunks of data over USART
//Properly formats the data from the EEPROM
void FormatDATA(unsigned char count, unsigned char * mem);
void W_EEPROM(unsigned short long addr, unsigned char * mem); //Writes to on-board EEPROM
void R_EEPROM(unsigned short long addr, unsigned char * mem); //Reads from on-board EEPROM
void EEPROMEnable(void);                //Pulls CS low and waits a short time
void EEPROMDisable(void);               //Pulls CS high and waits a short time
void WakeXBEE(void);                    //Wakes the XBee from sleep mode
void SleepXBEE(void);                   //Puts XBee into sleep mode
void TxERROR(unsigned char * mem);      //Checks if packet was received properly
unsigned char WriteSPI(unsigned char val);  //Writes to EEPROM and returns value
                                        // returned from SPI connection

#pragma code highVector=0x08

void atHighVector(void) {
    _asm GOTO HiPriISR _endasm
}
#pragma code

#pragma code lowVector=0x18

void atLowVector(void) {
    _asm GOTO LoPriISR _endasm
}
#pragma code

/*******************************************************************************
 * EEPROMEnable & EEPROMDisable routines
 * 
 *This was Matt's idea and I liked it so I totally stole it.
 *The idea is that we pull the chip low, then high, then low so as to ensure that
 * the system sees it. The Nops are used to give the system a chance to respond
 ******************************************************************************/
void EEPROMEnable(void) {
    LATAbits.LATA3 = 0;Nop();Nop();Nop();Nop();Nop();Nop();
    LATAbits.LATA3 = 1;Nop();Nop();Nop();Nop();Nop();Nop();
    LATAbits.LATA3 = 0;Nop();Nop();Nop();Nop();Nop();Nop();
}

void EEPROMDisable(void) {
    LATAbits.LATA3 = 1;Nop();Nop();Nop();Nop();Nop();Nop();
}

/*******************************************************************************
 * main subroutine
 * 
 * This routine will run continuously after program initialization
 * 
 * Within each iteration it will collect 1 temperature reading, 1 humidity
 *  reading, save the latest data, and check if it should transmit the last
 *  chunk of data from EEPROM
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void main(void) {
    Initial();                          
    while (1) {
        if ( USARTFlag ) {              //Checks if there was a new byte from USART
            CheckUSART();
        }
        if ( OKGOFlag ) {               //Check if "ok to send" has been received
            if ( SENDFlag ) {           //Check if latest data block is ready
                SendDATA(TTRAN);        //Send temperature data                
            }
        }
        
        if ( TimeStamp == 900 ) {
            WakeXBEE();                 //Wake up XBEE if more than 15 minutes has
        }                               // passed
        
        if ( TimeStamp > 3600 ) {       //One hour has passed
            T0CONbits.TMR0ON = 0;       //Turn off TMR0
            SendBYTE(0x46);             //Send finished command to tell GSMRS we are done
            while( 1 ) {                //Wait here forever
                if ( USARTFlag ) {      //If we receive something from GSMRS, check
                    CheckUSART();       // what it is
                }
                if ( ERRORFlag ) {
                    TxERROR(TTRAN);     //If packet is lost, resend lost packet
                }
                if ( FINISHED ) {       //If no more bad packet we put the XBee
                    SleepXBEE();        // to sleep and stop doing anything
                    while ( 1 ) {}
                }
            }
        }
        
        while ( SECFlag == 0 ) {}       //Waits until 1 second has passed
        ADCON0bits.GO = 1;              //Start collecting temperature data
        while ( ADCON0bits.GO ) {}      //Wait for collection to finish
        ADConverter();
        SECFlag = 0;                    //Reset 1 second counter
        Store();                        //Store latest readings in RAM or EEPROM
    }
}

/******************************************************************************
 * Initial subroutine
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR1, initializes SPI, and USART, sets up low and high 
 *  priority interrupts, sets up the LCD, and configures all I/O pins.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void Initial(void) {
    char i;
    OSCCON = 0b01100100;                //Oscillator control register, see data 
                                        // sheet, pp.43 - 45

    ODCON1 = 0x00;                      //Output Drain register, see data sheet
                                        // pp. 167
    
    //Set up all I/O ports
    InitIO();
    
    //Clear Holding register for incoming USART data
    for (i = 0; i < 10; i++) {
        Hold[i] = 0;
    }

    //Configuring Interrupts
    RCONbits.IPEN = 1;                  //Enable priority levels
    INTCON2bits.TMR0IP = 0;             //Assign low priority to TMR0 interrupt
    INTCONbits.TMR0IE = 1;              //Enable TMR0 interrupts
    IPR1bits.ADIP = 0;                  //Assign low priority to AD converter
    PIE1bits.ADIE = 1;                  //Enable AD interrupts
    IPR1bits.RC1IP = 1;                 //Assign high priority to USART
    PIE1bits.RC1IE = 1;                 //Enable USART interrupts
    IPR1bits.TMR1IP = 0;
    PIE1bits.TMR1IE = 1;
    INTCONbits.GIEL = 1;                //Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;                //Enable all interrupts

    //Set up Timer0
    T0CON = 0b00000101;                 //16-bit, Fosc/4, prescaler 64
    TMR0L = TMRL;                       //Load Timer0 with correct values
    TMR0H = TMRH;                       // for 1 second
    
    WakeXBEE();                         //Wake XBee from sleep
    
    Delay10KTCYx(10);

    //Set up the A/D converter
    ADCON0 = 0b00001001;                //Set up to read temp data on RA2
    ADCON1 = 0b00000000;                //VDD = 3.3V and VSS = GND
    ADCON2 = 0b10100001;                //Right Just., TAD = 8, FOSC/8
    ANCON0 = 0b00000100;                //Set AN2 as analog input

    //Set up the USART
    InitUSART();                        //Initialize the USART on USART1

    //Set up SPI
    InitSPI1();                         //Initialize the SPI on MSSP1

    T0CONbits.TMR0ON = 1;               //Turn on TMR0
}

/******************************************************************************
 * InitIO subroutine
 * 
 * This routine will initialize all I/O ports for the project.
 * 
 * NOTE:    USART is on USART1
 *          SPI is on MSSP1
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void InitIO(void) {
    //Configure the IO ports
    TRISA = 0b00000100;                     //Set PORTA as input for ADC, output
                                            // for EEPROM CS/RA3
    LATA = 0xFF;                            //Clear LATA to be safe
    TRISC = 0b10010000;                     //Configure TRISC for SPI on pins
    LATC = 0x00;                            // RC3/SCK1, RC4/SDI1, and RC5/SDO1
                                            // and USART on RC6/TX1 and RC7/RX1
    TRISD = 0b00011000;                     //Configure TRISD for XBee on pins
    LATD = 0x3F;                            // RD2/Sleep and RD3/CTS
}

/******************************************************************************
 * InitUSART subroutine
 * 
 * This routine will initialize the USART for future reading and writing on
 *  USART1.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void InitUSART(void) {
    TXSTA1 = 0b00100100;                    //Asynchronous,16-bit,High speed
    RCSTA1 = 0b10000000;                    //Enable,8-bit,Continuous,No framing,
                                            // No overrun
    BAUDCON1 = 0b00001000;                  //16-bit, everything else is default
    SPBRG1 = 0b11001111;                    //Set up the baud rate registers
    SPBRGH1 = 0b00000000;                   // both high and low
    PIR1bits.RC1IF = 0;                     //Clear Receive interrupt flag
    RCSTA1bits.CREN1 = 1;                   //Set the continuous enable bit
}

/******************************************************************************
 * Initialize SPI subroutine
 * 
 * This routine will initialize the SPI for future reading and writing. It also
 *  tells the EEPROM that we are allowing writing to all of the memory space.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void InitSPI1(void) {
    unsigned char i;
    SSP1STAT = 0b10000000;                  //Data sampled at end, transition from idle
    SSP1CON1 = 0b00110000;                  //Enable MSSP, idle state high, FOSC/4
    PIE1bits.SSP1IE = 0;                    //Disable MSSP1 interrupts
    IPR1bits.SSP1IP = 0;                    //Set low priority to MSSP1 interrupts if enabled
    
    EEPROMEnable();
    WriteSPI(ReadSTATUS);
    i = WriteSPI(0x00);
    EEPROMDisable();
    
    //This little block allows writing to the whole EEPROM chip
    EEPROMEnable();
    WriteSPI(WriteENABLE);
    EEPROMDisable();
    EEPROMEnable();
    WriteSPI(WriteSTATUS);
    WriteSPI(0x00);                         //Ensure that BP0, BP1, and BP2 are
    EEPROMDisable();                        // 0 to disable write protection

    Delay1KTCYx(1);
    
    EEPROMEnable();
    WriteSPI(ReadSTATUS);
    i = WriteSPI(0x00);
    EEPROMDisable();
    
    Delay1KTCYx(1);

    //Erase all of the EEPROM if ERASE = 1
    if ( ERASE ) {
        EEPROMEnable();
        WriteSPI(WriteENABLE);
        EEPROMDisable();
        EEPROMEnable();
        WriteSPI(EraseMEMORY);
        EEPROMDisable();

        Delay10KTCYx(20);                        //Wait for 100 ms for erase to finish
    }
}

/******************************************************************************
 * ADConverter subroutine
 * 
 * This routine will take the result of the A/D conversion and save it.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void ADConverter(void) {
    unsigned int Temp1 = ADRES;            //Save the input from the ADC high byte
    unsigned int Integer = (Temp1*0.81)-50; //Multiply by 1000 to get integer
                                            // and subtract "8" to account for strange
                                            // offset that we have noticed
    unsigned int Decimal = Integer % 10;    // value and then take modulus which
                                            // gives the decimal point
    unsigned int Integer1 = (Integer-Decimal)/10;//Subtract decimal value
                                            // and divide by 10 to get integer
    unsigned int Ones = Integer1 % 10;      //Take modulus again to get ones place
    unsigned int Integer2 = (Integer1-Ones)/10; //Subtract ones value
                                            // and divide by 10 to get tens value
    unsigned int Tens = Integer2;           //Save tens place
    
    ADTen = '0' + Tens;                     //These two lines convert the integer
    ADOne = '0' + Ones;                     // values to ascii characters
}

/******************************************************************************
 * CheckUSART subroutine
 *
 * This subroutine compares the string within the buffer to any of the commands
 *  we are looking for. If a match is found, a flag is set for future data 
 *  transmission. In this case, the only string to check for is "GO" which
 *  sets the FIRST flag indicating that we need to send all previously collected
 *  data, and it sets the OKGO flag which says to start transmitting.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void CheckUSART(void) {
    if (Hold[9] == '\n') {              //Check if the last byte received was an
                                        // end of line character 
        if ( Hold[8] == 0x54 ) {//( (Hold[7] == Go[0]) && (Hold[8] == Go[1]) ) {
            SendBYTE(0x41);             //Send "Ack" command to GSMRS
            SendBYTE('\n');
            OKGOFlag = 1;               //If GO is received, set OKGO flag
            FIRST = 1;                  //Since we only send GO once, this will
        }                               // be the first time; set FIRST flag so
                                        // that SendDATA will send all prior data
        if ( Hold[8] == 0x52 ) {        //Check if "resend" command is received
            SendBYTE(0x41);             //Send "Ack" command
            SendBYTE('\n');
            ERRORFlag = 1;              //Set ERRORFlag
        }
        
        if ( Hold[8] == 0x46 ) {
            SendBYTE(0x41);
            SendBYTE('\n');
            FINISHED = 1;
        }
    }
    USARTFlag = 0;                      //Clear "new byte" flag
}

/******************************************************************************
 * WakeXBEE subroutine
 *
 * This subroutine wakes up the XBee from sleep mode by sending logic low to pin
 *  9 on the XBee (SLEEP_RQ). It then waits until the XBee is fully awake by
 *  waiting for logic 0 on pin 12 (CTS) .
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void WakeXBEE(void) {
    LATDbits.LATD2 = 0;                 //Send wake-up command to XBee
    while( PORTDbits.PSP3 ) {}          //Wait for CTS to assert low
}

/******************************************************************************
 * SleepXBEE subroutine
 *
 * This subroutine causes the XBee to return to sleep mode by sending logic high
 *  to pin 9 on the XBee (SLEEP_RQ).
 * 
 * NOTE:    The XBee will automatically finish sending any data in the queue before
 *          going to sleep.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void SleepXBEE(void) {
    LATDbits.LATD2 = 1;                     //Send sleep command to XBee
    while( PORTDbits.PSP3 == 0 ) {}         //Wait for CTS to assert low
}

/******************************************************************************
 * FormatDATA subroutine
 *
 * This subroutine properly formats the data read from the EEPROM and sends each
 *  formatted byte to the XBee.
 * 
 * Inputs:      count = Steps through the larger "mem" memory location
 *              mem = RAM location to pull data from, format, and transmit
 * Outputs:     None
 ******************************************************************************/
void FormatDATA(unsigned char count, unsigned char * mem) {
    SendBYTE(mem[count]);       //Send temperature byte
    SendBYTE(' ');              //Send spacing character
    SendBYTE(mem[count+1]);     //Send high byte of timestamp
    SendBYTE(mem[count+2]);     //Send low byte of timestamp
}

/******************************************************************************
 * TxERROR subroutine
 *
 * This subroutine checks if there was an error in the data transmission. If so,
 *  it takes the address returned from the GSMRS and sends that packet again.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void TxERROR(unsigned char * mem) {
    unsigned short long addr;
    unsigned char count;
    
    while( USARTFlag == 0 ) {}      //Wait for response from GSMRS
    USARTFlag = 0;                  //Clear "new byte" flag
    addr = Hold[7]<<16 + Hold[8]<<8 + Hold[9];  //Format address from GSMRS
    R_EEPROM(addr, mem);            //Save erroneous packet to RAM 

    for(count=0; count<28; count+=3) {
        FormatDATA(count, mem);     //Format and transmit the data just read
                                    // from EEPROM
    }
}

/******************************************************************************
 * SendDATA subroutine
 * 
 * This routine will read in the latest chunk of data from the EEPROM, convert
 *  it to a BCD format, and then transmit it over the USART.
 * 
 * Inputs:      mem = RAM location of 60 bytes to be sent
 * Outputs:     None
 ******************************************************************************/
void SendDATA(unsigned char * mem) {
    unsigned int count1;                //Counter for stepping through EEPROM
    unsigned char count;                //Counter for stepping through mem
    unsigned short long num = 0;        //Used for EEPROM location

    if ( FIRST ) {                      //If it is the first time we have
                                        // called this function, then send
                                        // all data captured so far
        num = TempSAVE;                 //Start back where we left off if not all 
                                        // data was sent in 1 second
        for (count1=1; count1<=Total; count1++) {
            R_EEPROM(num, mem);         //Save single page of EEPROM to RAM
            for (count=0; count<28; count+=3) {
                FormatDATA(count, mem); //Format and transmit the data just read
                                        // from EEPROM
                SendBYTE('\n');         //Send spacing character
            }
            num += 30;                  //Increment address counter
            if ( (count1 == 133) || (count1 == 266) ) {
                num += 10;              //Move to new page in EEPROM
            }
            if ( (SECFlag) && (num != ADDR) ) {
                TempSAVE = num;
                return;
            }
        }
        FIRST = 0;                      //Clear flag for first time transmission
        SENDFlag = 0;                   //Clear flag for latest chunk of data
    } 
    else {
        num = ADDR - 30;                //Get address of most recent data set
        
        if ( Total > 133 ) {            //These two if statements handle the case
            num -= 10;                  // of jumping to a new page in EEPROM
        }
        if ( Total > 266 ) {
            num -= 10;
        }
        
        R_EEPROM(num, mem);             //Save single page of EEPROM
        for (count=0; count<28; count+=3) { //Send latest chunk of data
            FormatDATA(count, mem);     //Format and transmit the data just read
                                        // from EEPROM
            SendBYTE('\n');             //Send spacing character
        }
        SENDFlag = 0;                   //Clear flag for latest chunk of data
    }
}

/******************************************************************************
 * SendBYTE subroutine
 * 
 * This routine will send a single character across the USART.
 * 
 * Inputs:      STUFF = Byte to be transmitted over USART
 * Outputs:     None
 ******************************************************************************/
void SendBYTE(char STUFF) {
    while (PIR1bits.TX1IF == 0) {}      //Wait until transmit flag is cleared
    TXREG1 = STUFF;                     //Send current byte
}

/******************************************************************************
 * Store subroutine
 * 
 * This routine will store the latest values of temp and humidity into data
 *  memory and if the total number of bytes stored is 30, it will move that 
 *  data to the on-board EEPROM.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void Store(void) {
    unsigned int T1;                    //Used to make simple conversion so 
    unsigned char T;                    // that T can be made a BCD char

    T1=(ADTen-0x30)*16+(ADOne-0x30);    //These two lines convert
                                        // the values to BCD and then save them
    T = T1;                             // into their char variables
    
    TRAM[(3*T_Store)] = T;              //Save temperature reading to memory
    TRAM[(3*T_Store + 2)] = TimeStamp;  //Timestamp also saved
    TRAM[(3*T_Store + 1)] = TimeStamp >> 8; //Timestamp also saved
    
    if (T_Store == 9) {                 //10*3Bytes = 30 Bytes
        SENDFlag = 1;                   //Tells main loop to send last set of data
        W_EEPROM(ADDR, TRAM);           //Save 30 bytes to EEPROM
        T_Store = -1;                   //Reset the counter for number of bytes
        Total++;                        //Increment the total number of times we
                                        // have stored data to EEPROM
        if ( (Total == 133) || (Total == 266) ) {
            ADDR += 10;                 //If we are at 4 or 8 Kbytes, need to
        }                               // jump to the next page of memory
        ADDR += 30;                     //Increment address for EEPROM
    }
    T_Store++;                          //Increment counter since last storage
}

/******************************************************************************
 * WriteSPI subroutine
 * 
 * This routine will write a value over SPI.
 * 
 * Inputs:      val = Byte to be sent to peripheral
 * Outputs:     temp = Byte returned from peripheral
 ******************************************************************************/
unsigned char WriteSPI(unsigned char val) {
    unsigned char temp;
    SSP1BUF = val;                      //Place byte onto MOSI
    while (SSP1STATbits.BF == 0) {}     //Wait for transmit to complete
    temp = SSP1BUF;                     //Save the MISO result
    return temp;
}

/******************************************************************************
 * W_EEPROM subroutine
 * 
 * This routine will store the latest 30 bytes of temperature into EEPROM memory.
 * 
 * NOTE: When writing in 30 byte chunks, the AAI mode must be set to save time.
 *      This follows a specific procedure as outlined in the SST25VF080B 
 *      data sheet - pp. 13-15.
 * 
 * Inputs:      addr = EEPROM memory address for storing data
 *              mem = RAM location to pull data from to send to EEPROM
 * Outputs:     None
 ******************************************************************************/
void W_EEPROM(unsigned short long addr, unsigned char * mem) {
    unsigned char i;
    unsigned char high;
    unsigned char med;
    unsigned char low;
    
    high = addr>>16;                    //High byte of EEPROM memory location
    med = addr>>8;                      //Medium byte of EEPROM memory location
    low = addr;                         //Low byte of EEPROM memory location

    EEPROMEnable();                     //Select EEPROM
    WriteSPI(WriteENABLE);              //Set Write enable Latch (WREN)
    EEPROMDisable();                    //Push CS high according to data sheet
    EEPROMEnable();                     //Pull CS low to write
    WriteSPI(AutoWRITE);                //Begin AAI mode

    WriteSPI(high);                     //Send high address to EEPROM
    WriteSPI(med);                      //Send medium address to EEPROM
    WriteSPI(low);                      //Send low address to EEPROM
    WriteSPI(mem[0]);                   //Send first of two bytes
    WriteSPI(mem[1]);                   //Send second of two bytes

    EEPROMDisable();                    //Push CS high according to data sheet
    Delay1KTCYx(1);                     //Wait for write to finish
    for (i = 2; i < 29; i+=2) {
        EEPROMEnable();                 //Pull CS low to write
        WriteSPI(AutoWRITE);            //Continue AAI mode
        WriteSPI(mem[i]);               //Send first of two bytes
        WriteSPI(mem[i+1]);             //Send second of two bytes
        EEPROMDisable();                //Push CS high according to data sheet
        EEPROMEnable();
        WriteSPI(ReadSTATUS);
        while(1) {
            if( WriteSPI(0x00) != 0x42 ) {
                continue;
            } else {
                break;
            }
        }
        EEPROMDisable();
    }
    EEPROMEnable();
    WriteSPI(WriteDISABLE);             //Write disable to exit AAI mode
    EEPROMDisable();
    EEPROMEnable();
    WriteSPI(ReadSTATUS);
    i = WriteSPI(0x00);
    EEPROMDisable();
    
    Delay10KTCYx(2);                    //Wait for 6 ms for write to finish
}

/******************************************************************************
 * R_EEPROM subroutine
 * 
 * This routine will read the last set of data from EEPROM and save that data
 *  into holding registers in RAM for tranmission across USART.
 * 
 * Inputs:      addr = EEPROM memory address for pulling data from
 *              mem = RAM location to write data to
 * Outputs:     None
 ******************************************************************************/
void R_EEPROM(unsigned short long addr, unsigned char * mem) {
    unsigned char temp;
    unsigned char i;
    unsigned char high;
    unsigned char med;
    unsigned char low;

    high = addr>>16;                    //High byte of EEPROM memory location
    med = addr>>8;                      //Medium byte of EEPROM memory location
    low = addr;                         //Low byte of EEPROM memory location
    
    EEPROMEnable();                     //Pull CS low to write
    WriteSPI(ReadEEPROM);               //Set READ on EEPROM
    
    WriteSPI(high);                     //Send high address to EEPROM
    WriteSPI(med);                      //Send medium address to EEPROM
    WriteSPI(low);                      //Send low address to EEPROM
    
    for (i = 0; i < 30; i++) {
        temp = WriteSPI(0x00);          //Save the data to holding location in RAM
        mem[i] = temp;
    }
    EEPROMDisable();                    //Push CS high to deselect EEPROM
    Delay1KTCYx(1);                    //Wait for read to finish just in case
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * This routine handles the USART inputs on RCSTA1 and saves that value to the 
 *  Hold holding register.
 * 
 * Inputs:      RCFlag
 * Outputs:     None
 ******************************************************************************/
#pragma interrupt HiPriISR

void HiPriISR() {
    while (1) {
        if ( PIR1bits.RC1IF ) {         //Check if new byte received by USART
            USARThandler();
            continue;
        }
        break;
    }
}

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until the interrupt flags are all cleared.
 * 
 * Inputs:      Interrupt flag
 * Outputs:     None
 ******************************************************************************/
#pragma interruptlow LoPriISR nosave=TBLPTR, TBLPTRU, TABLAT, PCLATH, PCLATU, PROD, section("MATH_DATA")//, section(".tmpdata")

void LoPriISR() {
    while (1) {
        if ( INTCONbits.TMR0IF ) {      //Check if TMR0 overflow
            TMR0Handler();
        } else if ( PIR1bits.ADIF ) {   //Check if AD conversion is done
            ADhandler();
        } else if ( PIR1bits.TMR1IF ) {
            TMR1Handler();
        }
        break;
    }
}

/******************************************************************************
 * TMR0Handler subroutine
 *
 * Increments the TimeStamp counter for number of seconds since start, sets the 
 *  SECFlag to tell the main loop it can move on, starts the ADC and resets Timer0.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void TMR0Handler(void) {
    TMR0H = TMRH;                       //Reset Timer0 for another 1 second 
    TMR0L = TMRL;                       // run time
    TimeStamp++;                        //Used for all "timestamping"
    SECFlag = 1;                        //Tells main loop 1 second has passed
    INTCONbits.TMR0IF = 0;              //Clear Timer0 interrupt flag
}

/******************************************************************************
 * ADhandler interrupt service routine.
 *
 * Collect temperature reading and clear AD flag.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void ADhandler() {
    PIR1bits.ADIF = 0;                  //Clear the AD flag
}

/******************************************************************************
 * USART receive handler
 *
 * Saves the latest input byte from the USART into a buffer, sets the USARTFlag to
 *  tell the USART that a new byte was received, and clears associated flags.
 * 
 * Inputs:      None
 * Outputs:     None
 ******************************************************************************/
void USARThandler(void) {
    char temp = RCREG1;                 //Save USART input to temp variable
    int i;
    for (i = 0; i < 9; i++) {           //Shift contents of Hold register left
        Hold[i] = Hold[i + 1];          // by one and the add new byte from 
    }                                   // temp to the end of this buffer
    Hold[9] = temp;
    RCSTA1bits.FERR1 = 0;               //Clear overrun and framing error bits
    RCSTA1bits.OERR1 = 0;
    PIR1bits.RC1IF = 0;                 //Clear flag and return to polling routine
    USARTFlag = 1;                      //Set flag to indicate that a new byte
                                        // was received.
}

void TMR1Handler(void) {
    TMR1H = 0;
    TMR1L = 0;
    HowMuch++;
    PIR1bits.TMR1IF = 0;
}