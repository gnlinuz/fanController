/*
 * File:   nmain.c
 * Author: George Nikolaidis
 *
 * Created on 13 February 2024, 16:57
 * 
 * Fan control based on temperature
 * Using PWM at 10 Khz
 * Microchip PIC12F615, 
 * Dallas DS18B20 1-wire bus system temperature sensor
 */


// PIC12F615 Configuration Bit Settings
// Microchip suggests setting all unused pins as outputs and setting their level to zero.
// 'C' source line config statements

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSCIO oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config IOSCFS = 8MHZ    // Internal Oscillator Frequency Select (8 MHz)
#pragma config BOREN = OFF      // Brown-out Reset Selection bits (BOR disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#define _XTAL_FREQ 8000000
#define DISABLE_PWM_Service()               (CCP1CON = 0x0)
#define ENABLE_DIGITAL_IO_Pins()            (ANSEL = 0X0)
#define GPIF_INT_InterruptFlagClear()       (INTCONbits.GPIF = 0)
#define TMR2_OFF()                          (T2CONbits.T2ON = 0)
#define TMR2_ON()                           (T2CONbits.T2ON = 1)
#define SET_PRESCALER_1()                   (T2CON = 0x0)
#define ENABLE_CCP1_Output_Drive()          (TRISAbits.TRISIO2 = 0)
#define DISABLE_CCP1_Output_Drive()         (TRISAbits.TRISIO2 = 1)
#define SEND_LOW_CLOCK_PULSE()              (GPIObits.GP2 = 0)
#define LED_ON()                            (GPIObits.GP5 = 1)
#define LED_OFF()                           (GPIObits.GP5 = 0)

#define MASTER_LOW()                        (GPIObits.GP4 = 0x0)
#define MASTER_HIGH()                       (GPIObits.GP4 = 0x1)
#define RELEASE_BUS()                       (TRISAbits.TRISIO4 = 0x1)
#define MASTER_OUT()                        (TRISAbits.TRISIO4 = 0x0)
#define MASTER_READ_BIT                     (TRISAbits.TRISIO4)

#define TMR1_CLEAR_FLAG_INT()               (PIR1bits.TMR1IF = 0x0)
#define ENABLE_TMR1_INT()                   (PIE1bits.TMR1IE = 0x1)
#define TMR1_ON()                           (T1CONbits.TMR1ON = 0x1)
#define TMR1_OFF()                          (T1CONbits.TMR1ON = 0x0)
#define FOSC4_TMR1_CLOCK()                  (CMCON1bits.T1ACS = 0x0)
#define TOGGLE_OUTPUT()          do { GPIObits.GP2 = ~GPIObits.GP2; } while(0)

#define LED_Toggle()             do { GPIObits.GP5 = ~GPIObits.GP5; } while(0)

#define delayReset()             do { for(int i=0;i<960; i++); } while(0) /* delay for 480 us */
#define delayPresence()          do { for(int i=0;i<360; i++); } while(0) /* delay for 180 us */
#define DELAY_WRITE_BIT()        do { for(int i=0;i<120; i++); } while(0) /* delay for  60 us */
#define DELAY_READ_BIT()         do { for(int i=0;i<16; i++); }  while(0) /* delay for  8  us */
#define WRITE_LOW_DELAY()        do { for(int i=0;i<180; i++); } while(0) /* delay for  90 us */
#define WAIT_FOR_NEW_PWM_CYCLE() do { while(PIR1bits.TMR2IF != 1); } while(0)
#define DS18B20_BUSY()           do { while(TRISAbits.TRISIO4 != 1); } while(0)

/* Calculate the delay, ex. 60.000ns/500ns = 120 cycles, so 120 cycles of 500ns -> 60us */

unsigned char pwmSelect       = 0;
unsigned char pwmFreq[4]      = {0xC7,0x0,0xC,0x64};
unsigned char pwmDutyCycle[7] = {0x64,0x3C,0x5A,0x78,0x96,0xB4,0xC8};

unsigned char DS18B20_SKIP              = 0xCC; /* 11001100 */
unsigned char DS18B20_READ_SCRATCHPAD   = 0xBE; /* 10111110 */
unsigned char DS18B20_WRITE_SCRATCHPAD  = 0x4E; /* 01001110 */
unsigned char DS18B20_COPY              = 0x48; /* 01001000 */
unsigned char DS18B20_CONV              = 0x44; /* 01000100 */

unsigned char tmpBit        = 0;
unsigned char configByte    = 0;
unsigned char tempLSB       = 0;
unsigned char tempMSB       = 0;
unsigned char posNegTemp    = 1;
unsigned char halfTemp      = 0;
unsigned char convertedTemp = 0;

/*
10.4167Khz PR2=0xC7; TMR2=0x0; CCP1CON<DC1B1:DC1B0>=00 CCPR1L=01100100 0x64;
15%  duty cycle CCPR1L 00011110 64h 100dec
30%  duty cycle CCPR1L 00111100 3Ch 60dec
45%  duty cycle CCPR1L 01011010 5Ah 90dec
60%  duty cycle CCPR1L 01111000 78h 120dec
75%  duty cycle CCPR1L 10010110 96h 150dec
90%  duty cycle CCPR1L 10110100 B4h 180dec
100% duty cycle CCPR1L 11001000 C8h 200dec
 */



/* DS18B20 FUNCTIONS */
void sendReset(){
    
    MASTER_OUT();      /* Make PIN - 3 as output */
    MASTER_LOW();      /* Send LOW */
    delayReset();      /* Stay LOW for 480us */
    RELEASE_BUS();     /* Make PIN - 3 as input to be read to receive the presence from DS18B20 */
    delayPresence();   /* NEED TO REWRITE TRHIS Wait 180 us and read the presence */
    DS18B20_BUSY();    /* check if BUS is low (busy) */ 
    
}

void sendBit(unsigned char _bit){
   
    MASTER_OUT();            /* Make PIN - 3 as output */
    MASTER_LOW();            /* send low */
    NOP();
    NOP();                   /* wait in total for 1us */
    if(_bit) RELEASE_BUS();  /* release the Bus */
    DELAY_WRITE_BIT();       /* wait for 60us */
    RELEASE_BUS();           /* release the Bus */
    
}

void writeCommand(unsigned char data){
    
    for(int i=0;i<8;i++){
        sendBit(data & 0x01);   /* do AND on the LSB, write command sends LSB first */
        data = data >> 1;       /* right shift one bit until reach the MSB */
    }
    
}

 unsigned char readBit(){
     
    MASTER_OUT();               /* Make PIN - 3 as output */
    MASTER_LOW();               /* send low */
    NOP();
    NOP();                      /* wait in total for 1us */
    RELEASE_BUS();              /* release the Bus */
    DELAY_READ_BIT();           /* delay for 8us */
    tmpBit = MASTER_READ_BIT;
    DELAY_WRITE_BIT();          /* wait for 60us */
    return tmpBit;
    
}

unsigned char readByte(){
    
    unsigned char tmpByte = 0;
    for(int i=0;i<8;i++){
        tmpByte = tmpByte >> 1;         /* shift right by 1, read LSB first */
        if(readBit() & 0x01){
            tmpByte = tmpByte | 0x80;   /* do OR if if high read from the port */
        }
    }
    return tmpByte;
    
}

void resolutionCheck(unsigned char res){
    
    if(( res | 0x60 ) == 0x7F ) {               /* If resolution is 12 bit change it to 9 bit */
        sendReset();                            /* Send Reset plus the Presence to start communicating with DS18B20 */
        writeCommand(DS18B20_SKIP);             /* Send the SKIP command we are addressing only one sensor */
        writeCommand(DS18B20_WRITE_SCRATCHPAD); /* Write the 3 bytes of scratchpad TH,TL,CONF */
        for(int i=0;i<3;i++){
            writeCommand(0x1F);                 /* send the configuration byte of 9 bit resolution 00011111 
                                                 * on all bytes even TH,TL (ignore, not in use) */
        }
        sendReset();                            /* Send Reset plus the Presence to start communicating with DS18B20 */
        writeCommand(DS18B20_SKIP);             /* Send the SKIP command we are addressing only one sensor */
        writeCommand(DS18B20_COPY);             /* Send the COPY command to write the contents of byte 2,3,4 on EEPROM */
    }
    
}


void SYSTEM_Initialize(){
    
    /* CONFIGURATION OF GPIO
     * GP2 - PWM OUTPUT P1A - PIN 5
     * GP4 - DS18B20        - PIN 3
     * GP5 - OUTPUT LED     - PIN 2 
     */

    ENABLE_DIGITAL_IO_Pins();   /* disable analog enable digital pins */
    TRISA = 0xC;                /* 00001100 GP0,GP1,GP4,GP5 as OUTPUT, GP2,GP3 as INPUT
                                   initially you need to disable output for PWM on GP2
                                   as per documentation  */
    GPIObits.GP0 = 0x0;         /* make GP0 output low */
    GPIObits.GP1 = 0x0;         /* make GP1 output low */
    GPIObits.GP4 = 0x0;         /* make GP4 output low */
    GPIObits.GP5 = 0x0;         /* make GP5 output low */

    OPTION_REG   = 0x0;         /* 00000000, 0x0
                                 * GPIO pull-ups enable 0,
                                 * INTEDG on rising     0,
                                 * TOSC FOSC/4 TOSE     0,
                                 * PSA                  0,
                                 * PS                   000 1:2 */
    WPU = 0x12;                 /* 00010010 0x12
                                 * Weak pull-up enable on pins GP1,GP4 2 buttons */
    IOC = 0x12;                 /* Interrupt on change enable IOC on GP1, GP4 */
    INTCON = 0xC8;              /* 11001000  0xC8
                                 * GIE  1 Global Interrupt Enable bit
                                 * PEIE 1 Enables all unmasked interrupts
                                 * TOIE 0 Timer0 Overflow Interrupt Enable bit
                                 * INTE 1 GP2/INT External Interrupt Enable bit
                                 * GPIE 1 GPIO Change Interrupt Enable bit, IOC must EN
                                 * TOIF 0 Timer0 Overflow Interrupt Flag bit(2)
                                 * INTF 0 GP2/INT External Interrupt Flag bit
                                 * GPIF 0 GPIO Change Interrupt Flag bit */
    LED_ON();                   /* Led ON */
    DISABLE_CCP1_Output_Drive();
    PR2     = 0xC7;             /* Load value 199 decimal for PWM freq 10KHz 0xC7 */
    CCP1CON = 0xC;              /* 00001100 load duty cycle 50% */
    CCPR1L  = 0x64;             /* 01100100 load value on CCPR1L for DC 50% 0x64 */
    PIR1    = 0x0;              /* Clear interrupt flag TMR2IF, TMR2 to PR2 Match
                                 * Interrupt Flag bit(1) */
    SET_PRESCALER_1();          /* Set prescaler to 1:1 */
    TMR2_ON();                  /* TMR2 ON */
    WAIT_FOR_NEW_PWM_CYCLE();   /* as per documentation you have to wait after
                                 * a new PWM cycle has started, until Timer2
                                 * overflows and TMR2IF bit is set  */
    ENABLE_CCP1_Output_Drive(); /* Enable the CCP1 pin output driver by clearing
                                 * GP2 P1A bit and make it as output */
    
    /* Initialise DS18B20 */
    sendReset();                            /* Send Reset plus the Presence to start communicating with DS18B20 */
    writeCommand(DS18B20_SKIP);             /* Send the SKIP command we are addressing only one sensor */
    writeCommand(DS18B20_READ_SCRATCHPAD);  /* start reading the first 5 bytes from scratchpad 
                                             * the first two are the temperature (ignore) 
                                             * the next 2 bytes are the alarm/user bytes TH and TL (ignore)
                                             * the next 1 byte is the configuration of DS18B20 
                                             * when booting DS18B20 the data from EEPROM configuration
                                             * is transfered to scratchpad. By default the temperature
                                             * resolution is at 12 bits, but here we are not interested 
                                             * on high resolution therefore set it to 9 bits 
                                             * configuration byte= 0 R1 R0 11111
                                             *                   MSB           LSB
                                             * Set R1=0 and R0=0 sets the resolution to 9 bits */
    for(int i=0;i<5;i++){    
        configByte = readByte();            /* the first 4 bytes are overwritten (ignored) */                            
    }
    resolutionCheck(configByte);
}

/* PWM */
void selectPwmFreq(){
    
    /* 10.4167Khz PWM, PR2=0xC7 TMR2=0x0 CCP1CON<DC1B1:DC1B0>=00 CCPR1L=01100100 0x64 for 45% duty cycle */
    PR2     = 0xC7; 
    T2CON   = 0x0;
    CCP1CON = 0xC;
    CCPR1L  = 0x64;
    
}

void selectPwmDutyCycle(unsigned char pos){
    
    CCPR1L = pwmDutyCycle[pos]; /* Since using a 10 Khz PWM frequency the register  
                                   CCP1CON = 0xC remains with the same value (0000 1100) where the two bits are always 00 
                                   bit 5-4 DC1B<1:0>: PWM Duty Cycle Least Significant bits 
                                   only the CCPR1L changes between different duty cycles.
                                   According to the documentation the duty cycle registers can be changed at any time */
    
}

void pwmInitialise() {
    
    DISABLE_CCP1_Output_Drive();
    selectPwmFreq();            /* Set the 10 Khz PWM with 45% duty cycle */
    PIR1 = 0x0;                 /* Clear interrupt flag TMR2IF */
    TMR2_ON();                  /* TMR2 ON */
    WAIT_FOR_NEW_PWM_CYCLE();   /* as per documentation you have to wait after
                                 * a new PWM cycle has started, until Timer2
                                 * overflows and TMR2IF bit is set  */
    ENABLE_CCP1_Output_Drive(); /* Enable the CCP1 pin output driver by clearing
                                 * GP2 P1A bit and make it as output */
    
}

void stopPwm(){
    
    DISABLE_PWM_Service();      /* Stop PWM service */
    TMR2_OFF();                 /* Stop TMR2  clock */
    SEND_LOW_CLOCK_PULSE();
    
}

void statusPWM(){
    
    if(!CCP1CON)pwmInitialise();
    
}

unsigned char temperatureChk(unsigned char LSB, unsigned char MSB){
    
    if((MSB & 0x80) == 0x80 ){
        posNegTemp = 1; /* Temperature is above zero + */
    } else {
        posNegTemp = 0; /* Temperature is below zero - */
    }    
    if((LSB & 0x8)  == 0x8)   halfTemp   = 1; /* Temperature reading .5 value */
    
}

void getTemperature(unsigned char LSB, unsigned char MSB){

    convertedTemp = ((MSB & 0x7) << 4) | (LSB >> 4);   /* Combine the two bytes into one to get the temperature */
    if (convertedTemp < 0x1E || posNegTemp == 0) {
        if(CCP1CON & 0xC == 0xC) stopPwm();   /* Stop PWM temperature is lower than 30 Celsius */ 
    } 
    if (convertedTemp >= 0x1E && CCP1CON == 0x0)pwmInitialise();
            
    statusPWM();     /* Check the status of PWM if off then start it */
   
    if (convertedTemp >= 0x1E && convertedTemp < 0x23 )selectPwmDutyCycle(0); /* 30 - 35 duty cycle  15% */
    if (convertedTemp >= 0x23 && convertedTemp < 0x28 )selectPwmDutyCycle(1); /* 35 - 40 duty cycle  30% */
    if (convertedTemp >= 0x28 && convertedTemp < 0x2D )selectPwmDutyCycle(2); /* 40 - 45 duty cycle  45% */
    if (convertedTemp >= 0x2D && convertedTemp < 0x32 )selectPwmDutyCycle(3); /* 45 - 50 duty cycle  60% */
    if (convertedTemp >= 0x32 && convertedTemp < 0x37 )selectPwmDutyCycle(4); /* 50 - 55 duty cycle  75% */
    if (convertedTemp >= 0x37 && convertedTemp < 0x3C )selectPwmDutyCycle(5); /* 55 - 60 duty cycle  90% */
    if (convertedTemp >= 0x3C) selectPwmDutyCycle(6);                         /* 60 -    duty cycle 100% */
          
    /*
    switch(pwmSelect){
                case 36:
                    
                    break;
                case 22:
                    break;
    }
    */
}

void main(void) {
    
    SYSTEM_Initialize();                        /* System initialisation */
    while(1){
        sendReset();                            /* Send Reset plus the Presence to start communicating with DS18B20 */
        writeCommand(DS18B20_SKIP);             /* Send the SKIP command we are addressing only one sensor */
        writeCommand(DS18B20_CONV);             /* Send the CONVERT command to start temperature conversion */
        DS18B20_BUSY();                         /* check if BUS (is low) busy */ 
        sendReset();                            /* Send Reset plus the Presence to start communicating with DS18B20 */
        writeCommand(DS18B20_SKIP);             /* Send the SKIP command we are addressing only one sensor */
        tempLSB = readByte();                   /* Read byte 0 LSB of temperature */
        tempMSB = readByte();                   /* Read byte 1 MSB of temperature */
        temperatureChk(tempMSB, tempLSB);       /* Check if the temperature is positive or negative and if the reading has .5 */
        getTemperature(tempMSB, tempLSB);       /* depending the temperature the duty cycle changes */
    }
    
}

