
/*
 * File:   SPI26K83.c
 * Author: sylvain.henquin
 *
 * Created on 9 octobre 2023, 14:13
 * 
 * Communication SPI entre le PIC18F26k83 et l'EEPROM M93C66.
 * SDO = RC5
 * SDI = RC4
 * SCK = RC3
 * SS = RB3
 * Le composant est cï¿½blï¿½ pour travailler avec 512 registres de 8 bits
 * Les adresses internes ï¿½ l'EEPROM vont donc de 0x000 ï¿½ 0x1FF
 * 
 */

// PIC18F26K83 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = OFF     // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be set and cleared repeatedly)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = OFF     // Multi-vector enable bit (Interrupt contoller does not use vector table to prioritze interrupts)
#pragma config IVT1WAY = OFF    // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set repeatedly)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTC = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (HV on MCLR/VPP must be used for programming)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// CONFIG5H

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 64000000          //Oscillateur interne

//Association des sorties du I²C aux entrï¿½es du LCD utilisées dans lcd.h
#define RS PORTAbits.RA0
#define EN1 PORTAbits.RA1
#define D4 PORTAbits.RA2
#define D5 PORTAbits.RA3
#define D6 PORTAbits.RA4
#define D7 PORTAbits.RA5

#include <xc.h>
#include "lcd.h"        //Ne pas oublier d'ajouter ce fichier dans le dossier du projet !
#include <stdio.h>

void writeData(int data, int address);
void writeData16(float floatData, int address);
int loadData(int address);
float loadData16(int address);
void deleteAll();

void main(void) {
   
    char* ligne1[32];
    
    TRISA = 0b11000000;     //RA7 et RA6 : boutons, reste du PORTA connectée LCD
    TRISB = 0;              //RB5 et RB4 = LEDs ; RB3 = SS
    TRISC = 0b10010011;     //RC7 = RX1 ; RC6 = TX1 ; RC5 = SDO ; 
                            //RC4 = SDI ; RC3 = SCK ; RC2 = LED ;
                            //RC1 et RC0 = boutons
    ANSELA = 0;             //Pas d'entrï¿½es analogiques
    ANSELB = 0;             //Pas d'entrï¿½es analogiques   
    ANSELC = 0;             //Pas d'entrï¿½es analogiques
    ODCONB = 0b00000110;    //RB1 et RB2 en drain ouvert pour l'I2C
    PORTB = 0;
    PORTC = 0;
    PORTBbits.RB3 = 0;      //RB3 = SS
    
    //Config PPS 
    //UART
    U1RXPPS = 0b00010111;    //RC7 comme RX1 (valeur par défaut maintenue)
    RC6PPS  = 0b00010011;    //RC6 comme TX1 
    //I2C
    I2C2SCLPPS = 0b00001001; //RB1 = SCL2 (valeur par dï¿½faut)
    I2C2SDAPPS = 0b00001010; //RB2 = SDA2 (valeur par dï¿½faut)
    RB1PPS     = 0b00100011;
    RB2PPS     = 0b00100100;
    //SPI
    SPI1SDIPPS = 0b00010100; //RC4 = SDI (input)
    RC5PPS     = 0b00011111; //RC5 = SDO (output)
    RC3PPS     = 0b00011110; //RC3 = SCK (output)
        
    //Config UART1 (vers PC))
    U1CON0 = 0b10100000;    //BRGS=1 ; No auto-baud ; TXEN = 1 ; RXEN = 0 ; Asynchrone 8bits
    U1CON1 = 0b10000000;    //Serial ON ; on garde les autres bits à 0 (valeurs par dï¿½faut) car fonctions non utilisées
    U1CON2 = 0;             
    U1BRG  = 1666;          //Valeur sur 16bits (U1BRGH et U1BRGL) : Avec BRGS = 1 et Fosc = 64Mhz
                            //Donne 9600 Baud (9592 = 0,08% erreur)
    
    //Config I2C2
    I2C2CON0 = 0b10000100;       //I2C ON ; I2C Master mode, 7 bit-address
    I2C2CLK = 0b00000000;        //CLK = MFINTOSC (500kHz)
    //Les autres valeurs sont laissées par défaut
    
    //Config SPI
    SPI1CON0 = 0b00000010; //Mode master
    SPI1CON1 = 0b11000000; //ï¿½tat IDLE des bus = 0
            
    SPI1TWIDTH = 0;
    SPI1TCNTL  = 0;
    SPI1TCNTH  = 0;
    
    SPI1CLK  = 0b00000010; //Source de la CLK = MFINTOSC
    SPI1BAUD = 100;        //Fclk = ?? à calculer
    
            
    Lcd_Init();
    Lcd_Clear();
    PORTBbits.RB5 = 1;
    __delay_ms(100);
    
    SPI1CON0bits.EN = 1; //activation du module SPI
    
    //Activation de l'écriture dans l'EEPROM
    PORTBbits.RB3 = 1; 
    SPI1TWIDTH = 4;
    SPI1TCNTL  = 1; //12 bits à transmettre en tout 
    SPI1TCNTH  = 0;
    SPI1CON2bits.TXR = 1;
    SPI1TXB = 0b10011000; //Commande qui active l'écriture (avec octet ci-dessous)
    while(SPI1STATUSbits.TXBE == 0); //On attend que les 8 premiers bits soient envoyés
    SPI1TXB = 0b00001111; //Suite de la commande + dummy data
    while(SPI1STATUSbits.TXBE == 0); //On attend que les derniers bits soient envoyés
    SPI1CON2bits.TXR = 0;
    PORTBbits.RB3 = 0;
    SPI1CON0bits.EN = 0; //désactivation du module SPI
    
    float valeurTest = 18.68;
    while(1)
    {
        __delay_ms(100);
      
        U1CON0bits.RXEN = 1;
        if (U1FIFObits.RXBE == 0) //Flag à 1 lorsque le buffer de réception est vide
        { 
            writeData16(valeurTest,1);
            __delay_ms(100);
            float affiche16 = loadData16(1);
            Lcd_Clear();                        
            sprintf(ligne1,"val= %.2f", affiche16);   //Mise en forme de la chaîne de caractères ligne1
            Lcd_Set_Cursor(1,1);            
            Lcd_Write_String(ligne1);  
            __delay_ms(1000); 
            deleteAll(); 
            __delay_ms(100); 
            float deletedData = (int) loadData(1); // Vérifier que la data dans l'adresse est bien vidée (On doit parser en int)
            Lcd_Clear();                        
            sprintf(ligne1,"delete= %d", deletedData);   //Mise en forme de la chaï¿½ne de caractères ligne1
            Lcd_Set_Cursor(1,1);            
            Lcd_Write_String(ligne1);  
              
            __delay_ms(1000);  
            U1CON0bits.RXEN = 0;  //On désactive la réception pendant le traitement de l'octet reçu             
        }     
    }
    
    return;
}

void writeData(int data, int address){
    PORTBbits.RB3 = 1;
    SPI1TWIDTH = 4;
    SPI1TCNTL  = 2;     //20 bits à transmettre en tout : 101 0 00000000 10100100 
    SPI1TCNTH  = 0;
    SPI1CON2bits.TXR = 1;
    SPI1CON0bits.EN = 1; //activation du module SPI
    SPI1TXB = 0b10100000 + (address >> 4);            /
    while(SPI1STATUSbits.TXBE == 0); //On attend que le premier octet soit envoyé
    SPI1TXB = 0b00000000 + (address << 4) + (data >> 4);            //Suite de la commande
    while(SPI1STATUSbits.TXBE == 0); //On attend que le 2ï¿½me octet soit envoyé
    SPI1TXB = 0b00000000 + (data << 4) + 0000;            //Suite de la commande + dummy data
    while(SPI1STATUSbits.TXBE == 0); //On attend que le 2ï¿½me octet soit envoyé
    PORTBbits.RB5 = !PORTBbits.RB5;
    SPI1CON2bits.TXR = 0;
    PORTBbits.RB3 = 0;
    SPI1CON0bits.EN = 0; //dï¿½sactivation du module SPI
    __delay_ms(10);
}
void writeData16(float floatData, int address){
    int data = floatData * 100;
    int low = data& 0XFF;
    writeData(low, address);
    int high = (data >> 8) & 0xFF;
    address += 1;
    __delay_ms(10);
    writeData(high, address);
    __delay_ms(10);
}
int loadData(int address){
    
    char valEEPROM, valEEPROM1, valEEPROM2,valEEPROM3 = 0;
    
    PORTBbits.RB3 = 1;
    SPI1TWIDTH = 0;
    SPI1TCNTL  = 3;         //3 octets
    SPI1TCNTH  = 0;
    SPI1CON2bits.TXR = 1;
    SPI1CON2bits.RXR = 1;
    SPI1CON0bits.EN = 1;                //activation du module SPI
    SPI1TXB = 0b11000000 + (address >> 4);               //Commande pour lecture + adresse : 110 0 0000 0000
    while(SPI1STATUSbits.TXBE == 0);    //Premier octet envoyé
    valEEPROM1 = SPI1RXB;               //Premier octet reçu = dummy data
    SPI1TXB = 0b0000 + (address << 4) + 0000;               //Suite commande+Dummy data
    while(SPI1STATUSbits.TXBE == 0);    //Deuxième octet envoyé
    valEEPROM2 = SPI1RXB;               //Deuxième octet reçu = 4 bits dummy data + 4 bits poids forts donnée
    SPI1TXB = 0b11111111;               //Dummy data
    while(SPI1STATUSbits.TXBE == 0);    //Troisième octet envoyé
    valEEPROM3 = SPI1RXB;               //Troisième octet reçu = 4 bits poids faible donnée + 4 bits dummy data
    PORTCbits.RC2 = !PORTCbits.RC2;
    SPI1CON2bits.RXR = 0;
    SPI1CON2bits.TXR = 0;
    PORTBbits.RB3 = 0;
    SPI1CON0bits.EN = 0;    //dï¿½sactivation du module SPI
    __delay_ms(20);
            
    return valEEPROM = (valEEPROM2 << 4) + (valEEPROM3 >>4);
}
float loadData16(int address){
    int addressLowValue = loadData(address);
    int addressHighValue = loadData(address+1);
    int val = (addressHighValue<< 8) | 0x00 + addressLowValue;
    float floatValue = val; 
    float valFinale = floatValue / 100;
    return valFinale;
}
void deleteAll(){
     PORTBbits.RB3 = 1;
    SPI1TWIDTH = 0;
    SPI1TCNTL  = 1;    
    SPI1TCNTH  = 0;
    SPI1CON2bits.TXR = 1;
    SPI1CON0bits.EN = 1; //activation du module SPI
    SPI1TXB = 0b10010000;
    while(SPI1STATUSbits.TXBE == 0);
    SPI1CON2bits.TXR = 0;
    PORTBbits.RB3 = 0;
    SPI1CON0bits.EN = 0; //désactivation du module SPI
    PORTBbits.RB4 = !PORTBbits.RB4;
    __delay_ms(100);
    
}