
# 1 "../main.c"

# 18 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\xc.h"
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);


# 13 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\c90\xc8debug.h"
#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);

# 52 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\pic16f1936.h"
extern volatile unsigned char INDF0 __at(0x000);

asm("INDF0 equ 00h");


typedef union {
struct {
unsigned INDF0 :8;
};
} INDF0bits_t;
extern volatile INDF0bits_t INDF0bits __at(0x000);

# 72
extern volatile unsigned char INDF1 __at(0x001);

asm("INDF1 equ 01h");


typedef union {
struct {
unsigned INDF1 :8;
};
} INDF1bits_t;
extern volatile INDF1bits_t INDF1bits __at(0x001);

# 92
extern volatile unsigned char PCL __at(0x002);

asm("PCL equ 02h");


typedef union {
struct {
unsigned PCL :8;
};
} PCLbits_t;
extern volatile PCLbits_t PCLbits __at(0x002);

# 112
extern volatile unsigned char STATUS __at(0x003);

asm("STATUS equ 03h");


typedef union {
struct {
unsigned C :1;
unsigned DC :1;
unsigned Z :1;
unsigned nPD :1;
unsigned nTO :1;
};
struct {
unsigned CARRY :1;
unsigned :1;
unsigned ZERO :1;
};
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __at(0x003);

# 171
extern volatile unsigned short FSR0 __at(0x004);



extern volatile unsigned char FSR0L __at(0x004);

asm("FSR0L equ 04h");


typedef union {
struct {
unsigned FSR0L :8;
};
} FSR0Lbits_t;
extern volatile FSR0Lbits_t FSR0Lbits __at(0x004);

# 195
extern volatile unsigned char FSR0H __at(0x005);

asm("FSR0H equ 05h");


typedef union {
struct {
unsigned FSR0H :8;
};
} FSR0Hbits_t;
extern volatile FSR0Hbits_t FSR0Hbits __at(0x005);

# 215
extern volatile unsigned short FSR1 __at(0x006);



extern volatile unsigned char FSR1L __at(0x006);

asm("FSR1L equ 06h");


typedef union {
struct {
unsigned FSR1L :8;
};
} FSR1Lbits_t;
extern volatile FSR1Lbits_t FSR1Lbits __at(0x006);

# 239
extern volatile unsigned char FSR1H __at(0x007);

asm("FSR1H equ 07h");


typedef union {
struct {
unsigned FSR1H :8;
};
} FSR1Hbits_t;
extern volatile FSR1Hbits_t FSR1Hbits __at(0x007);

# 259
extern volatile unsigned char BSR __at(0x008);

asm("BSR equ 08h");


typedef union {
struct {
unsigned BSR0 :1;
unsigned BSR1 :1;
unsigned BSR2 :1;
unsigned BSR3 :1;
unsigned BSR4 :1;
};
struct {
unsigned BSR :5;
};
} BSRbits_t;
extern volatile BSRbits_t BSRbits __at(0x008);

# 311
extern volatile unsigned char WREG __at(0x009);

asm("WREG equ 09h");


typedef union {
struct {
unsigned WREG0 :8;
};
} WREGbits_t;
extern volatile WREGbits_t WREGbits __at(0x009);

# 331
extern volatile unsigned char PCLATH __at(0x00A);

asm("PCLATH equ 0Ah");


typedef union {
struct {
unsigned PCLATH :7;
};
} PCLATHbits_t;
extern volatile PCLATHbits_t PCLATHbits __at(0x00A);

# 351
extern volatile unsigned char INTCON __at(0x00B);

asm("INTCON equ 0Bh");


typedef union {
struct {
unsigned IOCIF :1;
unsigned INTF :1;
unsigned TMR0IF :1;
unsigned IOCIE :1;
unsigned INTE :1;
unsigned TMR0IE :1;
unsigned PEIE :1;
unsigned GIE :1;
};
struct {
unsigned :2;
unsigned T0IF :1;
unsigned :2;
unsigned T0IE :1;
};
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __at(0x00B);

# 429
extern volatile unsigned char PORTA __at(0x00C);

asm("PORTA equ 0Ch");


typedef union {
struct {
unsigned RA0 :1;
unsigned RA1 :1;
unsigned RA2 :1;
unsigned RA3 :1;
unsigned RA4 :1;
unsigned RA5 :1;
unsigned RA6 :1;
unsigned RA7 :1;
};
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __at(0x00C);

# 491
extern volatile unsigned char PORTB __at(0x00D);

asm("PORTB equ 0Dh");


typedef union {
struct {
unsigned RB0 :1;
unsigned RB1 :1;
unsigned RB2 :1;
unsigned RB3 :1;
unsigned RB4 :1;
unsigned RB5 :1;
unsigned RB6 :1;
unsigned RB7 :1;
};
} PORTBbits_t;
extern volatile PORTBbits_t PORTBbits __at(0x00D);

# 553
extern volatile unsigned char PORTC __at(0x00E);

asm("PORTC equ 0Eh");


typedef union {
struct {
unsigned RC0 :1;
unsigned RC1 :1;
unsigned RC2 :1;
unsigned RC3 :1;
unsigned RC4 :1;
unsigned RC5 :1;
unsigned RC6 :1;
unsigned RC7 :1;
};
} PORTCbits_t;
extern volatile PORTCbits_t PORTCbits __at(0x00E);

# 615
extern volatile unsigned char PORTE __at(0x010);

asm("PORTE equ 010h");


typedef union {
struct {
unsigned :3;
unsigned RE3 :1;
};
} PORTEbits_t;
extern volatile PORTEbits_t PORTEbits __at(0x010);

# 636
extern volatile unsigned char PIR1 __at(0x011);

asm("PIR1 equ 011h");


typedef union {
struct {
unsigned TMR1IF :1;
unsigned TMR2IF :1;
unsigned CCP1IF :1;
unsigned SSPIF :1;
unsigned TXIF :1;
unsigned RCIF :1;
unsigned ADIF :1;
unsigned TMR1GIF :1;
};
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __at(0x011);

# 698
extern volatile unsigned char PIR2 __at(0x012);

asm("PIR2 equ 012h");


typedef union {
struct {
unsigned CCP2IF :1;
unsigned :1;
unsigned LCDIF :1;
unsigned BCLIF :1;
unsigned EEIF :1;
unsigned C1IF :1;
unsigned C2IF :1;
unsigned OSFIF :1;
};
} PIR2bits_t;
extern volatile PIR2bits_t PIR2bits __at(0x012);

# 755
extern volatile unsigned char PIR3 __at(0x013);

asm("PIR3 equ 013h");


typedef union {
struct {
unsigned :1;
unsigned TMR4IF :1;
unsigned :1;
unsigned TMR6IF :1;
unsigned CCP3IF :1;
unsigned CCP4IF :1;
unsigned CCP5IF :1;
};
} PIR3bits_t;
extern volatile PIR3bits_t PIR3bits __at(0x013);

# 801
extern volatile unsigned char TMR0 __at(0x015);

asm("TMR0 equ 015h");


typedef union {
struct {
unsigned TMR0 :8;
};
} TMR0bits_t;
extern volatile TMR0bits_t TMR0bits __at(0x015);

# 821
extern volatile unsigned short TMR1 __at(0x016);

asm("TMR1 equ 016h");




extern volatile unsigned char TMR1L __at(0x016);

asm("TMR1L equ 016h");


typedef union {
struct {
unsigned TMR1L :8;
};
} TMR1Lbits_t;
extern volatile TMR1Lbits_t TMR1Lbits __at(0x016);

# 848
extern volatile unsigned char TMR1H __at(0x017);

asm("TMR1H equ 017h");


typedef union {
struct {
unsigned TMR1H :8;
};
} TMR1Hbits_t;
extern volatile TMR1Hbits_t TMR1Hbits __at(0x017);

# 868
extern volatile unsigned char T1CON __at(0x018);

asm("T1CON equ 018h");


typedef union {
struct {
unsigned TMR1ON :1;
unsigned :1;
unsigned nT1SYNC :1;
unsigned T1OSCEN :1;
unsigned T1CKPS0 :1;
unsigned T1CKPS1 :1;
unsigned TMR1CS0 :1;
unsigned TMR1CS1 :1;
};
struct {
unsigned :4;
unsigned T1CKPS :2;
unsigned TMR1CS :2;
};
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __at(0x018);

# 940
extern volatile unsigned char T1GCON __at(0x019);

asm("T1GCON equ 019h");


typedef union {
struct {
unsigned T1GSS0 :1;
unsigned T1GSS1 :1;
unsigned T1GVAL :1;
unsigned T1GGO_nDONE :1;
unsigned T1GSPM :1;
unsigned T1GTM :1;
unsigned T1GPOL :1;
unsigned TMR1GE :1;
};
struct {
unsigned T1GSS :2;
unsigned :1;
unsigned T1GGO :1;
};
} T1GCONbits_t;
extern volatile T1GCONbits_t T1GCONbits __at(0x019);

# 1017
extern volatile unsigned char TMR2 __at(0x01A);

asm("TMR2 equ 01Ah");


typedef union {
struct {
unsigned TMR2 :8;
};
} TMR2bits_t;
extern volatile TMR2bits_t TMR2bits __at(0x01A);

# 1037
extern volatile unsigned char PR2 __at(0x01B);

asm("PR2 equ 01Bh");


typedef union {
struct {
unsigned PR2 :8;
};
} PR2bits_t;
extern volatile PR2bits_t PR2bits __at(0x01B);

# 1057
extern volatile unsigned char T2CON __at(0x01C);

asm("T2CON equ 01Ch");


typedef union {
struct {
unsigned T2CKPS0 :1;
unsigned T2CKPS1 :1;
unsigned TMR2ON :1;
unsigned T2OUTPS0 :1;
unsigned T2OUTPS1 :1;
unsigned T2OUTPS2 :1;
unsigned T2OUTPS3 :1;
};
struct {
unsigned T2CKPS :2;
unsigned :1;
unsigned T2OUTPS :4;
};
} T2CONbits_t;
extern volatile T2CONbits_t T2CONbits __at(0x01C);

# 1128
extern volatile unsigned char CPSCON0 __at(0x01E);

asm("CPSCON0 equ 01Eh");


typedef union {
struct {
unsigned T0XCS :1;
unsigned CPSOUT :1;
unsigned CPSRNG0 :1;
unsigned CPSRNG1 :1;
unsigned :3;
unsigned CPSON :1;
};
struct {
unsigned :2;
unsigned CPSRNG :2;
};
} CPSCON0bits_t;
extern volatile CPSCON0bits_t CPSCON0bits __at(0x01E);

# 1182
extern volatile unsigned char CPSCON1 __at(0x01F);

asm("CPSCON1 equ 01Fh");


typedef union {
struct {
unsigned CPSCH0 :1;
unsigned CPSCH1 :1;
unsigned CPSCH2 :1;
};
struct {
unsigned CPSCH :3;
};
} CPSCON1bits_t;
extern volatile CPSCON1bits_t CPSCON1bits __at(0x01F);

# 1222
extern volatile unsigned char TRISA __at(0x08C);

asm("TRISA equ 08Ch");


typedef union {
struct {
unsigned TRISA0 :1;
unsigned TRISA1 :1;
unsigned TRISA2 :1;
unsigned TRISA3 :1;
unsigned TRISA4 :1;
unsigned TRISA5 :1;
unsigned TRISA6 :1;
unsigned TRISA7 :1;
};
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __at(0x08C);

# 1284
extern volatile unsigned char TRISB __at(0x08D);

asm("TRISB equ 08Dh");


typedef union {
struct {
unsigned TRISB0 :1;
unsigned TRISB1 :1;
unsigned TRISB2 :1;
unsigned TRISB3 :1;
unsigned TRISB4 :1;
unsigned TRISB5 :1;
unsigned TRISB6 :1;
unsigned TRISB7 :1;
};
} TRISBbits_t;
extern volatile TRISBbits_t TRISBbits __at(0x08D);

# 1346
extern volatile unsigned char TRISC __at(0x08E);

asm("TRISC equ 08Eh");


typedef union {
struct {
unsigned TRISC0 :1;
unsigned TRISC1 :1;
unsigned TRISC2 :1;
unsigned TRISC3 :1;
unsigned TRISC4 :1;
unsigned TRISC5 :1;
unsigned TRISC6 :1;
unsigned TRISC7 :1;
};
} TRISCbits_t;
extern volatile TRISCbits_t TRISCbits __at(0x08E);

# 1408
extern volatile unsigned char TRISE __at(0x090);

asm("TRISE equ 090h");


typedef union {
struct {
unsigned :3;
unsigned TRISE3 :1;
};
} TRISEbits_t;
extern volatile TRISEbits_t TRISEbits __at(0x090);

# 1429
extern volatile unsigned char PIE1 __at(0x091);

asm("PIE1 equ 091h");


typedef union {
struct {
unsigned TMR1IE :1;
unsigned TMR2IE :1;
unsigned CCP1IE :1;
unsigned SSPIE :1;
unsigned TXIE :1;
unsigned RCIE :1;
unsigned ADIE :1;
unsigned TMR1GIE :1;
};
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __at(0x091);

# 1491
extern volatile unsigned char PIE2 __at(0x092);

asm("PIE2 equ 092h");


typedef union {
struct {
unsigned CCP2IE :1;
unsigned :1;
unsigned LCDIE :1;
unsigned BCLIE :1;
unsigned EEIE :1;
unsigned C1IE :1;
unsigned C2IE :1;
unsigned OSFIE :1;
};
} PIE2bits_t;
extern volatile PIE2bits_t PIE2bits __at(0x092);

# 1548
extern volatile unsigned char PIE3 __at(0x093);

asm("PIE3 equ 093h");


typedef union {
struct {
unsigned :1;
unsigned TMR4IE :1;
unsigned :1;
unsigned TMR6IE :1;
unsigned CCP3IE :1;
unsigned CCP4IE :1;
unsigned CCP5IE :1;
};
} PIE3bits_t;
extern volatile PIE3bits_t PIE3bits __at(0x093);

# 1594
extern volatile unsigned char OPTION_REG __at(0x095);

asm("OPTION_REG equ 095h");


typedef union {
struct {
unsigned PS0 :1;
unsigned PS1 :1;
unsigned PS2 :1;
unsigned PSA :1;
unsigned TMR0SE :1;
unsigned TMR0CS :1;
unsigned INTEDG :1;
unsigned nWPUEN :1;
};
struct {
unsigned PS :3;
unsigned :1;
unsigned T0SE :1;
unsigned T0CS :1;
};
} OPTION_REGbits_t;
extern volatile OPTION_REGbits_t OPTION_REGbits __at(0x095);

# 1677
extern volatile unsigned char PCON __at(0x096);

asm("PCON equ 096h");


typedef union {
struct {
unsigned nBOR :1;
unsigned nPOR :1;
unsigned nRI :1;
unsigned nRMCLR :1;
unsigned :2;
unsigned STKUNF :1;
unsigned STKOVF :1;
};
} PCONbits_t;
extern volatile PCONbits_t PCONbits __at(0x096);

# 1728
extern volatile unsigned char WDTCON __at(0x097);

asm("WDTCON equ 097h");


typedef union {
struct {
unsigned SWDTEN :1;
unsigned WDTPS0 :1;
unsigned WDTPS1 :1;
unsigned WDTPS2 :1;
unsigned WDTPS3 :1;
unsigned WDTPS4 :1;
};
struct {
unsigned :1;
unsigned WDTPS :5;
};
} WDTCONbits_t;
extern volatile WDTCONbits_t WDTCONbits __at(0x097);

# 1787
extern volatile unsigned char OSCTUNE __at(0x098);

asm("OSCTUNE equ 098h");


typedef union {
struct {
unsigned TUN0 :1;
unsigned TUN1 :1;
unsigned TUN2 :1;
unsigned TUN3 :1;
unsigned TUN4 :1;
unsigned TUN5 :1;
};
struct {
unsigned TUN :6;
};
} OSCTUNEbits_t;
extern volatile OSCTUNEbits_t OSCTUNEbits __at(0x098);

# 1845
extern volatile unsigned char OSCCON __at(0x099);

asm("OSCCON equ 099h");


typedef union {
struct {
unsigned SCS0 :1;
unsigned SCS1 :1;
unsigned :1;
unsigned IRCF0 :1;
unsigned IRCF1 :1;
unsigned IRCF2 :1;
unsigned IRCF3 :1;
unsigned SPLLEN :1;
};
struct {
unsigned SCS :2;
unsigned :1;
unsigned IRCF :4;
};
} OSCCONbits_t;
extern volatile OSCCONbits_t OSCCONbits __at(0x099);

# 1917
extern volatile unsigned char OSCSTAT __at(0x09A);

asm("OSCSTAT equ 09Ah");


typedef union {
struct {
unsigned HFIOFS :1;
unsigned LFIOFR :1;
unsigned MFIOFR :1;
unsigned HFIOFL :1;
unsigned HFIOFR :1;
unsigned OSTS :1;
unsigned PLLR :1;
unsigned T1OSCR :1;
};
} OSCSTATbits_t;
extern volatile OSCSTATbits_t OSCSTATbits __at(0x09A);

# 1979
extern volatile unsigned short ADRES __at(0x09B);

asm("ADRES equ 09Bh");




extern volatile unsigned char ADRESL __at(0x09B);

asm("ADRESL equ 09Bh");


typedef union {
struct {
unsigned ADRESL :8;
};
} ADRESLbits_t;
extern volatile ADRESLbits_t ADRESLbits __at(0x09B);

# 2006
extern volatile unsigned char ADRESH __at(0x09C);

asm("ADRESH equ 09Ch");


typedef union {
struct {
unsigned ADRESH :8;
};
} ADRESHbits_t;
extern volatile ADRESHbits_t ADRESHbits __at(0x09C);

# 2026
extern volatile unsigned char ADCON0 __at(0x09D);

asm("ADCON0 equ 09Dh");


typedef union {
struct {
unsigned ADON :1;
unsigned GO_nDONE :1;
unsigned CHS0 :1;
unsigned CHS1 :1;
unsigned CHS2 :1;
unsigned CHS3 :1;
unsigned CHS4 :1;
};
struct {
unsigned :1;
unsigned ADGO :1;
unsigned CHS :5;
};
struct {
unsigned :1;
unsigned GO :1;
};
struct {
unsigned :1;
unsigned nDONE :1;
};
} ADCON0bits_t;
extern volatile ADCON0bits_t ADCON0bits __at(0x09D);

# 2115
extern volatile unsigned char ADCON1 __at(0x09E);

asm("ADCON1 equ 09Eh");


typedef union {
struct {
unsigned ADPREF0 :1;
unsigned ADPREF1 :1;
unsigned ADNREF :1;
unsigned :1;
unsigned ADCS0 :1;
unsigned ADCS1 :1;
unsigned ADCS2 :1;
unsigned ADFM :1;
};
struct {
unsigned ADPREF :2;
unsigned :2;
unsigned ADCS :3;
};
} ADCON1bits_t;
extern volatile ADCON1bits_t ADCON1bits __at(0x09E);

# 2187
extern volatile unsigned char LATA __at(0x10C);

asm("LATA equ 010Ch");


typedef union {
struct {
unsigned LATA0 :1;
unsigned LATA1 :1;
unsigned LATA2 :1;
unsigned LATA3 :1;
unsigned LATA4 :1;
unsigned LATA5 :1;
unsigned LATA6 :1;
unsigned LATA7 :1;
};
} LATAbits_t;
extern volatile LATAbits_t LATAbits __at(0x10C);

# 2249
extern volatile unsigned char LATB __at(0x10D);

asm("LATB equ 010Dh");


typedef union {
struct {
unsigned LATB0 :1;
unsigned LATB1 :1;
unsigned LATB2 :1;
unsigned LATB3 :1;
unsigned LATB4 :1;
unsigned LATB5 :1;
unsigned LATB6 :1;
unsigned LATB7 :1;
};
} LATBbits_t;
extern volatile LATBbits_t LATBbits __at(0x10D);

# 2311
extern volatile unsigned char LATC __at(0x10E);

asm("LATC equ 010Eh");


typedef union {
struct {
unsigned LATC0 :1;
unsigned LATC1 :1;
unsigned LATC2 :1;
unsigned LATC3 :1;
unsigned LATC4 :1;
unsigned LATC5 :1;
unsigned LATC6 :1;
unsigned LATC7 :1;
};
} LATCbits_t;
extern volatile LATCbits_t LATCbits __at(0x10E);

# 2373
extern volatile unsigned char LATE __at(0x110);

asm("LATE equ 0110h");


typedef union {
struct {
unsigned :3;
unsigned LATE3 :1;
};
} LATEbits_t;
extern volatile LATEbits_t LATEbits __at(0x110);

# 2394
extern volatile unsigned char CM1CON0 __at(0x111);

asm("CM1CON0 equ 0111h");


typedef union {
struct {
unsigned C1SYNC :1;
unsigned C1HYS :1;
unsigned C1SP :1;
unsigned :1;
unsigned C1POL :1;
unsigned C1OE :1;
unsigned C1OUT :1;
unsigned C1ON :1;
};
} CM1CON0bits_t;
extern volatile CM1CON0bits_t CM1CON0bits __at(0x111);

# 2451
extern volatile unsigned char CM1CON1 __at(0x112);

asm("CM1CON1 equ 0112h");


typedef union {
struct {
unsigned C1NCH0 :1;
unsigned C1NCH1 :1;
unsigned :2;
unsigned C1PCH0 :1;
unsigned C1PCH1 :1;
unsigned C1INTN :1;
unsigned C1INTP :1;
};
struct {
unsigned C1NCH :2;
unsigned :2;
unsigned C1PCH :2;
};
} CM1CON1bits_t;
extern volatile CM1CON1bits_t CM1CON1bits __at(0x112);

# 2517
extern volatile unsigned char CM2CON0 __at(0x113);

asm("CM2CON0 equ 0113h");


typedef union {
struct {
unsigned C2SYNC :1;
unsigned C2HYS :1;
unsigned C2SP :1;
unsigned :1;
unsigned C2POL :1;
unsigned C2OE :1;
unsigned C2OUT :1;
unsigned C2ON :1;
};
} CM2CON0bits_t;
extern volatile CM2CON0bits_t CM2CON0bits __at(0x113);

# 2574
extern volatile unsigned char CM2CON1 __at(0x114);

asm("CM2CON1 equ 0114h");


typedef union {
struct {
unsigned C2NCH0 :1;
unsigned C2NCH1 :1;
unsigned :2;
unsigned C2PCH0 :1;
unsigned C2PCH1 :1;
unsigned C2INTN :1;
unsigned C2INTP :1;
};
struct {
unsigned C2NCH :2;
unsigned :2;
unsigned C2PCH :2;
};
} CM2CON1bits_t;
extern volatile CM2CON1bits_t CM2CON1bits __at(0x114);

# 2640
extern volatile unsigned char CMOUT __at(0x115);

asm("CMOUT equ 0115h");


typedef union {
struct {
unsigned MC1OUT :1;
unsigned MC2OUT :1;
};
} CMOUTbits_t;
extern volatile CMOUTbits_t CMOUTbits __at(0x115);

# 2666
extern volatile unsigned char BORCON __at(0x116);

asm("BORCON equ 0116h");


typedef union {
struct {
unsigned BORRDY :1;
unsigned :6;
unsigned SBOREN :1;
};
} BORCONbits_t;
extern volatile BORCONbits_t BORCONbits __at(0x116);

# 2693
extern volatile unsigned char FVRCON __at(0x117);

asm("FVRCON equ 0117h");


typedef union {
struct {
unsigned ADFVR0 :1;
unsigned ADFVR1 :1;
unsigned CDAFVR0 :1;
unsigned CDAFVR1 :1;
unsigned TSRNG :1;
unsigned TSEN :1;
unsigned FVRRDY :1;
unsigned FVREN :1;
};
struct {
unsigned ADFVR :2;
unsigned CDAFVR :2;
};
} FVRCONbits_t;
extern volatile FVRCONbits_t FVRCONbits __at(0x117);

# 2769
extern volatile unsigned char DACCON0 __at(0x118);

asm("DACCON0 equ 0118h");


typedef union {
struct {
unsigned DACNSS :1;
unsigned :1;
unsigned DACPSS0 :1;
unsigned DACPSS1 :1;
unsigned :1;
unsigned DACOE :1;
unsigned DACLPS :1;
unsigned DACEN :1;
};
struct {
unsigned :2;
unsigned DACPSS :2;
};
} DACCON0bits_t;
extern volatile DACCON0bits_t DACCON0bits __at(0x118);

# 2830
extern volatile unsigned char DACCON1 __at(0x119);

asm("DACCON1 equ 0119h");


typedef union {
struct {
unsigned DACR0 :1;
unsigned DACR1 :1;
unsigned DACR2 :1;
unsigned DACR3 :1;
unsigned DACR4 :1;
};
struct {
unsigned DACR :5;
};
} DACCON1bits_t;
extern volatile DACCON1bits_t DACCON1bits __at(0x119);

# 2882
extern volatile unsigned char SRCON0 __at(0x11A);

asm("SRCON0 equ 011Ah");


typedef union {
struct {
unsigned SRPR :1;
unsigned SRPS :1;
unsigned SRNQEN :1;
unsigned SRQEN :1;
unsigned SRCLK0 :1;
unsigned SRCLK1 :1;
unsigned SRCLK2 :1;
unsigned SRLEN :1;
};
struct {
unsigned :4;
unsigned SRCLK :3;
};
} SRCON0bits_t;
extern volatile SRCON0bits_t SRCON0bits __at(0x11A);

# 2953
extern volatile unsigned char SRCON1 __at(0x11B);

asm("SRCON1 equ 011Bh");


typedef union {
struct {
unsigned SRRC1E :1;
unsigned SRRC2E :1;
unsigned SRRCKE :1;
unsigned SRRPE :1;
unsigned SRSC1E :1;
unsigned SRSC2E :1;
unsigned SRSCKE :1;
unsigned SRSPE :1;
};
} SRCON1bits_t;
extern volatile SRCON1bits_t SRCON1bits __at(0x11B);

# 3015
extern volatile unsigned char APFCON __at(0x11D);

asm("APFCON equ 011Dh");


typedef union {
struct {
unsigned CCP2SEL :1;
unsigned SSSEL :1;
unsigned C2OUTSEL :1;
unsigned SRNQSEL :1;
unsigned P2BSEL :1;
unsigned T1GSEL :1;
unsigned CCP3SEL :1;
};
} APFCONbits_t;
extern volatile APFCONbits_t APFCONbits __at(0x11D);

# 3071
extern volatile unsigned char ANSELA __at(0x18C);

asm("ANSELA equ 018Ch");


typedef union {
struct {
unsigned ANSA0 :1;
unsigned ANSA1 :1;
unsigned ANSA2 :1;
unsigned ANSA3 :1;
unsigned ANSA4 :1;
unsigned ANSA5 :1;
};
struct {
unsigned ANSELA :6;
};
} ANSELAbits_t;
extern volatile ANSELAbits_t ANSELAbits __at(0x18C);

# 3129
extern volatile unsigned char ANSELB __at(0x18D);

asm("ANSELB equ 018Dh");


typedef union {
struct {
unsigned ANSB0 :1;
unsigned ANSB1 :1;
unsigned ANSB2 :1;
unsigned ANSB3 :1;
unsigned ANSB4 :1;
unsigned ANSB5 :1;
};
struct {
unsigned ANSELB :6;
};
} ANSELBbits_t;
extern volatile ANSELBbits_t ANSELBbits __at(0x18D);

# 3187
extern volatile unsigned short EEADR __at(0x191);

asm("EEADR equ 0191h");




extern volatile unsigned char EEADRL __at(0x191);

asm("EEADRL equ 0191h");


typedef union {
struct {
unsigned EEADRL :8;
};
} EEADRLbits_t;
extern volatile EEADRLbits_t EEADRLbits __at(0x191);

# 3214
extern volatile unsigned char EEADRH __at(0x192);

asm("EEADRH equ 0192h");


typedef union {
struct {
unsigned EEADRH :7;
};
} EEADRHbits_t;
extern volatile EEADRHbits_t EEADRHbits __at(0x192);

# 3234
extern volatile unsigned short EEDAT __at(0x193);

asm("EEDAT equ 0193h");




extern volatile unsigned char EEDATL __at(0x193);

asm("EEDATL equ 0193h");


extern volatile unsigned char EEDATA __at(0x193);

asm("EEDATA equ 0193h");


typedef union {
struct {
unsigned EEDATL :8;
};
} EEDATLbits_t;
extern volatile EEDATLbits_t EEDATLbits __at(0x193);

# 3264
typedef union {
struct {
unsigned EEDATL :8;
};
} EEDATAbits_t;
extern volatile EEDATAbits_t EEDATAbits __at(0x193);

# 3279
extern volatile unsigned char EEDATH __at(0x194);

asm("EEDATH equ 0194h");


typedef union {
struct {
unsigned EEDATH :6;
};
} EEDATHbits_t;
extern volatile EEDATHbits_t EEDATHbits __at(0x194);

# 3299
extern volatile unsigned char EECON1 __at(0x195);

asm("EECON1 equ 0195h");


typedef union {
struct {
unsigned RD :1;
unsigned WR :1;
unsigned WREN :1;
unsigned WRERR :1;
unsigned FREE :1;
unsigned LWLO :1;
unsigned CFGS :1;
unsigned EEPGD :1;
};
} EECON1bits_t;
extern volatile EECON1bits_t EECON1bits __at(0x195);

# 3361
extern volatile unsigned char EECON2 __at(0x196);

asm("EECON2 equ 0196h");


typedef union {
struct {
unsigned EECON2 :8;
};
} EECON2bits_t;
extern volatile EECON2bits_t EECON2bits __at(0x196);

# 3381
extern volatile unsigned char RCREG __at(0x199);

asm("RCREG equ 0199h");


typedef union {
struct {
unsigned RCREG :8;
};
} RCREGbits_t;
extern volatile RCREGbits_t RCREGbits __at(0x199);

# 3401
extern volatile unsigned char TXREG __at(0x19A);

asm("TXREG equ 019Ah");


typedef union {
struct {
unsigned TXREG :8;
};
} TXREGbits_t;
extern volatile TXREGbits_t TXREGbits __at(0x19A);

# 3421
extern volatile unsigned short SP1BRG __at(0x19B);

asm("SP1BRG equ 019Bh");




extern volatile unsigned char SP1BRGL __at(0x19B);

asm("SP1BRGL equ 019Bh");


extern volatile unsigned char SPBRG __at(0x19B);

asm("SPBRG equ 019Bh");

extern volatile unsigned char SPBRGL __at(0x19B);

asm("SPBRGL equ 019Bh");


typedef union {
struct {
unsigned SPBRGL :8;
};
} SP1BRGLbits_t;
extern volatile SP1BRGLbits_t SP1BRGLbits __at(0x19B);

# 3455
typedef union {
struct {
unsigned SPBRGL :8;
};
} SPBRGbits_t;
extern volatile SPBRGbits_t SPBRGbits __at(0x19B);

# 3467
typedef union {
struct {
unsigned SPBRGL :8;
};
} SPBRGLbits_t;
extern volatile SPBRGLbits_t SPBRGLbits __at(0x19B);

# 3482
extern volatile unsigned char SP1BRGH __at(0x19C);

asm("SP1BRGH equ 019Ch");


extern volatile unsigned char SPBRGH __at(0x19C);

asm("SPBRGH equ 019Ch");


typedef union {
struct {
unsigned SPBRGH :8;
};
} SP1BRGHbits_t;
extern volatile SP1BRGHbits_t SP1BRGHbits __at(0x19C);

# 3505
typedef union {
struct {
unsigned SPBRGH :8;
};
} SPBRGHbits_t;
extern volatile SPBRGHbits_t SPBRGHbits __at(0x19C);

# 3520
extern volatile unsigned char RCSTA __at(0x19D);

asm("RCSTA equ 019Dh");


typedef union {
struct {
unsigned RX9D :1;
unsigned OERR :1;
unsigned FERR :1;
unsigned ADDEN :1;
unsigned CREN :1;
unsigned SREN :1;
unsigned RX9 :1;
unsigned SPEN :1;
};
} RCSTAbits_t;
extern volatile RCSTAbits_t RCSTAbits __at(0x19D);

# 3582
extern volatile unsigned char TXSTA __at(0x19E);

asm("TXSTA equ 019Eh");


typedef union {
struct {
unsigned TX9D :1;
unsigned TRMT :1;
unsigned BRGH :1;
unsigned SENDB :1;
unsigned SYNC :1;
unsigned TXEN :1;
unsigned TX9 :1;
unsigned CSRC :1;
};
} TXSTAbits_t;
extern volatile TXSTAbits_t TXSTAbits __at(0x19E);

# 3644
extern volatile unsigned char BAUDCON __at(0x19F);

asm("BAUDCON equ 019Fh");


typedef union {
struct {
unsigned ABDEN :1;
unsigned WUE :1;
unsigned :1;
unsigned BRG16 :1;
unsigned SCKP :1;
unsigned :1;
unsigned RCIDL :1;
unsigned ABDOVF :1;
};
} BAUDCONbits_t;
extern volatile BAUDCONbits_t BAUDCONbits __at(0x19F);

# 3696
extern volatile unsigned char WPUB __at(0x20D);

asm("WPUB equ 020Dh");


typedef union {
struct {
unsigned WPUB0 :1;
unsigned WPUB1 :1;
unsigned WPUB2 :1;
unsigned WPUB3 :1;
unsigned WPUB4 :1;
unsigned WPUB5 :1;
unsigned WPUB6 :1;
unsigned WPUB7 :1;
};
struct {
unsigned WPUB :8;
};
} WPUBbits_t;
extern volatile WPUBbits_t WPUBbits __at(0x20D);

# 3766
extern volatile unsigned char WPUE __at(0x210);

asm("WPUE equ 0210h");


typedef union {
struct {
unsigned :3;
unsigned WPUE3 :1;
};
} WPUEbits_t;
extern volatile WPUEbits_t WPUEbits __at(0x210);

# 3787
extern volatile unsigned char SSPBUF __at(0x211);

asm("SSPBUF equ 0211h");


typedef union {
struct {
unsigned SSPBUF :8;
};
} SSPBUFbits_t;
extern volatile SSPBUFbits_t SSPBUFbits __at(0x211);

# 3807
extern volatile unsigned char SSPADD __at(0x212);

asm("SSPADD equ 0212h");


typedef union {
struct {
unsigned SSPADD :8;
};
} SSPADDbits_t;
extern volatile SSPADDbits_t SSPADDbits __at(0x212);

# 3827
extern volatile unsigned char SSPMSK __at(0x213);

asm("SSPMSK equ 0213h");


typedef union {
struct {
unsigned SSPMSK :8;
};
} SSPMSKbits_t;
extern volatile SSPMSKbits_t SSPMSKbits __at(0x213);

# 3847
extern volatile unsigned char SSPSTAT __at(0x214);

asm("SSPSTAT equ 0214h");


typedef union {
struct {
unsigned BF :1;
unsigned UA :1;
unsigned R_nW :1;
unsigned S :1;
unsigned P :1;
unsigned D_nA :1;
unsigned CKE :1;
unsigned SMP :1;
};
} SSPSTATbits_t;
extern volatile SSPSTATbits_t SSPSTATbits __at(0x214);

# 3909
extern volatile unsigned char SSPCON1 __at(0x215);

asm("SSPCON1 equ 0215h");


extern volatile unsigned char SSPCON __at(0x215);

asm("SSPCON equ 0215h");


typedef union {
struct {
unsigned SSPM0 :1;
unsigned SSPM1 :1;
unsigned SSPM2 :1;
unsigned SSPM3 :1;
unsigned CKP :1;
unsigned SSPEN :1;
unsigned SSPOV :1;
unsigned WCOL :1;
};
} SSPCON1bits_t;
extern volatile SSPCON1bits_t SSPCON1bits __at(0x215);

# 3974
typedef union {
struct {
unsigned SSPM0 :1;
unsigned SSPM1 :1;
unsigned SSPM2 :1;
unsigned SSPM3 :1;
unsigned CKP :1;
unsigned SSPEN :1;
unsigned SSPOV :1;
unsigned WCOL :1;
};
} SSPCONbits_t;
extern volatile SSPCONbits_t SSPCONbits __at(0x215);

# 4031
extern volatile unsigned char SSPCON2 __at(0x216);

asm("SSPCON2 equ 0216h");


typedef union {
struct {
unsigned SEN :1;
unsigned RSEN :1;
unsigned PEN :1;
unsigned RCEN :1;
unsigned ACKEN :1;
unsigned ACKDT :1;
unsigned ACKSTAT :1;
unsigned GCEN :1;
};
} SSPCON2bits_t;
extern volatile SSPCON2bits_t SSPCON2bits __at(0x216);

# 4093
extern volatile unsigned char SSPCON3 __at(0x217);

asm("SSPCON3 equ 0217h");


typedef union {
struct {
unsigned DHEN :1;
unsigned AHEN :1;
unsigned SBCDE :1;
unsigned SDAHT :1;
unsigned BOEN :1;
unsigned SCIE :1;
unsigned PCIE :1;
unsigned ACKTIM :1;
};
} SSPCON3bits_t;
extern volatile SSPCON3bits_t SSPCON3bits __at(0x217);

# 4155
extern volatile unsigned short CCPR1 __at(0x291);

asm("CCPR1 equ 0291h");




extern volatile unsigned char CCPR1L __at(0x291);

asm("CCPR1L equ 0291h");


typedef union {
struct {
unsigned CCPR1L :8;
};
} CCPR1Lbits_t;
extern volatile CCPR1Lbits_t CCPR1Lbits __at(0x291);

# 4182
extern volatile unsigned char CCPR1H __at(0x292);

asm("CCPR1H equ 0292h");


typedef union {
struct {
unsigned CCPR1H :8;
};
} CCPR1Hbits_t;
extern volatile CCPR1Hbits_t CCPR1Hbits __at(0x292);

# 4202
extern volatile unsigned char CCP1CON __at(0x293);

asm("CCP1CON equ 0293h");


typedef union {
struct {
unsigned CCP1M0 :1;
unsigned CCP1M1 :1;
unsigned CCP1M2 :1;
unsigned CCP1M3 :1;
unsigned DC1B0 :1;
unsigned DC1B1 :1;
unsigned P1M0 :1;
unsigned P1M1 :1;
};
struct {
unsigned CCP1M :4;
unsigned DC1B :2;
unsigned P1M :2;
};
} CCP1CONbits_t;
extern volatile CCP1CONbits_t CCP1CONbits __at(0x293);

# 4284
extern volatile unsigned char PWM1CON __at(0x294);

asm("PWM1CON equ 0294h");


typedef union {
struct {
unsigned P1DC0 :1;
unsigned P1DC1 :1;
unsigned P1DC2 :1;
unsigned P1DC3 :1;
unsigned P1DC4 :1;
unsigned P1DC5 :1;
unsigned P1DC6 :1;
unsigned P1RSEN :1;
};
} PWM1CONbits_t;
extern volatile PWM1CONbits_t PWM1CONbits __at(0x294);

# 4346
extern volatile unsigned char CCP1AS __at(0x295);

asm("CCP1AS equ 0295h");


extern volatile unsigned char ECCP1AS __at(0x295);

asm("ECCP1AS equ 0295h");


typedef union {
struct {
unsigned PSS1BD0 :1;
unsigned PSS1BD1 :1;
unsigned PSS1AC0 :1;
unsigned PSS1AC1 :1;
unsigned CCP1AS0 :1;
unsigned CCP1AS1 :1;
unsigned CCP1AS2 :1;
unsigned CCP1ASE :1;
};
} CCP1ASbits_t;
extern volatile CCP1ASbits_t CCP1ASbits __at(0x295);

# 4411
typedef union {
struct {
unsigned PSS1BD0 :1;
unsigned PSS1BD1 :1;
unsigned PSS1AC0 :1;
unsigned PSS1AC1 :1;
unsigned CCP1AS0 :1;
unsigned CCP1AS1 :1;
unsigned CCP1AS2 :1;
unsigned CCP1ASE :1;
};
} ECCP1ASbits_t;
extern volatile ECCP1ASbits_t ECCP1ASbits __at(0x295);

# 4468
extern volatile unsigned char PSTR1CON __at(0x296);

asm("PSTR1CON equ 0296h");


typedef union {
struct {
unsigned STR1A :1;
unsigned STR1B :1;
unsigned STR1C :1;
unsigned STR1D :1;
unsigned STR1SYNC :1;
};
} PSTR1CONbits_t;
extern volatile PSTR1CONbits_t PSTR1CONbits __at(0x296);

# 4512
extern volatile unsigned short CCPR2 __at(0x298);

asm("CCPR2 equ 0298h");




extern volatile unsigned char CCPR2L __at(0x298);

asm("CCPR2L equ 0298h");


typedef union {
struct {
unsigned CCPR2L :8;
};
} CCPR2Lbits_t;
extern volatile CCPR2Lbits_t CCPR2Lbits __at(0x298);

# 4539
extern volatile unsigned char CCPR2H __at(0x299);

asm("CCPR2H equ 0299h");


typedef union {
struct {
unsigned CCPR2H :8;
};
} CCPR2Hbits_t;
extern volatile CCPR2Hbits_t CCPR2Hbits __at(0x299);

# 4559
extern volatile unsigned char CCP2CON __at(0x29A);

asm("CCP2CON equ 029Ah");


typedef union {
struct {
unsigned CCP2M0 :1;
unsigned CCP2M1 :1;
unsigned CCP2M2 :1;
unsigned CCP2M3 :1;
unsigned DC2B0 :1;
unsigned DC2B1 :1;
unsigned P2M0 :1;
unsigned P2M1 :1;
};
struct {
unsigned CCP2M :4;
unsigned DC2B :2;
unsigned P2M :2;
};
} CCP2CONbits_t;
extern volatile CCP2CONbits_t CCP2CONbits __at(0x29A);

# 4641
extern volatile unsigned char PWM2CON __at(0x29B);

asm("PWM2CON equ 029Bh");


typedef union {
struct {
unsigned P2DC0 :1;
unsigned P2DC1 :1;
unsigned P2DC2 :1;
unsigned P2DC3 :1;
unsigned P2DC4 :1;
unsigned P2DC5 :1;
unsigned P2DC6 :1;
unsigned P2RSEN :1;
};
} PWM2CONbits_t;
extern volatile PWM2CONbits_t PWM2CONbits __at(0x29B);

# 4703
extern volatile unsigned char CCP2AS __at(0x29C);

asm("CCP2AS equ 029Ch");


extern volatile unsigned char ECCP2AS __at(0x29C);

asm("ECCP2AS equ 029Ch");


typedef union {
struct {
unsigned PSS2BD0 :1;
unsigned PSS2BD1 :1;
unsigned PSS2AC0 :1;
unsigned PSS2AC1 :1;
unsigned CCP2AS0 :1;
unsigned CCP2AS1 :1;
unsigned CCP2AS2 :1;
unsigned CCP2ASE :1;
};
} CCP2ASbits_t;
extern volatile CCP2ASbits_t CCP2ASbits __at(0x29C);

# 4768
typedef union {
struct {
unsigned PSS2BD0 :1;
unsigned PSS2BD1 :1;
unsigned PSS2AC0 :1;
unsigned PSS2AC1 :1;
unsigned CCP2AS0 :1;
unsigned CCP2AS1 :1;
unsigned CCP2AS2 :1;
unsigned CCP2ASE :1;
};
} ECCP2ASbits_t;
extern volatile ECCP2ASbits_t ECCP2ASbits __at(0x29C);

# 4825
extern volatile unsigned char PSTR2CON __at(0x29D);

asm("PSTR2CON equ 029Dh");


typedef union {
struct {
unsigned STR2A :1;
unsigned STR2B :1;
unsigned STR2C :1;
unsigned STR2D :1;
unsigned STR2SYNC :1;
};
} PSTR2CONbits_t;
extern volatile PSTR2CONbits_t PSTR2CONbits __at(0x29D);

# 4869
extern volatile unsigned char CCPTMRS0 __at(0x29E);

asm("CCPTMRS0 equ 029Eh");


typedef union {
struct {
unsigned C1TSEL0 :1;
unsigned C1TSEL1 :1;
unsigned C2TSEL0 :1;
unsigned C2TSEL1 :1;
unsigned C3TSEL0 :1;
unsigned C3TSEL1 :1;
unsigned C4TSEL0 :1;
unsigned C4TSEL1 :1;
};
struct {
unsigned C1TSEL :2;
unsigned C2TSEL :2;
unsigned C3TSEL :2;
unsigned C4TSEL :2;
};
} CCPTMRS0bits_t;
extern volatile CCPTMRS0bits_t CCPTMRS0bits __at(0x29E);

# 4957
extern volatile unsigned char CCPTMRS1 __at(0x29F);

asm("CCPTMRS1 equ 029Fh");


typedef union {
struct {
unsigned C5TSEL0 :1;
unsigned C5TSEL1 :1;
};
struct {
unsigned C5TSEL :2;
};
} CCPTMRS1bits_t;
extern volatile CCPTMRS1bits_t CCPTMRS1bits __at(0x29F);

# 4991
extern volatile unsigned short CCPR3 __at(0x311);

asm("CCPR3 equ 0311h");




extern volatile unsigned char CCPR3L __at(0x311);

asm("CCPR3L equ 0311h");


typedef union {
struct {
unsigned CCPR3L :8;
};
} CCPR3Lbits_t;
extern volatile CCPR3Lbits_t CCPR3Lbits __at(0x311);

# 5018
extern volatile unsigned char CCPR3H __at(0x312);

asm("CCPR3H equ 0312h");


typedef union {
struct {
unsigned CCPR3H :8;
};
} CCPR3Hbits_t;
extern volatile CCPR3Hbits_t CCPR3Hbits __at(0x312);

# 5038
extern volatile unsigned char CCP3CON __at(0x313);

asm("CCP3CON equ 0313h");


typedef union {
struct {
unsigned CCP3M0 :1;
unsigned CCP3M1 :1;
unsigned CCP3M2 :1;
unsigned CCP3M3 :1;
unsigned DC3B0 :1;
unsigned DC3B1 :1;
unsigned P3M0 :1;
unsigned P3M1 :1;
};
} CCP3CONbits_t;
extern volatile CCP3CONbits_t CCP3CONbits __at(0x313);

# 5100
extern volatile unsigned char PWM3CON __at(0x314);

asm("PWM3CON equ 0314h");


typedef union {
struct {
unsigned P3DC0 :1;
unsigned P3DC1 :1;
unsigned P3DC2 :1;
unsigned P3DC3 :1;
unsigned P3DC4 :1;
unsigned P3DC5 :1;
unsigned P3DC6 :1;
unsigned P3RSEN :1;
};
} PWM3CONbits_t;
extern volatile PWM3CONbits_t PWM3CONbits __at(0x314);

# 5162
extern volatile unsigned char CCP3AS __at(0x315);

asm("CCP3AS equ 0315h");


extern volatile unsigned char ECCP3AS __at(0x315);

asm("ECCP3AS equ 0315h");


typedef union {
struct {
unsigned PSS3BD0 :1;
unsigned PSS3BD1 :1;
unsigned PSS3AC0 :1;
unsigned PSS3AC1 :1;
unsigned CCP3AS0 :1;
unsigned CCP3AS1 :1;
unsigned CCP3AS2 :1;
unsigned CCP3ASE :1;
};
} CCP3ASbits_t;
extern volatile CCP3ASbits_t CCP3ASbits __at(0x315);

# 5227
typedef union {
struct {
unsigned PSS3BD0 :1;
unsigned PSS3BD1 :1;
unsigned PSS3AC0 :1;
unsigned PSS3AC1 :1;
unsigned CCP3AS0 :1;
unsigned CCP3AS1 :1;
unsigned CCP3AS2 :1;
unsigned CCP3ASE :1;
};
} ECCP3ASbits_t;
extern volatile ECCP3ASbits_t ECCP3ASbits __at(0x315);

# 5284
extern volatile unsigned char PSTR3CON __at(0x316);

asm("PSTR3CON equ 0316h");


typedef union {
struct {
unsigned STR3A :1;
unsigned STR3B :1;
unsigned STR3C :1;
unsigned STR3D :1;
unsigned STR3SYNC :1;
};
} PSTR3CONbits_t;
extern volatile PSTR3CONbits_t PSTR3CONbits __at(0x316);

# 5328
extern volatile unsigned short CCPR4 __at(0x318);

asm("CCPR4 equ 0318h");




extern volatile unsigned char CCPR4L __at(0x318);

asm("CCPR4L equ 0318h");


typedef union {
struct {
unsigned CCPR4L :8;
};
} CCPR4Lbits_t;
extern volatile CCPR4Lbits_t CCPR4Lbits __at(0x318);

# 5355
extern volatile unsigned char CCPR4H __at(0x319);

asm("CCPR4H equ 0319h");


typedef union {
struct {
unsigned CCPR4H :8;
};
} CCPR4Hbits_t;
extern volatile CCPR4Hbits_t CCPR4Hbits __at(0x319);

# 5375
extern volatile unsigned char CCP4CON __at(0x31A);

asm("CCP4CON equ 031Ah");


typedef union {
struct {
unsigned CCP4M0 :1;
unsigned CCP4M1 :1;
unsigned CCP4M2 :1;
unsigned CCP4M3 :1;
unsigned DC4B0 :1;
unsigned DC4B1 :1;
};
} CCP4CONbits_t;
extern volatile CCP4CONbits_t CCP4CONbits __at(0x31A);

# 5425
extern volatile unsigned short CCPR5 __at(0x31C);

asm("CCPR5 equ 031Ch");




extern volatile unsigned char CCPR5L __at(0x31C);

asm("CCPR5L equ 031Ch");


typedef union {
struct {
unsigned CCPR5L :8;
};
} CCPR5Lbits_t;
extern volatile CCPR5Lbits_t CCPR5Lbits __at(0x31C);

# 5452
extern volatile unsigned char CCPR5H __at(0x31D);

asm("CCPR5H equ 031Dh");


typedef union {
struct {
unsigned CCPR5H :8;
};
} CCPR5Hbits_t;
extern volatile CCPR5Hbits_t CCPR5Hbits __at(0x31D);

# 5472
extern volatile unsigned char CCP5CON __at(0x31E);

asm("CCP5CON equ 031Eh");


typedef union {
struct {
unsigned CCP5M0 :1;
unsigned CCP5M1 :1;
unsigned CCP5M2 :1;
unsigned CCP5M3 :1;
unsigned DC5B0 :1;
unsigned DC5B1 :1;
};
} CCP5CONbits_t;
extern volatile CCP5CONbits_t CCP5CONbits __at(0x31E);

# 5522
extern volatile unsigned char IOCBP __at(0x394);

asm("IOCBP equ 0394h");


typedef union {
struct {
unsigned IOCBP0 :1;
unsigned IOCBP1 :1;
unsigned IOCBP2 :1;
unsigned IOCBP3 :1;
unsigned IOCBP4 :1;
unsigned IOCBP5 :1;
unsigned IOCBP6 :1;
unsigned IOCBP7 :1;
};
struct {
unsigned IOCBP :8;
};
} IOCBPbits_t;
extern volatile IOCBPbits_t IOCBPbits __at(0x394);

# 5592
extern volatile unsigned char IOCBN __at(0x395);

asm("IOCBN equ 0395h");


typedef union {
struct {
unsigned IOCBN0 :1;
unsigned IOCBN1 :1;
unsigned IOCBN2 :1;
unsigned IOCBN3 :1;
unsigned IOCBN4 :1;
unsigned IOCBN5 :1;
unsigned IOCBN6 :1;
unsigned IOCBN7 :1;
};
struct {
unsigned IOCBN :8;
};
} IOCBNbits_t;
extern volatile IOCBNbits_t IOCBNbits __at(0x395);

# 5662
extern volatile unsigned char IOCBF __at(0x396);

asm("IOCBF equ 0396h");


typedef union {
struct {
unsigned IOCBF0 :1;
unsigned IOCBF1 :1;
unsigned IOCBF2 :1;
unsigned IOCBF3 :1;
unsigned IOCBF4 :1;
unsigned IOCBF5 :1;
unsigned IOCBF6 :1;
unsigned IOCBF7 :1;
};
struct {
unsigned IOCBF :8;
};
} IOCBFbits_t;
extern volatile IOCBFbits_t IOCBFbits __at(0x396);

# 5732
extern volatile unsigned char TMR4 __at(0x415);

asm("TMR4 equ 0415h");


typedef union {
struct {
unsigned TMR4 :8;
};
} TMR4bits_t;
extern volatile TMR4bits_t TMR4bits __at(0x415);

# 5752
extern volatile unsigned char PR4 __at(0x416);

asm("PR4 equ 0416h");


typedef union {
struct {
unsigned PR4 :8;
};
} PR4bits_t;
extern volatile PR4bits_t PR4bits __at(0x416);

# 5772
extern volatile unsigned char T4CON __at(0x417);

asm("T4CON equ 0417h");


typedef union {
struct {
unsigned T4CKPS0 :1;
unsigned T4CKPS1 :1;
unsigned TMR4ON :1;
unsigned T4OUTPS0 :1;
unsigned T4OUTPS1 :1;
unsigned T4OUTPS2 :1;
unsigned T4OUTPS3 :1;
};
struct {
unsigned T4CKPS :2;
unsigned :1;
unsigned T4OUTPS :4;
};
} T4CONbits_t;
extern volatile T4CONbits_t T4CONbits __at(0x417);

# 5843
extern volatile unsigned char TMR6 __at(0x41C);

asm("TMR6 equ 041Ch");


typedef union {
struct {
unsigned TMR6 :8;
};
} TMR6bits_t;
extern volatile TMR6bits_t TMR6bits __at(0x41C);

# 5863
extern volatile unsigned char PR6 __at(0x41D);

asm("PR6 equ 041Dh");


typedef union {
struct {
unsigned PR6 :8;
};
} PR6bits_t;
extern volatile PR6bits_t PR6bits __at(0x41D);

# 5883
extern volatile unsigned char T6CON __at(0x41E);

asm("T6CON equ 041Eh");


typedef union {
struct {
unsigned T6CKPS0 :1;
unsigned T6CKPS1 :1;
unsigned TMR6ON :1;
unsigned T6OUTPS0 :1;
unsigned T6OUTPS1 :1;
unsigned T6OUTPS2 :1;
unsigned T6OUTPS3 :1;
};
struct {
unsigned T6CKPS :2;
unsigned :1;
unsigned T6OUTPS :4;
};
} T6CONbits_t;
extern volatile T6CONbits_t T6CONbits __at(0x41E);

# 5954
extern volatile unsigned char LCDCON __at(0x791);

asm("LCDCON equ 0791h");


typedef union {
struct {
unsigned LMUX0 :1;
unsigned LMUX1 :1;
unsigned CS0 :1;
unsigned CS1 :1;
unsigned :1;
unsigned WERR :1;
unsigned SLPEN :1;
unsigned LCDEN :1;
};
struct {
unsigned LMUX :2;
unsigned CS :2;
};
} LCDCONbits_t;
extern volatile LCDCONbits_t LCDCONbits __at(0x791);

# 6025
extern volatile unsigned char LCDPS __at(0x792);

asm("LCDPS equ 0792h");


typedef union {
struct {
unsigned LP0 :1;
unsigned LP1 :1;
unsigned LP2 :1;
unsigned LP3 :1;
unsigned WA :1;
unsigned LCDA :1;
unsigned BIASMD :1;
unsigned WFT :1;
};
struct {
unsigned LP :4;
};
} LCDPSbits_t;
extern volatile LCDPSbits_t LCDPSbits __at(0x792);

# 6095
extern volatile unsigned char LCDREF __at(0x793);

asm("LCDREF equ 0793h");


typedef union {
struct {
unsigned :1;
unsigned VLCD1PE :1;
unsigned VLCD2PE :1;
unsigned VLCD3PE :1;
unsigned :1;
unsigned LCDIRI :1;
unsigned LCDIRS :1;
unsigned LCDIRE :1;
};
} LCDREFbits_t;
extern volatile LCDREFbits_t LCDREFbits __at(0x793);

# 6147
extern volatile unsigned char LCDCST __at(0x794);

asm("LCDCST equ 0794h");


typedef union {
struct {
unsigned LCDCST0 :1;
unsigned LCDCST1 :1;
unsigned LCDCST2 :1;
};
struct {
unsigned LCDCST :3;
};
} LCDCSTbits_t;
extern volatile LCDCSTbits_t LCDCSTbits __at(0x794);

# 6187
extern volatile unsigned char LCDRL __at(0x795);

asm("LCDRL equ 0795h");


typedef union {
struct {
unsigned LRLAT0 :1;
unsigned LRLAT1 :1;
unsigned LRLAT2 :1;
unsigned :1;
unsigned LRLBP0 :1;
unsigned LRLBP1 :1;
unsigned LRLAP0 :1;
unsigned LRLAP1 :1;
};
struct {
unsigned LRLAT :3;
unsigned :1;
unsigned LRLBP :2;
unsigned LRLAP :2;
};
} LCDRLbits_t;
extern volatile LCDRLbits_t LCDRLbits __at(0x795);

# 6265
extern volatile unsigned char LCDSE0 __at(0x798);

asm("LCDSE0 equ 0798h");


typedef union {
struct {
unsigned SE0 :1;
unsigned SE1 :1;
unsigned SE2 :1;
unsigned SE3 :1;
unsigned SE4 :1;
unsigned SE5 :1;
unsigned SE6 :1;
unsigned SE7 :1;
};
} LCDSE0bits_t;
extern volatile LCDSE0bits_t LCDSE0bits __at(0x798);

# 6327
extern volatile unsigned char LCDSE1 __at(0x799);

asm("LCDSE1 equ 0799h");


typedef union {
struct {
unsigned SE8 :1;
unsigned SE9 :1;
unsigned SE10 :1;
unsigned SE11 :1;
unsigned SE12 :1;
unsigned SE13 :1;
unsigned SE14 :1;
unsigned SE15 :1;
};
} LCDSE1bits_t;
extern volatile LCDSE1bits_t LCDSE1bits __at(0x799);

# 6389
extern volatile unsigned char LCDDATA0 __at(0x7A0);

asm("LCDDATA0 equ 07A0h");


typedef union {
struct {
unsigned SEG0COM0 :1;
unsigned SEG1COM0 :1;
unsigned SEG2COM0 :1;
unsigned SEG3COM0 :1;
unsigned SEG4COM0 :1;
unsigned SEG5COM0 :1;
unsigned SEG6COM0 :1;
unsigned SEG7COM0 :1;
};
} LCDDATA0bits_t;
extern volatile LCDDATA0bits_t LCDDATA0bits __at(0x7A0);

# 6451
extern volatile unsigned char LCDDATA1 __at(0x7A1);

asm("LCDDATA1 equ 07A1h");


typedef union {
struct {
unsigned SEG8COM0 :1;
unsigned SEG9COM0 :1;
unsigned SEG10COM0 :1;
unsigned SEG11COM0 :1;
unsigned SEG12COM0 :1;
unsigned SEG13COM0 :1;
unsigned SEG14COM0 :1;
unsigned SEG15COM0 :1;
};
} LCDDATA1bits_t;
extern volatile LCDDATA1bits_t LCDDATA1bits __at(0x7A1);

# 6513
extern volatile unsigned char LCDDATA3 __at(0x7A3);

asm("LCDDATA3 equ 07A3h");


typedef union {
struct {
unsigned SEG0COM1 :1;
unsigned SEG1COM1 :1;
unsigned SEG2COM1 :1;
unsigned SEG3COM1 :1;
unsigned SEG4COM1 :1;
unsigned SEG5COM1 :1;
unsigned SEG6COM1 :1;
unsigned SEG7COM1 :1;
};
} LCDDATA3bits_t;
extern volatile LCDDATA3bits_t LCDDATA3bits __at(0x7A3);

# 6575
extern volatile unsigned char LCDDATA4 __at(0x7A4);

asm("LCDDATA4 equ 07A4h");


typedef union {
struct {
unsigned SEG8COM1 :1;
unsigned SEG9COM1 :1;
unsigned SEG10COM1 :1;
unsigned SEG11COM1 :1;
unsigned SEG12COM1 :1;
unsigned SEG13COM1 :1;
unsigned SEG14COM1 :1;
unsigned SEG15COM1 :1;
};
} LCDDATA4bits_t;
extern volatile LCDDATA4bits_t LCDDATA4bits __at(0x7A4);

# 6637
extern volatile unsigned char LCDDATA6 __at(0x7A6);

asm("LCDDATA6 equ 07A6h");


typedef union {
struct {
unsigned SEG0COM2 :1;
unsigned SEG1COM2 :1;
unsigned SEG2COM2 :1;
unsigned SEG3COM2 :1;
unsigned SEG4COM2 :1;
unsigned SEG5COM2 :1;
unsigned SEG6COM2 :1;
unsigned SEG7COM2 :1;
};
} LCDDATA6bits_t;
extern volatile LCDDATA6bits_t LCDDATA6bits __at(0x7A6);

# 6699
extern volatile unsigned char LCDDATA7 __at(0x7A7);

asm("LCDDATA7 equ 07A7h");


typedef union {
struct {
unsigned SEG8COM2 :1;
unsigned SEG9COM2 :1;
unsigned SEG10COM2 :1;
unsigned SEG11COM2 :1;
unsigned SEG12COM2 :1;
unsigned SEG13COM2 :1;
unsigned SEG14COM2 :1;
unsigned SEG15COM2 :1;
};
} LCDDATA7bits_t;
extern volatile LCDDATA7bits_t LCDDATA7bits __at(0x7A7);

# 6761
extern volatile unsigned char LCDDATA9 __at(0x7A9);

asm("LCDDATA9 equ 07A9h");


typedef union {
struct {
unsigned SEG0COM3 :1;
unsigned SEG1COM3 :1;
unsigned SEG2COM3 :1;
unsigned SEG3COM3 :1;
unsigned SEG4COM3 :1;
unsigned SEG5COM3 :1;
unsigned SEG6COM3 :1;
unsigned SEG7COM3 :1;
};
} LCDDATA9bits_t;
extern volatile LCDDATA9bits_t LCDDATA9bits __at(0x7A9);

# 6823
extern volatile unsigned char LCDDATA10 __at(0x7AA);

asm("LCDDATA10 equ 07AAh");


typedef union {
struct {
unsigned SEG8COM3 :1;
unsigned SEG9COM3 :1;
unsigned SEG10COM3 :1;
unsigned SEG11COM3 :1;
unsigned SEG12COM3 :1;
unsigned SEG13COM3 :1;
unsigned SEG14COM3 :1;
unsigned SEG15COM3 :1;
};
} LCDDATA10bits_t;
extern volatile LCDDATA10bits_t LCDDATA10bits __at(0x7AA);

# 6885
extern volatile unsigned char STATUS_SHAD __at(0xFE4);

asm("STATUS_SHAD equ 0FE4h");


typedef union {
struct {
unsigned C_SHAD :1;
unsigned DC_SHAD :1;
unsigned Z_SHAD :1;
};
} STATUS_SHADbits_t;
extern volatile STATUS_SHADbits_t STATUS_SHADbits __at(0xFE4);

# 6917
extern volatile unsigned char WREG_SHAD __at(0xFE5);

asm("WREG_SHAD equ 0FE5h");


typedef union {
struct {
unsigned WREG_SHAD :8;
};
} WREG_SHADbits_t;
extern volatile WREG_SHADbits_t WREG_SHADbits __at(0xFE5);

# 6937
extern volatile unsigned char BSR_SHAD __at(0xFE6);

asm("BSR_SHAD equ 0FE6h");


typedef union {
struct {
unsigned BSR_SHAD :5;
};
} BSR_SHADbits_t;
extern volatile BSR_SHADbits_t BSR_SHADbits __at(0xFE6);

# 6957
extern volatile unsigned char PCLATH_SHAD __at(0xFE7);

asm("PCLATH_SHAD equ 0FE7h");


typedef union {
struct {
unsigned PCLATH_SHAD :7;
};
} PCLATH_SHADbits_t;
extern volatile PCLATH_SHADbits_t PCLATH_SHADbits __at(0xFE7);

# 6977
extern volatile unsigned char FSR0L_SHAD __at(0xFE8);

asm("FSR0L_SHAD equ 0FE8h");


typedef union {
struct {
unsigned FSR0L_SHAD :8;
};
} FSR0L_SHADbits_t;
extern volatile FSR0L_SHADbits_t FSR0L_SHADbits __at(0xFE8);

# 6997
extern volatile unsigned char FSR0H_SHAD __at(0xFE9);

asm("FSR0H_SHAD equ 0FE9h");


typedef union {
struct {
unsigned FSR0H_SHAD :8;
};
} FSR0H_SHADbits_t;
extern volatile FSR0H_SHADbits_t FSR0H_SHADbits __at(0xFE9);

# 7017
extern volatile unsigned char FSR1L_SHAD __at(0xFEA);

asm("FSR1L_SHAD equ 0FEAh");


typedef union {
struct {
unsigned FSR1L_SHAD :8;
};
} FSR1L_SHADbits_t;
extern volatile FSR1L_SHADbits_t FSR1L_SHADbits __at(0xFEA);

# 7037
extern volatile unsigned char FSR1H_SHAD __at(0xFEB);

asm("FSR1H_SHAD equ 0FEBh");


typedef union {
struct {
unsigned FSR1H_SHAD :8;
};
} FSR1H_SHADbits_t;
extern volatile FSR1H_SHADbits_t FSR1H_SHADbits __at(0xFEB);

# 7057
extern volatile unsigned char STKPTR __at(0xFED);

asm("STKPTR equ 0FEDh");


typedef union {
struct {
unsigned STKPTR :5;
};
} STKPTRbits_t;
extern volatile STKPTRbits_t STKPTRbits __at(0xFED);

# 7077
extern volatile unsigned char TOSL __at(0xFEE);

asm("TOSL equ 0FEEh");


typedef union {
struct {
unsigned TOSL :8;
};
} TOSLbits_t;
extern volatile TOSLbits_t TOSLbits __at(0xFEE);

# 7097
extern volatile unsigned char TOSH __at(0xFEF);

asm("TOSH equ 0FEFh");


typedef union {
struct {
unsigned TOSH :7;
};
} TOSHbits_t;
extern volatile TOSHbits_t TOSHbits __at(0xFEF);

# 7123
extern volatile __bit ABDEN __at(0xCF8);


extern volatile __bit ABDOVF __at(0xCFF);


extern volatile __bit ACKDT __at(0x10B5);


extern volatile __bit ACKEN __at(0x10B4);


extern volatile __bit ACKSTAT __at(0x10B6);


extern volatile __bit ACKTIM __at(0x10BF);


extern volatile __bit ADCS0 __at(0x4F4);


extern volatile __bit ADCS1 __at(0x4F5);


extern volatile __bit ADCS2 __at(0x4F6);


extern volatile __bit ADDEN __at(0xCEB);


extern volatile __bit ADFM __at(0x4F7);


extern volatile __bit ADFVR0 __at(0x8B8);


extern volatile __bit ADFVR1 __at(0x8B9);


extern volatile __bit ADGO __at(0x4E9);


extern volatile __bit ADIE __at(0x48E);


extern volatile __bit ADIF __at(0x8E);


extern volatile __bit ADNREF __at(0x4F2);


extern volatile __bit ADON __at(0x4E8);


extern volatile __bit ADPREF0 __at(0x4F0);


extern volatile __bit ADPREF1 __at(0x4F1);


extern volatile __bit AHEN __at(0x10B9);


extern volatile __bit ANSA0 __at(0xC60);


extern volatile __bit ANSA1 __at(0xC61);


extern volatile __bit ANSA2 __at(0xC62);


extern volatile __bit ANSA3 __at(0xC63);


extern volatile __bit ANSA4 __at(0xC64);


extern volatile __bit ANSA5 __at(0xC65);


extern volatile __bit ANSB0 __at(0xC68);


extern volatile __bit ANSB1 __at(0xC69);


extern volatile __bit ANSB2 __at(0xC6A);


extern volatile __bit ANSB3 __at(0xC6B);


extern volatile __bit ANSB4 __at(0xC6C);


extern volatile __bit ANSB5 __at(0xC6D);


extern volatile __bit BCLIE __at(0x493);


extern volatile __bit BCLIF __at(0x93);


extern volatile __bit BF __at(0x10A0);


extern volatile __bit BIASMD __at(0x3C96);


extern volatile __bit BOEN __at(0x10BC);


extern volatile __bit BORRDY __at(0x8B0);


extern volatile __bit BRG16 __at(0xCFB);


extern volatile __bit BRGH __at(0xCF2);


extern volatile __bit BSR0 __at(0x40);


extern volatile __bit BSR1 __at(0x41);


extern volatile __bit BSR2 __at(0x42);


extern volatile __bit BSR3 __at(0x43);


extern volatile __bit BSR4 __at(0x44);


extern volatile __bit C1HYS __at(0x889);


extern volatile __bit C1IE __at(0x495);


extern volatile __bit C1IF __at(0x95);


extern volatile __bit C1INTN __at(0x896);


extern volatile __bit C1INTP __at(0x897);


extern volatile __bit C1NCH0 __at(0x890);


extern volatile __bit C1NCH1 __at(0x891);


extern volatile __bit C1OE __at(0x88D);


extern volatile __bit C1ON __at(0x88F);


extern volatile __bit C1OUT __at(0x88E);


extern volatile __bit C1PCH0 __at(0x894);


extern volatile __bit C1PCH1 __at(0x895);


extern volatile __bit C1POL __at(0x88C);


extern volatile __bit C1SP __at(0x88A);


extern volatile __bit C1SYNC __at(0x888);


extern volatile __bit C1TSEL0 __at(0x14F0);


extern volatile __bit C1TSEL1 __at(0x14F1);


extern volatile __bit C2HYS __at(0x899);


extern volatile __bit C2IE __at(0x496);


extern volatile __bit C2IF __at(0x96);


extern volatile __bit C2INTN __at(0x8A6);


extern volatile __bit C2INTP __at(0x8A7);


extern volatile __bit C2NCH0 __at(0x8A0);


extern volatile __bit C2NCH1 __at(0x8A1);


extern volatile __bit C2OE __at(0x89D);


extern volatile __bit C2ON __at(0x89F);


extern volatile __bit C2OUT __at(0x89E);


extern volatile __bit C2OUTSEL __at(0x8EA);


extern volatile __bit C2PCH0 __at(0x8A4);


extern volatile __bit C2PCH1 __at(0x8A5);


extern volatile __bit C2POL __at(0x89C);


extern volatile __bit C2SP __at(0x89A);


extern volatile __bit C2SYNC __at(0x898);


extern volatile __bit C2TSEL0 __at(0x14F2);


extern volatile __bit C2TSEL1 __at(0x14F3);


extern volatile __bit C3TSEL0 __at(0x14F4);


extern volatile __bit C3TSEL1 __at(0x14F5);


extern volatile __bit C4TSEL0 __at(0x14F6);


extern volatile __bit C4TSEL1 __at(0x14F7);


extern volatile __bit C5TSEL0 __at(0x14F8);


extern volatile __bit C5TSEL1 __at(0x14F9);


extern volatile __bit CARRY __at(0x18);


extern volatile __bit CCP1AS0 __at(0x14AC);


extern volatile __bit CCP1AS1 __at(0x14AD);


extern volatile __bit CCP1AS2 __at(0x14AE);


extern volatile __bit CCP1ASE __at(0x14AF);


extern volatile __bit CCP1IE __at(0x48A);


extern volatile __bit CCP1IF __at(0x8A);


extern volatile __bit CCP1M0 __at(0x1498);


extern volatile __bit CCP1M1 __at(0x1499);


extern volatile __bit CCP1M2 __at(0x149A);


extern volatile __bit CCP1M3 __at(0x149B);


extern volatile __bit CCP2AS0 __at(0x14E4);


extern volatile __bit CCP2AS1 __at(0x14E5);


extern volatile __bit CCP2AS2 __at(0x14E6);


extern volatile __bit CCP2ASE __at(0x14E7);


extern volatile __bit CCP2IE __at(0x490);


extern volatile __bit CCP2IF __at(0x90);


extern volatile __bit CCP2M0 __at(0x14D0);


extern volatile __bit CCP2M1 __at(0x14D1);


extern volatile __bit CCP2M2 __at(0x14D2);


extern volatile __bit CCP2M3 __at(0x14D3);


extern volatile __bit CCP2SEL __at(0x8E8);


extern volatile __bit CCP3AS0 __at(0x18AC);


extern volatile __bit CCP3AS1 __at(0x18AD);


extern volatile __bit CCP3AS2 __at(0x18AE);


extern volatile __bit CCP3ASE __at(0x18AF);


extern volatile __bit CCP3IE __at(0x49C);


extern volatile __bit CCP3IF __at(0x9C);


extern volatile __bit CCP3M0 __at(0x1898);


extern volatile __bit CCP3M1 __at(0x1899);


extern volatile __bit CCP3M2 __at(0x189A);


extern volatile __bit CCP3M3 __at(0x189B);


extern volatile __bit CCP3SEL __at(0x8EE);


extern volatile __bit CCP4IE __at(0x49D);


extern volatile __bit CCP4IF __at(0x9D);


extern volatile __bit CCP4M0 __at(0x18D0);


extern volatile __bit CCP4M1 __at(0x18D1);


extern volatile __bit CCP4M2 __at(0x18D2);


extern volatile __bit CCP4M3 __at(0x18D3);


extern volatile __bit CCP5IE __at(0x49E);


extern volatile __bit CCP5IF __at(0x9E);


extern volatile __bit CCP5M0 __at(0x18F0);


extern volatile __bit CCP5M1 __at(0x18F1);


extern volatile __bit CCP5M2 __at(0x18F2);


extern volatile __bit CCP5M3 __at(0x18F3);


extern volatile __bit CDAFVR0 __at(0x8BA);


extern volatile __bit CDAFVR1 __at(0x8BB);


extern volatile __bit CFGS __at(0xCAE);


extern volatile __bit CHS0 __at(0x4EA);


extern volatile __bit CHS1 __at(0x4EB);


extern volatile __bit CHS2 __at(0x4EC);


extern volatile __bit CHS3 __at(0x4ED);


extern volatile __bit CHS4 __at(0x4EE);


extern volatile __bit CKE __at(0x10A6);


extern volatile __bit CKP __at(0x10AC);


extern volatile __bit CPSCH0 __at(0xF8);


extern volatile __bit CPSCH1 __at(0xF9);


extern volatile __bit CPSCH2 __at(0xFA);


extern volatile __bit CPSON __at(0xF7);


extern volatile __bit CPSOUT __at(0xF1);


extern volatile __bit CPSRNG0 __at(0xF2);


extern volatile __bit CPSRNG1 __at(0xF3);


extern volatile __bit CREN __at(0xCEC);


extern volatile __bit CS0 __at(0x3C8A);


extern volatile __bit CS1 __at(0x3C8B);


extern volatile __bit CSRC __at(0xCF7);


extern volatile __bit C_SHAD __at(0x7F20);


extern volatile __bit DACEN __at(0x8C7);


extern volatile __bit DACLPS __at(0x8C6);


extern volatile __bit DACNSS __at(0x8C0);


extern volatile __bit DACOE __at(0x8C5);


extern volatile __bit DACPSS0 __at(0x8C2);


extern volatile __bit DACPSS1 __at(0x8C3);


extern volatile __bit DACR0 __at(0x8C8);


extern volatile __bit DACR1 __at(0x8C9);


extern volatile __bit DACR2 __at(0x8CA);


extern volatile __bit DACR3 __at(0x8CB);


extern volatile __bit DACR4 __at(0x8CC);


extern volatile __bit DC __at(0x19);


extern volatile __bit DC1B0 __at(0x149C);


extern volatile __bit DC1B1 __at(0x149D);


extern volatile __bit DC2B0 __at(0x14D4);


extern volatile __bit DC2B1 __at(0x14D5);


extern volatile __bit DC3B0 __at(0x189C);


extern volatile __bit DC3B1 __at(0x189D);


extern volatile __bit DC4B0 __at(0x18D4);


extern volatile __bit DC4B1 __at(0x18D5);


extern volatile __bit DC5B0 __at(0x18F4);


extern volatile __bit DC5B1 __at(0x18F5);


extern volatile __bit DC_SHAD __at(0x7F21);


extern volatile __bit DHEN __at(0x10B8);


extern volatile __bit D_nA __at(0x10A5);


extern volatile __bit EEIE __at(0x494);


extern volatile __bit EEIF __at(0x94);


extern volatile __bit EEPGD __at(0xCAF);


extern volatile __bit FERR __at(0xCEA);


extern volatile __bit FREE __at(0xCAC);


extern volatile __bit FVREN __at(0x8BF);


extern volatile __bit FVRRDY __at(0x8BE);


extern volatile __bit GCEN __at(0x10B7);


extern volatile __bit GIE __at(0x5F);


extern volatile __bit GO __at(0x4E9);


extern volatile __bit GO_nDONE __at(0x4E9);


extern volatile __bit HFIOFL __at(0x4D3);


extern volatile __bit HFIOFR __at(0x4D4);


extern volatile __bit HFIOFS __at(0x4D0);


extern volatile __bit INTE __at(0x5C);


extern volatile __bit INTEDG __at(0x4AE);


extern volatile __bit INTF __at(0x59);


extern volatile __bit IOCBF0 __at(0x1CB0);


extern volatile __bit IOCBF1 __at(0x1CB1);


extern volatile __bit IOCBF2 __at(0x1CB2);


extern volatile __bit IOCBF3 __at(0x1CB3);


extern volatile __bit IOCBF4 __at(0x1CB4);


extern volatile __bit IOCBF5 __at(0x1CB5);


extern volatile __bit IOCBF6 __at(0x1CB6);


extern volatile __bit IOCBF7 __at(0x1CB7);


extern volatile __bit IOCBN0 __at(0x1CA8);


extern volatile __bit IOCBN1 __at(0x1CA9);


extern volatile __bit IOCBN2 __at(0x1CAA);


extern volatile __bit IOCBN3 __at(0x1CAB);


extern volatile __bit IOCBN4 __at(0x1CAC);


extern volatile __bit IOCBN5 __at(0x1CAD);


extern volatile __bit IOCBN6 __at(0x1CAE);


extern volatile __bit IOCBN7 __at(0x1CAF);


extern volatile __bit IOCBP0 __at(0x1CA0);


extern volatile __bit IOCBP1 __at(0x1CA1);


extern volatile __bit IOCBP2 __at(0x1CA2);


extern volatile __bit IOCBP3 __at(0x1CA3);


extern volatile __bit IOCBP4 __at(0x1CA4);


extern volatile __bit IOCBP5 __at(0x1CA5);


extern volatile __bit IOCBP6 __at(0x1CA6);


extern volatile __bit IOCBP7 __at(0x1CA7);


extern volatile __bit IOCIE __at(0x5B);


extern volatile __bit IOCIF __at(0x58);


extern volatile __bit IRCF0 __at(0x4CB);


extern volatile __bit IRCF1 __at(0x4CC);


extern volatile __bit IRCF2 __at(0x4CD);


extern volatile __bit IRCF3 __at(0x4CE);


extern volatile __bit LATA0 __at(0x860);


extern volatile __bit LATA1 __at(0x861);


extern volatile __bit LATA2 __at(0x862);


extern volatile __bit LATA3 __at(0x863);


extern volatile __bit LATA4 __at(0x864);


extern volatile __bit LATA5 __at(0x865);


extern volatile __bit LATA6 __at(0x866);


extern volatile __bit LATA7 __at(0x867);


extern volatile __bit LATB0 __at(0x868);


extern volatile __bit LATB1 __at(0x869);


extern volatile __bit LATB2 __at(0x86A);


extern volatile __bit LATB3 __at(0x86B);


extern volatile __bit LATB4 __at(0x86C);


extern volatile __bit LATB5 __at(0x86D);


extern volatile __bit LATB6 __at(0x86E);


extern volatile __bit LATB7 __at(0x86F);


extern volatile __bit LATC0 __at(0x870);


extern volatile __bit LATC1 __at(0x871);


extern volatile __bit LATC2 __at(0x872);


extern volatile __bit LATC3 __at(0x873);


extern volatile __bit LATC4 __at(0x874);


extern volatile __bit LATC5 __at(0x875);


extern volatile __bit LATC6 __at(0x876);


extern volatile __bit LATC7 __at(0x877);


extern volatile __bit LATE3 __at(0x883);


extern volatile __bit LCDA __at(0x3C95);


extern volatile __bit LCDCST0 __at(0x3CA0);


extern volatile __bit LCDCST1 __at(0x3CA1);


extern volatile __bit LCDCST2 __at(0x3CA2);


extern volatile __bit LCDEN __at(0x3C8F);


extern volatile __bit LCDIE __at(0x492);


extern volatile __bit LCDIF __at(0x92);


extern volatile __bit LCDIRE __at(0x3C9F);


extern volatile __bit LCDIRI __at(0x3C9D);


extern volatile __bit LCDIRS __at(0x3C9E);


extern volatile __bit LFIOFR __at(0x4D1);


extern volatile __bit LMUX0 __at(0x3C88);


extern volatile __bit LMUX1 __at(0x3C89);


extern volatile __bit LP0 __at(0x3C90);


extern volatile __bit LP1 __at(0x3C91);


extern volatile __bit LP2 __at(0x3C92);


extern volatile __bit LP3 __at(0x3C93);


extern volatile __bit LRLAP0 __at(0x3CAE);


extern volatile __bit LRLAP1 __at(0x3CAF);


extern volatile __bit LRLAT0 __at(0x3CA8);


extern volatile __bit LRLAT1 __at(0x3CA9);


extern volatile __bit LRLAT2 __at(0x3CAA);


extern volatile __bit LRLBP0 __at(0x3CAC);


extern volatile __bit LRLBP1 __at(0x3CAD);


extern volatile __bit LWLO __at(0xCAD);


extern volatile __bit MC1OUT __at(0x8A8);


extern volatile __bit MC2OUT __at(0x8A9);


extern volatile __bit MFIOFR __at(0x4D2);


extern volatile __bit OERR __at(0xCE9);


extern volatile __bit OSFIE __at(0x497);


extern volatile __bit OSFIF __at(0x97);


extern volatile __bit OSTS __at(0x4D5);


extern volatile __bit P1DC0 __at(0x14A0);


extern volatile __bit P1DC1 __at(0x14A1);


extern volatile __bit P1DC2 __at(0x14A2);


extern volatile __bit P1DC3 __at(0x14A3);


extern volatile __bit P1DC4 __at(0x14A4);


extern volatile __bit P1DC5 __at(0x14A5);


extern volatile __bit P1DC6 __at(0x14A6);


extern volatile __bit P1M0 __at(0x149E);


extern volatile __bit P1M1 __at(0x149F);


extern volatile __bit P1RSEN __at(0x14A7);


extern volatile __bit P2BSEL __at(0x8EC);


extern volatile __bit P2DC0 __at(0x14D8);


extern volatile __bit P2DC1 __at(0x14D9);


extern volatile __bit P2DC2 __at(0x14DA);


extern volatile __bit P2DC3 __at(0x14DB);


extern volatile __bit P2DC4 __at(0x14DC);


extern volatile __bit P2DC5 __at(0x14DD);


extern volatile __bit P2DC6 __at(0x14DE);


extern volatile __bit P2M0 __at(0x14D6);


extern volatile __bit P2M1 __at(0x14D7);


extern volatile __bit P2RSEN __at(0x14DF);


extern volatile __bit P3DC0 __at(0x18A0);


extern volatile __bit P3DC1 __at(0x18A1);


extern volatile __bit P3DC2 __at(0x18A2);


extern volatile __bit P3DC3 __at(0x18A3);


extern volatile __bit P3DC4 __at(0x18A4);


extern volatile __bit P3DC5 __at(0x18A5);


extern volatile __bit P3DC6 __at(0x18A6);


extern volatile __bit P3M0 __at(0x189E);


extern volatile __bit P3M1 __at(0x189F);


extern volatile __bit P3RSEN __at(0x18A7);


extern volatile __bit PCIE __at(0x10BE);


extern volatile __bit PEIE __at(0x5E);


extern volatile __bit PEN __at(0x10B2);


extern volatile __bit PLLR __at(0x4D6);


extern volatile __bit PS0 __at(0x4A8);


extern volatile __bit PS1 __at(0x4A9);


extern volatile __bit PS2 __at(0x4AA);


extern volatile __bit PSA __at(0x4AB);


extern volatile __bit PSS1AC0 __at(0x14AA);


extern volatile __bit PSS1AC1 __at(0x14AB);


extern volatile __bit PSS1BD0 __at(0x14A8);


extern volatile __bit PSS1BD1 __at(0x14A9);


extern volatile __bit PSS2AC0 __at(0x14E2);


extern volatile __bit PSS2AC1 __at(0x14E3);


extern volatile __bit PSS2BD0 __at(0x14E0);


extern volatile __bit PSS2BD1 __at(0x14E1);


extern volatile __bit PSS3AC0 __at(0x18AA);


extern volatile __bit PSS3AC1 __at(0x18AB);


extern volatile __bit PSS3BD0 __at(0x18A8);


extern volatile __bit PSS3BD1 __at(0x18A9);


extern volatile __bit RA0 __at(0x60);


extern volatile __bit RA1 __at(0x61);


extern volatile __bit RA2 __at(0x62);


extern volatile __bit RA3 __at(0x63);


extern volatile __bit RA4 __at(0x64);


extern volatile __bit RA5 __at(0x65);


extern volatile __bit RA6 __at(0x66);


extern volatile __bit RA7 __at(0x67);


extern volatile __bit RB0 __at(0x68);


extern volatile __bit RB1 __at(0x69);


extern volatile __bit RB2 __at(0x6A);


extern volatile __bit RB3 __at(0x6B);


extern volatile __bit RB4 __at(0x6C);


extern volatile __bit RB5 __at(0x6D);


extern volatile __bit RB6 __at(0x6E);


extern volatile __bit RB7 __at(0x6F);


extern volatile __bit RC0 __at(0x70);


extern volatile __bit RC1 __at(0x71);


extern volatile __bit RC2 __at(0x72);


extern volatile __bit RC3 __at(0x73);


extern volatile __bit RC4 __at(0x74);


extern volatile __bit RC5 __at(0x75);


extern volatile __bit RC6 __at(0x76);


extern volatile __bit RC7 __at(0x77);


extern volatile __bit RCEN __at(0x10B3);


extern volatile __bit RCIDL __at(0xCFE);


extern volatile __bit RCIE __at(0x48D);


extern volatile __bit RCIF __at(0x8D);


extern volatile __bit RD __at(0xCA8);


extern volatile __bit RE3 __at(0x83);


extern volatile __bit RSEN __at(0x10B1);


extern volatile __bit RX9 __at(0xCEE);


extern volatile __bit RX9D __at(0xCE8);


extern volatile __bit R_nW __at(0x10A2);


extern volatile __bit SBCDE __at(0x10BA);


extern volatile __bit SBOREN __at(0x8B7);


extern volatile __bit SCIE __at(0x10BD);


extern volatile __bit SCKP __at(0xCFC);


extern volatile __bit SCS0 __at(0x4C8);


extern volatile __bit SCS1 __at(0x4C9);


extern volatile __bit SDAHT __at(0x10BB);


extern volatile __bit SE0 __at(0x3CC0);


extern volatile __bit SE1 __at(0x3CC1);


extern volatile __bit SE10 __at(0x3CCA);


extern volatile __bit SE11 __at(0x3CCB);


extern volatile __bit SE12 __at(0x3CCC);


extern volatile __bit SE13 __at(0x3CCD);


extern volatile __bit SE14 __at(0x3CCE);


extern volatile __bit SE15 __at(0x3CCF);


extern volatile __bit SE2 __at(0x3CC2);


extern volatile __bit SE3 __at(0x3CC3);


extern volatile __bit SE4 __at(0x3CC4);


extern volatile __bit SE5 __at(0x3CC5);


extern volatile __bit SE6 __at(0x3CC6);


extern volatile __bit SE7 __at(0x3CC7);


extern volatile __bit SE8 __at(0x3CC8);


extern volatile __bit SE9 __at(0x3CC9);


extern volatile __bit SEG0COM0 __at(0x3D00);


extern volatile __bit SEG0COM1 __at(0x3D18);


extern volatile __bit SEG0COM2 __at(0x3D30);


extern volatile __bit SEG0COM3 __at(0x3D48);


extern volatile __bit SEG10COM0 __at(0x3D0A);


extern volatile __bit SEG10COM1 __at(0x3D22);


extern volatile __bit SEG10COM2 __at(0x3D3A);


extern volatile __bit SEG10COM3 __at(0x3D52);


extern volatile __bit SEG11COM0 __at(0x3D0B);


extern volatile __bit SEG11COM1 __at(0x3D23);


extern volatile __bit SEG11COM2 __at(0x3D3B);


extern volatile __bit SEG11COM3 __at(0x3D53);


extern volatile __bit SEG12COM0 __at(0x3D0C);


extern volatile __bit SEG12COM1 __at(0x3D24);


extern volatile __bit SEG12COM2 __at(0x3D3C);


extern volatile __bit SEG12COM3 __at(0x3D54);


extern volatile __bit SEG13COM0 __at(0x3D0D);


extern volatile __bit SEG13COM1 __at(0x3D25);


extern volatile __bit SEG13COM2 __at(0x3D3D);


extern volatile __bit SEG13COM3 __at(0x3D55);


extern volatile __bit SEG14COM0 __at(0x3D0E);


extern volatile __bit SEG14COM1 __at(0x3D26);


extern volatile __bit SEG14COM2 __at(0x3D3E);


extern volatile __bit SEG14COM3 __at(0x3D56);


extern volatile __bit SEG15COM0 __at(0x3D0F);


extern volatile __bit SEG15COM1 __at(0x3D27);


extern volatile __bit SEG15COM2 __at(0x3D3F);


extern volatile __bit SEG15COM3 __at(0x3D57);


extern volatile __bit SEG1COM0 __at(0x3D01);


extern volatile __bit SEG1COM1 __at(0x3D19);


extern volatile __bit SEG1COM2 __at(0x3D31);


extern volatile __bit SEG1COM3 __at(0x3D49);


extern volatile __bit SEG2COM0 __at(0x3D02);


extern volatile __bit SEG2COM1 __at(0x3D1A);


extern volatile __bit SEG2COM2 __at(0x3D32);


extern volatile __bit SEG2COM3 __at(0x3D4A);


extern volatile __bit SEG3COM0 __at(0x3D03);


extern volatile __bit SEG3COM1 __at(0x3D1B);


extern volatile __bit SEG3COM2 __at(0x3D33);


extern volatile __bit SEG3COM3 __at(0x3D4B);


extern volatile __bit SEG4COM0 __at(0x3D04);


extern volatile __bit SEG4COM1 __at(0x3D1C);


extern volatile __bit SEG4COM2 __at(0x3D34);


extern volatile __bit SEG4COM3 __at(0x3D4C);


extern volatile __bit SEG5COM0 __at(0x3D05);


extern volatile __bit SEG5COM1 __at(0x3D1D);


extern volatile __bit SEG5COM2 __at(0x3D35);


extern volatile __bit SEG5COM3 __at(0x3D4D);


extern volatile __bit SEG6COM0 __at(0x3D06);


extern volatile __bit SEG6COM1 __at(0x3D1E);


extern volatile __bit SEG6COM2 __at(0x3D36);


extern volatile __bit SEG6COM3 __at(0x3D4E);


extern volatile __bit SEG7COM0 __at(0x3D07);


extern volatile __bit SEG7COM1 __at(0x3D1F);


extern volatile __bit SEG7COM2 __at(0x3D37);


extern volatile __bit SEG7COM3 __at(0x3D4F);


extern volatile __bit SEG8COM0 __at(0x3D08);


extern volatile __bit SEG8COM1 __at(0x3D20);


extern volatile __bit SEG8COM2 __at(0x3D38);


extern volatile __bit SEG8COM3 __at(0x3D50);


extern volatile __bit SEG9COM0 __at(0x3D09);


extern volatile __bit SEG9COM1 __at(0x3D21);


extern volatile __bit SEG9COM2 __at(0x3D39);


extern volatile __bit SEG9COM3 __at(0x3D51);


extern volatile __bit SEN __at(0x10B0);


extern volatile __bit SENDB __at(0xCF3);


extern volatile __bit SLPEN __at(0x3C8E);


extern volatile __bit SMP __at(0x10A7);


extern volatile __bit SPEN __at(0xCEF);


extern volatile __bit SPLLEN __at(0x4CF);


extern volatile __bit SRCLK0 __at(0x8D4);


extern volatile __bit SRCLK1 __at(0x8D5);


extern volatile __bit SRCLK2 __at(0x8D6);


extern volatile __bit SREN __at(0xCED);


extern volatile __bit SRLEN __at(0x8D7);


extern volatile __bit SRNQEN __at(0x8D2);


extern volatile __bit SRNQSEL __at(0x8EB);


extern volatile __bit SRPR __at(0x8D0);


extern volatile __bit SRPS __at(0x8D1);


extern volatile __bit SRQEN __at(0x8D3);


extern volatile __bit SRRC1E __at(0x8D8);


extern volatile __bit SRRC2E __at(0x8D9);


extern volatile __bit SRRCKE __at(0x8DA);


extern volatile __bit SRRPE __at(0x8DB);


extern volatile __bit SRSC1E __at(0x8DC);


extern volatile __bit SRSC2E __at(0x8DD);


extern volatile __bit SRSCKE __at(0x8DE);


extern volatile __bit SRSPE __at(0x8DF);


extern volatile __bit SSPEN __at(0x10AD);


extern volatile __bit SSPIE __at(0x48B);


extern volatile __bit SSPIF __at(0x8B);


extern volatile __bit SSPM0 __at(0x10A8);


extern volatile __bit SSPM1 __at(0x10A9);


extern volatile __bit SSPM2 __at(0x10AA);


extern volatile __bit SSPM3 __at(0x10AB);


extern volatile __bit SSPOV __at(0x10AE);


extern volatile __bit SSSEL __at(0x8E9);


extern volatile __bit STKOVF __at(0x4B7);


extern volatile __bit STKUNF __at(0x4B6);


extern volatile __bit STR1A __at(0x14B0);


extern volatile __bit STR1B __at(0x14B1);


extern volatile __bit STR1C __at(0x14B2);


extern volatile __bit STR1D __at(0x14B3);


extern volatile __bit STR1SYNC __at(0x14B4);


extern volatile __bit STR2A __at(0x14E8);


extern volatile __bit STR2B __at(0x14E9);


extern volatile __bit STR2C __at(0x14EA);


extern volatile __bit STR2D __at(0x14EB);


extern volatile __bit STR2SYNC __at(0x14EC);


extern volatile __bit STR3A __at(0x18B0);


extern volatile __bit STR3B __at(0x18B1);


extern volatile __bit STR3C __at(0x18B2);


extern volatile __bit STR3D __at(0x18B3);


extern volatile __bit STR3SYNC __at(0x18B4);


extern volatile __bit SWDTEN __at(0x4B8);


extern volatile __bit SYNC __at(0xCF4);


extern volatile __bit T0CS __at(0x4AD);


extern volatile __bit T0IE __at(0x5D);


extern volatile __bit T0IF __at(0x5A);


extern volatile __bit T0SE __at(0x4AC);


extern volatile __bit T0XCS __at(0xF0);


extern volatile __bit T1CKPS0 __at(0xC4);


extern volatile __bit T1CKPS1 __at(0xC5);


extern volatile __bit T1GGO __at(0xCB);


extern volatile __bit T1GGO_nDONE __at(0xCB);


extern volatile __bit T1GPOL __at(0xCE);


extern volatile __bit T1GSEL __at(0x8ED);


extern volatile __bit T1GSPM __at(0xCC);


extern volatile __bit T1GSS0 __at(0xC8);


extern volatile __bit T1GSS1 __at(0xC9);


extern volatile __bit T1GTM __at(0xCD);


extern volatile __bit T1GVAL __at(0xCA);


extern volatile __bit T1OSCEN __at(0xC3);


extern volatile __bit T1OSCR __at(0x4D7);


extern volatile __bit T2CKPS0 __at(0xE0);


extern volatile __bit T2CKPS1 __at(0xE1);


extern volatile __bit T2OUTPS0 __at(0xE3);


extern volatile __bit T2OUTPS1 __at(0xE4);


extern volatile __bit T2OUTPS2 __at(0xE5);


extern volatile __bit T2OUTPS3 __at(0xE6);


extern volatile __bit T4CKPS0 __at(0x20B8);


extern volatile __bit T4CKPS1 __at(0x20B9);


extern volatile __bit T4OUTPS0 __at(0x20BB);


extern volatile __bit T4OUTPS1 __at(0x20BC);


extern volatile __bit T4OUTPS2 __at(0x20BD);


extern volatile __bit T4OUTPS3 __at(0x20BE);


extern volatile __bit T6CKPS0 __at(0x20F0);


extern volatile __bit T6CKPS1 __at(0x20F1);


extern volatile __bit T6OUTPS0 __at(0x20F3);


extern volatile __bit T6OUTPS1 __at(0x20F4);


extern volatile __bit T6OUTPS2 __at(0x20F5);


extern volatile __bit T6OUTPS3 __at(0x20F6);


extern volatile __bit TMR0CS __at(0x4AD);


extern volatile __bit TMR0IE __at(0x5D);


extern volatile __bit TMR0IF __at(0x5A);


extern volatile __bit TMR0SE __at(0x4AC);


extern volatile __bit TMR1CS0 __at(0xC6);


extern volatile __bit TMR1CS1 __at(0xC7);


extern volatile __bit TMR1GE __at(0xCF);


extern volatile __bit TMR1GIE __at(0x48F);


extern volatile __bit TMR1GIF __at(0x8F);


extern volatile __bit TMR1IE __at(0x488);


extern volatile __bit TMR1IF __at(0x88);


extern volatile __bit TMR1ON __at(0xC0);


extern volatile __bit TMR2IE __at(0x489);


extern volatile __bit TMR2IF __at(0x89);


extern volatile __bit TMR2ON __at(0xE2);


extern volatile __bit TMR4IE __at(0x499);


extern volatile __bit TMR4IF __at(0x99);


extern volatile __bit TMR4ON __at(0x20BA);


extern volatile __bit TMR6IE __at(0x49B);


extern volatile __bit TMR6IF __at(0x9B);


extern volatile __bit TMR6ON __at(0x20F2);


extern volatile __bit TRISA0 __at(0x460);


extern volatile __bit TRISA1 __at(0x461);


extern volatile __bit TRISA2 __at(0x462);


extern volatile __bit TRISA3 __at(0x463);


extern volatile __bit TRISA4 __at(0x464);


extern volatile __bit TRISA5 __at(0x465);


extern volatile __bit TRISA6 __at(0x466);


extern volatile __bit TRISA7 __at(0x467);


extern volatile __bit TRISB0 __at(0x468);


extern volatile __bit TRISB1 __at(0x469);


extern volatile __bit TRISB2 __at(0x46A);


extern volatile __bit TRISB3 __at(0x46B);


extern volatile __bit TRISB4 __at(0x46C);


extern volatile __bit TRISB5 __at(0x46D);


extern volatile __bit TRISB6 __at(0x46E);


extern volatile __bit TRISB7 __at(0x46F);


extern volatile __bit TRISC0 __at(0x470);


extern volatile __bit TRISC1 __at(0x471);


extern volatile __bit TRISC2 __at(0x472);


extern volatile __bit TRISC3 __at(0x473);


extern volatile __bit TRISC4 __at(0x474);


extern volatile __bit TRISC5 __at(0x475);


extern volatile __bit TRISC6 __at(0x476);


extern volatile __bit TRISC7 __at(0x477);


extern volatile __bit TRISE3 __at(0x483);


extern volatile __bit TRMT __at(0xCF1);


extern volatile __bit TSEN __at(0x8BD);


extern volatile __bit TSRNG __at(0x8BC);


extern volatile __bit TUN0 __at(0x4C0);


extern volatile __bit TUN1 __at(0x4C1);


extern volatile __bit TUN2 __at(0x4C2);


extern volatile __bit TUN3 __at(0x4C3);


extern volatile __bit TUN4 __at(0x4C4);


extern volatile __bit TUN5 __at(0x4C5);


extern volatile __bit TX9 __at(0xCF6);


extern volatile __bit TX9D __at(0xCF0);


extern volatile __bit TXEN __at(0xCF5);


extern volatile __bit TXIE __at(0x48C);


extern volatile __bit TXIF __at(0x8C);


extern volatile __bit UA __at(0x10A1);


extern volatile __bit VLCD1PE __at(0x3C99);


extern volatile __bit VLCD2PE __at(0x3C9A);


extern volatile __bit VLCD3PE __at(0x3C9B);


extern volatile __bit WA __at(0x3C94);


extern volatile __bit WCOL __at(0x10AF);


extern volatile __bit WDTPS0 __at(0x4B9);


extern volatile __bit WDTPS1 __at(0x4BA);


extern volatile __bit WDTPS2 __at(0x4BB);


extern volatile __bit WDTPS3 __at(0x4BC);


extern volatile __bit WDTPS4 __at(0x4BD);


extern volatile __bit WERR __at(0x3C8D);


extern volatile __bit WFT __at(0x3C97);


extern volatile __bit WPUB0 __at(0x1068);


extern volatile __bit WPUB1 __at(0x1069);


extern volatile __bit WPUB2 __at(0x106A);


extern volatile __bit WPUB3 __at(0x106B);


extern volatile __bit WPUB4 __at(0x106C);


extern volatile __bit WPUB5 __at(0x106D);


extern volatile __bit WPUB6 __at(0x106E);


extern volatile __bit WPUB7 __at(0x106F);


extern volatile __bit WPUE3 __at(0x1083);


extern volatile __bit WR __at(0xCA9);


extern volatile __bit WREN __at(0xCAA);


extern volatile __bit WRERR __at(0xCAB);


extern volatile __bit WUE __at(0xCF9);


extern volatile __bit ZERO __at(0x1A);


extern volatile __bit Z_SHAD __at(0x7F22);


extern volatile __bit nBOR __at(0x4B0);


extern volatile __bit nDONE __at(0x4E9);


extern volatile __bit nPD __at(0x1B);


extern volatile __bit nPOR __at(0x4B1);


extern volatile __bit nRI __at(0x4B2);


extern volatile __bit nRMCLR __at(0x4B3);


extern volatile __bit nT1SYNC __at(0xC2);


extern volatile __bit nTO __at(0x1C);


extern volatile __bit nWPUEN __at(0x4AF);


# 30 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\pic.h"
#pragma intrinsic(__nop)
extern void __nop(void);

# 78
__attribute__((__unsupported__("The " "FLASH_READ" " macro function is no longer supported. Please use the MPLAB X MCC."))) unsigned char __flash_read(unsigned short addr);

__attribute__((__unsupported__("The " "FLASH_WRITE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_write(unsigned short addr, unsigned short data);

__attribute__((__unsupported__("The " "FLASH_ERASE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_erase(unsigned short addr);

# 114 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\eeprom_routines.h"
extern void eeprom_write(unsigned char addr, unsigned char value);
extern unsigned char eeprom_read(unsigned char addr);


# 91 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\pic.h"
#pragma intrinsic(_delay)
extern __nonreentrant void _delay(unsigned long);
#pragma intrinsic(_delaywdt)
extern __nonreentrant void _delaywdt(unsigned long);

#pragma intrinsic(_delay3)
extern __nonreentrant void _delay3(unsigned char);

# 137
extern __bank0 unsigned char __resetbits;
extern __bank0 __bit __powerdown;
extern __bank0 __bit __timeout;

# 255 "../config.h"
extern const unsigned short PWM_trans_table[202] = {
0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
181, 181, 181, 181, 181, 181, 181, 181, 181, 181,
181, 182, 184, 185, 186, 187, 189, 190, 191, 193, 194, 195, 196, 198, 199, 200, 202, 203, 204, 205,
207, 208, 209, 211, 212, 213, 214, 216, 217, 218, 220, 221, 222, 224, 225, 226, 227, 229, 230, 231,
233, 234, 235, 236, 238, 239, 240, 242, 243, 244, 245, 247, 248, 249, 251, 252, 253, 254, 256, 257,
258, 260, 261, 262, 263, 265, 266, 267, 269, 270, 271, 272, 274, 275, 276, 278, 279, 280, 281, 283,
284, 285, 287, 288, 289, 290, 292, 293, 294, 296, 297, 298, 300, 301, 302, 303, 305, 306, 307, 309,
310, 311, 312, 314, 315, 316, 318, 319, 320, 321, 323, 324, 325, 327, 328, 329, 330, 332, 333, 334,
336, 337, 338, 339, 341, 342, 343, 345, 346, 347, 348, 350, 351, 352, 354, 355, 356, 357, 359, 360,
361, 363, 364, 365, 367, 368, 369, 370, 372, 373, 374, 376, 377, 378, 379, 381, 382, 383, 385, 386,
387, 388, 390, 391, 392, 394, 395, 396, 397, 399, 400, 400,
400, 400, 400, 400, 400, 400, 400, 400, 400, 400
};

# 39 "../project.h"
typedef union
{
unsigned char b;
struct
{
unsigned B0 : 1;
unsigned B1 : 1;
unsigned B2 : 1;
unsigned B3 : 1;
unsigned B4 : 1;
unsigned B5 : 1;
unsigned B6 : 1;
unsigned B7 : 1;

} bits;
} _u_bits;



typedef union
{
unsigned int w;
struct
{
unsigned char lo;
unsigned char hi;
} b;
} _u_wb;



typedef union
{
unsigned long lng;
struct
{
_u_wb low;
_u_wb hiw;
} w;
} _u_lng;

# 37 "../timer.h"
extern void timer_init( unsigned char ui8_TmrNb );
extern unsigned int get_timer( unsigned char ui8_TmrNb );
extern void clear_timer( unsigned char ui8_TmrNb );
extern void start_timer( unsigned char ui8_TmrNb, unsigned char ui8_start_spec_time );
extern void Oscill_Source_Block( void );

# 37 "../pwm.h"
extern void PWM_Capture_init( unsigned char CCP_Nb );
extern void PWM_CTRL( void );
extern void PWM_Write_Out( unsigned char ui8_DutyCycle_Out );
extern void interrupt_PWMCapture( void );
extern unsigned char PWMReadDC( void );
extern unsigned int ui16_PWM_Freq_In;
extern double Duty_Cycle_In_Ratio;
extern unsigned char ui8_Duty_Cycle_In_Ratio;
extern unsigned char ui8_PWMoutvalue;
extern unsigned char ui8_Duty_Cycle_In_Ratio;
extern unsigned char ui8_CMD_Mode;

# 39 "../project.h"
typedef union
{
unsigned char b;
struct
{
unsigned B0 : 1;
unsigned B1 : 1;
unsigned B2 : 1;
unsigned B3 : 1;
unsigned B4 : 1;
unsigned B5 : 1;
unsigned B6 : 1;
unsigned B7 : 1;

} bits;
} _u_bits;



typedef union
{
unsigned int w;
struct
{
unsigned char lo;
unsigned char hi;
} b;
} _u_wb;



typedef union
{
unsigned long lng;
struct
{
_u_wb low;
_u_wb hiw;
} w;
} _u_lng;

# 213 "../bldc.h"
extern void init_bldc( void );
extern void interrrupt_bldc( void );
extern void InitMotorStop( void );
extern void InitMotorRun( void );
extern signed short phase_delay_counter_debug;
extern unsigned char BlankingCountdbg;

extern unsigned short ui16_phase_angle;
extern unsigned int B[ 8 ];
extern _u_bits MotorFlags;
extern unsigned short ui16_speed_fil;
extern unsigned short ui16_speed_rar;
extern volatile _u_wb ui16_IPhase1_bldc;
extern volatile _u_wb ui16_IPhase2_bldc;
extern volatile _u_wb ui16_IPhase3_bldc;
extern volatile _u_wb ui16_IPhase_bldc;

extern unsigned short ui16_UPhase_bldc;

extern unsigned short ui16_NTC_Temp_bldc;
extern unsigned short ui16_CPU_Temp_bldc;

extern volatile _u_wb ui16_Ubat_bldc;
extern volatile _u_wb ui16_Ubemf_bldc;
extern unsigned short ui8_zero_cros_cnt;
extern unsigned short ui16_duty_cycle_BLDC;

# 245
void InitSystem( void );
void InitDriver( void );
void Commutate( void );
void ControlStartUp( void );
void StallControl( void );
void AdcManager( void );
void WarmUpControl( void );
void TimeBaseManager( void );
void CalcFilter( void );
void ControlSlowStart( void );

# 56 "../lin.h"
typedef enum _BOOL
{
FALSE = 0,
TRUE
}
BOOL;
typedef unsigned char BYTE;

# 72
extern void Transmit_LIN_8Bytes( BYTE ID, BYTE B0, BYTE B1, BYTE B2, BYTE B3, BYTE B4, BYTE B5, BYTE B6,
BYTE B7 );
extern void Receive_ETAT_PADD( void );
extern void Receive_Diag( char id );
extern void Transmit_LIN_Byte( BYTE ID, BYTE B0 );
extern void Transmit_LIN_3Bytes( BYTE ID, BYTE B0, BYTE B1, BYTE B2 );
extern void EnableMCP201( void );

# 291
typedef union ELINMINT_ID
{
unsigned char ID;

struct
{
unsigned ID0 : 1;
unsigned ID1 : 1;
unsigned ID2 : 1;
unsigned ID3 : 1;
unsigned ID4 : 1;
unsigned ID5 : 1;
unsigned ID6 : 1;
unsigned ID7 : 1;
} IDbits;
} ELINMINT_ID;

# 314
typedef union ELINMINT_MESSAGE_SIZE
{
unsigned char SIZE;

struct
{
unsigned SIZE0 : 1;
unsigned SIZE1 : 1;
unsigned SIZE2 : 1;
unsigned SIZE3 : 1;
unsigned SIZE4 : 1;
unsigned SIZE5 : 1;
unsigned SIZE6 : 1;
unsigned SIZE7 : 1;
} SIZEbits;
} ELINMINT_MESSAGE_SIZE;

# 336
typedef union ELINMINT_CRC
{
int CRC;
BYTE CRCL;
struct
{
unsigned CRC0 : 1;
unsigned CRC1 : 1;
unsigned CRC2 : 1;
unsigned CRC3 : 1;
unsigned CRC4 : 1;
unsigned CRC5 : 1;
unsigned CRC6 : 1;
unsigned CRC7 : 1;
unsigned CRC8 : 1;
unsigned CRC9 : 1;
unsigned CRC10 : 1;
unsigned CRC11 : 1;
unsigned CRC12 : 1;
unsigned CRC13 : 1;
unsigned CRC14 : 1;
unsigned CRC15 : 1;
} CRCbits;
} ELINMINT_CRC;

# 369
typedef union ELINMINT_STATUS
{
BYTE ELINMIntStatusByte;
struct
{
unsigned TX : 1;
unsigned RX : 1;
unsigned ERROR : 1;
unsigned IDLE : 1;
unsigned ERROR_BIT0 : 1;
unsigned ERROR_BIT1 : 1;
unsigned ERROR_BIT2 : 1;
unsigned ERROR_BIT3 : 1;
} ELINMINTSTS;
} ELINMINT_STATUS;

# 390
typedef union ELINMINT_STATUS1
{
BYTE ELINMIntStatusByte;
struct
{
unsigned WAKEUP : 1;
unsigned HEADER : 1;
unsigned FRAME : 1;
unsigned EXTENDED : 1;
unsigned WAKEUP_RECEIVED : 1;
unsigned WAKEUP_SENT : 1;
unsigned SLEEP_TIMEOUT : 1;
} ELINMINTSTS;
} ELINMINT_STATUS1;

# 427
BYTE _ELINMIntInitialize( void );

# 446
void ELINMIntHandler( void );

# 469
void _ELINMIntSendMessage( BYTE id, char size, unsigned int fmin, unsigned int fmax );

# 488
void _ELINMIntResetProtocol( BYTE code );

# 508
void _ELINMIntReceiveMessage( BYTE tag, BYTE id, char size );

# 536
BYTE * _ELINMIntGetPointer( char _ELINMInt_tag, BYTE _ELINMInt_position );

# 554
BYTE _ELINMIntCalcIDParity( ELINMINT_ID ELINM_idtr );

# 1001
extern ELINMINT_STATUS _ELINMIntStatus;
extern ELINMINT_STATUS1 _ELINMIntStatus1;
extern BYTE _ELINMIntMessageTag;
extern BYTE _ELINMIntMessageBuffer[ ];

# 27 "../diag.h"
extern unsigned int ui16_PWM_Freq_In;

extern unsigned char Error_PICetatMonitor;
extern unsigned char DC_pic_etat_monitor;


extern void DiagInit( void );
extern void SetDiagAlarm( void );
extern unsigned char read_eeprom_data( unsigned char ui8_adress );
extern void ReadCal_Value( void );
extern void DiagPicEtatMonitor( void );
extern void EOL( void );

# 74
extern _u_bits ui8_error_Flags;

# 21 "../adc.h"
extern void ADC_Init( void );
extern unsigned int ADC_Read( void );
extern void ADC_Wait( void );
extern signed int NTC_Temperature( void );
extern void FILTER1( unsigned char ui8_channel );
extern void FILTER2( unsigned char ui8_channel );
extern void Get_Analog_Value( void );
extern void FILTER_Init( void );

extern unsigned int ui16_fir_Bat_mittel;
extern _u_wb ui16_fir_IPhase_mean;
extern unsigned int ui16_CPU_Temp_bldc_mean;
extern unsigned int ui16_NTC_Temp_bldc_mean;
extern unsigned int ui16_CPU_Temp_bldc_mean_cal;
extern unsigned int ui16_NTC_Temp_bldc_mean_cal;
extern unsigned char ui8_IPhase_sel;
extern unsigned int filter1;

# 21 "../eeprom.h"
extern unsigned char read_eeprom_data( unsigned char ui8_adress );
extern void write_eeprom_data( unsigned char ui8_adress, unsigned char ui8_adress_data );

# 49 "../cksum.h"
extern unsigned unsigned int cksum(void);
extern unsigned unsigned int _checksum0;

# 17 "../T_Link\tl_basetypes.h"
typedef unsigned char Bool;
typedef float Float32;
typedef double Float64;
typedef signed short int Int16;
typedef signed long int Int32;
typedef signed char Int8;
typedef unsigned short int UInt16;
typedef unsigned long int UInt32;
typedef unsigned char UInt8;
typedef void Void;

# 61 "../T_Link\BVH2_Appl_Layer.h"
struct tag_SIBFS_Current_Analysis_High_b_tp {
unsigned int Cb1_Current_Analysis_High_ns : 4;
unsigned int Cb1_glflag : 2;
unsigned int Cb1_Current_Analysis_High : 1;
};

struct tag_SIBFS_Current_Analysis_low_b_tp {
unsigned int Cb10_greenState : 1;
unsigned int Cb11_Wait : 1;
unsigned int Cb12_CntOverCurrent : 1;
unsigned int Cb13_redState : 1;
unsigned int Cb9_Current_Analysis_low : 1;
};

struct tag_SIBFS_Dry_RunningAlarm_b_tp {
unsigned int Cb19_Dry_RunningAlarm : 1;
unsigned int Cb20_greenState : 1;
unsigned int Cb21_DryRunningAlarm : 1;
unsigned int Cb22_DryRun66 : 1;
unsigned int Cb23_DryRun55 : 1;
};

struct tag_SIBFS_Dry_Running_b_tp {
unsigned int Cb14_Dry_Running : 1;
unsigned int Cb15_greenState : 1;
unsigned int Cb16_DryRunning : 1;
unsigned int Cb17_redState : 1;
unsigned int Cb18_CntOverCurrent : 1;
};

struct tag_SIBFS_Motor_Stalled_b_tp {
unsigned int Cb25_Motor_sta__Statemachine_ns : 3;
unsigned int Cb24_glflag : 2;
unsigned int Cb24_Motor_Stalled : 1;
unsigned int Cb25_Motor_stalled_Statemachine : 1;
unsigned int Cb32_default : 1;
unsigned int Cb33_Stop : 1;
};

struct tag_SIBFS_PWM_Detection_b_tp {
unsigned int Cb35_PWMinput_handling_ns : 4;
unsigned int Cb35_PWMinput_handling : 1;
};

struct tag_SIBFS_Pic_etat_monitor_b_tp {
unsigned int Cb44_Pic_etat_monitor_ns : 3;
unsigned int Cb44_glflag : 2;
unsigned int Cb44_Pic_etat_monitor : 1;
};

struct tag_SIBFS_Temperature_Alarm_b_tp {
unsigned int Cb51_Temperature_Alarm : 1;
unsigned int Cb52_CntOverTemp : 1;
unsigned int Cb53_reset : 1;
unsigned int Cb54_greenTemp : 1;
unsigned int Cb55_redTemp : 1;
};

struct tag_SIBFS_UbatHandling_b_tp {
unsigned int Aux_sflag3 : 3;
unsigned int Cb57_Ubat_Handling : 1;
unsigned int Cb58_SaturationHigh : 1;
unsigned int Cb59_SaturationLow : 1;
unsigned int Cb60_NormalUbat : 1;
};

# 136
extern const UInt16 Sb2_Fixed_Power;

# 141
extern  UInt16 ui16_Current_Thresh;
extern  UInt16 ui16_PWM_Freq_mat;
extern  UInt16 ui16_Speed_demand_mat;
extern  UInt16 ui16_Speed_demand_mat_Max;
extern  UInt16 ui16_Speed_demand_mat_min;
extern  UInt16 ui16_Speed_mat;
extern  UInt16 ui16_dryRun_Thresh;
extern  UInt16 ui16_mat_Current;
extern  UInt16 ui16_mat_inpTemp;
extern  UInt16 ui8_BattVolt_mat;
extern  UInt16 ui8_Ki_mat;
extern  UInt16 ui8_fixed_start_speed_mat;
extern  Bool bl_Pumpoff_Alarm;
extern  Bool bool_CPU_TempAlarm;
extern  Bool bool_CPU_TempRedAlarm;
extern  Bool bool_ControlLoopMode;
extern  Bool bool_DryRunningAlarm;
extern  Bool bool_HighCurrentAlarm;
extern  Bool bool_MotorStalled;
extern  Bool bool_PIC_Alarm;
extern  Bool bool_PWMin_Freq_err_Alarm;
extern  Bool bool_PWMin_err_Alarm;
extern  Bool bool_StalledMotorStop;
extern  Bool bool_UbatAlarm;
extern  Bool bool_mat_currAlarm_bldc;
extern  Bool bool_mat_pic_etat;
extern  Bool bool_start_demand_mat;
extern  UInt8 ui8_Kp_mat;
extern  UInt8 ui8_PWM_dc_mat;
extern  UInt8 ui8_ResetMatlab;
extern  UInt8 ui8_debug_out0;
extern  UInt16 ui16_duty_cycle_mat;

# 183
extern Void BVH2_Appl_Layer(Void);


# 95 "../main.c"
#pragma config FOSC = INTOSC
#pragma config WDTE = 1
#pragma config MCLRE = 0
#pragma config CLKOUTEN = 0


# 111
#pragma config PLLEN = 1
#pragma config BORV = LO
#pragma config LVP = 0

# 119
unsigned int ui16_Timer_VaL1;
unsigned char ui8_Task_Cont3ms;
unsigned char ui8_Task_Cont5ms;
unsigned char ui8_Task_Cont100ms;
unsigned int ui16_Task_Cont500ms;
unsigned char ui8_Sync_Cont10ms;
unsigned char ui8_lastTaskvalue;
unsigned int ui16_I_cal_Ph1 = 1;
unsigned int ui16_I_cal_Ph2 = 1;
unsigned int ui16_I_cal_Ph3 = 1;
unsigned int ui16_Temp_cal;
unsigned char ui8_DebugCnt;
unsigned char ui8_i_wob = 0;
unsigned char ui8_selected_lid = 0x80;
unsigned int ui16_wait = 0;
unsigned int sum;
unsigned int checksum[ 3 ];

unsigned char ui8_current_cal[3];
unsigned char ui8_calib_flag;

extern unsigned char ui8_Task_Cont1ms;
extern unsigned char ui8_Ki_Lin;
extern unsigned char ui8_Kp_Lin;
extern unsigned char ui8_temp_calibration;
extern unsigned char ui8_b_DResServID_c;
extern unsigned char ui8_b_DResLocID_c;
extern unsigned char ui8_b_DResB0_c;
extern unsigned char ui8_b_DResB1_c;
extern unsigned char ui8_b_DResB2_c;
extern unsigned char ui8_b_DResB3_c;
extern unsigned char ui8_b_DResB4_c;
extern unsigned char ui8_b_DResB5_c;
extern unsigned short ui16_duty_cycle_mat;

int last_pwm;
unsigned char pwm_cmd;




void init_ports( void );
void system_init( void );
int rate_limit(int pwm);

extern void Task1ms( void );

# 178
void I_calibrationInit(void)

{

if ( read_eeprom_data( 0x03 ) != 0x00 && read_eeprom_data( 0x03 ) != 0xFF)

{

ui16_I_cal_Ph1 = ( read_eeprom_data( 0x02 )<<8 ) | read_eeprom_data( 0x03 );


}


else
{

ui16_I_cal_Ph1 = 0x0100;


}


if ( read_eeprom_data( 0x05 ) != 0x00 && read_eeprom_data( 0x05 ) != 0xFF)

{

ui16_I_cal_Ph2 = ( read_eeprom_data( 0x04 )<<8 ) | read_eeprom_data( 0x05 );


}


else
{

ui16_I_cal_Ph2 = 0x0100;


}


if ( read_eeprom_data( 0x07 ) != 0x00 && read_eeprom_data( 0x07 ) != 0xFF)

{

ui16_I_cal_Ph3 = ( read_eeprom_data( 0x06 )<<8 ) | read_eeprom_data( 0x07 );

}


else
{

ui16_I_cal_Ph3 = 0x0100;


}

# 328
LATC2 = 0;
LATB2 = 0;
LATB1 = 0;
LATC = LATC | 0b00011010;

ADC_Wait();
ADC_Wait();

( ADCON0 = ( 0x04 << 2 ) | 0x01 );
ADC_Wait();
( GO_nDONE = 1 );


ui8_current_cal[2] = ADC_Read();

( ADCON0 = ( 0x0D << 2 ) | 0x01 );
ADC_Wait();
( GO_nDONE = 1 );


ui8_current_cal[1] = ADC_Read();

( ADCON0 = ( 0x02 << 2 ) | 0x01 );
ADC_Wait();
( GO_nDONE = 1 );


ui8_current_cal[0] = ADC_Read();

# 361
}

# 408
void init_ports( void )

{

# 415
ANSELA = 0b00100111;
ANSELB = 0b00101001;


LATA = 0;
LATB = 0;
LATC = 0;

# 502
TRISA = 0b01111111;

# 511
TRISB = 0b00111001;

# 520
TRISC = 0b00000000;

# 535
}

# 549
void system_init( void )

{

OPTION_REG = 0b10000001;

# 562
ui8_ResetMatlab = 1;
GIE = 0;
INTCON = 0;
PIE1 = 0;
PIE2 = 0;
PIE3 = 0;

ui16_Timer_VaL1 = 0;
ui8_Task_Cont3ms = 0;
ui8_Task_Cont5ms = 0;
ui8_Task_Cont100ms = 0;
ui8_DebugCnt = 0;

ui8_Sync_Cont10ms = 0;

ui16_dryRun_Thresh = 73;

ui16_Current_Thresh = 360;

init_ports( );

# 592
Oscill_Source_Block( );
timer_init( 1 );
timer_init( 4 );
timer_init( 6 );


PWM_Capture_init( 5 );
ADC_Init( );
DiagInit( );
FILTER_Init( );


ui16_Temp_cal = 0;

# 614
I_calibrationInit();

init_bldc( );

PEIE = 1;
GIE = 1;

}

# 634
void main( void )

{

# 646
system_init( );
clear_timer( 4 );
ui8_lastTaskvalue = 0;

# 699
while( 1 )

{

if( ( ui8_Task_Cont1ms - ui8_lastTaskvalue ) != 0)

{

# 711
ui8_lastTaskvalue = ui8_Task_Cont1ms;


ui8_Task_Cont3ms++;
ui8_Task_Cont5ms++;
ui8_Task_Cont100ms++;

# 935
PR2 = ( unsigned char ) ( 32000000UL / ( 16 * 20000UL ) ) * 5 / 4;
ui16_duty_cycle_BLDC = ui16_duty_cycle_mat*5/4;

# 953
Get_Analog_Value( );

# 999
if( ui8_Task_Cont3ms > 2 )

{

ui8_Task_Cont3ms = 0;

# 1038
}

# 1044
if( ui8_Task_Cont5ms > 4 )

{



ui8_Task_Cont5ms = 0;


asm("clrwdt");




pwm_cmd = PWMReadDC( );
ui8_PWM_dc_mat = rate_limit(pwm_cmd);


ui16_PWM_Freq_mat = ui16_PWM_Freq_In;
ui16_Speed_mat = ui16_speed_fil;
ui16_mat_inpTemp = ui16_NTC_Temp_bldc_mean_cal;
ui8_Ki_mat = 5;
ui8_Kp_mat = 0;

# 1114
ui16_Speed_demand_mat = PWM_trans_table[ ui8_PWM_dc_mat ];


ui16_Speed_demand_mat_Max = ( PWM_trans_table[ 200 ] );


ui16_Speed_demand_mat_min = ( PWM_trans_table[ 20 ] );

# 1139
ui8_BattVolt_mat = ( unsigned char )( ui16_fir_Bat_mittel>>2 );
ui16_mat_Current = ui16_fir_IPhase_mean.w;
bool_mat_currAlarm_bldc = MotorFlags.bits.B0;

# 1152
bool_ControlLoopMode = 0;

# 1160
BVH2_Appl_Layer( );


ui8_ResetMatlab = 0;

# 1169
if( ui16_duty_cycle_mat == 0 )

{

InitMotorStop( );


}


else
{

InitMotorRun( );


}

# 1192
ui8_error_Flags.bits.B0 = bool_PWMin_err_Alarm;
ui8_error_Flags.bits.B1 = bool_CPU_TempAlarm;
ui8_error_Flags.bits.B7 = bool_CPU_TempRedAlarm;
ui8_error_Flags.bits.B2 = bool_HighCurrentAlarm;
ui8_error_Flags.bits.B3 = bool_MotorStalled;
ui8_error_Flags.bits.B4 = bool_DryRunningAlarm;
ui8_error_Flags.bits.B6 = bool_UbatAlarm;
ui8_error_Flags.bits.B5 = bool_PWMin_Freq_err_Alarm;




MotorFlags.bits.B0 = 0;

# 1260
}

# 1266
if( ui8_Task_Cont100ms == 25 )

{

# 1279
SetDiagAlarm( );

# 1343
}

# 1349
if( ui8_Task_Cont100ms == 37 )

{

# 1381
}

# 1387
if( ui8_Task_Cont100ms == 50 )

{

# 1431
}

# 1437
if( ui8_Task_Cont100ms == 75 )

{

# 1481
}

# 1487
if( ui8_Task_Cont100ms > 100 )

{



ui8_Task_Cont100ms = 0;

# 1533
}

# 1561
bool_start_demand_mat = 0;

# 1567
}


else
{



}


}


}



int sign(int x) {
return (x > 0) - (x < 0);
}

int rate_limit(int pwm) {
int delta_pwm = last_pwm - pwm;
if(delta_pwm < 0) {
int sign_delta_pwm = sign(delta_pwm);
int abs_delta_pwm = delta_pwm;
if(delta_pwm < 0) {abs_delta_pwm = delta_pwm * -1;}
int pwm_inc = 5;
int sign_pwm_inc = pwm_inc * sign_delta_pwm;
sign_pwm_inc = last_pwm - sign_pwm_inc;

if (pwm_inc >= abs_delta_pwm) {
last_pwm = pwm;
}
else {
last_pwm = sign_pwm_inc;
}
}
else {
last_pwm = pwm;
}


return last_pwm;
}
