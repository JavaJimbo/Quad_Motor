/***************************************************************************************
 * Project:     Quad Motor 220 - NO USB for PIC 32MX220
 *              Compiled using XC32 Version 1.30 on MPLABX V4.01
 *              For Quad Motor Controller board V1.0
 * FileName:    main.c 
 *  
 * 1-15-17: 
 * 2-25-17: Add decoding and 
 * 2-28-17: Added PIDcontrol() but haven't tested it yet.xxx
 * 4-22-17: For Quad Motor Board V1.0
 * 4-23-17: Got most functions working on Rev 1 Quad board.
 * Removed PCA9685 code
 * 4-24-17: USB works. Encoders work. 
 * 5-6-17:  Testing with all four motors at once.
 * 5-9-17:  Works with USB Joystick - goes forward and reverse and steers right and left.
 * 7-13-17: Flip flop works with TI chip
 * 2-20-18: 
 * 4-5-18:  Got this working with Xbee Dual Joystick 
 *          for four wheeler using Quad Motor Controller board V1.0
 * 4-6-18:  Moved motor math from joystick to Quad board, so joystick data is transmitted.
 * 4-8-18:  Works with XBBE Dual Joystick - PID control steering four wheels.
 ****************************************************************************************/
#define TESTOUT LATCbits.LATC0
#define FORWARD 0
#define REVERSE 1
#define PWM_MAX 3000
#define DIVIDER 256

#define DISABLE_OUT PORTBbits.RB4
#define FAULT_IN PORTCbits.RC1     

#define EncoderOne TMR1
#define EncoderTwo TMR4
#define EncoderThree TMR3
#define EncoderFour TMR5

#define ENC1_LATCH PORTAbits.RA4
#define ENC2_LATCH PORTBbits.RB2
#define ENC3_LATCH PORTBbits.RB1
#define ENC4_LATCH PORTBbits.RB0

#define ENC1_DIR PORTBbits.RB15
#define ENC2_DIR PORTBbits.RB14
#define ENC3_DIR PORTBbits.RB13
#define ENC4_DIR PORTCbits.RC9

#define PWM1 OC4RS
#define PWM2 OC3RS
#define PWM3 OC2RS
#define PWM4 OC1RS

#define DIR1_OUT LATAbits.LATA7
#define DIR2_OUT LATCbits.LATC5
#define DIR3_OUT LATAbits.LATA10
#define DIR4_OUT LATCbits.LATC7

#define STX 36
#define ETX 13
#define DLE 16
#define MAXPACKET 80
#define MAXVELOCITY 500
#define DRIVEDIRECT 145
#define ROOMBA 0
#define RASPI 240
#define ROBOTNIK 19
#define SETPID 69
#define START 128
#define STOP 173
#define POWERDOWN 133
#define RESET 7
#define SAFE 131
#define FULL 132
#define QUIT 128
#define SHUTDOWN 160
#define STANDBY 0
#define RUN 111

#define SYS_FREQ 60000000
#define GetPeripheralClock() SYS_FREQ 

#include "Delay.h"
 #include <plib.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier for 220 - yields 60 Mhz
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx3      // ICE/ICD Comm Channel Select
#pragma config JTAGEN   = OFF           // Use JTAG pins for normal IO
#pragma config DEBUG    = OFF            // Enable/disable debugging


/*** DEFINES *****************************************************************/
#define HOSTuart UART2
#define SYS_FREQ 60000000  // With 8 Mhz crystal and FPLLMUL = MUL_15
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR     
#define MAXBUFFER 64

#define false FALSE
#define true TRUE

#define STX 36
#define ETX 13
#define DLE 16

#define NUMMOTORS 4

/** V A R I A B L E S ********************************************************/

struct PIDtype
{
    long sumError;
    short derIndex;
    unsigned char saturationFlag;
    short kP;
    short kI;
    short kD;
    unsigned short PWMoffset;
} PID[NUMMOTORS];
#define MAXNUM 16
unsigned char NUMbuffer[MAXNUM + 1];

#define MAXDER 3
short errorDerivative[NUMMOTORS][MAXDER];

void initializeErrorArrays(void){
unsigned short i, j;

    for(j = 0; j < NUMMOTORS; j++){
        for(i = 0; i < MAXDER; i++) errorDerivative[j][i] = 0;
    }
}

short uartTimeout = 0;
unsigned char startFlag = false;
unsigned char escapeFlag = false;
unsigned short UARTRxIndex = 0;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
extern unsigned char getCRC8(unsigned char *ptrMessage, short numBytes);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
// void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);

short PIDcontrol(short servoID, short error);
void putch(unsigned char ch);
void putch1(unsigned char ch);


union convertType {
    unsigned char byte[2];
    short integer;
} convert;

#define MAXPWM 3000

unsigned char intFlag = FALSE;

#define CommandUART UART2
unsigned char CommandBuffer[MAXBUFFER + 1];
unsigned char CommandBufferFull = FALSE;

//unsigned char HOSTRxBuffer[MAXBUFFER + 1];
//unsigned char HOSTBufferFull = FALSE;

#define CommandByte      CommandBuffer[0]
#define SubCommandByte   CommandBuffer[1]
#define TransmitLength   CommandBuffer[2]
#define LSBLeftJoystickX CommandBuffer[3]
#define MSBLeftJoystickX CommandBuffer[4]
#define LSBLeftJoystickY CommandBuffer[5]
#define MSBLeftJoystickY CommandBuffer[6]
#define LSBRightJoystickX CommandBuffer[7]
#define MSBRightJoystickX CommandBuffer[8]
#define LSBRightJoystickY CommandBuffer[9]
#define MSBRightJoystickY CommandBuffer[10]
#define LSB_CRCresult     CommandBuffer[11]
#define MSB_CRCresult     CommandBuffer[12]

unsigned char displayFlag = FALSE;

int main(void) 
{
    // short jogPWM = 0;
    unsigned char i = 0, p = 0, q = 0; 
    short PWMvalue = 0;
    short intLeftJoystickY = 0, intLeftJoystickX = 0, intRightJoystickY = 0, intRightJoystickX = 0;
    unsigned short CRCresult;  
    long ActualPosition1 = 0, ActualPosition2 = 0, ActualPosition3 = 0, ActualPosition4 = 0;
    long CommandPosition1 = 0, CommandPosition2 = 0, CommandPosition3 = 0, CommandPosition4 = 0;    
    long lngError = 0;
    short Error1 = 0, Error2 = 0, Error3 = 0, Error4 = 0;
    unsigned char runMode = TRUE;
        
    PWM1 = PWM2 = PWM3 = PWM4 = 0;

    for (i = 0; i < NUMMOTORS; i++)
    {
        PID[i].sumError = 0;
        PID[i].derIndex = 0;
        PID[i].saturationFlag = FALSE;
        PID[i].kP = 200;
        PID[i].kI = 10;
        PID[i].kD = 200;
        PID[i].PWMoffset = 100;
    }
    
    DelayMs(200);
    UserInit();      
    printf("\rTesting PID");
    
    while (1) {        
        DelayMs(1);
        if (intFlag && runMode)
        {
            intFlag = FALSE;
            // if (jogPWM != 0) intRightJoystickY = jogPWM;
            CommandPosition1 = CommandPosition1 + (intRightJoystickY * 8) - (intRightJoystickX * 4);
            CommandPosition2 = CommandPosition2 + (intRightJoystickY * 8) - (intRightJoystickX * 4);
            CommandPosition3 = CommandPosition3 + (intRightJoystickY * 8) + (intRightJoystickX * 4);
            CommandPosition4 = CommandPosition4 + (intRightJoystickY * 8) + (intRightJoystickX * 4
                    );
            
            if (DIR1_OUT == FORWARD) 
                ActualPosition1 = ActualPosition1 + (long) (EncoderOne * DIVIDER); 
            else ActualPosition1 = ActualPosition1 - (long) (EncoderOne * DIVIDER); 
            EncoderOne = 0;                                    
            
            if (DIR2_OUT == FORWARD) 
                ActualPosition2 = ActualPosition2 + (long) (EncoderTwo * DIVIDER);
            else ActualPosition2 = ActualPosition2 - (long) (EncoderTwo * DIVIDER);
            EncoderTwo = 0;
            
            if (DIR3_OUT == FORWARD) 
                ActualPosition3 = ActualPosition3 - (long) (EncoderThree * DIVIDER);
            else ActualPosition3 = ActualPosition3 + (long) (EncoderThree * DIVIDER);
            EncoderThree = 0;
            
            if (DIR4_OUT == FORWARD) 
                ActualPosition4 = ActualPosition4 - (long) (EncoderFour * DIVIDER);
            else ActualPosition4 = ActualPosition4 + (long) (EncoderFour * DIVIDER);
            EncoderFour = 0;
            
            lngError = ActualPosition1 - CommandPosition1;
            if (lngError > 0x7FFF) lngError = 0x7FFF;
            else if (lngError < -0x7FFF) lngError = -0x7FFF;
            Error1 = (short) lngError;
         
            lngError = ActualPosition2 - CommandPosition2;
            if (lngError > 0x7FFF) lngError = 0x7FFF;
            else if (lngError < -0x7FFF) lngError = -0x7FFF;
            Error2 = (short) lngError;
         
            lngError = ActualPosition3 - CommandPosition3;
            if (lngError > 0x7FFF) lngError = 0x7FFF;
            else if (lngError < -0x7FFF) lngError = -0x7FFF;
            Error3 = (short) lngError;
         
            lngError = ActualPosition4 - CommandPosition4;
            if (lngError > 0x7FFF) lngError = 0x7FFF;
            else if (lngError < -0x7FFF) lngError = -0x7FFF;
            Error4 = (short) lngError;

            PWMvalue = PIDcontrol(0, Error1);
            if (PWMvalue < 0)          
            {            
                DIR1_OUT = REVERSE;
                PWMvalue = 0 - PWMvalue;
            }
            else DIR1_OUT = FORWARD;                      
            PWM1 = PWMvalue;

            PWMvalue = PIDcontrol(1, Error2);
            if (PWMvalue < 0)          
            {            
                DIR2_OUT = REVERSE;
                PWMvalue = 0 - PWMvalue;
            }
            else DIR2_OUT = FORWARD;                      
            PWM2 = PWMvalue;
            
            PWMvalue = PIDcontrol(2, Error3);
            if (PWMvalue < 0)          
            {            
                DIR3_OUT = FORWARD;
                PWMvalue = 0 - PWMvalue;
            }
            else DIR3_OUT = REVERSE;                      
            PWM3 = PWMvalue;
            
            PWMvalue = PIDcontrol(3, Error4);
            if (PWMvalue < 0)          
            {            
                DIR4_OUT = FORWARD;
                PWMvalue = 0 - PWMvalue;
            }
            else DIR4_OUT = REVERSE;                      
            PWM4 = PWMvalue;                        
        }
        else if (runMode == FALSE)
        {
            CommandPosition1 = ActualPosition1 = 0;
            PWM1 = PWM2 = PWM3 = PWM4 = 0;
        }
        
        /*
        if (HOSTBufferFull)
        {
            HOSTBufferFull = FALSE;
            printf("\rReceived: ");
            q = 0;
            command = 0;
            for (p = 0; p < MAXBUFFER; p++) {
                ch = toupper (HOSTRxBuffer[p]);
                if (isalpha(ch)) command = ch;
                else if (ch == ' ') command = ' ';
                putch(ch);
                if (ch == '\r' || ch == ' ')break;
                if ((isdigit(ch) || ch == '-') && q < MAXNUM) NUMbuffer[q++] = ch;
            }
            if (q) {
                NUMbuffer[q] = '\0';
                value = atoi(NUMbuffer);
            }
            if (command) {
                switch (command) {
                    case 'J':
                        jogPWM = value;
                        printf("\rJog: %d", jogPWM);
                        break;
                    case 'P':
                        if (q) PID[0].kP = value;
                        break;
                    case 'I':
                        if (q) PID[0].kI = value;
                        break;
                    case 'D':
                        if (q) PID[0].kD = value;
                        break;
                    case 'O':
                        if (q) PID[0].PWMoffset = value;
                        break;
                    case ' ':
                        if (runMode){
                            runMode = FALSE;
                            printf("\rHALT");
                        }
                        else {
                            runMode = TRUE;
                            printf("\rRUN");
                        }
                        break;
                    case 'M':
                        if (displayFlag){
                            displayFlag = FALSE;
                            printf("\rDisplay OFF");
                        }
                        else {
                            displayFlag = TRUE;
                            printf("\rDisplay ON");
                        }                            
                    default:
                        printf("\rCommand: %c", command);
                        break;
                }
                printf("\rkP=%d, kI=%d, kD=%d, OFFSET: %d", PID[0].kP, PID[0].kI, PID[0].kD, PID[0].PWMoffset);
                command = 0;
            }
            putch('\r');
        }
        */
        
        if (CommandBufferFull) 
        {
            CommandBufferFull = FALSE;
            
            if (CommandByte == ROOMBA) {                            
                convert.byte[0] = LSBLeftJoystickX;
                convert.byte[1] = MSBLeftJoystickX;
                intLeftJoystickX = convert.integer;
            
                convert.byte[0] = LSBLeftJoystickY;
                convert.byte[1] = MSBLeftJoystickY;
                intLeftJoystickY = convert.integer;
            
                convert.byte[0] = LSBRightJoystickX;
                convert.byte[1] = MSBRightJoystickX;
                intRightJoystickX = convert.integer;
            
                convert.byte[0] = LSBRightJoystickY;
                convert.byte[1] = MSBRightJoystickY;
                intRightJoystickY = convert.integer;
            
                convert.byte[0] = LSB_CRCresult;
                convert.byte[1] = MSB_CRCresult;
                CRCresult = convert.integer;            
            } // End if command == ROOMBA
        } // End if UART buffer full
        
    } // End while(1))
} // End main())


void putch(unsigned char ch) {
    while (!IFS1bits.U2TXIF); // set when register is empty 
    U2TXREG = ch;
}

void putch1(unsigned char ch) {
    while (!IFS1bits.U1TXIF); // set when register is empty 
    U1TXREG = ch;
}


void  UserInit(void)
{
    unsigned char ch;

    mJTAGPortEnable(false);
    PORTSetPinsDigitalOut(IOPORT_C, BIT_5 | BIT_7);
    PORTSetPinsDigitalOut(IOPORT_A, BIT_7 | BIT_10);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_4);

    DIR1_OUT = DIR2_OUT = DIR3_OUT = DIR4_OUT = 0;

    PORTSetPinsDigitalIn(IOPORT_C, BIT_1 | BIT_9);
    PORTSetPinsDigitalOut(IOPORT_C, BIT_0);

    ANSELBbits.ANSB15 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB13 = 0;
    ANSELCbits.ANSC3 = 0;  // For Pin 36 - CommandUART RX
    
    PORTSetPinsDigitalIn(IOPORT_B, BIT_13 | BIT_14 | BIT_15);
    DISABLE_OUT = 0;

    PORTSetPinsDigitalIn(IOPORT_B, BIT_13 | BIT_14 | BIT_15);

    // Set up main UART    
    PPSOutput(4, RPC2, U2TX); // REV 1.0 
    PPSInput(2, U2RX, RPA8);
    // PPSOutput(4, RPC9, U2TX);    
    // PPSInput(2, U2RX, RPC8);
    PPSOutput(1, RPB3, U1TX); // Rev 1.0
    PPSInput(3, U1RX, RPC3); // Rev 1.0
    
    /*
    // Configure UART #1
    UARTConfigure(CommandUART, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(CommandUART, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(CommandUART, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(CommandUART, SYS_FREQ, 57600);
    UARTEnable(CommandUART, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #1 Interrupts
    INTEnable(INT_U1TX, INT_DISABLED);
    INTEnable(INT_U1RX, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(CommandUART), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(CommandUART), INT_SUB_PRIORITY_LEVEL_0);
    */
    
    // Configure UART #2 (HOST UART))
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_U2RX, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);    

    do {
        ch = UARTGetDataByte(HOSTuart);
    } while (ch);

    // Set digital inputs
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;

    // Set timer counter inputs
    PPSInput(4, T5CK, RPB0);
    PPSInput(2, T3CK, RPB1);
    PPSInput(3, T4CK, RPB2);

    // Set up timers as counters
    T1CON = 0x00;
    T1CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T1CONbits.TCKPS1 = 0; // 1:1 Prescaler
    T1CONbits.TCKPS0 = 0;
    T1CONbits.TSYNC = 1;
    PR1 = 0xFFFF;
    T1CONbits.TON = 1; // Let her rip 

    T3CON = 0x00;
    T3CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T3CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T3CONbits.TCKPS1 = 0;
    T3CONbits.TCKPS0 = 0;
    PR3 = 0xFFFF;
    T3CONbits.TON = 1; // Let her rip 

    T4CON = 0x00;
    T4CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T4CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T4CONbits.TCKPS1 = 0;
    T4CONbits.TCKPS0 = 0;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 0xFFFF;
    T4CONbits.TON = 1; // Let her rip 

    T5CON = 0x00;
    T5CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T5CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T5CONbits.TCKPS1 = 0;
    T5CONbits.TCKPS0 = 0;
    PR5 = 0xFFFF;
    T5CONbits.TON = 1; // Let her rip 

    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;
    PR2 = 3000; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip   
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    
    // Set up PWM OC1
    PPSOutput(1, RPB7, OC1);
    OC1CON = 0x00;
    OC1CONbits.OC32 = 0; // 16 bit PWM
    OC1CONbits.ON = 1; // Turn on PWM
    OC1CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC1CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC1CONbits.OCM1 = 1;
    OC1CONbits.OCM0 = 0;
    OC1RS = 0;

    // Set up PWM OC2
    PPSOutput(2, RPC8, OC2);
    OC2CON = 0x00;
    OC2CONbits.OC32 = 0; // 16 bit PWM
    OC2CONbits.ON = 1; // Turn on PWM
    OC2CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC2CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC2CONbits.OCM1 = 1;
    OC2CONbits.OCM0 = 0;
    OC2RS = 0;

    // Set up PWM OC3
    PPSOutput(4, RPC4, OC3);
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up PWM OC4 on D6 on the Olimex 220 board:
    PPSOutput(3, RPC6, OC4);
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 0;

    OC1RS = 0;
    OC2RS = 0;
    OC4RS = 0;
    OC3RS = 0;

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();
}//end UserInit


// Timer 2 generates an interrupt every 50 microseconds approximately
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void) 
{
    static int intCounter = 0;
    
    mT2ClearIntFlag(); // Clear interrupt flag
    if (TESTOUT) TESTOUT = 0;
    else TESTOUT = 1;
    
    intCounter++;
    if (intCounter >= 80)
    {
        intCounter = 0;
        intFlag = TRUE;
    }
    if (uartTimeout)
    {
        uartTimeout--;
        if (uartTimeout == 0)
        {
            startFlag = false;
            escapeFlag = false;
            UARTRxIndex = 0;        
        }
    }
}




 short PIDcontrol(short servoID, short error){
    short PWMout;
    long PIDcorrection, lngError;    
    long diffError, pastError;
    long PCorr = 0, ICorr = 0, DCorr = 0;
    short intPCorr, intICorr, intDCorr, intError;
    static short DisplayCounter = 25;
        
    lngError = (long) error;   
    if (!PID[servoID].saturationFlag) PID[servoID].sumError = PID[servoID].sumError + lngError;    
    
    pastError = (long) errorDerivative[servoID][PID[servoID].derIndex];    
    diffError = lngError - pastError;
    
    errorDerivative[servoID][PID[servoID].derIndex] = error;
    PID[servoID].derIndex++; if (PID[servoID].derIndex >= MAXDER) PID[servoID].derIndex = 0;     
    
    PCorr = lngError * (long) PID[servoID].kP;
    ICorr = PID[servoID].sumError * (long) PID[servoID].kI;
    DCorr = diffError * (long) PID[servoID].kD;

    PIDcorrection = -(PCorr + ICorr + DCorr);
    PIDcorrection = PIDcorrection / DIVIDER;     
    
    if (PIDcorrection < 0) 
        PWMout = (short) PIDcorrection - PID[servoID].PWMoffset;            
    else PWMout = (short) PIDcorrection + PID[servoID].PWMoffset;
    
    if (PWMout > PWM_MAX) 
    {
        PWMout = PWM_MAX;
        PID[servoID].saturationFlag = TRUE;
    }
    else if (PWMout < -PWM_MAX) {
        PWMout = 0 - PWM_MAX;
        PID[servoID].saturationFlag = TRUE;
    }
    else PID[servoID].saturationFlag = FALSE;    
    
    intPCorr = (short) (PCorr / DIVIDER);   
    intICorr = (short) (ICorr / DIVIDER);
    intDCorr = (short) (DCorr / DIVIDER);    
    intError = error / DIVIDER;

    if (displayFlag){
        if (DisplayCounter) DisplayCounter--;
        else {
            printf("\rERR: %d, P: %d, I %d, D: %d, PWM: %d", intError, intPCorr, intICorr, intDCorr, PWMout);
            DisplayCounter = 25;
        }           
    }
    
    return (PWMout);
}

#define UART_TIMEOUT 20
void __ISR(_UART_2_VECTOR, ipl2) IntUartCommandHandler(void) 
{
unsigned char ch;

    if (INTGetFlag(INT_SOURCE_UART_RX(CommandUART))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(CommandUART));
        if (U2STAbits.OERR || U2STAbits.FERR) 
        {
            if (UARTReceivedDataIsAvailable(CommandUART))
                ch = UARTGetDataByte(CommandUART);
            U2STAbits.OERR = 0;
            UARTRxIndex = 0;
        }

        if (UARTReceivedDataIsAvailable(CommandUART)) 
        {
            ch = UARTGetDataByte(CommandUART);
            uartTimeout = UART_TIMEOUT;

            // Store next char if packet is valid and board number matches
            if (startFlag && UARTRxIndex < MAXBUFFER) CommandBuffer[UARTRxIndex] = ch;

            // If preceding character wasn't an escape char:
            // check whether it is STX, ETX or DLE,
            // otherwise if board number matches then store and advance for next char
            if (escapeFlag == false || startFlag == false) 
            {
                if (ch == DLE) escapeFlag = true;
                else if (ch == STX) 
                {
                    UARTRxIndex = 0;
                    startFlag = true;
                } 
                else if (ch == ETX) 
                {
                    startFlag = false;  
                    CommandBufferFull = TRUE;                    
                    UARTRxIndex = 0;
                    uartTimeout = 0;
                } 
                else if (startFlag) UARTRxIndex++;
            }// Otherwise if preceding character was an escape char:	
            else 
            {
                escapeFlag = false;
                if (startFlag) UARTRxIndex++;
            }
        } // If RX data available
    } // End if RX intereeupt
    if (INTGetFlag(INT_SOURCE_UART_TX(CommandUART))) 
        INTClearFlag(INT_SOURCE_UART_TX(CommandUART));            
}


/*
#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) 
{
unsigned char ch;
static unsigned short i = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));                 
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(HOSTuart)) 
        {
            ch = UARTGetDataByte(HOSTuart);            
            if (ch != 0 && ch != '\n') {            
                if (ch == BACKSPACE) 
                {
                    if (i != 0) i--;
                    HOSTRxBuffer[i] = '\0';
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, ' ');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, BACKSPACE);                
                } 
                else if (i < MAXBUFFER) 
                {
                    HOSTRxBuffer[i] = toupper(ch);
                    i++;
                }            
                if ('\r' == ch || ' ' == ch) 
                {
                    HOSTBufferFull = TRUE;
                    HOSTRxBuffer[i] = '\0';
                    i = 0;
                }
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));            
}
*/