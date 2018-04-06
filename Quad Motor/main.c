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
 ****************************************************************************************/
#define FORWARD 0
#define REVERSE 1
#define PWM_MAX 4095

#define M1 0
#define M2 1
#define M3 2
#define M4 3

#define LEFTFRONTPWM M2
#define LEFTFRONTENC TMR5        

#define RIGHTFRONTPWM M1
#define RIGHTFRONTENC TMR2
    
#define LEFTREARPWM M3
#define LEFTREARENC TMR4   
    
#define RIGHTREARPWM M4
#define RIGHTREARENC TMR3  

#define USE_PWM
// #define USE_AD
// #define TEST_EEPROM

// #define I2C_CLOCK_FREQ              100000
// #define EEPROM_I2C_BUS              I2C1
// #define EEPROM_ADDRESS              0x50     // 0b1010000 Serial EEPROM address    
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

#define START_ONE 80
#define START_TWO 80
#define START_THREE 20
#define START_FOUR 20
#define TIMEOUT 200
#define UART_TIMEOUT 400
#define MAXBITLENGTH 20

#define STX 36
#define ETX 13
#define DLE 16


/** V A R I A B L E S ********************************************************/
unsigned char HOSTRxBuffer[MAXBUFFER + 1];
unsigned char HOSTTxBuffer[MAXBUFFER + 1];
#define NUM_MOTORS 4
#define NUM_MOTOR_REGISTERS (NUM_MOTORS * 2)
unsigned char MotorData[NUM_MOTOR_REGISTERS];

unsigned short RxLength = 0;
unsigned short TxLength = 0;

unsigned short previousExpected = 0, numExpectedBytes = 0;
unsigned char error = 0;
unsigned char RXstate = 0;
unsigned char timeoutFlag = FALSE;
unsigned short numBytesReceived = 0;
short uartTimeout = 0;
unsigned char startFlag = false;
unsigned char escapeFlag = false;
unsigned short RxIndex = 0;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
extern unsigned char getCRC8(unsigned char *ptrMessage, short numBytes);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
// void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);

unsigned char PCAReadByte(unsigned char device, unsigned char PCAcontrolRegister, unsigned char *ptrData);
unsigned char PCAWriteByte(unsigned char device, unsigned char PCAcontrolRegister, unsigned char data);

unsigned char setPCA9685outputs(unsigned char device, unsigned short channel, unsigned short turnON, unsigned short turnOFF);
unsigned char initializePCA9685(unsigned char device);
unsigned int decodePacket(unsigned char *ptrInPacket, unsigned int numInBytes, unsigned char *ptrData);

void Halt(void);
unsigned char testRUN(short PWMvalue, unsigned char direction);
long getPositionError(short i, short targetSpeed);
long PIDcontrol(long error);
unsigned char setMotorPWM(short side, short PWMvalue, unsigned char direction);
void putch(unsigned char ch);

unsigned int decodePacket(unsigned char *ptrInPacket, unsigned int inPacketSize, unsigned char *ptrData);

#define PWM_OFFSET 800

long kP = 50, kI = 0, kD = 0;


union convertType {
    unsigned char byte[2];
    short integer;
} convert;

#define NUM_ENCODERS 2
#define LEFT 0
#define RIGHT 1
long actualPos[NUM_ENCODERS], targetPos[NUM_ENCODERS];

long errorLeft, errorRight, Lcorr, Rcorr, PWMleft, PWMright, posLeft, posRight;

#define MAXPWM 3000
unsigned char ch = 0;


#define RESET_DIRECTION 3

unsigned short Timer2Counter = 0;
unsigned char Timer2Flag = false;
unsigned short tenths = 0, seconds = 0, minutes = 0;
unsigned char stopWatchOn = false;

unsigned char dataFlag = FALSE;

int main(void) {
    unsigned char command = 0, subCommand = 0;
    unsigned char MOT1Direction = 0, MOT2Direction = 0, MOT3Direction = 0, MOT4Direction = 0;
    unsigned char PrevMOT1Direction = RESET_DIRECTION, PrevMOT2Direction = RESET_DIRECTION, PrevMOT3Direction = RESET_DIRECTION, PrevMOT4Direction = RESET_DIRECTION;
    unsigned char NewDirection = 0;
    long wheel1 = 0, wheel2 = 0, wheel3 = 0, wheel4 = 0;
    long velocity1 = 0, velocity2 = 0, velocity3 = 0, velocity4 = 0;
    short displayCounter = 0;
    unsigned char displayMode = FALSE;
    unsigned char rightMotorMSB = 0, rightMotorLSB = 0, leftMotorMSB = 0, leftMotorLSB = 0;
    short PWMvalue = 0;
    unsigned char testChar = 'A';
    
    PWM1 = PWM2 = PWM3 = PWM4 = 0;

    DelayMs(200);
    UserInit();    
    
    while (1) {        
        DelayMs(1);
        if (dataFlag) {
            dataFlag = FALSE;
            command = HOSTRxBuffer[0];
            subCommand = HOSTRxBuffer[1];
            if (command == ROOMBA) {
                rightMotorMSB = MotorData[0];
                rightMotorLSB = MotorData[1];
                leftMotorMSB = MotorData[2];
                leftMotorLSB = MotorData[3];
                
                convert.byte[1] = rightMotorMSB;
                convert.byte[0] = rightMotorLSB;
                PWMvalue = abs(convert.integer) * 3;
                if (PWMvalue > MAXPWM) PWMvalue = MAXPWM;                  
                PWM1 = PWM2 = PWMvalue;
                
                DIR1_OUT = DIR2_OUT = REVERSE;
                DIR3_OUT = DIR4_OUT = FORWARD;
                if (convert.integer < 0) DIR1_OUT = DIR2_OUT = REVERSE;
                else DIR1_OUT = DIR2_OUT = FORWARD;
                
                convert.byte[1] = leftMotorMSB;
                convert.byte[0] = leftMotorLSB;  
                PWMvalue = abs(convert.integer) * 3;
                if (PWMvalue > MAXPWM) PWMvalue = MAXPWM;                    
                PWM3 = PWM4 = PWMvalue;
                
                if (convert.integer < 0) DIR3_OUT = DIR4_OUT = FORWARD;
                else DIR3_OUT = DIR4_OUT = REVERSE;
                
            } else if (subCommand == QUIT) {
                ;
            } else if (subCommand == SHUTDOWN) {
                ;
            }            
        }                
    }
}


void putch(unsigned char ch) {
    while (!IFS1bits.U2TXIF); // set when register is empty 
    U2TXREG = ch;
}



/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 * PPSInput(2,U2RX,RPA9);       // Assign U2RX to pin RPA09
 * PPSInput(3,U2CTS,RPC3);      // Assign U2CTS to pin RPC3
 * PPSOutput(4,RPC4,U2TX);      // Assign U2TX to pin RPC4
 * PPSOutput(1,RPB15,U2RTS);    // Assign U2RTS to pin RPB15    
 *
 *****************************************************************************/


void  UserInit(void)
{
    unsigned char ch;

    mJTAGPortEnable(false);
    PORTSetPinsDigitalOut(IOPORT_C, BIT_5 | BIT_7);
    PORTSetPinsDigitalOut(IOPORT_A, BIT_7 | BIT_10);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_4);

    DIR1_OUT = DIR2_OUT = DIR3_OUT = DIR4_OUT = 0;

    PORTSetPinsDigitalIn(IOPORT_C, BIT_1 | BIT_9);

    ANSELBbits.ANSB15 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB13 = 0;
    PORTSetPinsDigitalIn(IOPORT_B, BIT_13 | BIT_14 | BIT_15);
    DISABLE_OUT = 0;

    PORTSetPinsDigitalIn(IOPORT_B, BIT_13 | BIT_14 | BIT_15);

    // Set up main UART    
    PPSOutput(4, RPC2, U2TX); // REV 1.0 
    PPSInput(2, U2RX, RPA8);
    // PPSOutput(4, RPC9, U2TX);    
    // PPSInput(2, U2RX, RPC8);

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
    OC1CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC1CONbits.OCM1 = 1;
    OC1CONbits.OCM0 = 0;
    OC1RS = 0;

    // Set up PWM OC2
    PPSOutput(2, RPC8, OC2);
    OC2CON = 0x00;
    OC2CONbits.OC32 = 0; // 16 bit PWM
    OC2CONbits.ON = 1; // Turn on PWM
    OC2CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC2CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC2CONbits.OCM1 = 1;
    OC2CONbits.OCM0 = 0;
    OC2RS = 0;

    // Set up PWM OC3
    PPSOutput(4, RPC4, OC3);
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up PWM OC4 on D6 on the Olimex 220 board:
    PPSOutput(3, RPC6, OC4);
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
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

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {


    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;
            RxIndex = 0;
        }


        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = UARTGetDataByte(HOSTuart);
            uartTimeout = 20;

            // Store next char if packet is valid and board number matches
            if (startFlag && RxIndex < MAXBUFFER) HOSTRxBuffer[RxIndex] = ch;

            // If preceding character wasn't an escape char:
            // check whether it is STX, ETX or DLE,
            // otherwise if board number matches then store and advance for next char
            if (escapeFlag == false || startFlag == false) {
                if (ch == DLE) escapeFlag = true;
                else if (ch == STX) {
                    RxIndex = 0;
                    startFlag = true;
                } else if (ch == ETX) {
                    startFlag = false;  
                    if (dataFlag == FALSE)
                    {
                        dataFlag = TRUE;
                        MotorData[0] = HOSTRxBuffer[3];
                        MotorData[1] = HOSTRxBuffer[4];
                        MotorData[2] = HOSTRxBuffer[5];
                        MotorData[3] = HOSTRxBuffer[6];
                    }
                    RxIndex = 0;
                    uartTimeout = 0;
                } else if (startFlag) RxIndex++;
            }// Otherwise if preceding character was an escape char:	
            else {
                escapeFlag = false;
                if (startFlag) RxIndex++;
            }
        }
        if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
            INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
        }
    }
}


#define MULTIPLIER 100

long getPositionError(short wheel, short targetSpeed) 
{
    long error, newPos, lngTargetSpeed;

    if (wheel >= NUM_ENCODERS) return (0);

    if (wheel == LEFT) {
        newPos = (long) LEFTREARENC;
        //LEFTREARENC = 0;
    } else {
        newPos = (long) RIGHTREARENC;
        //RIGHTREARENC = 0;        
    }

    lngTargetSpeed = (long) abs(targetSpeed);

    actualPos[wheel] = (newPos * (long) MULTIPLIER);
    targetPos[wheel] = targetPos[wheel] + lngTargetSpeed;

    if (wheel == LEFT) posLeft = actualPos[wheel];
    else posRight = actualPos[wheel];

    error = actualPos[wheel] - targetPos[wheel];

    if (wheel == LEFT) {
        errorLeft = error / MULTIPLIER;
    } else {
        errorRight = error / MULTIPLIER;
    }

    return (error);
}

#define DIVIDER 10000

long PIDcontrol(long error) {
    long PIDcorrection;
    long PCorr = 0;

    PCorr = error * kP;
    PIDcorrection = PCorr;
    PIDcorrection = PIDcorrection / DIVIDER;
    if (PIDcorrection > PWM_MAX) PIDcorrection = PWM_MAX;
    if (PIDcorrection < -PWM_MAX) PIDcorrection = -PWM_MAX;

    return (PIDcorrection);
}

// Timer 2 generates an interrupt every 51 microseconds approximately
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void) {
    mT2ClearIntFlag(); // Clear interrupt flag
    if (uartTimeout)
    {
        uartTimeout--;
        if (uartTimeout == 0)
        {
            startFlag = false;
            escapeFlag = false;
            RxIndex = 0;        
        }
    }
}