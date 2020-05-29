/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 *  MSP432 I2C - EUSCI_B0 I2C Master TX  bytes to MSP432 Slave - Repeated Start
 *
 *  Description: This demo connects two MSP432 's via the I2C bus. The master
 *  transmits to the slave. This is the MASTER CODE. It continuously
 *  transmits an array of data and demonstrates how to implement an I2C
 *  master transmitter sending multiple bytes followed by a repeated start,
 *  followed by a read of multiple bytes.  This is a common operation for
 *  reading register values from I2C slave devices such as sensors. The
 *  transaction for the I2C that is written looks as follows:
 *
 *  _________________________________________________________
 *  |  Start   |      |  Start   |                   |       |
 *  | 0x48Addr | 0x04 | 0x48Addr |  <10 Byte Read>   | Stop  |
 *  |    W     |      |    R     |                   |       |
 *  |__________|______|__________|___________________|_______|
 *
 *  ACLK = n/a, MCLK = HSMCLK = SMCLK = BRCLK = default DCO = ~3.0MHz
 *
 *                                /|\  /|\
 *                MSP432P401      10k  10k      MSP432P401
 *                   slave         |    |         master
 *             -----------------   |    |   -----------------
 *            |     P1.6/UCB0SDA|<-|----+->|P1.6/UCB0SDA     |
 *            |                 |  |       |                 |
 *            |                 |  |       |                 |
 *            |     P1.7/UCB0SCL|<-+------>|P1.7/UCB0SCL     |
 *            |                 |          |                 |
 *
 *****************************************************************************/
/* DriverLib Defines */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "calculations.h"

int vspfunc(char *format, ...);
void printfuart(uint32_t moduleInstance, char *format, ...);

/* Slave Address for I2C Slave */
#define SLAVE_ADDRESS       0x28
#define NUM_OF_REC_BYTES    5

// Sample rate
#define SAMPLE_RATE         1000
// 100 Hz logging uses 65% of the interrupt time to print, don't both going higher.
//        could change if we print less data
#define LOGGING_RATE        10

// PWM for Alarm
#define PWM_ALARM           31

// Offset for noisy data
float offset = 6; // have as user defined input or #define??

/* I2C Variables */
static char s[8];
const uint8_t TXData[] = { 0x04 };
static volatile uint8_t RXData0[NUM_OF_REC_BYTES];
static volatile uint8_t RXData1[NUM_OF_REC_BYTES];
static volatile uint32_t xferIndex0;
static volatile uint32_t xferIndex1;
static volatile bool stopSent;
static volatile int k = 0;
static volatile int c = 0; // constants counter
static volatile int count = 0;

//filter constants, fc = 10Hz, 2 pole, fsample = 100 Hz
//float A[2] = {-1.142980502539901,0.412801598096189};
//float B[3] = {0.067455273889072,0.134910547778144,0.067455273889072};

//filter constants, fc = 10Hz, 2 pole, fsample = 500 Hz
//float A[2] = {-1.822694925196308,0.837181651256023};
//float B[3] = {0.003621681514929,0.007243363029857,0.003621681514929};

//filter constants, fc = 10Hz, 2 pole, fsample = 1000 Hz
float A[2] = { -1.911197067426073, 0.914975834801433 };
float B[3] = { 9.446918438401619e-04, 0.001889383687680, 9.446918438401619e-04 };

//flags
bool init = true;
bool start_alarm = false;
static bool exportExhaleFlowFlag = false;
static bool exportInhaleFlowFlag = false;
static bool exportVolumeFlag = false;
bool exportFlowFlag = false;
bool inhale_flag = false;
bool inhale_flag_past = false;
bool exhale_flag = false;
bool exhale_flag_past = false;

//store pressure, flow and volumes
static int16_t p_currentIn = 0, p_currentEx = 0;
static float p_currentIn_ = 0, p_currentEx_ = 0;
static float p_currentIn_f = 0, p_currentEx_f = 0;
float p_pastIn = 0;
float p_pastEx = 0;
float p_pastIn_f = 0;
float p_pastEx_f = 0;
float p_pastpastIn = 0.0;
float p_pastpastEx = 0.0;
float p_pastpastIn_f = 0.0;
float p_pastpastEx_f = 0.0;
static float totalVolumeExhalePast = 0;
static float totalVolumeInhalePast = 0;
static float totalVolumeInhale = 0;
static float totalVolumeExhale = 0;
static float volFlowrateInhalePast = 0;
static float volFlowrateExhalePast = 0;

static float exportVolumeExhale = 0;
static float exportVolumeInhale = 0;

//state machine variables
float q_v_in = 0;
float q_v_ex = 0;

// initialize variables
//p_currentIn = 0;
//p_currentEx = 0;
float dataCount = 0;
uint8_t readdata = 0;
char init_const[50]; // total of 46 expected, did 50 to be safe
uint32_t init_counter = 0;

// Initial Conditions set by doctor
float init_O2 = 0;              // 6 chars for all these, 0.XXXX
float init_CO2 = 0;
float init_H2O = 0;
float init_N2 = 0;
float init_T_air_ex = 0;        // maybe 1 decimal, XXX.X --> 5 chars
float init_LowVolume = 0;         // XXX.XX --> 6 chars
float init_HighVolume = 0;
float init_PercentDiff = 0;     // XX.XX --> 5 chars,

//states
enum State
{
    initialize, calculation, debounce, rest
};

enum State pastState = initialize;
bool newExp = false;
float p_maxIn = 0;
float p_maxEx = 0;
float volFlowrateInhale = 0;
float volFlowrateExhale = 0;
float volumeInhale = 0;
float volumeExhale = 0;

float frequency = SAMPLE_RATE; // Hz
float N_N2_in = 0.80;
float N_CO2_in = 0;
float N_O2_in = 0.20; // user input
float N_H2O_in = 0; // ? unknown right now

float N_N2_ex = 0.80;
float N_CO2_ex = 0.05; // user input
float N_O2_ex = 0.16; // ? unknown right now
float N_H2O_ex = 0; // ? unknown right now

float T_air_in = 293; // Air temperature during inhale [K]
float T_air_ex = 293; // Air temperature during exhale [K]
double i = 0; // Logging Counter
double j = 0; // Sampling Counter

float dir_c = 0;

enum State state = initialize;

/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        3000000,                                // SMCLK = 3MHz
        EUSCI_B_I2C_SET_DATA_RATE_400KBPS,      // Desired I2C Clock of 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
        };

// UART Configuration
const eUSCI_UART_ConfigV1 uartConfig = {
EUSCI_A_UART_CLOCKSOURCE_SMCLK,                             // SMCLK Source
        1, //19,                                                         // Clock Prescalar   --------
        10, //8,                                                          // FirstModReg       -------- >>> these 3 set baudrate (9600)
        0, //85,                                                          // SecondModReg      --------
        EUSCI_A_UART_NO_PARITY,                                     // No Parity
        EUSCI_A_UART_LSB_FIRST,                                // Send LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                                 // 1 Stop Bit
        EUSCI_A_UART_MODE,                                          // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION           // Oversampling
        };

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfig = {
TIMER_A_CLOCKSOURCE_ACLK,               //clockSource
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          //clockSourceDivider
        62,                                     //timerPeriod
        TIMER_A_CAPTURECOMPARE_REGISTER_1,      //compareRegister
        TIMER_A_OUTPUTMODE_RESET_SET,           //compareOutputMode
        0                                       //dutyCycle , 31 for 50%
        };

int main(void)
{
    /* Disabling the Watchdog  */
    MAP_WDT_A_holdTimer();

    // Enable floating Point Operations
    MAP_FPU_enableModule();

    ///////////////////////////oscilloscope toggling////////////////

    /* Configuring P3.2 as output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN2);
    /* Configuring P3.3 as output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3);
    /* Configuring P3.6 as output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);

    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_16);

    ///////////////////////////Timer32//////////////

    //Sampling Interrupt Initialization
    MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
    TIMER32_PERIODIC_MODE);
    MAP_Interrupt_enableInterrupt(INT_T32_INT1);
    int rate = (CS_getMCLK() / SAMPLE_RATE);
    MAP_Timer32_setCount(TIMER32_0_BASE, rate); //100 Hz ... 1 Hz = 3000000 counts
    MAP_Timer32_enableInterrupt(TIMER32_0_BASE);
    MAP_Timer32_startTimer(TIMER32_0_BASE, false);

    //Logging Interrupt Initialization
    MAP_Timer32_initModule(TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
    TIMER32_PERIODIC_MODE);
    MAP_Interrupt_enableInterrupt(INT_T32_INT2);
    int rate1 = (CS_getMCLK() / LOGGING_RATE);
    MAP_Timer32_setCount(TIMER32_1_BASE, rate1); //100 Hz ... 1 Hz = 3000000 counts
    MAP_Timer32_enableInterrupt(TIMER32_1_BASE);
    MAP_Timer32_startTimer(TIMER32_1_BASE, false);

    ///////////////////////////PWM for buzzer///////////////////

    /* Setting MCLK to REFO at 128Khz for LF mode
     * Setting ACLK to 128Khz */
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //MAP_PCM_setPowerState(PCM_AM_LF_VCORE0);

    /* Configuring GPIO2.4 as peripheral output for PWM */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
    GPIO_PRIMARY_MODULE_FUNCTION);

    /* start PWM signal */
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    /* Enabling interrupts and starting the watchdog timer */
    //MAP_Interrupt_enableSleepOnIsrExit();
    //MAP_Interrupt_enableMaster();
    ///////////////////////////UART/////////////////////////////
    // Selecting P1.2 and P1.3 in UART mode
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3,
            GPIO_PRIMARY_MODULE_FUNCTION);

    //enable LED for blinking on reset
    GPIO_setAsOutputPin(
    GPIO_PORT_P1,
                        GPIO_PIN0);

    // Initialize and enable UART
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
    MAP_UART_enableModule(EUSCI_A0_BASE);
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);

    ///////////////////////////I2C/////////////////////////////

    /* Select Port 1 for I2C - Set Pin 6, 7 to input Primary Module Function,
     *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL respectively).
     */
    //PORT 1 -- BUS 0
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1,
            GPIO_PIN6 + GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION);
    //PORT 6 -- BUS 1
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P6,
            GPIO_PIN5 + GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);

    stopSent = false;

    // set aside bytes of memory to store incoming values
    memset(RXData0, 0x00, NUM_OF_REC_BYTES);
    memset(RXData1, 0x00, NUM_OF_REC_BYTES);

    /* Initializing I2C Master to SMCLK at 100khz with no autostop */
    MAP_I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    MAP_I2C_initMaster(EUSCI_B1_BASE, &i2cConfig);
//
//    /* Specify slave address */
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_ADDRESS);
    MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, SLAVE_ADDRESS);
//
//    /* Enable I2C Module to start operations */
    MAP_I2C_enableModule(EUSCI_B0_BASE);
    MAP_Interrupt_enableInterrupt(INT_EUSCIB0);
    MAP_I2C_enableModule(EUSCI_B1_BASE);
    MAP_Interrupt_enableInterrupt(INT_EUSCIB1);

//    while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE));
//    while (MAP_I2C_masterIsStopSent(EUSCI_B1_BASE)); //NEW

//    MAP_Interrupt_enableSleepOnIsrExit();

    /* Send the two bytes of the "start continuous mode" command */
    //MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, 0x36);
    //MAP_I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, 0x03); //15 for DP avg till read, 03 for Mass Flow avgTR
    //__delay_cycles(150000);
    // // commands to get the clock speeds
    uint32_t aclk = MAP_CS_getACLK(); //128kHz
    uint32_t mclk = MAP_CS_getMCLK(); //48MHz
    uint32_t smclk = MAP_CS_getSMCLK(); //3MHz

    // Interrupt priorities
    /* Configuring interrupt priorities  */
    int a = MAP_Interrupt_getPriority(INT_T32_INT1);
    Interrupt_setPriority(INT_T32_INT2, 1 << 5);
    int b = MAP_Interrupt_getPriority(INT_T32_INT2);

    ////////////////////////get values of custom inputs from GUI////////////////////////

    while (1)
    {
//        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P3, GPIO_PIN2);

        MAP_PCM_gotoLPM0InterruptSafe();
    }
}

/*******************************************************************************
 * eUSCIB0 ISR. The repeated start and transmit/receive operations happen
 * within this ISR.
 *******************************************************************************/
void EUSCIB0_IRQHandler(void)
{
    uint_fast16_t status;

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B0_BASE);

    /* Receives bytes into the receive buffer. If we have received all bytes,
     * send a STOP condition */
    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {
        if (xferIndex0 == NUM_OF_REC_BYTES - 2)
        {
            MAP_I2C_disableInterrupt(EUSCI_B0_BASE,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0);
            MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_STOP_INTERRUPT);

            /*
             * Switch order so that stop is being set during reception of last
             * byte read byte so that next byte can be read.
             */
            MAP_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
            RXData0[xferIndex0++] = MAP_I2C_masterReceiveMultiByteNext(
            EUSCI_B0_BASE);
        }
        else
        {
            RXData0[xferIndex0++] = MAP_I2C_masterReceiveMultiByteNext(
            EUSCI_B0_BASE);
        }
    }
    else if (status & EUSCI_B_I2C_STOP_INTERRUPT)
    {
        MAP_Interrupt_disableSleepOnIsrExit();
        MAP_I2C_disableInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_STOP_INTERRUPT);
    }

    p_currentIn = 0; //, p_currentEx = 0;

    p_currentIn = p_currentIn | RXData0[1];
    p_currentIn = p_currentIn << 8;
    p_currentIn = p_currentIn | RXData0[2];

    p_currentIn = p_currentIn & 0b0011111111111111;

    float output_max = 14745;
    float output_min = 1638;
    float pressure_max = 2500; // in Pascals
    float pressure_min = -2500; // in Pascals

    // calculate with bias offset
    p_currentIn_ =
            0.215 - 3.33
                    + ((((float) (p_currentIn) - output_min)
                            * (pressure_max - pressure_min))
                            / (output_max - output_min)) + pressure_min;
}

void EUSCIB1_IRQHandler(void)
{
    uint_fast16_t status;

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B1_BASE);

    /* Receives bytes into the receive buffer. If we have received all bytes,
     * send a STOP condition */
    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {
        if (xferIndex1 == NUM_OF_REC_BYTES - 2)
        {
            MAP_I2C_disableInterrupt(EUSCI_B1_BASE,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0);
            MAP_I2C_enableInterrupt(EUSCI_B1_BASE, EUSCI_B_I2C_STOP_INTERRUPT);

            /*
             * Switch order so that stop is being set during reception of last
             * byte read byte so that next byte can be read.
             */
            MAP_I2C_masterReceiveMultiByteStop(EUSCI_B1_BASE);
            RXData1[xferIndex1++] = MAP_I2C_masterReceiveMultiByteNext(
            EUSCI_B1_BASE);
        }
        else
        {
            RXData1[xferIndex1++] = MAP_I2C_masterReceiveMultiByteNext(
            EUSCI_B1_BASE);
        }
    }
    else if (status & EUSCI_B_I2C_STOP_INTERRUPT)
    {
        MAP_Interrupt_disableSleepOnIsrExit();
        MAP_I2C_disableInterrupt(EUSCI_B1_BASE,
        EUSCI_B_I2C_STOP_INTERRUPT);
    }

    p_currentEx = 0;

    p_currentEx = p_currentEx | RXData1[1];
    p_currentEx = p_currentEx << 8;
    p_currentEx = p_currentEx | RXData1[2];

    p_currentEx = p_currentEx & 0b0011111111111111;

    float output_max = 14745;
    float output_min = 1638;
    float pressure_max = 2500; // in Pascals
    float pressure_min = -2500; // in Pascals

    // set previous data to p_past1 and p_past 2

    // calculate with bias offset
    p_currentEx_ =
            -2.69 + 3.262
                    + ((((float) (p_currentEx) - output_min)
                            * (pressure_max - pressure_min))
                            / (output_max - output_min)) + pressure_min;
}

/* EUSCI A0 UART ISR - receives data from PC host */

void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        MAP_UART_transmitData(EUSCI_A0_BASE,
        MAP_UART_receiveData(EUSCI_A0_BASE));
        if (MAP_UART_receiveData(EUSCI_A0_BASE) == 'R')
        {
            if (pwmConfig.dutyCycle != 0)
            {
                pwmConfig.dutyCycle = 0;
                /* start PWM signal */
                MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
            }
            else
            {
                pwmConfig.dutyCycle = 31;
                /* start PWM signal */
                MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
            }
        }

        init_const[init_counter] = MAP_UART_receiveData(EUSCI_A0_BASE);
        init_counter++;

        // Need a way to end line, use 'enter key'= '0xD' in ASCII
        if (MAP_UART_receiveData(EUSCI_A0_BASE) == '$')
        {
            init_O2 = atof(strtok(init_const, ",")); // retrieve O2
            init_CO2 = atof(strtok(NULL, ",")); // retrieve CO2
            init_H2O = atof(strtok(NULL, ",")); // retrieve CO2
            init_T_air_ex = atof(strtok(NULL, ",")); // retrieve CO2
            init_N2 = atof(strtok(NULL, ",")); // retrieve CO2
            init_LowVolume = atof(strtok(NULL, ",")); // retrieve Low FLow
            init_HighVolume = atof(strtok(NULL, ",")); // retrieve High FLow
            init_PercentDiff = atof(strtok(NULL, "$")); // retrieve fifth token
            init = false; // init is finished, start main loop
            init_counter = 0;
        }
    }

}

/* Timer32 Sampling ISR */

void T32_INT1_IRQHandler(void)
{
    if (init)
    {
        //wait for UART receive interrupt to send you startup info (%CO2, O2...)
        // when it does, init = false (set this in interrupt handler)
    }
    else
    {
        /* Making sure the last transaction has been completely sent out */
        MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);

        // set timing pin
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN3);
        j++;
        frequency = SAMPLE_RATE; // Hz

        // inhale conditions -- should be pretty close to "atmospheric", might change per hospital
        N_N2_in = init_N2; //0.79;//init_N2; //
        N_CO2_in = init_CO2; //0.0;//init_CO2; ////
        N_O2_in = init_O2; //0.21;//init_O2; // // user input
        N_H2O_in = init_H2O; //0.0;//init_H2O; // // ? unknown right now

        T_air_in = init_T_air_ex; //293;//init_T_air_ex;//293; //init_T_air_ex;// Air temperature during inhale [K]

        // exhale conditions -- probably need to be set for every patient
        N_N2_ex = init_N2; //0.79; //init_N2;
        N_CO2_ex = init_CO2; //0; //init_CO2; // // user input
        N_O2_ex = init_O2; //0.21; // init_O2; //// ? unknown right now
        N_H2O_ex = init_H2O; //0; //init_H2O; // // ? unknown right now

        T_air_ex = init_T_air_ex; //293; //init_T_air_ex; // // Air temperature during exhale [K] ***SETTABLE PARAMETER***

        //alarm conditions
        // init
        //init_HighVolume = 430; //remove if you want to set via gui
        //init_LowVolume = 200;
        // init_PercentDiff = .4;

        // filter data with fc = 10Hz, 2 pole Low pass filter
        p_currentIn_f = filter(A, B, p_currentIn_, p_pastIn, p_pastpastIn,
                               p_pastIn_f, p_pastpastIn_f);

        p_currentEx_f = filter(A, B, p_currentEx_, p_pastEx, p_pastpastEx,
                               p_pastEx_f, p_pastpastEx_f);

        if (((p_currentIn_f + offset) > p_currentEx_f)
                && ((p_currentIn_f) < (p_currentEx_f + offset)))
        {
            // do what you've been doing
            inhale_flag = inhale_flag_past;
            exhale_flag = exhale_flag_past;
        }
        else if ((p_currentIn_f + offset) > p_currentEx_f)
        {
            // switch to inhale
            inhale_flag = true;
            exhale_flag = false;
        }
        else if ((p_currentIn_f) < (p_currentEx_f + offset))
        {
            // switch to exhale
            exhale_flag = true;
            inhale_flag = false;
        }

        // calculate and export flowrate data
        if (inhale_flag) //currently an inhale
        {
            dir_c = 3.5391; //3.6266;
            volFlowrateInhale = getFlowRate(p_currentIn_f, N_N2_in, N_O2_in,
                                            N_CO2_in, N_H2O_in, T_air_in,
                                            dir_c);

            if (exhale_flag_past)
            {
                // Export volumeEx
                exportVolumeFlag = true;
                totalVolumeInhale = totalVolumeInhalePast;
                exportVolumeInhale = totalVolumeInhale;
                exportVolumeExhale = totalVolumeExhale;
                // check if we need to alarm doctors
                if (init_PercentDiff
                        <= abs(exportVolumeInhale - exportVolumeExhale)
                                / exportVolumeInhale)
                {
                    start_alarm = true;
                }
                if (init_LowVolume >= exportVolumeExhale)
                {
                    start_alarm = true;
                }
                if (init_HighVolume <= exportVolumeExhale)
                {
                    start_alarm = true;
                }

                if (start_alarm)
                {
                    pwmConfig.dutyCycle = 31;
                    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
                    start_alarm = false;

                    //Heartbeat
                    GPIO_toggleOutputOnPin(
                    GPIO_PORT_P3,
                                           GPIO_PIN6);
                }

                //set past values
                totalVolumeInhalePast = totalVolumeInhale;
                totalVolumeExhalePast = totalVolumeExhale;
                totalVolumeInhale = 0;
                totalVolumeExhale = 0;
            }

            //calc and sum inhale volume
            volumeInhale = getVolume(volFlowrateInhale, volFlowrateInhalePast,
                                     frequency);
            totalVolumeInhale = totalVolumeInhale + volumeInhale;

            exportInhaleFlowFlag = true;
            exportExhaleFlowFlag = false;
        }
        else if (exhale_flag) // currently an exhale
        {
            dir_c = 3.5416; //3.6198;
            volFlowrateExhale = getFlowRate(p_currentEx_f, N_N2_ex, N_O2_ex,
                                            N_CO2_ex, N_H2O_ex, T_air_ex,
                                            dir_c);

            if (inhale_flag_past)
            {
                // Export volumeIn
                exportVolumeFlag = true;
                totalVolumeExhale = totalVolumeExhalePast;
                exportVolumeInhale = totalVolumeInhale;
                exportVolumeExhale = totalVolumeExhale;

                // alarm conditions
                if (init_PercentDiff
                        <= abs(exportVolumeInhale - exportVolumeExhale)
                                / exportVolumeInhale)
                {
                    start_alarm = true;
                }
                if (init_LowVolume >= exportVolumeInhale)
                {
                    start_alarm = true;
                }
                if (init_HighVolume <= exportVolumeInhale)
                {
                    start_alarm = true;
                }

                if (start_alarm)
                {
                    pwmConfig.dutyCycle = 31;
                    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
                    start_alarm = false;
                    //Heartbeat
                    GPIO_toggleOutputOnPin(
                    GPIO_PORT_P3,
                                           GPIO_PIN6);
                }

                totalVolumeInhalePast = totalVolumeInhale;
                totalVolumeExhalePast = totalVolumeExhale;
                totalVolumeInhale = 0;
                totalVolumeExhale = 0;
            }

            //calc and sum inhale volume
            volumeExhale = getVolume(volFlowrateExhale, volFlowrateExhalePast,
                                     frequency);
            totalVolumeExhale = totalVolumeExhale + volumeExhale;

            exportExhaleFlowFlag = true;
            exportInhaleFlowFlag = false;
        }

        // set all current values to past values
        p_pastpastEx = p_pastEx;
        p_pastpastIn = p_pastIn;
        p_pastpastEx_f = p_pastEx_f;
        p_pastpastIn_f = p_pastIn_f;

        p_pastEx = p_currentEx_;
        p_pastIn = p_currentIn_;
        p_pastEx_f = p_currentEx_f;
        p_pastIn_f = p_currentIn_f;
        volFlowrateExhalePast = volFlowrateExhale;
        volFlowrateInhalePast = volFlowrateInhale;

        inhale_flag_past = inhale_flag;
        exhale_flag_past = exhale_flag;

        /* initiate the read */
        xferIndex0 = 0;
        xferIndex1 = 0;

        while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE)
                && MAP_I2C_masterIsStopSent(EUSCI_B1_BASE))
            ;

        MAP_I2C_masterReceiveStart(EUSCI_B1_BASE);
        MAP_I2C_enableInterrupt(EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);

        MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
        MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);

        // turn off timing pin
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3);
    }
}

/* Timer32 Logging ISR */
void T32_INT2_IRQHandler(void)
{
    // output to UART

    //clear interrupt flag
    MAP_Timer32_clearInterruptFlag(TIMER32_1_BASE);

    // turn on timing check pin
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);

    // message = "flowrate,volumeIn,VolumeEx\n"

    // count times in interrupt
    i++;

    // export correct flowrate
    if (exportInhaleFlowFlag)
    {
        // Converting float to string
        // Printing string to PC through UART - USB
        vspfunc("%f", (volFlowrateInhale));
        //vspfunc("%f", (N_N2_in));
        printfuart(EUSCI_A0_BASE, "%s,", s);

        if (volFlowrateInhale == 0)
        {
            count++;
            if (count > 20)
            {
                pwmConfig.dutyCycle = 31;
                MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
            }
        }
        else
        {
            count = 0;
        }

        // report everything
//        vspfunc("%f", p_currentIn_);
//        printfuart(EUSCI_A0_BASE, "%s,", s);
//        vspfunc("%f", -1 * p_currentEx_);
//        printfuart(EUSCI_A0_BASE, "%s,", s);
//        vspfunc("%f", p_currentIn_f);
//        printfuart(EUSCI_A0_BASE, "%s,", s);
//        vspfunc("%f", -1*p_currentEx_f);
//        printfuart(EUSCI_A0_BASE, "%s,", s);
//        vspfunc("%f", (volFlowrateInhale));
//        printfuart(EUSCI_A0_BASE, "%s,", s);
//        vspfunc("%f", (-1 * volFlowrateExhale));
//        printfuart(EUSCI_A0_BASE, "%s,", s);

        exportInhaleFlowFlag = false;
    }
    else if (exportExhaleFlowFlag)
    {
        // Converting float to string
        // Printing string to PC through UART - USB
        // 249246 clock cycles
        vspfunc("%f", (-1 * volFlowrateExhale));
        printfuart(EUSCI_A0_BASE, "%s,", s);

        if (volFlowrateInhale == 0)
        {
            count++;
            if (count > 20)
            {
                pwmConfig.dutyCycle = 31;
                MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
            }
        }
        else
        {
            count = 0;
        }

        // report everything
//        vspfunc("%f", p_currentIn_);
//        printfuart(EUSCI_A0_BASE, "%s,", s);
//        vspfunc("%f", -1 * p_currentEx_);
//        printfuart(EUSCI_A0_BASE, "%s,", s);
//        vspfunc("%f", p_currentIn_f);
//        printfuart(EUSCI_A0_BASE, "%s,", s);
//        vspfunc("%f", -1*p_currentEx_f);
//        printfuart(EUSCI_A0_BASE, "%s,", s);
//        vspfunc("%f", (volFlowrateInhale));
//        printfuart(EUSCI_A0_BASE, "%s,", s);
//        vspfunc("%f", (-1 * volFlowrateExhale));
//        printfuart(EUSCI_A0_BASE, "%s,", s);

        exportExhaleFlowFlag = false;
    }

    // export correct volume
    if (exportVolumeFlag)
    {
        vspfunc("%f", exportVolumeInhale);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", exportVolumeExhale);
        printfuart(EUSCI_A0_BASE, "%s\n", s);
        exportVolumeFlag = false;
    }
    else
    {
        vspfunc("%f", exportVolumeInhale);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", exportVolumeExhale);
        printfuart(EUSCI_A0_BASE, "%s\n", s);
    }

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);
}

// Function to convert float to string
int vspfunc(char *format, ...)
{
    va_list aptr;
    int ret;

    va_start(aptr, format);
    ret = vsprintf(s, format, aptr);
    va_end(aptr);

    return (ret);
}

void UARTSendString(char pui8Buffer[])
{
    //
    // Loop while there are more characters to send.
    //
    uint8_t i;
    for (i = 0; i < strlen(pui8Buffer); i++)
    {
        UART_transmitData(EUSCI_A0_BASE, pui8Buffer[i]);
    }
}

//void check_Alarm(void){ // add in various inputs
//    // if (flow_rate is too low or too high) {
//        /* send PWM */
//        //  pwmConfig.dutyCycle = PWM_Alarm;
//        //  MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig)
//    // }
//}
