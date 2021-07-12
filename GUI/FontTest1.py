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
#define SLAVE_ADDRESS       0x78
#define NUM_OF_REC_BYTES    5

/* Sample Rate*/
#define SAMPLE_RATE         1000

/* Logging Rate */
#define LOGGING_RATE        10

/* PWM for Alarm */
#define PWM_ALARM           31

/* Calibration time in seconds*/
#define CALIBRATION_TIME    10
/* Offset for noisy data */
float offset = 6;

/* String array to output data */
static char s[8];

/* I2C Variables */
static volatile uint8_t RXData0[NUM_OF_REC_BYTES]; // data read in from I2C sensor
static volatile uint8_t RXData1[NUM_OF_REC_BYTES];
static volatile uint32_t xferIndex0;    // index of data being read in
static volatile uint32_t xferIndex1;
static volatile bool stopSent; // flag to end receiving data

//filter constants, fc = 10Hz, 2 pole, fsample = 100 Hz
//float A[2] = {-1.142980502539901,0.412801598096189};
//float B[3] = {0.067455273889072,0.134910547778144,0.067455273889072};

//filter constants, fc = 10Hz, 2 pole, fsample = 500 Hz
//float A[2] = {-1.822694925196308,0.837181651256023};
//float B[3] = {0.003621681514929,0.007243363029857,0.003621681514929};

/* filter constants, fc = 10Hz, 2 pole, fsample = 1000 Hz */
float A[2] = { -1.911197067426073, 0.914975834801433 };
float B[3] = { 9.446918438401619e-04, 0.001889383687680, 9.446918438401619e-04 };

/* flags */
bool alarm = false;
bool init = false;
bool calib = false;
bool ready = false;
bool start_alarm = false;
static bool exportExhaleFlowFlag = false;
static bool exportInhaleFlowFlag = false;
static bool exportVolumeFlag = false;
bool inhale_flag = false;
bool inhale_flag_past = false;
bool exhale_flag = false;
bool exhale_flag_past = false;
float alarm_cause = -1;
float calib_counter = 0;
float calib_limit = CALIBRATION_TIME * SAMPLE_RATE;

/* Initalize variables */
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
volatile uint16_t adcResult;
float analog_press = 0;
float no_flow_counter = 0;
float no_flow_limit = 10000;
float trans1_offset = 0;
float trans2_offset = 0;
float analog_offset = 0;
float trans1_sum = 0;
float trans2_sum = 0;
float analog_sum = 0;
float p_calibIn = 0;
float p_calibEx = 0;

/* Initalize variables for UART Transmission*/
char init_const[50];
uint32_t init_counter = 0;

/* Initialize Calculation Conditions */
float init_O2 = 0;
float init_CO2 = 0;
float init_H2O = 0;
float init_N2 = 0;
float init_T_air_ex = 298;
float init_LowVolume = 0;
float init_HighVolume = 0;
float init_PercentDiff = 0;

float volFlowrateInhale = 0;
float volFlowrateExhale = 0;
float volumeInhale = 0;
float volumeExhale = 0;

/* Constants for the Calculations */
float frequency = SAMPLE_RATE;
float N_N2_in = 0.80;
float N_CO2_in = 0;
float N_O2_in = 0.20;
float N_H2O_in = 0;
float N_N2_ex = 0.80;
float N_CO2_ex = 0.05;
float N_O2_ex = 0.16;
float N_H2O_ex = 0;
float T_air_in = 293;
float T_air_ex = 293;
float dir_c = 0;

/* Debugging counters */
double i = 0; // Logging Counter
double j = 0; // Sampling Counter

////////////////////////////Module Inits ///////////////////////////////
/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig = {
EUSCI_B_I2C_CLOCKSOURCE_SMCLK,                  // SMCLK Clock Source
        3000000,                                // SMCLK = 3MHz
        EUSCI_B_I2C_SET_DATA_RATE_400KBPS,      // Desired I2C Clock of 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
        };

/* Timer_A Continuous Mode Configuration Parameter */
const Timer_A_UpModeConfig upModeConfig = {
TIMER_A_CLOCKSOURCE_ACLK,            // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // ACLK/1 = 32Khz
        16384,
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0
        TIMER_A_DO_CLEAR                     // Clear Counter
        };

/* Timer_A Compare Configuration Parameter */
const Timer_A_CompareModeConfig compareConfig = {
TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR1
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_SET_RESET,               // Toggle output but
        16384                                       // 16000 Period
        };

/* UART Configuration */
const eUSCI_UART_ConfigV1 uartConfig = {
EUSCI_A_UART_CLOCKSOURCE_SMCLK,                             // SMCLK Source
        1, //19,                                            // Clock Prescalar   --------
        10, //8,                                            // FirstModReg       -------- >>> these 3 set baudrate (9600)
        0, //85,                                            // SecondModReg      --------
        EUSCI_A_UART_NO_PARITY,                             // No Parity
        EUSCI_A_UART_LSB_FIRST,                             // Send LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                          // 1 Stop Bit
        EUSCI_A_UART_MODE,                                  // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,      // Oversampling
        EUSCI_A_UART_8_BIT_LEN };

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfig = {
TIMER_A_CLOCKSOURCE_ACLK,                       //clockSource
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          //clockSourceDivider
        62,                                     //timerPeriod
        TIMER_A_CAPTURECOMPARE_REGISTER_1,      //compareRegister
        TIMER_A_OUTPUTMODE_RESET_SET,           //compareOutputMode
        31                                       //dutyCycle , 31 for 50%
        };

/*
 * Main Function runs on flash.  Initializes all the modules used for this code.  Sets up clock speeds.
 * Configure Pins and Interrupts that will be in use.  Finally enter an infinite loop looking for
 * interrupts.
 *
 * last edited: 5/18/2021
 */
int main(void)
{
    /* Disabling the Watchdog  */
    MAP_WDT_A_holdTimer();

    /* Enable floating Point Operations */
    MAP_FPU_enableModule();

    ////////////////////////////Init the Clocks being Used ///////////////////

    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_16);
    /* Setting MCLK to REFO at 128Khz for LF mode Setting ACLK to 128Khz */
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Check Clock Speeds */
    uint32_t aclk = MAP_CS_getACLK();   // 128000 Hz
    uint32_t mclk = MAP_CS_getMCLK();   // 48000000 Hz
    uint32_t smclk = MAP_CS_getSMCLK(); // 3000000 Hz

    ///////////////////////////oscilloscope toggling for debugging////////////////

    /* Configuring P3.2 as output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN2);
    /* Configuring P3.3 as output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN3);
    /* Configuring P3.6 as output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);

    ///////////////////////////Timer32///////////////////////////////

    /* Sampling Timer Interrupt Initialization */
    MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
    TIMER32_PERIODIC_MODE);
    MAP_Interrupt_enableInterrupt(INT_T32_INT1);
    int count = (CS_getMCLK() / SAMPLE_RATE);  // 48000
    MAP_Timer32_setCount(TIMER32_0_BASE, count); // trigger interrupt every 48000 counts
    MAP_Timer32_enableInterrupt(TIMER32_0_BASE);
    MAP_Timer32_startTimer(TIMER32_0_BASE, false);

    /* Logging Timer Interrupt Initialization */
    MAP_Timer32_initModule(TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
    TIMER32_PERIODIC_MODE);
    MAP_Interrupt_enableInterrupt(INT_T32_INT2);
    int rate1 = (CS_getMCLK() / LOGGING_RATE); // 480000
    MAP_Timer32_setCount(TIMER32_1_BASE, rate1); // trigger interrupt every 480000 counts
    MAP_Timer32_enableInterrupt(TIMER32_1_BASE);
    MAP_Timer32_startTimer(TIMER32_1_BASE, false);

    ///////////////////////////PWM for buzzer///////////////////

    /* Configuring GPIO2.4 as peripheral output for PWM */
    /*
     MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
     GPIO_PORT_P2,
     GPIO_PIN4,
     GPIO_PRIMARY_MODULE_FUNCTION);
     */

    /* start PWM signal */
    //MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN4);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
    alarm = false;

    ///////////////////////////UART Configuration/////////////////////////////
    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3,
            GPIO_PRIMARY_MODULE_FUNCTION);

    /* Initialize and enable UART */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
    MAP_UART_enableModule(EUSCI_A0_BASE);

    /* Allow UART to trigger interrupt */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);

    ///////////////////////////ADC Configuration/////////////////////////////
    /* Initializing ADC (MCLK/1/1) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
                         0);

    /* Configuring ADC Memory */
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A6,
                                        false);

    /* Setting up GPIO pins as analog inputs (and references) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P4,
            GPIO_PIN7,
            GPIO_TERTIARY_MODULE_FUNCTION);

    /* Configuring Timer_A in continuous mode and sourced from ACLK */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upModeConfig);

    /* Configuring Timer_A0 in CCR1 to trigger at 16000 (0.5s) */
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig);

    /* Configuring the sample trigger to be sourced from Timer_A0  and setting it
     * to automatic iteration after it is triggered*/
    MAP_ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false);

    /* Enabling the interrupt when a conversion on channel 6 is complete and
     * enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT0);
    MAP_ADC14_enableConversion();

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);

    /* Starting the Timer */
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    ///////////////////////////I2C/////////////////////////////

    /* Select Port 1 for I2C - Set Pin 6, 7 to input Primary Module Function,
     *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL respectively).
     */

    /* PORT 1 -- BUS 0 */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1,
            GPIO_PIN6 + GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION);

    /* PORT 6 -- BUS 1 */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P6,
            GPIO_PIN5 + GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);

    stopSent = false;

    /* set aside bytes of memory to store incoming values */
    memset((void*) RXData0, 0x00, NUM_OF_REC_BYTES);
    memset((void*) RXData1, 0x00, NUM_OF_REC_BYTES);

    /* Initializing I2C Master to SMCLK at 100khz with no autostop */
    MAP_I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    MAP_I2C_initMaster(EUSCI_B1_BASE, &i2cConfig);

    /* Specify slave address */
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_ADDRESS);
    MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, SLAVE_ADDRESS);

    /* Enable I2C Module to start operations */
    MAP_I2C_enableModule(EUSCI_B0_BASE);
    MAP_Interrupt_enableInterrupt(INT_EUSCIB0);
    MAP_I2C_enableModule(EUSCI_B1_BASE);
    MAP_Interrupt_enableInterrupt(INT_EUSCIB1);
    MAP_Interrupt_enableMaster();

    ///////////////////// Interrupt priorities ///////////////////////////
    /* Configuring interrupt priorities  */
    //Interrupt_setPriority(INT_T32_INT2, 2 << 5);    // Logging Interrupt
    //Interrupt_setPriority(INT_T32_INT1, 2 << 5);    // Sampling Timer
    //Interrupt_setPriority(INT_EUSCIA0, 2 << 5);     // UART
    //Interrupt_setPriority(INT_EUSCIB0, 1 << 5);     // I2C 1
    //Interrupt_setPriority(INT_EUSCIB1, 1 << 5);     // I2C 2
    //Interrupt_setPriority(INT_ADC14, 1 << 5);       // ADC
    /* Check Priorities Used for Debugging*/
    int a = MAP_Interrupt_getPriority(INT_T32_INT1);
    int b = MAP_Interrupt_getPriority(INT_T32_INT2);
    int c = MAP_Interrupt_getPriority(INT_EUSCIA0);
    int d = MAP_Interrupt_getPriority(INT_EUSCIB0);
    int e = MAP_Interrupt_getPriority(INT_EUSCIB1);
    int f = MAP_Interrupt_getPriority(INT_ADC14);

    /* Enter an infinite loop waiting for an interrupt to trigger */
    while (1)
    {
        MAP_PCM_gotoLPM0InterruptSafe();
    }
}

/*******************************************************************************
 * Interrupt handler for repeated stop and transmit data for I2C transducer on
 * bus 0.  Receives all bytes from the buffer then translates to a float.  This
 * float is then adjusted to account for bias in the transducer.
 *
 * last edited: 5/12/2021
 *******************************************************************************/
void EUSCIB0_IRQHandler(void)
{

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2); //debugging pin
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2); //debugging pin
    uint_fast16_t status; // Interrupt status message

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B0_BASE); // Read status

    /* Receives bytes into the receive buffer. If we have received all bytes,
     * send a STOP condition */
    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {
        if (xferIndex0 == NUM_OF_REC_BYTES - 2) // If the last set of data being read
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
            EUSCI_B0_BASE); // Put data read from sensor into Data array
        }
        else // If this is not the last data set being received
        {
            RXData0[xferIndex0++] = MAP_I2C_masterReceiveMultiByteNext(
            EUSCI_B0_BASE); // Put data read from sensor into data array
        }
    }
    /* If not data in the receive buffer exit */
    else if (status & EUSCI_B_I2C_STOP_INTERRUPT)
    {
        MAP_Interrupt_disableSleepOnIsrExit();
        MAP_I2C_disableInterrupt(EUSCI_B0_BASE,
        EUSCI_B_I2C_STOP_INTERRUPT);
    }
    /* Run calculations on data to get it as a binary */
    p_currentIn = 0;

    /* Interpret data as an int */
    p_currentIn = p_currentIn | RXData0[1];
    p_currentIn = p_currentIn << 8;
    p_currentIn = p_currentIn | RXData0[2];
    p_currentIn = p_currentIn & 0b0011111111111111;

    /* Constants to adjust data */
    float output_max = 14746;
    float output_min = 1638;
    float pressure_max = 2490.89; //2500; // in Pascals
    float pressure_min = -2490.89; //-2500; // in Pascals
    /* calculate pressure read in with bias offset */
    p_currentIn_ = ((((float) (p_currentIn) - output_min)
            * (pressure_max - pressure_min)) / (output_max - output_min))
            + pressure_min;

}

/*******************************************************************************
 * Interrupt handler for repeated stop and transmit data for I2C transducer on
 * bus 1.  Receives all bytes from the buffer then translates to a float.  This
 * float is then adjusted to account for bias in the transducer.
 *
 * last edited: 5/12/2021
 *******************************************************************************/
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

    float output_max = 14746;
    float output_min = 1638;
    float pressure_max = 2490.89; // in Pascals
    float pressure_min = -2490.89; // in Pascals

    /* calculate with bias offset */
    p_currentEx_ = ((((float) (p_currentEx) - output_min)
            * (pressure_max - pressure_min)) / (output_max - output_min))
            + pressure_min;

}

/*******************************************************************************
 * Interrupt handler for Analog to Digital Conversions (ADC).  This interrupt
 * is triggered by clock A (Timer_A0_Base).  When triggered the result stored in
 * ADC Memory 0 is interpreted as a float in PSI.
 *
 * last edited: 5/12/2021
 *******************************************************************************/
void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = MAP_ADC14_getEnabledInterruptStatus(); // Read the status message that called interrupt
    MAP_ADC14_clearInterruptFlag(status); // Clear the status message so that interrupt can be called again

    if (status & ADC_INT0) //If interrupt is from ADC0
    {
        adcResult = MAP_ADC14_getResult(ADC_MEM0); // Read Result from ADC0
        analog_press = adc_to_pressure(adcResult) + analog_offset; // Calculate the pressure of that ADC conversion
    }
}

/*******************************************************************************
 * Interrupt handler for handling messages received from GUI over UART.  There
 * are 3 main cases that the GUI has.
 * 1) Alarm reset signal is sent.  In this case flip the state of the alarm
 * 2) Freeze state signal is sent.  In this case stop receiving & printing new
 * data and wait
 * 3) Init data is received.  Save all constants then begin measuring new data
 *
 * last edited: 5/18/2021
 *******************************************************************************/
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE); // Read UART interrupt status
    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status); // Clear status to be able to read another interrupt

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) // If the status message shows data in input buffer
    {
        MAP_UART_transmitData(EUSCI_A0_BASE,
        MAP_UART_receiveData(EUSCI_A0_BASE)); // Receive the data that is being read in from uart buffer

        if (MAP_UART_receiveData(EUSCI_A0_BASE) == 'R') // If the message received is the character 'R'
        {
            //if (pwmConfig.dutyCycle != 0) // If the alarm is not off
            if (alarm)
            {
                MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN4);
                MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);
                alarm = false;
                /*
                 pwmConfig.dutyCycle = 0; // Set the duty cycle to 0
                 MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig); // Send 0 duty cycle to alarm turning it off
                 no_flow_counter = 0; // reset no flow counter
                 alarm_cause = -1; // reset alarm cause
                 */

            }
            else // If the alarm is off
            {
                MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
                        GPIO_PORT_P2,
                        GPIO_PIN4,
                        GPIO_PRIMARY_MODULE_FUNCTION);
                pwmConfig.dutyCycle = PWM_ALARM; // Set the duty cycle to 31
                MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig); // Send the duty cycle to alarm turning it on
                alarm = true;
            }
        }
        else if (MAP_UART_receiveData(EUSCI_A0_BASE) == 'X')
        { //If the message received is the character 'X'
            init = false; //stop the system
            alarm_cause = -1; //stop the alarm from triggering
        }
        else if (MAP_UART_receiveData(EUSCI_A0_BASE) == 'S')
        {  //If the message received is the character 'S'
            init = true; //start the system
            alarm_cause = -1;
        }
        else // If the message is anything other than just 'R', 'X', or 'S'
        {
            init_const[init_counter] = MAP_UART_receiveData(EUSCI_A0_BASE); // Store data read in into the array
            init_counter++; // increment array counter

            // If the stop character '$' is read then interpret data
            if (MAP_UART_receiveData(EUSCI_A0_BASE) == '$')
            {
                init_O2 = atof(strtok(init_const, ",")); // retrieve O2
                init_CO2 = atof(strtok(NULL, ",")); // retrieve CO2
                init_H2O = atof(strtok(NULL, ",")); // retrieve H2O
                init_T_air_ex = atof(strtok(NULL, ",")); // retrieve Temp of Air
                init_N2 = atof(strtok(NULL, ",")); // retrieve N2
                init_LowVolume = atof(strtok(NULL, ",")); // retrieve Low FLow
                init_HighVolume = atof(strtok(NULL, ",")); // retrieve High FLow
                init_PercentDiff = atof(strtok(NULL, "$")); // retrieve Percent difference
                calib = true; // begin calibration
                trans1_offset = 0;
                trans2_offset = 0;
                analog_offset = 0;
                trans1_sum = 0;
                trans2_sum = 0;
                analog_sum = 0;
                calib_counter = 0;
                init_counter = 0; // count how many times this has been called for debugging

                // inhale conditions
                N_N2_in = 0.79;
                N_CO2_in = 0.0;
                N_O2_in = 0.21;
                N_H2O_in = 0.0;
                T_air_in = 293; // Air temperature during inhale [K]

                // exhale conditions
                N_N2_ex = 0.79;
                N_CO2_ex = 0;
                N_O2_ex = 0.21;
                N_H2O_ex = 0;
                T_air_ex = 293; // Air temperature during exhale [K]
            }
        }
    }

}

/*******************************************************************************
 * Interrupt handler for sampling data. Triggered by Timer 32 at 1000 Hz. If
 * GUI has not initialized constants do nothing. If initialized then calculate
 * filtered pressure data, volume, and state of breathing (i.e. Inhale or Exhale)
 * If any of the values are outside limits trigger the alarm.
 *
 * last edited: 6/23/2021
 *******************************************************************************/
void T32_INT1_IRQHandler(void)
{
    if (calib)
    {
        /* Making sure the last transaction has been completely sent out */
        MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);

        if (calib_counter == calib_limit)
        {
            alarm_cause = 101;

            trans1_offset = 0;//(trans1_sum / calib_limit) * -1;
            trans2_offset = 0;//(trans2_sum / calib_limit) * -1;
            analog_offset = (analog_sum / calib_limit) * -1;
            calib = false;
        }
        else
        {
            // filter data with fc = 10Hz, 2 pole Low pass filter
            p_currentIn_f = filter(A, B, p_currentIn_, p_pastIn, p_pastpastIn,
                                   p_pastIn_f, p_pastpastIn_f);

            p_currentEx_f = filter(A, B, p_currentEx_, p_pastEx, p_pastpastEx,
                                   p_pastEx_f, p_pastpastEx_f);

            trans1_sum += p_currentIn_f;
            trans2_sum += p_currentEx_f;
            analog_sum += analog_press;

            // Set all current values to past values
            p_pastpastEx = p_pastEx;
            p_pastpastIn = p_pastIn;
            p_pastpastEx_f = p_pastEx_f;
            p_pastpastIn_f = p_pastIn_f;
            p_pastEx = p_currentEx_;
            p_pastIn = p_currentIn_;
            p_pastEx_f = p_currentEx_f;
            p_pastIn_f = p_currentIn_f;

            //Re-initialize the reading index
            xferIndex0 = 0;
            xferIndex1 = 0;

            //Wait for stop signals to be sent to both transducers
            while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE)
                    && MAP_I2C_masterIsStopSent(EUSCI_B1_BASE))
                ;

            //send start receive from transducer 1
            MAP_I2C_masterReceiveStart(EUSCI_B1_BASE);
            MAP_I2C_enableInterrupt(EUSCI_B1_BASE,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0);

            //send start receive from transducer 2
            MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
            MAP_I2C_enableInterrupt(EUSCI_B0_BASE,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0);

            alarm_cause = 100;
            calib_counter++;
        }

    }
    else if (init)
    {
        /* Making sure the last transaction has been completely sent out */
        MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);

        // filter data with fc = 10Hz, 2 pole Low pass filter
        p_currentIn_f = filter(A, B, p_currentIn_, p_pastIn, p_pastpastIn,
                               p_pastIn_f, p_pastpastIn_f);
        //p_currentIn_f += trans1_offset;
        p_calibIn = p_currentIn_f + trans1_offset;

        p_currentEx_f = filter(A, B, p_currentEx_, p_pastEx, p_pastpastEx,
                               p_pastEx_f, p_pastpastEx_f);
        //p_currentEx_f += trans2_offset;
        p_calibEx = p_currentEx_f + trans2_offset;

        /* Decide what state the breathing cycle is in */
        if (((p_calibIn + offset) > p_calibEx)
                && ((p_calibIn) < (p_calibEx + offset)))
        {
            // do what you've been doing
            inhale_flag = inhale_flag_past;
            exhale_flag = exhale_flag_past;
        }
        else if ((p_calibIn + offset) > p_calibEx)
        {
            // switch to inhale
            inhale_flag = true;
            exhale_flag = false;
        }
        else if ((p_calibIn) < (p_calibEx + offset))
        {
            // switch to exhale
            exhale_flag = true;
            inhale_flag = false;
        }
        /* Check if there is no flow through the ventilator */
        if ((p_currentIn_ > -10 && p_currentIn_ < 10)
                && (p_currentEx_ > -10 && p_currentEx_ < 10))
        {
            no_flow_counter++; //increment the no flow counter

            if (no_flow_counter == no_flow_limit) // if the no flow counter is at the limit
            {
                start_alarm = true; // begin the alarm
                alarm_cause = 999; // set alarm cause to no flow limit reached
                no_flow_counter = 0; // reset the no flow counter
            }
        }
        else // If flow is detected
        {
            no_flow_counter = 0; //reset no flow counter
        }

        /*if (start_alarm) // if alarm called to start
         {
         MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
         GPIO_PORT_P2,
         GPIO_PIN4,
         GPIO_PRIMARY_MODULE_FUNCTION);
         pwmConfig.dutyCycle = PWM_ALARM; // set the duty cycle
         MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig); // send the duty cycle to alarm turning it on
         start_alarm = false; // reset the alarm flag
         alarm = true;
         }*/

        /* calculate and export flowrate data */
        if (inhale_flag) //currently an inhale
        {
            /* Calculate the flowrate */
            dir_c = 3.5391; //3.6266;
            volFlowrateInhale = getFlowRate(p_calibIn, N_N2_in, N_O2_in,
                                            N_CO2_in, N_H2O_in, T_air_in, dir_c,
                                            1);
            if (exhale_flag_past) //If past state was an exhale
            {
                // Export volume Exhale
                exportVolumeFlag = true; // Initialize the flag
                /* Set data to relevant outputs */
                totalVolumeInhale = totalVolumeInhalePast;
                exportVolumeInhale = totalVolumeInhale;
                exportVolumeExhale = totalVolumeExhale;

                /* check if alarm should be triggered */
                if (init_PercentDiff
                        <= abs(exportVolumeInhale - exportVolumeExhale)
                                / exportVolumeInhale) // Percent different check
                {
                    start_alarm = true; // Alarm flag set
                    alarm_cause = 1; // set alarm cause to percent difference to large
                }
                if (init_LowVolume >= exportVolumeExhale) // low flow check
                {
                    start_alarm = true; // Alarm set flag
                    alarm_cause = 2; // set alarm cause to low flow
                }
                if (init_HighVolume <= exportVolumeExhale) // high flow check
                {
                    start_alarm = true; // Alarm set flag
                    alarm_cause = 3; // set alarm cause to high flow
                }
                /*if (start_alarm) // if Alarm flag
                 {
                 MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
                 GPIO_PORT_P2,
                 GPIO_PIN4,
                 GPIO_PRIMARY_MODULE_FUNCTION);
                 pwmConfig.dutyCycle = PWM_ALARM; // Set duty cycle
                 MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig); // Send duty cycle to alarm turning it on
                 start_alarm = false; //reset alarm flag
                 alarm = true;
                 }*/

                /* set past values */
                totalVolumeInhalePast = totalVolumeInhale;
                totalVolumeExhalePast = totalVolumeExhale;
                totalVolumeInhale = 0;
                totalVolumeExhale = 0;
            }

            // calculate and sum inhale volume
            volumeInhale = getVolume(volFlowrateInhale, volFlowrateInhalePast,
                                     frequency);
            totalVolumeInhale = totalVolumeInhale + volumeInhale;

            /* change flags */
            exportInhaleFlowFlag = true;
            exportExhaleFlowFlag = false;
        }
        else if (exhale_flag) // currently an exhale
        {
            /* Calculate Flow Rate */
            dir_c = 3.5416; //3.6198;
            volFlowrateExhale = getFlowRate(p_calibEx, N_N2_ex, N_O2_ex,
                                            N_CO2_ex, N_H2O_ex, T_air_ex, dir_c,
                                            0);

            if (inhale_flag_past)
            {
                /* Export volumeIn */
                exportVolumeFlag = true;
                totalVolumeExhale = totalVolumeExhalePast;
                exportVolumeInhale = totalVolumeInhale;
                exportVolumeExhale = totalVolumeExhale;

                /* alarm conditions Exact same as above*/
                if (init_PercentDiff
                        <= abs(exportVolumeInhale - exportVolumeExhale)
                                / exportVolumeInhale)
                {
                    start_alarm = true;
                    alarm_cause = 1;
                }
                if (init_LowVolume >= exportVolumeInhale)
                {
                    start_alarm = true;
                    alarm_cause = 2;
                }

                if (init_HighVolume <= exportVolumeInhale)
                {
                    start_alarm = true;
                    alarm_cause = 3;
                }

                /*if (start_alarm)
                 {
                 MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
                 GPIO_PORT_P2,
                 GPIO_PIN4,
                 GPIO_PRIMARY_MODULE_FUNCTION);
                 pwmConfig.dutyCycle = 31;
                 MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
                 start_alarm = false;
                 alarm = true;
                 }*/

                /* Set past values */
                totalVolumeInhalePast = totalVolumeInhale;
                totalVolumeExhalePast = totalVolumeExhale;
                totalVolumeInhale = 0;
                totalVolumeExhale = 0;
            }

            /* calc and sum inhale volume */
            volumeExhale = getVolume(volFlowrateExhale, volFlowrateExhalePast,
                                     frequency);
            totalVolumeExhale = totalVolumeExhale + volumeExhale;

            /* reset flags */
            exportExhaleFlowFlag = true;
            exportInhaleFlowFlag = false;
        }

        /* set all current values to past values */
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

        /* Reset Flags */
        inhale_flag_past = inhale_flag;
        exhale_flag_past = exhale_flag;

        /* Re-initialize the reading index */
        xferIndex0 = 0;
        xferIndex1 = 0;

        /* Wait for stop signals to be sent to both transducers */
        while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE)
                && MAP_I2C_masterIsStopSent(EUSCI_B1_BASE))
            ;

        /* send start receive from transducer 1 */
        MAP_I2C_masterReceiveStart(EUSCI_B1_BASE);
        MAP_I2C_enableInterrupt(EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);

        /* send start receive from transducer 2 */
        MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
        MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);

        // turn off timing pin used for debugging
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3);

    }
}

/*******************************************************************************
 * Interrupt handler for Logging data. Triggered by Timer 32 at 100 Hz.
 * Depending on the state decided in the sampling data export the data over
 * UART in a certain order.  The general format is as follows:
 * 1) Volume 1
 * 2) Volume 2
 * 3) Filtered Pressure 1
 * 4) Filtered Pressure 2
 * 5) Alarm Cause
 * 6) Static Pressure
 *
 * last edited: 6/23/2021
 *******************************************************************************/
void T32_INT2_IRQHandler(void)
{
    MAP_Timer32_clearInterruptFlag(TIMER32_1_BASE); //clear interrupt flag

    i++;

    /* export correct flowrate */
    if (exportInhaleFlowFlag)
    {
        /*
         * Converting float to string & Printing string to PC through UART - USB
         */

        vspfunc("%f", alarm_cause);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", (volFlowrateInhale));
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", (volFlowrateExhale));
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", p_currentIn_f);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", -1 * p_currentEx_f);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", analog_press);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", exportVolumeInhale);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", exportVolumeExhale);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", trans1_offset);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", trans2_offset);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", p_calibIn);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", p_calibEx);
        printfuart(EUSCI_A0_BASE, "%s\n", s);

        exportInhaleFlowFlag = false; // reset the flag
    }
    else if (exportExhaleFlowFlag)
    {
        /*
         * Converting float to string & Printing string to PC through UART - USB
         */
        vspfunc("%f", alarm_cause);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", (-1 * volFlowrateExhale));
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", (volFlowrateInhale));
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", p_currentIn_f);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", -1 * p_currentEx_f);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", analog_press);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", exportVolumeInhale);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", exportVolumeExhale);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", trans1_offset);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", trans2_offset);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", p_calibIn);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", p_calibEx);
        printfuart(EUSCI_A0_BASE, "%s\n", s);

        exportExhaleFlowFlag = false; // reset flag
    }
    else
    {
        vspfunc("%f", alarm_cause);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s,", s);
        vspfunc("%f", 0);
        printfuart(EUSCI_A0_BASE, "%s\n", s);
    }

}

/*******************************************************************************
 * Helper function for logging data that changes float data types to character
 * arrays (Strings).
 *
 * last edited: 5/12/2021
 *******************************************************************************/
int vspfunc(char *format, ...)
{
    va_list aptr;
    int ret;

    va_start(aptr, format);
    ret = vsprintf(s, format, aptr);
    va_end(aptr);

    return (ret);
}
