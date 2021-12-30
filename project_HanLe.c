#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "clock.h"
#include "uart0.h"
#include "clock.h"
#include "wait.h"
#include "odometer.h"
#include "tm4c123gh6pm.h"

#define MAX_CHARS 80
#define MAX_FIELDS 5
#define DEFAULT_VALUE 0xFFFF
#define MAX_TIMER_VALUE     0xFFFFFFFF
#define SLEEP        (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 0*4)))
#define TRIG        (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))
#define ECHO        (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4)))

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

// PortB masks
#define LEFT_MASK 32
#define LEFT_MASK_1 16
#define SLEEP_MASK 1

// PortE masks
#define RIGHT_MASK_1 16
#define RIGHT_MASK 32
#define ECHO_MASK 1
#define TRIG_MASK 2

// PortC masks
// Pin 6
#define COUNT_UP_MASK 64
//Pin 4
#define COUNT_DOWN_MASK 16

// PortF masks
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8
#define PUSH_BUTTON_MASK 16
//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;


typedef struct _instruction
{
    uint8_t command;
    uint8_t subcommand;
    uint16_t argument;
} instruction;

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

void setCounterUpMode()
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = 0;                               //
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
}

void setCounterDownMode()
{
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_CAP;           // configure for edge count mode, count down
    WTIMER0_CTL_R = 0;                               //
    WTIMER0_TAV_R = 0;                               // zero counter for first period
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
}


// Initialize Hardware
void initOdometer()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    //SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 ; //| SYSCTL_RCGCTIMER_R0;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

   // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | BLUE_LED_MASK;  // bits 1 and 2 are outputs, other pins are inputs
    GPIO_PORTF_DIR_R &= ~PUSH_BUTTON_MASK;               // bit 4 is an input
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)

    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
                                                         // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;                // enable internal pull-up for push button

    // Configure FREQ_IN for frequency counter
    GPIO_PORTC_AFSEL_R |= COUNT_UP_MASK;              // select alternative functions for FREQ_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to FREQ_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= COUNT_UP_MASK;                // enable bit 6 for digital input

    GPIO_PORTC_PUR_R |= COUNT_UP_MASK | COUNT_DOWN_MASK;
    setCounterUpMode();

    GPIO_PORTC_AFSEL_R |= COUNT_DOWN_MASK;              // select alternative functions for FREQ_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;           // map alt fns to FREQ_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0;
    GPIO_PORTC_DEN_R |= COUNT_DOWN_MASK;                // enable bit 6 for digital input

    setCounterDownMode();
}

void initializePwm(void)
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure three backlight LEDs
    GPIO_PORTB_DIR_R |= LEFT_MASK | LEFT_MASK_1 | SLEEP_MASK;                       // make bit5 an output
    GPIO_PORTB_DR2R_R |= LEFT_MASK | LEFT_MASK_1 | SLEEP_MASK;                      // set drive strength to 2mA
    GPIO_PORTB_DEN_R |= LEFT_MASK | LEFT_MASK_1 | SLEEP_MASK;                       // enable digital
    GPIO_PORTB_AFSEL_R |= LEFT_MASK | LEFT_MASK_1;                     // select auxilary function
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB5_M | GPIO_PCTL_PB4_M);                      // enable PWM
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB5_M0PWM3 | GPIO_PCTL_PB4_M0PWM2;

    GPIO_PORTE_DIR_R |= RIGHT_MASK | RIGHT_MASK_1;  // make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R |= RIGHT_MASK | RIGHT_MASK_1; // set drive strength to 2mA
    GPIO_PORTE_DEN_R |= RIGHT_MASK | RIGHT_MASK_1;  // enable digital
    GPIO_PORTE_AFSEL_R |= RIGHT_MASK | RIGHT_MASK_1;// select auxilary function
    GPIO_PORTE_PCTL_R &= ~(GPIO_PCTL_PE4_M | GPIO_PCTL_PE5_M);    // enable PWM
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5;

    // Configure PWM module 0 to drive RGB backlight
    // LEFT   on M0PWM2 (PB4), M0PWM1a
    // LEFT   on M0PWM3 (PB5), M0PWM1b
    // RIGHT  on M0PWM4 (PE4), M0PWM2a
    // RIGHT  on M0PWM5 (PE5), M0PWM2b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_1_CTL_R = 0;                                // turn-off PWM0 generator 1 (drives outs 2 and 3)
    PWM0_2_CTL_R = 0;                                // turn-off PWM0 generator 2 (drives outs 4 and 5)

    PWM0_1_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;

    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 3 on PWM0, gen 1b, cmpb
    PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
                                                     // output 4 on PWM0, gen 2a, cmpa
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 5 on PWM0, gen 2b, cmpb
    PWM0_1_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_2_LOAD_R = 1024;
    PWM0_INVERT_R = PWM_INVERT_PWM2INV | PWM_INVERT_PWM3INV | PWM_INVERT_PWM4INV | PWM_INVERT_PWM5INV;
                                                     // invert outputs so duty cycle increases with increasing compare values
    PWM0_1_CMPA_R = 0;
    PWM0_1_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM0_2_CMPB_R = 0;                               // green off
    PWM0_2_CMPA_R = 0;                               // blue off

    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN;
                                                     // enable outputs
}
void initTimer()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 ;
    _delay_cycles(3);

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;                               // configure as 32-bit counter (A only)
    TIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR; // configure for edge time mode, count up

    TIMER1_TAV_R = 0;                               // zero counter for first period
}

void setPwmDutyCycle(uint8_t id, uint16_t pwmA, uint16_t pwmB)
{
    if(id == 0)
    {
        PWM0_1_CMPA_R = pwmA;
        PWM0_1_CMPB_R = pwmB;
    }
    if(id == 1)
    {
        PWM0_2_CMPA_R = pwmA;
        PWM0_2_CMPB_R = pwmB;
    }
}

void stop()
{
    setPwmDutyCycle(1, 0, 0);
    setPwmDutyCycle(0, 0, 0);
}

uint32_t waitDistance(uint32_t distance)
{
    uint32_t dist_cm;

        TIMER1_TAV_R = 0;
        TRIG = 1;
        waitMicrosecond(30);
        TRIG = 0;
        while(!ECHO) {}                          //wait ECHO to go high
        TIMER1_CTL_R |= TIMER_CTL_TAEN;         //start timer
        while(ECHO) {}                         //wait ECHO to go low
        dist_cm = (TIMER1_TAV_R * 0.025) / 58;
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;        //stop timer

        return dist_cm;
}

void forward(uint16_t dist_cm)
{
    bool b = 1;
    int32_t x, y;
    uint16_t dist = dist_cm *46 / (6.476) / (3.14);
    selectEncoderIncMode(0);
    setEncoderPosition(0,0);
    setEncoderPosition(1,0);
    setPwmDutyCycle(1, 920, 0);
    setPwmDutyCycle(0, 0, 920);
    while(b)
    {
        x = getEncoderPosition(0);
        y = getEncoderPosition(1);

        if(x == dist || y == dist ) {stop(); b = 0;}
        if(waitDistance(20) == 20) {stop(); b = 0;}
    }
}

void reverse(uint16_t dist_cm)
{
    bool b = 1;
    int32_t x, y;
    uint16_t dist = dist_cm *46 / (6.476) / (3.14);
    selectEncoderIncMode(0);
    setEncoderPosition(0,0);
    setEncoderPosition(1,0);
    setPwmDutyCycle(1, 0, 920);
    setPwmDutyCycle(0, 920, 0);
    x = getEncoderPosition(0);
    y = getEncoderPosition(1);
    while(b)
    {
        x = getEncoderPosition(0);
        y = getEncoderPosition(1);
        if(x == dist || y == dist) {stop(); b = 0;}
    }
}

void cw(uint16_t degree)
{
    bool b = 1;
    int32_t x, y;
    uint16_t deg_count = degree*(16)*(46)/(180)/(6.476);
    selectEncoderIncMode(0);
    setEncoderPosition(0,0);
    setEncoderPosition(1,0);
    setPwmDutyCycle(1, 0, 0);
    setPwmDutyCycle(0, 0, 920);
    x = getEncoderPosition(0);
    y = getEncoderPosition(1);
    while(b)
    {
        x = getEncoderPosition(0);
        y = getEncoderPosition(1);
        if(x == deg_count || y == deg_count) {stop(); b = 0;}
    }
}

void ccw(uint16_t degree)
{
    bool b = 1;
    int32_t x, y;
    uint16_t deg_count = degree*(16)*(46)/(180)/(6.476);
    selectEncoderIncMode(0);
    setEncoderPosition(0,0);
    setEncoderPosition(1,0);
    setPwmDutyCycle(1, 920, 0);
    setPwmDutyCycle(0, 0, 0);
    x = getEncoderPosition(0);
    y = getEncoderPosition(1);
    while(b)
    {
        x = getEncoderPosition(0);
        y = getEncoderPosition(1);
        if(x == deg_count || y == deg_count) {stop(); b = 0;}
    }
}


void getsUart0(USER_DATA* data)
{
    char c;
    uint8_t count = 0;
    while(true)
    {
        if(count == MAX_CHARS)
        {
            data->buffer[count] ='\0';
            break;
        }
        c = getcUart0();
        if(c == 8 || c == 127)
        {
            count--;
        }
        else if(c == 13)
        {
            data->buffer[count] ='\0';
            break;
        }
        else if(c >= 32)
        {
            data->buffer[count] = c;
            count++;
        }
    }
    return;
}

void parseFields(USER_DATA* data)
{
    uint8_t pos = 0;
    data->fieldCount = 0;
    while(data->buffer[pos] != '\0')
    {
        if(data->fieldCount > MAX_FIELDS)
        {
            break;
        }
        if(isalpha(data->buffer[pos]) != 0)
        {
             data->fieldType[data->fieldCount] = 'a';
             data->fieldPosition[data->fieldCount] = pos;
             if(data->buffer[pos-1] == '\0')
                 data->fieldCount++;
        }
        else if(isdigit(data->buffer[pos]) != 0)
        {
             data->fieldType[data->fieldCount] = 'n';
             data->fieldPosition[data->fieldCount] = pos;
             if(data->buffer[pos-1] == '\0')
                 data->fieldCount++;
        }
        else
        {
            data->buffer[pos] = '\0';
        }
        pos++;
    }
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber <= data->fieldCount)
    {
        if(data->fieldType[fieldNumber] == 'a')
        {
            return &data->buffer[data->fieldPosition[fieldNumber]];
        }
        else
            return 0;
    }
    else
        return 0;
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    char num[50] = " ";
    if(data->fieldType[fieldNumber] == 'n')
    {
        strcpy(num, &data->buffer[data->fieldPosition[fieldNumber]]);
        return atoi(num);
    }
    else
        return 0;
}

bool strcompare(const char strCommand[], char string[])
{
    uint8_t i = 0;
    while(string[i] != '\0')
    {
        if(strCommand[i] != string[i])
            return false;
        i++;
    }
    return true;
}

void encoder(USER_DATA* data, instruction instr[], uint8_t n)
{
    char* string =  getFieldString(data, 0);
    uint16_t arg;

    arg = getFieldInteger(data, 1);
        if(arg == '\0')
            instr[n].argument = DEFAULT_VALUE;
        else
            instr[n].argument = arg;

        if(strcompare("forward", string) == 1)       instr[n].command = 0;
        else if(strcompare("reverse", string) == 1)  instr[n].command = 1;
        else if(strcompare("cw", string) == 1)       instr[n].command = 2;
        else if(strcompare("ccw", string) == 1)      instr[n].command = 3;
        else if(strcompare("wait", string) == 1)
        {
            instr[n].command = 4;
            if(strcompare("pb", getFieldString(data, 1)) == 1)
                instr[n].subcommand = 0;
            else
            {
                instr[n].subcommand = 1;
                instr[n].argument = getFieldInteger(data, 2);
            }
        }
        else if(strcompare("pause", string) == 1)    instr[n].command = 5;
        else if(strcompare("stop", string) == 1)     instr[n].command = 6;
}

void printList(instruction instr[], uint8_t n)
{
    uint8_t i;
    uint16_t arg;
    char str[50];
    for(i = 0; i < n; i++)
    {
        arg = instr[i].argument;
        if(instr[i].command == 0)       sprintf(str, "\n%d forward %d\n", i+1, arg);
        else if(instr[i].command == 1)  sprintf(str, "\n%d reverse %d\n", i+1, arg);
        else if(instr[i].command == 2)  sprintf(str, "\n%d cw %d\n", i+1, arg);
        else if(instr[i].command == 3)  sprintf(str, "\n%d ccw %d\n", i+1, arg);
        else if(instr[i].command == 4)
        {
            if(instr[i].subcommand == 0) sprintf(str, "\n%d wait pb \n", i+1);
            else sprintf(str, "\n%d wait distance %d\n", i+1, arg);
        }
        else if(instr[i].command == 5)  sprintf(str, "\n%d pause %d\n", i+1, arg);
        else if(instr[i].command == 6)  sprintf(str, "\n%d stop \n", i+1);
        putsUart0(str);
     }
    putcUart0('\n');
}

void deleteList(instruction instr[], uint8_t count, uint8_t LINE_NUMBER)
{
    uint8_t i=LINE_NUMBER;
    while(i<count)
    {
        instr[i-1].command = instr[i].command;
        instr[i-1].argument = instr[i].argument;
        i++;
    }
    instr[count-1].command = NULL;
    instr[count-1].argument = NULL;
}

void insertList(USER_DATA* data, instruction instr[], uint8_t count, uint8_t LINE_NUMBER)
{
    uint8_t i=LINE_NUMBER;
    while(count >= i)
    {
        instr[count].command = instr[count-1].command;
        instr[count].argument = instr[count-1].argument;
        count--;
    }
    char* string =  getFieldString(data, 2);
    uint16_t arg;
    if(strcompare("wait", string) == 1)
    {
        instr[i-1].command = 4;
        if(strcompare("pb", getFieldString(data, 3)) == 1)
            instr[i-1].subcommand = 0;
        else
        {
            instr[i-1].subcommand = 1;
            instr[i-1].argument = getFieldInteger(data, 4);
        }
    }
    else
    {
    arg = getFieldInteger(data, 3);
    if(arg == NULL)
        instr[i-1].argument = DEFAULT_VALUE;
    else
        instr[i-1].argument = arg;
    if(strcompare("forward", string) == 1)       instr[i-1].command = 0;
    else if(strcompare("reverse", string) == 1)  instr[i-1].command = 1;
    else if(strcompare("cw", string) == 1)       instr[i-1].command = 2;
    else if(strcompare("ccw", string) == 1)      instr[i-1].command = 3;
    else if(strcompare("pause", string) == 1)    instr[i-1].command = 5;
    else if(strcompare("stop", string) == 1)     instr[i-1].command = 6;
    }
}

void pause(uint32_t TIME_MS)
{
    waitMicrosecond(TIME_MS);
}

void waitPbPress()
{
    while(GPIO_PORTF_DATA_R & PUSH_BUTTON_MASK);
}

void run(instruction instr[], uint8_t count)
{
    SLEEP = 1;
    uint8_t i;
    for(i=0; i<count; i++)
    {
        if(instr[i].command == 0)       forward(instr[i].argument);
        else if(instr[i].command == 1)  reverse(instr[i].argument);
        else if(instr[i].command == 2)  cw(instr[i].argument);
        else if(instr[i].command == 3)  ccw(instr[i].argument);
        else if(instr[i].command == 4)
        {
            if(instr[i].subcommand == 0)
                waitPbPress();
            if(instr[i].subcommand == 1)
                waitDistance(instr[i].argument);
        }
        else if(instr[i].command == 5)  pause(instr[i].argument);
        else if(instr[i].command == 6)  stop();
        waitMicrosecond(1000000);
    }

    //SLEEP = 0;
}

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    GPIO_PORTF_DIR_R &= ~PUSH_BUTTON_MASK;               // bit 4 is an input
    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK;                                                           // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;                // enable internal pull-up for push button

    //Configure TRIG as an digital output
    GPIO_PORTE_DIR_R |= TRIG_MASK;                       // make bit 2 an output
    GPIO_PORTE_DR2R_R |= TRIG_MASK | ECHO_MASK;                      // set drive strength to 2mA
    GPIO_PORTE_DIR_R &= ~ECHO_MASK;
    GPIO_PORTE_DEN_R |= TRIG_MASK | ECHO_MASK;
}

int main(void)
{
    initHw();
    initUart0();
    initOdometer();
    initializePwm();
    initTimer();

    setUart0BaudRate(115200, 40e6);

    USER_DATA data = {0};
    instruction instr[10] = {0};
    uint8_t count = 0;

    while(true)
    {

        //bool valid = 0;
        // Display greeting
        putcUart0('>');

       // Get the string from the user
        getsUart0(&data);
        // Echo back to the user of the TTY interface for testing

        #ifdef DEBUG
        putsUart0(data.buffer);
        putcUart0('\n');
        #endif
       // Parse fields
        parseFields(&data);
        // Echo back the parsed field data (type and fields)
        #ifdef DEBUG
        uint8_t i;
        for (i = 0; i < data.fieldCount; i++)
        {
            putcUart0(data.fieldType[i]);
            putcUart0('\t');
            putsUart0(&data.buffer[fieldPosition[i]]);
            putcUart0('\n');
        }
        #endif

        if(strcompare("list", getFieldString(&data, 0)) == 1)
        {
            printList(instr, count);
        }
        else if(strcompare("delete", getFieldString(&data, 0)) == 1)
        {
            deleteList(instr, count, getFieldInteger(&data, 1));
            count--;
            printList(instr, count);
        }
        else if(strcompare("insert", getFieldString(&data, 0)) == 1)
        {
            insertList(&data, instr, count, getFieldInteger(&data, 1));
            count++;
            printList(instr, count);
        }
        else if(strcompare("run", getFieldString(&data, 0)) == 1)
        {
            run(instr, count);
        }
        else
            encoder(&data, instr, count++);

    }

}
