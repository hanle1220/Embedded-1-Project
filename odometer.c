#include <stdint.h>
#include "odometer.h"
#include "clock.h"
#include "tm4c123gh6pm.h"

void setEncoderPosition(uint8_t id, int32_t position)
{
    if(id == 1)
        WTIMER0_TAV_R = position;
    else
        WTIMER1_TAV_R = position;
}

int32_t getEncoderPosition(uint8_t id)
{
    int32_t position;
    if(id == 1)
        position = WTIMER1_TAV_R;
    else
        position = WTIMER0_TAV_R;

    return position;
}
void selectEncoderIncMode(uint8_t id)
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;  //Turn off timer
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;

    if(id == 0)
        WTIMER0_TAMR_R |= TIMER_TAMR_TACDIR;
    else
        WTIMER1_TAMR_R |= TIMER_TAMR_TACDIR;

    WTIMER1_CTL_R |= TIMER_CTL_TAEN; //Turn on timer
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;
}

void selectEncoderDecMode(uint8_t id)
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;  //Turn off timer
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;

    if(id == 0)
        WTIMER0_TAMR_R &= ~TIMER_TAMR_TACDIR;
    else
        WTIMER1_TAMR_R &= ~TIMER_TAMR_TACDIR;

    WTIMER1_CTL_R |= TIMER_CTL_TAEN; //Turn on timer
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;
}
