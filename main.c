/**
*Author - SR Sivaram and Gautam Shaw
*Date   - 9/11/2018
*Objective: Capture the time-stamps using TIVA launchpad from TEXAS Instruments. 
*It will read the counter value  whenever  there is  an interupt to the TIVA board. 
*Clock rate: 80 MHzhence resolution of time stamps will be 12.5 ns. 
*/



#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "inc/tm4c123gh6pge.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/flash.h"
#include "inc/hw_ssi.h"
#include "Functions.h"



void main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); //  Clock Set at 80 MHz

    ConfigureUART();
    Configure_Timer();
    Configure_GPIO();
    UARTprintf("Initialized... \n");
    UARTprintf("TiVA TimeStamp Collection V1.0. \n");
    UARTprintf("Press 'S' to Start Data Collection \n");
    UARTprintf("Press 'T' to Set data runs Default is 100 \n");
    UARTprintf("Please don't press any Keys during data collection to get Accurate results \n");
    while(1) // While loop to keep processor busy while Idle
    {
        if(Flag == 1)
        {
            Timer_Enable();
        }
    }
}
