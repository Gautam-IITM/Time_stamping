/*
 * ISR.h
 *
 *  Created on: Jul 6, 2017
 *      Author: Sivaram
 */

#ifndef ISR_H_
#define ISR_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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

// Initializations

uint16_t i = 0, it = 0, j = 0 ;
uint32_t Temp = 0;
uint16_t Nos = 100;
uint32_t Data1[7500];
char Data[20];
uint8_t Flag = 0;

void Timer_ISR (void)                                                  //Variable Timer ISR to get data from Counter
{
    TimerIntClear(WTIMER0_BASE,TIMER_TIMA_TIMEOUT);                   //Clearing Interrupt Flag
    if (j < Nos)
    {
        j++;
        WTIMER0_TAV_R=0;                                              //Resetting the Counter
        WTIMER0_TAPS_R=0;
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_2);                // Clearing Interrupt Status
        IntPendClear(INT_GPIOD);                                      // Clearing Interrupts
        Print_Data();
    }
    else
    {
        Timer_Disable();
        WTIMER0_TAV_R=0;                                              //Resetting the Counter
        WTIMER0_TAPS_R=0;
        GPIO_ISR_Disable();
        Print_Data();
    }
}

void UART_ISR (void)                                                 //UART ISR To Get Data From User
{
   uint32_t ui32Status;                                              //Clearing Interrupt flag
   char Tr ;
   ui32Status = UARTIntStatus(UART0_BASE, true);
   UARTIntClear(UART0_BASE, ui32Status);
   Tr = UARTCharGet(UART0_BASE);                                     //Receiving data from buffer
   if (Tr == 'T')
   {
       UARTprintf("Please Enter Number of runs \n");                 // Receiving Data from User
       do
       {
           i = UARTgets(Data,20);
           Nos = (uint32_t)atoi(Data);
           if (Nos == 0)
           {
               UARTprintf("Invalid Entry Please Enter a Proper Integer...  \n");
           }
       }
       while(Nos == 0);
       UARTprintf("Data Received is %d \n",Nos);
   }
   else if (Tr == 'T')
   {
       Flag = 1;
   }
   IntPendClear(INT_UART0);
   ui32Status = UARTIntStatus(UART0_BASE, true);
   UARTIntClear(UART0_BASE, ui32Status);
   SysCtlDelay(10);                                          // Delay for Registers to be set properly
}

void Time_Stamp_ISR (void)                                     // ISR to take timestamps
{
    Temp = TimerValueGet(WTIMER0_BASE,TIMER_A);
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_2);             // Clearing Interrupt Status
    IntPendClear(INT_GPIOD);                                   // Clearing Interrupts
    if (it <7500)
    {
        Data1[it] = Temp;
        it++;
    }
    else
    {
        SysCtlDelay(10);
    }
}


#endif /* ISR_H_ */
