/*
 * Functions.h
 *
 *  Created on: Jul 6, 2017
 *      Author: Sivaram
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "driverlib/systick.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/flash.h"
#include "inc/hw_ssi.h"
#include "driverlib/pwm.h"
#include "driverlib/watchdog.h"
#include "ISR.h"

void Configure_Timer(void);
void GPIO_ISR_Disable (void);
void GPIO_ISR_Enable (void);
void Timer_Disable (void);
void Timer_Enable (void);


void Configure_Timer(void)                                                            // Variable timer Configuring
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);                                     // Enable Timer Module
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_PERIODIC);                                  // Timer to be periodic
    TimerLoadSet(WTIMER0_BASE, TIMER_A, (SysCtlClockGet()*16));                        // Timer Value set (Period of the timer) to reset the timer and prevent it from overflowing
    IntPrioritySet(INT_WTIMER0A, 0x00);                                                // Timer Gets the Highest Priority
    IntEnable(INT_WTIMER0A);                                                           // Enabling Interrupt for the Timer
    TimerEnable(WTIMER0_BASE, TIMER_A);                                                // Timer A Enable
    TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);                                  // Setting interrupt condition as timer timeout
}



void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);                                    // Setting Baud Rate and UART Clock Speed
    UARTFlowControlSet(UART0_BASE,UART_FLOWCONTROL_NONE);                    // UART Flow Control Set to None
    IntEnable(INT_UART0);                                                    // UART Interrupt Enabled
    //IntPrioritySet(INT_UART0, 0x00);                                         // Interrupt Priority set as Highest
    UARTFIFODisable(UART0_BASE);
    UARTIntEnable(UART0_BASE, UART_INT_RX );                                 // Setting the Interrupt Condition - When a byte is recieved
    UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);            // FIFO Set as length 1

}

void GPIO_ISR_Disable (void)
{
    GPIOIntDisable(GPIO_PORTD_BASE, GPIO_INT_PIN_2);           // Interrupt Disable
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_2);             // Clearing Interrupt Status
    IntPendClear(INT_GPIOD);                                   // Clearing Interrupts
}

void GPIO_ISR_Enable (void)
{
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_2);           // Interrupt Enable
}

void Timer_Disable (void)
{
    TimerDisable(WTIMER0_BASE, TIMER_A);                                     // Timer A Disable
}

void Timer_Enable (void)
{
    TimerEnable(WTIMER0_BASE, TIMER_A);                                     // Timer A Disable
}

void Print_Data (void)
{
    for (i = 0; i < 7500; i++)
    {
        UARTprintf("%d\n",Data1[i]);
        Data1[i] = 0;
    }
    UARTprintf("-------------------------------------------------------\n");
}

void Configure_GPIO(void)                                                    // Configuring GPIO Pins
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);                             // Enabling GPIO Port D
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                             // Port F Enabled
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2);                       // PF4 Set As Input (Push Button)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,                                   // Red,Blue and Green LED (PF1,2,3) set as Output
                          GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTD_BASE,GPIO_PIN_2 ,GPIO_RISING_EDGE);            // Setting Interrupt Condition For PF4 as Falling Edge
    GPIOIntRegister(GPIO_PORTD_BASE, Time_Stamp_ISR);                        // Registering Interrupt With the Corresponding Function
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_2);                          // Interrupt Enable
}

#endif /* FUNCTIONS_H_ */
