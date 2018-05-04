/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/02/13 5:09p $
 * @brief    Use the timer pin P3.2 to demonstrate timer trigger counting
 *           mode function. And displays the measured input frequency to
 *           UART console.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"


void TMR0_IRQHandler(void)
{
    // printf takes long time and affect the freq. calculation, we only print out once a while
    static int cnt = 0;

    cnt++;
    if(cnt == 10)
    {
        printf("Input frequency is %dHz\n", 22118400 / TIMER0->CAP);
        cnt = 0;
    }
    // Clear Timer 0 capture interrupt flag
    TIMER0->EINTSTS = TIMER_EINTSTS_CAPIF_Msk;

}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->REGLCTL != 1)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT);

    /* Enable external 12MHz XTAL (UART), and HIRC */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    while((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk |CLK_STATUS_XTLSTB_Msk)) !=
            (CLK_STATUS_HIRCSTB_Msk |CLK_STATUS_XTLSTB_Msk));

    /* Enable UART and Timer 0 clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_TMR0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P1_MFP = SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD;

    /* Set P3 multi function pin for Timer 0 capture pin */
    SYS->P3_MFP = SYS_MFP_P32_TM0_EXT;

    /* Lock protected registers */
    SYS->REGLCTL = 0;
}

void UART_Init(void)
{
    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART0->LINE = UART_LINE_WLS_Msk;
    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART0->BAUD = UART_BAUD_BAUDM0_Msk | UART_BAUD_BAUDM1_Msk | (0xBE);
}

int main(void)
{
    int volatile i;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Init();

    printf("\nThis sample code demonstrate timer trigger counting mode.\n");
    printf("Please connect input source with Timer 0 capture pin T0EX (P3.2), press any key to continue\n");
    getchar();

    /*
      Timer 0 clock source is 22118400 Hz
    */
    TIMER0->CTL = TIMER_CTL_CNTEN_Msk | TIMER_PERIODIC_MODE;

    // Set compare value as large as possible
    TIMER0->CMP = 0xFFFFFF;

    // Configure Timer 0 trigger counting mode, capture TDR value on falling edge, enable capture interrupt
    TIMER0->EXTCTL = TIMER_CAPTURE_TRIGGER_COUNTING_MODE |
                     TIMER_CAPTURE_FALLING_EDGE |
                     TIMER_EXTCTL_CAPIEN_Msk |
                     TIMER_EXTCTL_CAPEN_Msk |
                     TIMER_CAPTURE_FALLING_EDGE;


    NVIC_EnableIRQ(TMR0_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


