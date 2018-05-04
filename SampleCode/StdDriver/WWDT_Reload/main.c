/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/01 4:27p $
 * @brief    Demonstrate the WWDT counter reload function
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "Mini58Series.h"
#include "stdio.h"

void WDT_IRQHandler(void)
{

    // Reload WWDT counter and clear WWDT interrupt flag
    WWDT_RELOAD_COUNTER();
    WWDT_CLEAR_INT_FLAG();
    printf("WWDT counter reload\n");

}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT);

    /* Enable external 12MHz XTAL (UART), HIRC, and  LIRC (fro WDT) */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);


    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_XTAL, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P1_MFP = SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD;
    /* Lock protected registers */
    SYS_LockReg();
}


int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\nThis sample code demonstrate WWDT reload function\n");

    // WWDT register is locked, so it is necessary to unlock protect register before configure WDT
    SYS_UnlockReg();
    // WWDT timeout every 768 * 64 WWDT clock, compare interrupt trigger at 768 * 32 WWDT clock, enable WWDT counter compare interrupt
    WWDT_Open(WWDT_PRESCALER_768, 0x20, TRUE);

    // Enable WWDT timeout interrupt, which share the same interrupt number with WDT
    NVIC_EnableIRQ(WDT_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


