
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/05/28 10:37a $ 
 * @brief    Use WDT to wake up system from Power-down mode periodically.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/  
#include <stdio.h>
#include "Mini58Series.h"


void WDT_IRQHandler(void)
{

    // Clear WDT interrupt flag
    WDT_CLEAR_TIMEOUT_INT_FLAG();
  
    // Check WDT wake up flag
    if(WDT_GET_TIMEOUT_WAKEUP_FLAG()) {
        printf("Wake up by WDT\n");
        // Clear WDT wake up flag
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }    

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
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_WDTCKEN_Msk;

    /* Select UART clock source from external crystal*/
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UARTSEL_Msk) | CLK_CLKSEL1_UARTSEL_XTAL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
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
    
    printf("\nThis sample code demonstrate using WDT to wake system up from power down mode\n");
   
    // WDT register is locked, so it is necessary to unlock protect register before configure WDT
    SYS_UnlockReg();    
    // WDT timeout every 2^14 WDT clock, disable system reset, enable wake up system
    WDT_Open(WDT_TIMEOUT_2POW14, 0, FALSE, TRUE);

    // Enable WDT timeout interrupt
    WDT_EnableInt();
    NVIC_EnableIRQ(WDT_IRQn);
    
    while(1) {
        // Wait 'til UART FIFO empty to get a cleaner console out
        while(!UART_IS_TX_EMPTY(UART0));
        CLK_PowerDown();        
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


