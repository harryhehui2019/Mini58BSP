/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/02/13 5:08p $ 
 * @brief    Use polling mode to check WDT time-out state and reset WDT 
 *           after time out occurs.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/  
#include <stdio.h>
#include "Mini58Series.h"


void SYS_Init(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->REGLCTL != 1) {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT);
		
    /* Enable HIRC, and LIRC (fro WDT) */
    CLK->PWRCTL = CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk;

    /* Waiting for clock ready */
    while((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk)) != 
            (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk));


    /* Enable UART and WDT clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_WDTCKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


/*---------------------------------------------------------------------------------------------------------*/
/* Init I/O Multi-function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P1_MFP = SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD;

    /* Lock protected registers */
    SYS->REGLCTL = 0;
}

void UART_Init(void)
{
    // Set UART to 8 bit character length, 1 stop bit, and no parity 
    UART0->LINE = UART_LINE_WLS_Msk;
    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk | (0xBE);
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
    UART_Init();
    
    printf("\nThis sample code demonstrate using WDT in polling mode\n");
   
    // WDT register is locked, so it is necessary to unlock protect register before configure WDT
    while(SYS->REGLCTL != 1) {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    // WDT timeout every 2^14 WDT clock, enable system reset
    WDT->CTL = WDT_TIMEOUT_2POW14 | WDT_CTL_WDTEN_Msk | WDT_CTL_RSTEN_Msk | WDT_CTL_INTEN_Msk;
    
    while(1) {
        // WDT timeout flag set
        if(WDT->CTL & WDT_CTL_IF_Msk) {
            // Reset WDT and clear time out flag
            WDT->CTL |= WDT_CTL_RSTCNT_Msk;
            printf("Reset WDT counter\n");
        }
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


