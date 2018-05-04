/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/06/02 9:10p $
 * @brief    Demonstrate PWM Precise Center Aligned feature
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"

#define PLLCTL_SETTING  CLK_PLLCTL_100MHz_HXT
#define PLL_CLOCK       50000000

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

    /* Enable external 12MHz XTAL, HIRC */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    while((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_XTLSTB_Msk)) !=
            (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_XTLSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLKDIV_Msk ) | CLK_CLKDIV_HCLK(2);
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_PWMCH01CKEN_Msk | CLK_APBCLK_PWMCH23CKEN_Msk | CLK_APBCLK_PWMCH45CKEN_Msk;

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

    /* Set P0 multi-function pins for PWM Channel 5  */
    SYS->P0_MFP |= SYS_MFP_P04_PWM0_CH5;
    /* Set P2 multi-function pins for PWM Channel 0~4  */
    SYS->P2_MFP = SYS_MFP_P22_PWM0_CH0 | SYS_MFP_P23_PWM0_CH1 |
                  SYS_MFP_P24_PWM0_CH2 | SYS_MFP_P25_PWM0_CH3 | SYS_MFP_P26_PWM0_CH4;


    /* Lock protected registers */
    SYS->REGLCTL = 0;
}

void UART_Init(void)
{
    // 115200 bps
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk | (((__XTAL + (115200/2)) / 115200)-2);
    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART0->LINE = 0x3 | (0x0 << UART_LINE_PBE_Pos) | (0x0 << UART_LINE_NSB_Pos) ;
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

    printf("\nThis sample code will output PWM channel 0 to with different duty\n");
    printf(", and enable/disable Precise Center Align function.\n");
    printf("Polling 1 period interrupt flag to get PWM channel 0 output.\n");;

    // PWM-Timer 0 enable and Auto-reload
    PWM->CTL = PWM_CTL_CNTEN0_Msk | PWM_CTL_CNTMODE0_Msk;

    // Set channel 0 prescaler to 2. Actual value fill into register needs to minus 1.
    PWM->CLKPSC = 0x1;

    // Set channel 0 clock divider to 1
    PWM->CLKDIV = (PWM_CLK_DIV_1 << PWM_CLKDIV_CLKDIV0_Pos);

    // Set the PWM aligned type
    PWM->CTL = (PWM->CTL & ~PWM_CTL_CNTTYPE_Msk) | (PWM_CENTER_ALIGNED);

    // Enable PWM channel 0 output
    PWM->POEN = PWM_POEN_POEN0_Msk;

    // Start
    PWM->CTL |= PWM_CTL_CNTEN0_Msk;

    /*
     Precise Center Align and Center Align PWM channel 0 waveform of this sample shown below:

     |<- CNR-(2*(CMR+1))  ->|  CNR-(2*(CMR+1) = 401 -(2*(100+1)) CLKs
                   |<-     CNR - (2*(CMR+1))   ->|  CNR-(2*(CMR+1) = 402 -(2*(99+1)) CLKs
                                         |<- 2 *(CNR-CMR) clk  ->|  2 * (CNR - CMR) = 2 * (401-100) CLKs
                                                           |<-2 *(CNR - CMR) clk ->|  2 * (CNR - CMR) = 2 * (402-99) CLKs
           ________          ____________          ________       ____________       ________          ____________          ________       ____________
      ____| 7.96us |_8.08us_|   24.08us  |_8.08us_| 8.08us |_8us_|   24.24us  |_8us_| 7.96us |_8.08us_|   24.08us  |_8.08us_| 8.08us |_8us_|   24.24us  |_8us_

    */


    while(1)
    {
        // Enable PWM Precise Center Aligned Type
        PWM->PCACTL = PWM_PCACTL_PCAEN_Msk;

        // PWM Channel 0 Output : high width = 7.96u, low width = 8.08u
        PWM->CMPDAT0 = 100;
        PWM->PERIOD0= 401;

        // Polling, Wait 1 period interrupt flags
        while((PWM->INTSTS & (PWM_INTSTS_ZIF0_Msk << 0)) == 0);
        PWM->INTSTS = (PWM_INTSTS_ZIF0_Msk << 0);

        // Disable PWM Precise Center Aligned Type
        PWM->PCACTL &= ~(PWM_PCACTL_PCAEN_Msk);

        // PWM Channel 0 Output : high width = 24.08u, low width = 8.08u
        PWM->CMPDAT0 = 100;
        PWM->PERIOD0 = 401;

        // Polling, Wait 1 period interrupt flags
        while((PWM->INTSTS & (PWM_INTSTS_ZIF0_Msk << 0)) == 0);
        PWM->INTSTS = (PWM_INTSTS_ZIF0_Msk << 0);

        // Enable PWM Precise Center Aligned Type
        PWM->PCACTL = PWM_PCACTL_PCAEN_Msk;

        // PWM Channel 0 Output : high width = 8.08u, low width = 8u
        PWM->CMPDAT0 = 99;
        PWM->PERIOD0 = 402;

        // Polling, Wait 1 period interrupt flags
        while((PWM->INTSTS & (PWM_INTSTS_ZIF0_Msk << 0)) == 0);
        PWM->INTSTS = (PWM_INTSTS_ZIF0_Msk << 0);

        // Disable PWM Precise Center Aligned Type
        PWM->PCACTL &= ~(PWM_PCACTL_PCAEN_Msk);

        // PWM Channel 0 Output : high width = 24.24u, low width = 8u
        PWM->CMPDAT0 = 99;
        PWM->PERIOD0 = 402;

        // Polling, Wait 1 period interrupt flags
        while((PWM->INTSTS & (PWM_INTSTS_ZIF0_Msk << 0)) == 0);
        PWM->INTSTS = (PWM_INTSTS_ZIF0_Msk << 0);
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


