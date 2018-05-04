/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/05/28 10:37a $
 * @brief    Demonstrate PWM Precise Center Align feature
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"

#define PLL_CLOCK           50000000

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable external 12MHz XTAL (UART), HIRC */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV_HCLK(1));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

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

    printf("\nThis sample code will output PWM channel 0 to with different duty\n");
    printf(", and enable/disable Precise Center Align function.\n");
    printf("Polling 1 period interrupt flag to get PWM channel 0 output.\n");

    // PWM-Timer 0 enable and Auto-reload
    PWM->CTL = PWM_CTL_CNTEN0_Msk | PWM_CTL_CNTMODE0_Msk;
    PWM_SET_PRESCALER(PWM, 0, 1);
    PWM_SET_DIVIDER(PWM, 0, PWM_CLK_DIV_1);

    // Set the PWM aligned type
    PWM_SET_ALIGNED_TYPE(PWM, 0, PWM_CENTER_ALIGNED);

    // Enable PWM channel 0 output
    PWM_EnableOutput(PWM, BIT0);

    // Start
    PWM_Start(PWM, BIT0);

    /*
    Precise Center Align and Center Align PWM channel 0 waveform of this sample shown below:

    |<- CNR-(2*(CMR+1))  ->|  CNR-(2*(CMR+1) = 401 -(2*(100+1)) CLKs
               |<-  CNR -( 2 *( CMR + 1))   ->|  CNR-(2*(CMR+1) = 402 -(2*(99+1)) CLKs
                                     |<-  2 *(CNR-CMR) clk ->|  2 * (CNR - CMR) = 2 * (401-100) CLKs
                                                       |<- 2 * (CNR-CMR) clk  ->|   2 * (CNR - CMR) = 2 * (402-99) CLKs
       ________          ____________          ________       ____________       ________          ____________          ________       ____________
    ____| 7.96us |_8.08us_|   24.08us  |_8.08us_| 8.08us |_8us_|   24.24us  |_8us_| 7.96us |_8.08us_|   24.08us  |_8.08us_| 8.08us |_8us_|   24.24us  |_8us_

    */

    while(1)
    {
        // Enable PWM Precise Center Aligned Type
        PWM->PCACTL = PWM_PCACTL_PCAEN_Msk;

        // PWM Channel 0 Output : duty = 7.96u, low = 8.08u
        PWM_SET_CMR(PWM, 0, 100);
        PWM_SET_CNR(PWM, 0, 401);

        // Polling, Wait 1 period interrupt flags
        while(PWM_GetPeriodIntFlag(PWM, 0) == 0);
        PWM_ClearPeriodIntFlag(PWM, 0);

        // Disable PWM Precise Center Aligned Type
        PWM->PCACTL &= ~(PWM_PCACTL_PCAEN_Msk);

        // PWM Channel 0 Output : duty = 24.08u, low = 8.08u
        PWM_SET_CMR(PWM, 0, 100);
        PWM_SET_CNR(PWM, 0, 401);

        // Polling, Wait 1 period interrupt flags
        while(PWM_GetPeriodIntFlag(PWM, 0) == 0);
        PWM_ClearPeriodIntFlag(PWM, 0);

        // Enable PWM Precise Center Aligned Type
        PWM->PCACTL = PWM_PCACTL_PCAEN_Msk;

        // PWM Channel 0 Output : duty = 8.08u, low = 8u
        PWM_SET_CMR(PWM, 0, 99);
        PWM_SET_CNR(PWM, 0, 402);

        // Polling, Wait 1 period interrupt flags
        while(PWM_GetPeriodIntFlag(PWM, 0) == 0);
        PWM_ClearPeriodIntFlag(PWM, 0);

        // Disable PWM Precise Center Aligned Type
        PWM->PCACTL &= ~(PWM_PCACTL_PCAEN_Msk);

        // PWM Channel 0 Output : duty = 24.24u, low = 8u
        PWM_SET_CMR(PWM, 0, 99);
        PWM_SET_CNR(PWM, 0, 402);

        // Polling, Wait 1 period interrupt flags
        while(PWM_GetPeriodIntFlag(PWM, 0) == 0);
        PWM_ClearPeriodIntFlag(PWM, 0);
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


