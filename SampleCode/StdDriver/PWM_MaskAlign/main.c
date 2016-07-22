
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/05/28 10:37a $
 * @brief    Demonstrate the PWM Output 0%, 50% and 100%.
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

void PWM_Polling(uint8_t Channel)
{
    // Polling, Wait 1 period interrupt flags
    while(PWM_GetPeriodIntFlag(PWM, Channel) == 0);
    PWM_ClearPeriodIntFlag(PWM, Channel);

    // Polling, Wait 2 period interrupt flags
    while(PWM_GetPeriodIntFlag(PWM, Channel) == 0);
    PWM_ClearPeriodIntFlag(PWM, Channel);
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

    // PWM-Timer 0 enable and Auto-reload
    PWM->CTL = PWM_CTL_CNTEN0_Msk | PWM_CTL_CNTMODE0_Msk | PWM_CTL_CNTEN1_Msk | PWM_CTL_CNTMODE1_Msk | PWM_CTL_CNTEN2_Msk | PWM_CTL_CNTMODE2_Msk;

    PWM_SET_PRESCALER(PWM, 0, 1);
    PWM_SET_PRESCALER(PWM, 1, 1);
    PWM_SET_PRESCALER(PWM, 2, 1);

    PWM_SET_DIVIDER(PWM, 0, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWM, 1, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWM, 2, PWM_CLK_DIV_1);

    // Enable PWM channel 0 ~3 output
    PWM_EnableOutput(PWM, BIT0);
    PWM_EnableOutput(PWM, BIT1);
    PWM_EnableOutput(PWM, BIT2);

    // Start
    PWM_Start(PWM, BIT0);
    PWM_Start(PWM, BIT1);
    PWM_Start(PWM, BIT2);

    printf("\nThis sample code will output PWM channel 1 to with 3 duty(0%%, 50%% and 100%%).\n");
    printf("Edge align type with Duty 0%% or Center align type with Duty 100%% need to \n");
    printf("use Phase Change Mask Aligned to achieve.\n");
    printf("Polling PWM channel 0 period interrupt flag to get PWM channel 1,2 output.\n");

    /*
     Edge Align PWM channel 1 waveform of this sample shown below:
     Duty 0%                     -> Duty 50%
                                                          -> Duty 0%(Enable Mask Aligned)
                                                                                 -> Duty 100%
                _____       _____       _____       _____       _____       _____       _____       _____
     PWM0 _8us_| 8us |_8us_| 8us |_8us_| 8us |_8us_| 8us |_8us_| 8us |_8us_| 8us |_8us_| 8us |_8us_| 8us |
                                        _____       _____                         ________
     PWM1 __15.99us__|__15.99us__|_8us_| 8us |_8us_| 8us |___________32us________|  16us  |__________

    */

    PWM_SET_ALIGNED_TYPE(PWM, 0, PWM_EDGE_ALIGNED);
    PWM_SET_CMR(PWM, 0, 200);
    PWM_SET_CNR(PWM, 0, 400);

    PWM_SET_ALIGNED_TYPE(PWM, 1, PWM_EDGE_ALIGNED);

    // PWM Channel 1 Duty = 0%, Disable Output Mask Aligned and Output the original channel 1 waveform
    PWM->MSKALIGN |= PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN0_Msk;
    PWM_SET_CMR(PWM, 1, 0);
    PWM_SET_CNR(PWM, 1, 400);
    PWM_Polling(0);

    // PWM Channel 1 Duty = 50%, Disable Output Mask Aligned and Output the original channel 1 waveform
    PWM->MSKALIGN |= PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN0_Msk;
    PWM_SET_CMR(PWM, 1, 200);
    PWM_Polling(0);

    // Edge Aligned Type Duty 0% need to use mask aligned and PWM channel outputs enable control to achieve
    // PWM Channel 1 Duty 0%, Enable Output Mask Aligned and Output the MSKDAT waveform
    PWM->MSKALIGN = PWM_MSKALIGN_ALIGN1_Msk;
    PWM->MSKALIGN |= PWM_MSKALIGN_MSKEN0_Msk;
    PWM->MSKALIGN = PWM->MSKALIGN & ~(PWM_MSKALIGN_MSKDAT1_Msk);
    PWM_Polling(0);

    // PWM Channel 1 Duty 100%, Disable Output Mask Aligned and Output the original channel 1 waveform
    PWM->MSKALIGN = PWM->MSKALIGN | (PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN1_Msk);
    PWM_SET_CMR(PWM, 1, 400);
    PWM_Polling(0);


    /*
     Center Align PWM channel 2 waveform of this sample shown below:
     Duty 0%              -> Duty 50%                      -> Duty 100%          -> Duty 50%                      -> Duty 100%(Enable Mask Aligned)

                          ________          ________       __________ __________       ______        ______       ___________________
    PWM2 ______64us______|  16us  |__16us__|  16us  |_8us_|    32us  |   32us   |_8us_| 16us |_16us_| 16us |_8us_|

    */

    PWM_SET_ALIGNED_TYPE(PWM, 0, PWM_CENTER_ALIGNED);
    PWM_SET_ALIGNED_TYPE(PWM, 2, PWM_CENTER_ALIGNED);

    // PWM Channel 2 Duty 0%, Disable Output Mask Aligned and Output the original channel 2 waveform
    PWM->MSKALIGN = PWM->MSKALIGN | (PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN2_Msk);
    PWM_SET_CMR(PWM, 2, 400);
    PWM_SET_CNR(PWM, 2, 400);
    PWM_Polling(0);

    // PWM Channel 2 Duty 50%, Disable Output Mask Aligned and Output the original channel 2 waveform
    PWM->MSKALIGN = PWM->MSKALIGN | (PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN2_Msk);
    PWM_SET_CMR(PWM, 2, 200);
    PWM_Polling(0);

    // PWM Channel 2 Duty 100%, Disable Output Mask Aligned and Output the original channel 2 waveform
    PWM->MSKALIGN = PWM->MSKALIGN | (PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN2_Msk);
    PWM_SET_CMR(PWM, 2, 0);
    PWM_Polling(0);

    // PWM Channel 2 Duty 50%, Disable Output Mask Aligned and Output the original channel 2 waveform
    PWM->MSKALIGN = PWM->MSKALIGN | (PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN2_Msk);
    PWM_SET_CMR(PWM, 2, 200);
    PWM_Polling(0);

    // Center Aligned Type Duty 100% need to use outputs mask aligned and PWM channel outputs enable control to achieve
    // PWM Channel 2 Duty 100%, Enable Output Mask Aligned and Output the MSKDAT waveform
    PWM->MSKALIGN = PWM_MSKALIGN_ALIGN2_Msk;
    PWM->MSKALIGN |= PWM_MSKALIGN_MSKEN0_Msk;
    PWM->MSKALIGN |= PWM_MSKALIGN_MSKDAT2_Msk;
    PWM_Polling(0);

    PWM_Stop(PWM, BIT0);
    PWM_Stop(PWM, BIT1);
    PWM_Stop(PWM, BIT2);

    while(1);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


