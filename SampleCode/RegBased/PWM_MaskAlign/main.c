
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/02 9:07p $
 * @brief    Demonstrate the PWM Output 0%, 50% and 100%.
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

void PWM_Polling(uint8_t Channel)
{
    // Polling, Wait 1 period interrupt flags
    while((PWM->INTSTS & (PWM_INTSTS_ZIF0_Msk << Channel)) == 0);
    PWM->INTSTS = (PWM_INTSTS_ZIF0_Msk << Channel);

    // Polling, Wait 2 period interrupt flags
    while((PWM->INTSTS & (PWM_INTSTS_ZIF0_Msk << Channel)) == 0);
    PWM->INTSTS = (PWM_INTSTS_ZIF0_Msk << Channel);
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

    // PWM-Timer 0~2 enable and Auto-reload
    PWM->CTL = PWM_CTL_CNTEN0_Msk | PWM_CTL_CNTMODE0_Msk | PWM_CTL_CNTEN1_Msk | PWM_CTL_CNTMODE1_Msk | PWM_CTL_CNTEN2_Msk | PWM_CTL_CNTMODE2_Msk;

    // Set channel 0~2 prescaler to 2. Actual value fill into register needs to minus 1.
    PWM->CLKPSC = 0x101;

    // Set channel 0~2 clock divider to 1
    PWM->CLKDIV |= (PWM_CLK_DIV_1 << PWM_CLKDIV_CLKDIV0_Pos);
    PWM->CLKDIV |= (PWM_CLK_DIV_1 << PWM_CLKDIV_CLKDIV1_Pos);
    PWM->CLKDIV |= (PWM_CLK_DIV_1 << PWM_CLKDIV_CLKDIV2_Pos);

    // Enable PWM channel 0~2 output
    PWM->POEN = PWM_POEN_POEN0_Msk | PWM_POEN_POEN1_Msk | PWM_POEN_POEN2_Msk;

    // Start
    PWM->CTL |= (PWM_CTL_CNTEN0_Msk | PWM_CTL_CNTEN1_Msk | PWM_CTL_CNTEN2_Msk);

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

    // Set the PWM aligned type
    PWM->CTL = (PWM->CTL & ~PWM_CTL_CNTTYPE_Msk) | (PWM_EDGE_ALIGNED);
    PWM->CMPDAT0 = 200;
    PWM->PERIOD0 = 400;

    // PWM Channel 1 Duty = 0%, Disable Output Mask Aligned and Output the original channel 1 waveform
    PWM->MSKALIGN |= PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN0_Msk;
    PWM->CMPDAT1 = 0;
    PWM->PERIOD1 = 400;
    PWM_Polling(0);

    // PWM Channel 1 Duty = 50%, Disable Output Mask Aligned and Output the original channel 1 waveform
    PWM->MSKALIGN |= PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN0_Msk;
    PWM->CMPDAT1 = 200;
    PWM_Polling(0);

    // Edge Aligned Type Duty 0% need to use mask aligned and PWM channel outputs enable control to achieve
    // PWM Channel 1 Duty 0%, Enable Output Mask Aligned and Output the MSKDAT waveform
    PWM->MSKALIGN = (0x1ul << 17);//PWM_MSKALIGN_ALIGN1_Msk;
    PWM->MSKALIGN |= PWM_MSKALIGN_MSKEN0_Msk;
    PWM->MSKALIGN = PWM->MSKALIGN & ~(PWM_MSKALIGN_MSKDAT1_Msk);
    PWM_Polling(0);

    // PWM Channel 1 Duty 100%, Disable Output Mask Aligned and Output the original channel 1 waveform
    PWM->MSKALIGN = PWM->MSKALIGN | (PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN1_Msk);
    PWM->CMPDAT1 = 400;
    PWM_Polling(0);


    /*
     Center Align PWM channel 2 waveform of this sample shown below:
     Duty 0%              -> Duty 50%                      -> Duty 100%          -> Duty 50%                      -> Duty 100%(Enable Mask Aligned)

                          ________          ________       __________ __________       ______        ______       ___________________
    PWM2 ______64us______|  16us  |__16us__|  16us  |_8us_|    32us  |   32us   |_8us_| 16us |_16us_| 16us |_8us_|

    */
    PWM->CTL = (PWM->CTL & ~PWM_CTL_CNTTYPE_Msk) | (PWM_CENTER_ALIGNED);

    // PWM Channel 2 Duty 0%, Disable Output Mask Aligned and Output the original channel 2 waveform
    PWM->MSKALIGN = PWM->MSKALIGN | (PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN2_Msk);
    PWM->CMPDAT2 = 400;
    PWM->PERIOD2 = 400;
    PWM_Polling(0);

    // PWM Channel 2 Duty 50%, Disable Output Mask Aligned and Output the original channel 2 waveform
    PWM->MSKALIGN = PWM->MSKALIGN | (PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN2_Msk);
    PWM->CMPDAT2 = 200;
    PWM_Polling(0);

    // PWM Channel 2 Duty 100%, Disable Output Mask Aligned and Output the original channel 2 waveform
    PWM->MSKALIGN = PWM->MSKALIGN | (PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN2_Msk);
    PWM->CMPDAT2 = 0;
    PWM_Polling(0);

    // PWM Channel 2 Duty 50%, Disable Output Mask Aligned and Output the original channel 2 waveform
    PWM->MSKALIGN = PWM->MSKALIGN | (PWM_MSKALIGN_MSKEN0_Msk | PWM_MSKALIGN_MSKEN2_Msk);
    PWM->CMPDAT2 = 200;
    PWM_Polling(0);

    // Center Aligned Type Duty 100% need to use outputs mask aligned and PWM channel outputs enable control to achieve
    // PWM Channel 2 Duty 100%, Enable Output Mask Aligned and Output the MSKDAT waveform
    PWM->MSKALIGN = 0x1ul << 18;//PWM_MSKALIGN_ALIGN2_Msk;
    PWM->MSKALIGN |= PWM_MSKALIGN_MSKEN0_Msk;
    PWM->MSKALIGN |= PWM_MSKALIGN_MSKDAT2_Msk;
    PWM_Polling(0);

    // Stop
    PWM->PERIOD0 = 0;
    PWM->PERIOD1 = 0;
    PWM->PERIOD2 = 0;

    while(1);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


