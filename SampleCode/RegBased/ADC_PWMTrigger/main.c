
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/12/18 6:49p $
 * @brief    Demonstrate PWM0 channel 0 trigger ADC.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"

void ADC_IRQHandler(void)
{
    P01 = P01 ^ 1;

    /* Get ADC convert result */
    printf("Convert result is %x\n", ADC->DAT & 0x3FF);

    // Clear ADC convert complete flag
    ADC->STATUS |= ADC_STATUS_ADIF_Msk;
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

    /* Enable external 12MHz XTAL, HIRC */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    while((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_XTLSTB_Msk)) !=
            (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_XTLSTB_Msk));

    /* Set core clock as HXT */
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HXT;

    /* Enable UART, ADC and PWM clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_ADCCKEN_Msk | CLK_APBCLK_PWMCH01CKEN_Msk;

    /* Select UART Clock source from external 12 MHz */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UARTSEL_Pos);//

    /* ADC clock source Clock source from external 12 MHz crystal clock, set divider to (6 + 1) */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_ADCSEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_ADCSEL_Pos);
    CLK->CLKDIV |= (6 << CLK_CLKDIV_ADCDIV_Pos);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD and ADC channel 5 input */
    SYS->P1_MFP = SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD | SYS_MFP_P15_ADC_CH5;

    /* Analog pin OFFD to prevent leakage */
    P1->DINOFF |= (1 << 5) << GP_DINOFF_DINOFF0_Pos;

    /* Set P2 multi-function pins for PWM Channel 0. */
    SYS->P2_MFP = SYS_MFP_P22_PWM0_CH0;

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS->REGLCTL = 0;
}

void UART_Init(void)
{
    // 115200 bps
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk | (((__XTAL + (115200/2)) / 115200)- 2);
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

    printf("\nThis sample code demonstrate PWM channel 0 trigger ADC function\n");

    /* Configure P0.5 as Output mode */
    P0->MODE = (P0->MODE & ~(GP_MODE_MODE1_Msk)) | (GPIO_MODE_OUTPUT << GP_MODE_MODE1_Pos);
    P01 = 0;

    /* Enable channel 5 */
    ADC->CHEN = (1 << 5);

    /* Enable A/D Converter, A/D Interrupt , PWM trigger A/D conversion */
    ADC->CTL = ADC_CTL_ADCEN_Msk | ADC_CTL_ADCIEN_Msk | ADC_CTL_HWTRGSEL_Msk | ADC_CTL_HWTRGEN_Msk;

    /* Enable ADC convert complete interrupt */
    NVIC_EnableIRQ(ADC_IRQn);

    /* PWM frequency is 600Hz, duty 70% */
    PWM->CLKPSC = 0x101;
    /* Set channel 0 clock divider to 1 */
    PWM->CLKDIV = (PWM_CLK_DIV_1 << PWM_CLKDIV_CLKDIV0_Pos);
    /* Enable PWM channel 0 auto-reload mode */
    PWM->CTL = PWM_CTL_CNTMODE0_Msk;
    PWM->CMPDAT0 = 2999;
    PWM->PERIOD0 = 9999;

    /* Set PWM channel 0 to center-aligned mode */
    PWM->CTL = (PWM->CTL & ~PWM_CTL_CNTTYPE_Msk) | PWM_CTL_CNTTYPE_Msk;

    /* Enable PWM channel 0 center-triggered ADC */
    PWM->ADCTCTL0 = (PWM->ADCTCTL0 & ~(PWM_ADCTCTL0_CPTRGEN0_Msk)) | PWM_ADCTCTL0_CPTRGEN0_Msk;

    /* Enable PWM channel 0 output */
    PWM->POEN = PWM_POEN_POEN0_Msk;

    /* Start PWM */
    PWM->CTL |= PWM_CTL_CNTEN0_Msk;

    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


