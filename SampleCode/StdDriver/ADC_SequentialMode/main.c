/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/10/22 5:35p $
 * @brief    Demonstrate ADC PWM Sequential Mode conversion and shows the result on UART console.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"

#define PLL_CLOCK           50000000

void ADC_IRQHandler(void)
{
    uint32_t u32Flag;

    // Get ADC convert complete interrupt flag
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_ADIF_INT);

    P05 = P05 ^ 1;

    ADC_CLR_INT_FLAG(ADC, u32Flag);
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

    /* Enable external 12MHz XTAL, HIRC */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV_HCLK(1));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* STCLK to XTL STCLK to XTL */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_XTAL);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(ADC_MODULE);
    CLK_EnableModuleClock(PWMCH01_MODULE);
    CLK_EnableModuleClock(PWMCH23_MODULE);
    CLK_EnableModuleClock(PWMCH45_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_XTAL,CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(ADC_MODULE,CLK_CLKSEL1_ADCSEL_XTAL,CLK_CLKDIV_ADC(6));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD and ADC channel 1 input*/
    SYS->P1_MFP = SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD  | SYS_MFP_P10_ADC_CH1;

    /* Set P5 multi-function pins for ADC channel 0 input*/
    SYS->P5_MFP  = SYS_MFP_P53_ADC_CH0;

    /* Set P0 multi-function pins for PWM Channel 5  */
    SYS->P0_MFP |= SYS_MFP_P04_PWM0_CH5;
    /* Set P2 multi-function pins for PWM Channel 0~4  */
    SYS->P2_MFP = SYS_MFP_P22_PWM0_CH0 | SYS_MFP_P23_PWM0_CH1 |
                  SYS_MFP_P24_PWM0_CH2 | SYS_MFP_P25_PWM0_CH3 | SYS_MFP_P26_PWM0_CH4;

    /* Analog pin OFFD to prevent leakage */
    P1->DINOFF |= (1 << 5) << GP_DINOFF_DINOFF0_Pos;

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();

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

    printf("\nThis sample code demonstrate ADC PWM Sequential Mode conversion and printf the result on UART\n");

    GPIO_SetMode(P0, BIT5, GPIO_MODE_OUTPUT);
    P05 = 0;

    // Enable channel 0
    ADC_Open(ADC, NULL, NULL, BIT0);
    // Enable channel 1
    ADC_Open(ADC, NULL, NULL, BIT1);

    // Enable A/D Converter, A/D Interrupt , External Trigger and A/D conversion is started by PWM trigger
    ADC->CTL = ADC_CTL_ADCEN_Msk | ADC_CTL_ADCIEN_Msk | ADC_CTL_HWTRGSEL_Msk | ADC_CTL_HWTRGEN_Msk;

    // Enable ADC convert complete interrupt
    NVIC_EnableIRQ(ADC_IRQn);

    /* ADC PWM Sequential Mode Control */
    // ADC Sequential mode Enable
    ADC->SEQCTL |= ADC_SEQCTL_SEQEN_Msk;

    // ADC Sequential Mode Type, shunt23 type
    ADC->SEQCTL = (ADC->SEQCTL & ~(ADC_SEQCTL_SEQTYPE_Msk));

    // Delay ADC start conversion time after PWM trigger
    ADC->TRGDLY = 0xFF;

    // ADC Sequential Mode Selection, ADC_INT after Channel 0 then Channel 1 conversion finishes
    ADC->SEQCTL = (ADC->SEQCTL & ~(ADC_SEQCTL_MODESEL_Msk)) | (0 << ADC_SEQCTL_MODESEL_Pos);

    // ADC PWM first Trigger Type and Source Selection, PWM0 Falling
    ADC->SEQCTL = (ADC->SEQCTL & ~(ADC_SEQCTL_TRG1CTL_Msk)) | (ADC_SEQMODE_PWM0_FALLING << ADC_SEQCTL_TRG1CTL_Pos);

    // ADC PWM Second Trigger Type and Source Selection, PWM2 Falling
    ADC->SEQCTL = (ADC->SEQCTL & ~(ADC_SEQCTL_TRG2CTL_Msk)) | (ADC_SEQMODE_PWM2_FALLING << ADC_SEQCTL_TRG2CTL_Pos);


    // PWM0 frequency is 1200Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM, 0, 1200, 30);

    // PWM2 frequency is 3000Hz, duty 50%
    PWM_ConfigOutputChannel(PWM, 2, 3000, 50);

    // Enable output of all PWM channels
    PWM_EnableOutput(PWM, 0x3F);

    // Start
    PWM_Start(PWM, 0x3F);


    while(1)
    {
        // Get ADC convert result
        printf("A/D PWM Sequential Mode First Result  is %x\n", ADC->SEQDAT1);
        printf("A/D PWM Sequential Mode Second Result is %x\n", ADC->SEQDAT2);
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


