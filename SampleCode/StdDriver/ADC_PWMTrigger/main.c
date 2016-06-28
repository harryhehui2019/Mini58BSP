
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/12/18 5:37p $ 
 * @brief    Demonstrate PWM0 channel 0 trigger ADC.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/  
#include <stdio.h>
#include "Mini58Series.h"

void ADC_IRQHandler(void)
{
    uint32_t u32Flag;
    
    /* Get ADC comparator interrupt flag */
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_ADIF_INT);
    
    /* Get ADC convert result */
    printf("Convert result is %x\n", (uint32_t)ADC_GET_CONVERSION_DATA(ADC, 0));
    
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
	
    /* Enable external 12MHz XTAL (UART), HIRC */
    CLK->PWRCTL = CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk);
	
    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_XTAL,CLK_CLKDIV_HCLK(1));
	
    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_ADCCKEN_Msk | CLK_APBCLK_PWMCH01CKEN_Msk;
    
    /* Select UART clock source from external crystal */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UARTSEL_Msk) | CLK_CLKSEL1_UARTSEL_XTAL;
		/* Select ADC clock source from external crystal */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_ADCSEL_Msk) | CLK_CLKSEL1_ADCSEL_XTAL;
		
    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_XTAL,CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(ADC_MODULE,CLK_CLKSEL1_ADCSEL_XTAL,CLK_CLKDIV_ADC(6));
		
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD and ADC channel 5 input */
    SYS->P1_MFP = SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD | SYS_MFP_P15_ADC_CH5;

    /* Analog pin OFFD to prevent leakage */
    P1->DINOFF |= (1 << 5) << GP_DINOFF_DINOFF0_Pos;
		
    /* Set P2 multi-function pins for PWM Channel 0. */
    SYS->P2_MFP = SYS_MFP_P22_PWM0_CH0;

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
	
    printf("\nThis sample code demonstrate PWM channel 0 trigger ADC function\n");

    /* Enable channel 5 */
    ADC_Open(ADC, 0, 0, 0x01 << 5);

    /* Power on ADC */
    ADC_POWER_ON(ADC);
		
    /* Enable PWM trigger */
    ADC_EnableHWTrigger(ADC, ADC_TRIGGER_BY_PWM, ADC_FALLING_EDGE_TRIGGER);
												 
    /* Enable ADC convert complete interrupt  */
    ADC_EnableInt(ADC, ADC_ADIF_INT);
    NVIC_EnableIRQ(ADC_IRQn);
	
    /* PWM frequency is 100Hz, duty 30% */
    PWM_ConfigOutputChannel(PWM, 0, 100, 30);
    /* Enable output PWM channel 0 */
    PWM_EnableOutput(PWM, 0x1);

    /* Set PWM channel 0 to center-aligned mode */
    PWM_SET_ALIGNED_TYPE(PWM, 0, PWM_CENTER_ALIGNED);
  
    /* Enable PWM channel 0 center-triggered ADC */
    PWM_EnableADCTrigger(PWM, 0, PWM_TRIGGER_ADC_CNTR_IS_CNR);
		
    /* PWM Start */
    PWM_Start(PWM, 0x1);
		
    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


