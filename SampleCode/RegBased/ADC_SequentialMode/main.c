/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 16/06/08 1:18p $ 
 * @brief    Demonstrate ADC PWM Sequential Mode conversion and shows the result on UART console.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/  
#include <stdio.h>
#include "Mini58Series.h"

#define PLLCTL_SETTING  CLK_PLLCTL_100MHz_HXT
#define PLL_CLOCK       50000000

void ADC_IRQHandler(void)
{		
    P05 = P05 ^ 1;

    // Clear ADC convert complete flag
    ADC->STATUS |= ADC_STATUS_ADIF_Msk;
}

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
		
    /* Enable UART, ADC and PWM clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_ADCCKEN_Msk | CLK_APBCLK_PWMCH01CKEN_Msk | CLK_APBCLK_PWMCH23CKEN_Msk | CLK_APBCLK_PWMCH45CKEN_Msk;
    
    /* Select UART Clock source from external 12 MHz or 32 KHz crystal clock */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UARTSEL_Pos);// 
		
    /* ADC clock source Clock source from external 12 MHz or 32 KHz crystal clock, set divider to (6 + 1) */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_ADCSEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_ADCSEL_Pos); 
    CLK->CLKDIV |= (6 << CLK_CLKDIV_ADCDIV_Pos);    

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD and ADC channel 1 input*/
    SYS->P1_MFP = SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD | SYS_MFP_P10_ADC_CH1;
		
		/* Set P5 multi-function pins for ADC channel 0 input*/
    SYS->P5_MFP  = SYS_MFP_P53_ADC_CH0;
		
    /* Set P0 multi-function pins for PWM Channel 5  */
    SYS->P0_MFP |= SYS_MFP_P04_PWM0_CH5; 
    /* Set P2 multi-function pins for PWM Channel 0~4  */
    SYS->P2_MFP = SYS_MFP_P22_PWM0_CH0 | SYS_MFP_P23_PWM0_CH1 | 
                  SYS_MFP_P24_PWM0_CH2 | SYS_MFP_P25_PWM0_CH3 | SYS_MFP_P26_PWM0_CH4;
									
    /* Analog pin OFFD to prevent leakage */
    P5->DINOFF |= (1 << 3) << GP_DINOFF_DINOFF3_Pos;
		P1->DINOFF |= (1 << 0) << GP_DINOFF_DINOFF0_Pos;

    /* To update the variable SystemCoreClock */
    SystemCoreClockUpdate();
		
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

    printf("\nThis sample code demonstrate ADC PWM Sequential Mode conversion and print the result on UART\n");

    /* Configure P0.5 as Output mode */
    P0->MODE = (P0->MODE & ~(GP_MODE_MODE5_Msk)) | (GPIO_MODE_OUTPUT << GP_MODE_MODE5_Pos);
    P05 = 0;  
	
    // Enable channel 0, 1
    ADC->CHEN = BIT0 | BIT1;
    
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
		    			
    // Set channel 0,2 pre-scale to 2. Actual value fill into register needs to minus 1.
    PWM->CLKPSC = 0x101;
    // Set channel 0 clock divider to 1
    PWM->CLKDIV = (PWM_CLK_DIV_1 << PWM_CLKDIV_CLKDIV0_Pos) | (PWM_CLK_DIV_1 << PWM_CLKDIV_CLKDIV2_Pos);
    // Enable PWM channel 0, 2 auto-reload mode
    PWM->CTL = PWM_CTL_CNTMODE0_Msk | PWM_CTL_CNTMODE2_Msk;
		
    /* 
      Configure PWM channel 2 init period and duty. 
      Period is HCLK / (pre-scale * clock divider * (CNR + 1))
      Duty ratio = (CMR + 1) / (CNR + 1)
      Period = 50 MHz / (2 * 1 * (9999 + 1)) =  2500 Hz
      Duty ratio = (2999 + 1) / (9999 + 1) = 30%
    */
    PWM->CMPDAT0 = 2999;
    PWM->PERIOD0 = 9999;   
    
    /* 
      Configure PWM channel 2 init period and duty. 
      Period is HCLK / (pre-scale * clock divider * (CNR + 1))
      Duty ratio = (CMR + 1) / (CNR + 1)
      Period = 50 MHz / (2 * 1 * (4999 + 1)) =  5000 Hz
      Duty ratio = (2499 + 1) / (4999 + 1) = 50%
    */
    PWM->CMPDAT2 = 2499;
    PWM->PERIOD2 = 4999;   

    // Enable PWM channel 0 and 2 output
    PWM->POEN = PWM_POEN_POEN0_Msk | PWM_POEN_POEN2_Msk;
    
    // Start
    PWM->CTL |= PWM_CTL_CNTEN0_Msk | PWM_CTL_CNTEN2_Msk;    

    while(1)
    {      			
        // Get ADC convert result
        printf("A/D PWM Sequential Mode First Result  is %x\n", ADC->SEQDAT1);
        printf("A/D PWM Sequential Mode Second Result is %x\n", ADC->SEQDAT2);
    }

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


