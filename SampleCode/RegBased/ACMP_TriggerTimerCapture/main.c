/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/07/03 7:10p $
 * @brief    Show how to use Analog comparator (ACMP) state change to
 *           trigger timer capture function. P1.5 is used as comparator
 *           positive input and VBG as negative input.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"


void TMR0_IRQHandler(void)
{
    // printf takes long time and affect the freq. calculation, we only print out once a while
    static int cnt = 0;

    cnt++;
    if(cnt == 60) {
        printf("Input frequency is %dHz\n", 22118400 / TIMER0->CAP);
        cnt = 0;
    }
    // Clear Timer 0 capture interrupt flag
    TIMER0->EINTSTS = TIMER_EINTSTS_CAPIF_Msk;

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

    /*  Read User Config to select internal high speed RC  */
    SystemInit();

    /* Enable HIRC */
    CLK->PWRCTL = CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) == 0);
    /* Enable Timer, UART and ACMP clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_ACMPCKEN_Msk | CLK_APBCLK_TMR0CKEN_Msk | CLK_APBCLK_PWMCH01CKEN_Msk;

    /* Select system clock source from HIRC*/
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_Msk;

    /* Select Timer 0 clock source from HCLK, to use ACMP trigger timer, timer clock source must be HCLK */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | CLK_CLKSEL1_TMR0SEL_HCLK;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD and ACMP CPP0*/
    SYS->P1_MFP = SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD | SYS_MFP_P15_ACMP0_P0;

    /* Set P3.6 to ACMP CPO0 function and P3.2 to Timer 0 capture pin*/
    SYS->P3_MFP = SYS_MFP_P36_ACMP0_O | SYS_MFP_P32_TM0_EXT;

    /* Analog pin OFFD to prevent leakage */
    P1->DINOFF |= (1 << 5) << GP_DINOFF_DINOFF0_Pos;

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

    PWM->PHCHG |= PWM_PHCHG_ACMP0TEN_Msk | PWM_PHCHG_AUTOCLR0_Msk;//Enable signal path from ACMP to Timer

    printf("\nThis sample code demonstrate ACMP0 function. Using CPP0 (P1.5) as ACMP0\n");
    printf("positive input and internal bandgap voltage as the negative input\n");
    printf("The compare result reflects on CPO0 (P3.6)\n");
    printf("Timer detects ACMP signal to calculate its output frequency\n");

    /* Configure ACMP0 Comparator 0. Enable ACMP0, enable interrupt and select internal reference voltage as negative input */
    ACMP->CTL[0] = ACMP_CTL_ACMPEN_Msk | ACMP_CTL_ACMPIE_Msk | ACMP_CTL_NEGSEL_Msk | ACMP_CTL_FTRGEN_Msk;

    TIMER0->CTL = TIMER_CTL_CAPSRC_Msk | TIMER_CTL_CNTEN_Msk | TIMER_PERIODIC_MODE | TIMER_CTL_INTEN_Msk;

    // Set compare value as large as possible
    TIMER0->CMP = 0xFFFFFF;

    // Configure Timer 0 trigger counting mode, capture TDR value on falling edge, enable capture interrupt
    TIMER0->EXTCTL = TIMER_CAPTURE_TRIGGER_COUNTING_MODE |
                     TIMER_CAPTURE_FALLING_THEN_RISING_EDGE |
                     TIMER_EXTCTL_CAPIEN_Msk |
                     TIMER_EXTCTL_CAPEN_Msk |
                     TIMER_CAPTURE_FALLING_EDGE;

    NVIC_EnableIRQ(TMR0_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


