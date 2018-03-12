/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/05/27 5:39p $
 * @brief
 *           Change system clock to different PLL frequency and output system clock from CLKO pin.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "Mini58Series.h"

#define PLLCTL_SETTING  CLK_PLLCTL_100MHz_HXT
#define PLL_CLOCK       50000000

#define SIGNATURE       0x125ab234
#define FLAG_ADDR       0x20000FFC

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple calculation test function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void Delay(uint32_t x)
{
    int32_t i;

    for(i = 0; i < x; i++)
    {
        __NOP();
        __NOP();
    }
}

uint32_t g_au32PllSetting[] =
{
    CLK_PLLCTL_72MHz_HXT,   /* PLL = 72MHz */
    CLK_PLLCTL_96MHz_HXT,   /* PLL = 96MHz */
    CLK_PLLCTL_100MHz_HXT,  /* PLL = 100MHz */

    CLK_PLLCTL_72MHz_HIRC,  /* PLL = 71.884800MHz */
    CLK_PLLCTL_96MHz_HIRC,  /* PLL = 96.129968MHz */
    CLK_PLLCTL_100MHz_HIRC, /* PLL = 99.532800MHz */
};

void SYS_SET_PLL(uint32_t ctl)
{
    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware. */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = ctl;

    /* Wait for PLL clock ready */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
}
void SYS_PLL_Demo(void)
{
    int32_t  i;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PLL clock configuration test                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n-------------------------[ Test PLL ]-----------------------------\n");

    for(i = 0; i < sizeof(g_au32PllSetting) / sizeof(g_au32PllSetting[0]) ; i++)
    {
        /* Select HCLK clock source to HXT and HCLK source divider as 1 */
        CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HXT;
        CLK->CLKDIV  = (CLK->CLKDIV  & (~CLK_CLKDIV_HCLKDIV_Msk))  | CLK_CLKDIV_HCLK(1);

        /* Set PLL clock */
        SYS_SET_PLL(g_au32PllSetting[i]);

        /* Select HCLK clock source to PLL and HCLK source divider as 2 */
        CLK->CLKDIV  = (CLK->CLKDIV  & (~CLK_CLKDIV_HCLKDIV_Msk))  | CLK_CLKDIV_HCLK(2);
        CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

        printf("  Change system clock to %d Hz ...................... ", (CLK_GetPLLClockFreq()) >> 1);

        /* Enable CKO clock source */
        CLK->APBCLK |= CLK_APBCLK_CLKOCKEN_Msk;

        /* CKO = clock source / 2^(1 + 1) */
        CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | (1);

        /* Select CLKO clock source as HCLK */
        CLK->CLKSEL2 = (CLK->CLKSEL1 & (~CLK_CLKSEL2_CLKOSEL_Msk)) | CLK_CLKSEL2_CLKOSEL_HCLK;

        /* The delay loop is used to check if the CPU speed is increasing */
        Delay(0x400000);
        printf("[OK]\n");

        /* Disable CLKO clock */
        CLK->APBCLK &= (~CLK_APBCLK_CLKOCKEN_Msk);
    }
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Set P5 multi-function pins for crystal output/input */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLKDIV_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(1);

    /* Set PLL to Power-down mode */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL = (CLK->PWRCTL & ~CLK_PWRCTL_XTLEN_Msk) | CLK_PWRCTL_XTLEN_HXT;

    /* Wait for HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_XTLSTB_Msk));

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

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0CKEN_Msk;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_XTAL;
    CLK->CLKDIV &= ~CLK_CLKDIV_UARTDIV_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_UART(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD);

    /* Set P3 multi-function pins for Clock Output */
    SYS->P3_MFP &= ~SYS_MFP_P36_Msk;
    SYS->P3_MFP = SYS_MFP_P36_CLKO;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__XTAL, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    while(SYS->REGLCTL != 1)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS->REGLCTL = 0;

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|     Mini58 System Driver Sample Code  |\n");
    printf("+---------------------------------------+\n");

    /*---------------------------------------------------------------------------------------------------------*/
    /* Misc system function test                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers for Brown-Out Detector settings */
    while(SYS->REGLCTL != 1)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    /* Check if the write-protected registers are unlocked before BOD setting and CPU Reset */
    if(SYS->REGLCTL != 0)
    {
        printf("Protected Address is Unlocked\n");
    }

    /* Run PLL Demo */
    SYS_PLL_Demo();

    /* Wait for message send out */
    while(!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV  = (CLK->CLKDIV  & (~CLK_CLKDIV_HCLKDIV_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Set PLL to Power down mode and HW will also clear PLLSTB bit in CLKSTATUS register */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;
    printf("Exit\n");
    while(1);
}




