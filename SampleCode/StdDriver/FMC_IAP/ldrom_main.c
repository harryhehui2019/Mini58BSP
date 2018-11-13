/**************************************************************************//**
 * @file     LDROM_main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/05/26 12:22p $
 * @brief    This sample code includes LDROM image (fmc_ld_iap)
 *           and APROM image (fmc_ap_main).
 *           It shows how to branch between APROM and LDROM. To run
 *           this sample code, the boot mode must be "Boot from APROM
 *           with IAP".
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"

#define PLL_CLOCK                   50000000


typedef void (FUNC_PTR)(void);

FUNC_PTR    *func;
uint32_t    sp;


void SYS_Init(void)
{
    /* Read User Config to select internal high speed RC */
    SystemInit();

    /* Set P5 multi-function pins for crystal output/input */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL &= ~CLK_PWRCTL_XTLEN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_XTLEN_Pos); // XTAL12M (HXT) Enabled

    /* Waiting for 12MHz clock ready */
    while (!(CLK->STATUS & CLK_STATUS_XTLSTB_Msk));

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_XTAL;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0CKEN_Msk; // UART Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UARTSEL_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0,P1 multi-function pins for UART RXD and TXD  */
    SYS->P0_MFP = 0x303;
    UART0->LINE = 0x3;
    UART0->BAUD = 0x30000066;
}


#ifdef __ARMCC_VERSION
void __asm __set_SP(uint32_t _sp)
{
    MSR MSP, r0
    BX lr
}
#endif


void SendChar_ToUART(int ch)
{
    while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
    UART0->DAT = ch;
    if (ch == '\n')
    {
        while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
        UART0->DAT = '\r';
    }
}


void print_msg(char *str)
{
    for ( ; *str ; str++)
        SendChar_ToUART(*str);
}

int main()
{
    FUNC_PTR    *func;

    SYS_Init();
    UART_Init();

    print_msg("\n\n");
    print_msg("MINI58 FMC IAP Sample Code [LDROM code]\n");

    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    print_msg("\n\nPress any key to branch to APROM...\n");
    while (UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk);

    print_msg("\n\nChange VECMAP and branch to APROM...\n");
    while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk));

    sp = FMC_Read(FMC_APROM_BASE);
    func = (FUNC_PTR *)FMC_Read(FMC_APROM_BASE + 4);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) /* for GNU C compiler */
    asm("msr msp, %0" : : "r" (sp));
#else
    __set_SP(sp);
#endif

    /*  NOTE!
     *     Before change VECMAP, user MUST disable all interrupts.
     */
    /* FMC_SetVectorPageAddr(FMC_APROM_BASE) */
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADDR = FMC_APROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    func();

    while (1);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
