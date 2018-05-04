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


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));

    /* Set P5 multi-function pins for crystal output/input */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT);

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_XTLEN_HXT);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_XTAL, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_UART0_RXD | SYS_MFP_P00_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    UART0->BAUD = 0x30000066;
    UART0->LINE = 0x3;
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

    print_msg("\n\nChange VECMAP and branch to LDROM...\n");
    while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk));

    func = (FUNC_PTR *) FMC_Read(FMC_APROM_BASE+4);
    __set_SP( FMC_Read(FMC_APROM_BASE) );

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
