/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 15/05/27 2:11p $
 * @brief
 *           Change system clock to different PLL frequency and output system clock from CLKO pin.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "Mini58Series.h"


#define PLL_CLOCK           50000000


void Delay(uint32_t x)
{
    int32_t i;

    for(i = 0; i < x; i++) {
        __NOP();
        __NOP();
    }
}

uint32_t g_au32PllSetting[] = {
    25000000,  /* PLL = 25MHz */
    36000000,  /* PLL = 36MHz */
    48000000,  /* PLL = 48MHz */
    50000000,  /* PLL = 50MHz */
};


void SYS_PLL_Demo(void)
{
    int32_t  i;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PLL clock configuration test                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n-------------------------[ Test PLL ]-----------------------------\n");

    for(i = 0; i < sizeof(g_au32PllSetting) / sizeof(g_au32PllSetting[0]) ; i++) {
        /* Select HCLK clock source from PLL.
           PLL will be configured to twice specified frequency.
           And HCLK clock source divider will be set to 2.
        */
        CLK_SetCoreClock(g_au32PllSetting[i]);

        printf("%d  Change system clock to %d Hz ...................... ",i,SystemCoreClock);

        /* Output selected clock to CKO, CKO Clock = HCLK / 2^(1 + 1) */
        CLK_EnableCKO(CLK_CLKSEL2_CLKOSEL_HCLK, 1, 0);

        /* The delay loop is used to check if the CPU speed is increasing */
        Delay(0x400000);
        printf("[OK]\n");
        /* Disable CLKO clock */
        CLK_DisableCKO();
    }
}

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

    /* Lock protected registers */
    SYS_LockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P0 multi-function pins for UART RXD and TXD */
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD);
    //SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P00_Msk);
    //SYS->P0_MFP |= (SYS_MFP_P01_UART0_RXD | SYS_MFP_P00_UART0_TXD);

    /* Set P3 multi-function pins for Clock Output */
    SYS->P3_MFP = SYS_MFP_P36_CLKO;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|   MINI58 System Driver Sample Code    |\n");
    printf("+---------------------------------------+\n");


    /* Unlock protected registers for Brown-Out Detector settings */
    SYS_UnlockReg();

    /* Check if the write-protected registers are unlocked before BOD setting and CPU Reset */
    if(SYS_IsRegLocked() == 0) {
        printf("Protected Address is Unlocked\n");
    }

    /* Run PLL Test */
    SYS_PLL_Demo();
    
    printf("Exit\n");

    while(1);
}

