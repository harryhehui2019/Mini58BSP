/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 3 $
 * $Date: 15/05/28 10:37a $
 * @brief    Read/write EEPROM using GPIO pins to simulate I2C interface.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"
#include "i2c_software_gpio_with_timer.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT);

    /* Enable external 12MHz XTAL, HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to XTL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_XTAL,CLK_CLKDIV_HCLK(1));

    /* STCLK to XTL STCLK to XTL */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_XTAL);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_XTAL,CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD and TXD */
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

int32_t main (void)
{
    uint8_t Tx_Data[6];

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-------------------------------------------------------+\n");
    printf("|    Software I2C sample code                           |\n");
    printf("|    SW Master -> I2C EEPROM                            |\n");
    printf("+-------------------------------------------------------+\n");

    I2C_SW_I_Open(50000);
    Tx_Data[0]=0;
    Tx_Data[1]=0;
    Tx_Data[2]=0xAA;
    Tx_Data[3]=0xBB;
    Tx_Data[4]=0x55;
    Tx_Data[5]=0xCC;

    printf("Write data into EEPROM\n");
    printf("Data:0x%x,0x%x,0x%x,0x%x\n",Tx_Data[2],Tx_Data[3],Tx_Data[4],Tx_Data[5] );
    I2C_SW_I_Send(0x50,Tx_Data,6);
    while(I2C_SW_I_IsBZ());
    if(I2C_SW_I_Count()!=6)
        while(1);
    CLK_SysTickDelay(5000);

    printf("Write address into EEPROM\n");
    I2C_SW_I_Send(0x50,Tx_Data,2);
    while(I2C_SW_I_IsBZ());
    if(I2C_SW_I_Count()!=2)
        while(1);

    printf("Read data form EEPROM\n");
    I2C_SW_I_Get(0x50,Tx_Data,4);
    while(I2C_SW_I_IsBZ());
    printf("Data:0x%x,0x%x,0x%x,0x%x\n",Tx_Data[0],Tx_Data[1],Tx_Data[2],Tx_Data[3] );
    if(I2C_SW_I_Count()!=4)
        while(1);
    while(1);
}
