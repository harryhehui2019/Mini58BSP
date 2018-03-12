/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 5 $
 * $Date: 15/05/28 11:35a $
 * @brief    Demonstrate how to use GPIO pins to simulate I2C interface
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"

#include "i2c_software_gpio.h"
#include "i2c_hardware.h"

void SYS_Init(void)
{
    int32_t i32TimeOutCnt;

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

    /* Read User Config to select internal high speed RC */
    SystemInit();

    /* Enable external 12MHz XTAL, HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_XTL12M | CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    i32TimeOutCnt = __HSI / 200; /* About 5ms */
    while((CLK->STATUS & (CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk)) !=
            (CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk))
    {
        if(i32TimeOutCnt-- <= 0)
            break;
    }

    /* Switch HCLK clock source to XTL, STCLK to XTL */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLKSEL_XTAL | CLK_CLKSEL0_HCLKSEL_XTAL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_I2C0CKEN_Msk | CLK_APBCLK_UART0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_XTAL;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3.4 and P3.5 for I2C SDA and SCL */
    SYS->P3_MFP = SYS_MFP_P34_I2C0_SDA | SYS_MFP_P35_I2C0_SCL;

    /* Set P1 multi-function pins for UART RXD and TXD */
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD);

    /* Lock protected registers */
    SYS->REGLCTL = 0;

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART and set UART Baudrate */
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk | (((__XTAL + (115200/2)) / 115200)-2);
    UART0->LINE = 0x3 | (0x0 << UART_LINE_PBE_Pos) | (0x0 << UART_LINE_NSB_Pos) ;
}

int32_t main (void)
{
    uint8_t Tx_Data[6];
    uint8_t Rx_Data[6];

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("+-------------------------------------------------------+\n");
    printf("|    Software I2C sample code                           |\n");
    printf("|    SW Master                                          |\n");
    printf("|    HW Slave                                           |\n");
    printf("+-------------------------------------------------------+\n");

    printf("Initiate I2C for slave\n");
    InitI2C_HW();

    printf("Initiate software I2C for Master\n");
    I2C_SW_Open(100000);

    printf("Please connect:\n");
    printf("SW SDA - P1.4   <->   HW SDA - P3.4 \n");
    printf("SW SCL - P1.5   <->   HW SCL - P3.5 \n");

    Tx_Data[0]=0;
    Tx_Data[1]=0;
    Tx_Data[2]=0xA5;
    Tx_Data[3]=0xcc;
    Tx_Data[4]=0xbb;
    Tx_Data[5]=0xdd;

    printf("Access I2C slave:\n");

    I2C_SW_Send(0x15,Tx_Data,6);
    CLK_SysTickDelay(5000);
    I2C_SW_Get(0x15,Rx_Data,4);

    if((Rx_Data[0] != 0xaa) || (Rx_Data[1] != 0x22) || (Rx_Data[2] != 0x33) || (Rx_Data[3] != 0x44))
        printf("Data Error!!\n");
    else
        printf("Pass!!\n");

    while(1);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
