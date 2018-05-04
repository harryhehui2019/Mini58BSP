/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 5 $
 * $Date: 15/05/28 11:35a $
 * @brief    Read/write EEPROM via I2C interface using polling mode.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Mini58Series.h"

uint8_t gu8Count = 0, isPress = FALSE;
uint8_t g_u8Buf[256] = {0};

extern void EEPROM_Write(uint32_t u32Addr, uint8_t u8Data);
extern uint8_t EEPROM_Read(uint32_t u32Addr);
extern uint8_t EEPROM_SequentialRead(uint32_t u32Addr, uint8_t *pu8Buf, uint32_t u32Size);
extern void EEPROM_PageWrite(uint32_t u32Addr, uint8_t *pu8Buf);
extern void EEPROM_Init(void);

void delay_loop(void)
{
    uint32_t i, j;
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 60000; j++);
    }
}

int32_t I2C_24LC64_AutoTest(void)
{
    int32_t i, i32Err;

    /* Programming EEPROM */
    for(i=0; i<256; i++)
        EEPROM_Write(i, i);

    /* Verify */
    i32Err = 0;
    for(i=0; i<256; i++)
    {
        if(EEPROM_Read(i) != i)
        {
            i32Err = 1;
            break;
        }
    }

    if(i32Err)
    {
        printf("Write Fail!\n");
        return -1;
    }
    else
    {
        printf("Verify OK!\n");
    }

    /* Delay for 2 seconds */
    for(i=0; i<20; i++)
        CLK_SysTickDelay(100000);

    EEPROM_SequentialRead(0, g_u8Buf, 256);
    /* Verify */
    i32Err = 0;
    for(i=0; i<256; i++)
    {
        if(g_u8Buf[i] != i)
        {
            i32Err = 1;
            break;
        }
    }

    if(i32Err)
    {
        printf("Seq. Read Fail\n");
        return -1;
    }
    else
    {
        printf("Seq. Read OK!\n");
    }

    /* Delay for 2 seconds */
    for(i=0; i<20; i++)
        CLK_SysTickDelay(100000);

    for(i=0; i<256; i++)
        g_u8Buf[i] = i;
    for(i=0; i<8; i++)
        EEPROM_PageWrite(i * 32, &g_u8Buf[i*32]);

    memset(g_u8Buf, 0, 256);

    EEPROM_SequentialRead(0, g_u8Buf, 256);
    /* Verify */
    i32Err = 0;
    for(i=0; i<256; i++)
    {
        if(EEPROM_Read(i) != (i & 0xFF))
        {
            i32Err = 1;
            break;
        }
    }

    if(i32Err)
    {
        printf("Page Write Fail\n");
        return -1;
    }
    else
    {
        printf("Page Write OK!\n");
    }

    return i32Err;

}

int32_t I2C_24LC64_ManualTest(void)
{
    uint32_t i2cdata = 0, i;

    for(i=0; i<10; i++)
    {
        EEPROM_Write(i, i);
        i2cdata = EEPROM_Read(i);
        printf("Address :%x, Write: %x, Read: %x\n", i, i, i2cdata);
        delay_loop();
    }

    return 0;
}

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

int main(void)
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    UART_Init();

    printf("+-------------------------------------------------------+\n");
    printf("|    Mini58 I2C Driver Sample Code with EEPROM 24LC64   |\n");
    printf("+-------------------------------------------------------+\n");
    /*
        This sample code should work with EEPROM 24LC64
        to show how to program EEPROM through I2C interface.

        The demo will program EEPROM and verify the written data.
        Finally, user may press "SW_INT" key to write and read a byte.
     */

    EEPROM_Init();

    /* Test EEPROM read/write automatically */
    I2C_24LC64_AutoTest();

    /* Delay for 2 seconds */
    for(i=0; i<20; i++)
        CLK_SysTickDelay(100000);

    /* Test EEPROM read/write by key pressing */
    I2C_24LC64_ManualTest();

    while(1);
}

