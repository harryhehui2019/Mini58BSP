/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 5 $
 * $Date: 15/05/28 4:15p $
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

void EINT0_IRQHandler(void)
{
    P3->INTSRC = 1 << 2;
    gu8Count++;
    isPress = TRUE;
}

void GPIO_Init(void)
{
    /* Enable debounce function of P3.2 (EINT0) */
    GPIO_ENABLE_DEBOUNCE(P3, 2);

    /* Set debounce time. it is about 6.4 ms */
    GPIO->DBCTL = GPIO_DBCTL_DBCLKSRC_IRC10K | GPIO_DBCTL_DBCLKSEL_64;

    /* Enable P3.2 to be EINT0 */
    GPIO_EnableInt(P3, 2, GPIO_INT_RISING);
    NVIC_EnableIRQ(EINT0_IRQn);
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
        printf("I2C EEPROM Write Fail !!\n");
        return -1;
    }
    else
    {
        printf("I2C EEPROM Verify OK!\n");
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
        printf("I2C EEPROM Seq. Read Fail!\n");
        return -1;
    }
    else
    {
        printf("I2C EEPROM Seq. Read OK!\n");
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
        printf("I2C EEPROM Page Write Fail!\n");
        return -1;
    }
    else
    {
        printf("I2C EEPROM Page Write OK!\n");
    }

    return i32Err;

}

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
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_XTAL,CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3.4 and P3.5 for I2C SDA and SCL */
    SYS->P3_MFP = SYS_MFP_P34_I2C0_SDA | SYS_MFP_P35_I2C0_SCL;

    /* Set P1 multi-function pins for UART RXD and TXD */
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();

    /* Update System Core Clock */
    SystemCoreClockUpdate();
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

    printf("+-------------------------------------------------------+\n");
    printf("|    Mini58 I2C Driver Sample Code with EEPROM 24LC64   |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init GPIO P3.2 as EINT0 */
    GPIO_Init();
    /*
        This sample code should work with EEPROM 24LC64
        to show how to program EEPROM through I2C interface.

        The demo will program EEPROM and verify the written data.
        Finally, user may press "SW_INT" key to write and read a byte.
     */

    EEPROM_Init();

    /* Test EEPROM read/write automatically */
    I2C_24LC64_AutoTest();

    while(1);
}

