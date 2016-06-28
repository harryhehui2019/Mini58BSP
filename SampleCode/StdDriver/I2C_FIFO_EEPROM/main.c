/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 3 $
 * $Date: 15/05/28 10:37a $
 * @brief    Read/write EEPROM via I2C interface using FIFO mode.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Mini58Series.h"

#define EEPROM_READ_ADDR      0xA1 /* Address of slave for read  */
#define EEPROM_WRITE_ADDR     0xA0 /* Address of slave for write */
uint8_t WBuf[3], RBuf[3];

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

void ACK_Polling(void)
{
    uint32_t u32Status;

    /* Disable FIFO mode , don't need FIFO here */
    I2C_DISABLE_FIFO(I2C);

    do {
        /* Send start */
        I2C_START(I2C);                             // S
        I2C_WAIT_READY(I2C);                        // (INT), S

        /* Send control byte */
        I2C_SET_DATA(I2C, EEPROM_WRITE_ADDR);       // ConByte(W)
        I2C_SET_CONTROL_REG(I2C, I2C_SI);
        I2C_WAIT_READY(I2C);                        // (INT), ConByte(W)
        u32Status = I2C_GET_STATUS(I2C);
        I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI); // STOP
    } while( u32Status!= 0x18);

    /* Enable FIFO mode again */
    I2C_ENABLE_FIFO(I2C);
}

void EEPROM_Write(void)
{
    /* Send start */
    I2C_START(I2C);                             // S
    I2C_WAIT_READY(I2C);                        // (INT), S

    /* Send control byte */
    I2C_SET_DATA(I2C, EEPROM_WRITE_ADDR);       // (DATA), ConByte(W)
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_SET_DATA(I2C, (0x00 >> 8) & 0xFFUL);    // (DATA), Add-H
    I2C_WAIT_READY(I2C);                        // (INT), ConByte(W)

    I2C_SET_DATA(I2C, 0x01 & 0xFFUL);           // (DATA), Add-L
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), Add-H

    I2C_SET_DATA(I2C, WBuf[0]);                 // (DATA), data0
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), Add-L

    I2C_SET_DATA(I2C, WBuf[1]);                 // (DATA), data1
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), data0

    I2C_SET_DATA(I2C, WBuf[2]);                 // (DATA), data2
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), data1

    I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI); // STOP
    I2C_WAIT_READY(I2C);                        // (INT), data2

    I2C_SET_CONTROL_REG(I2C, I2C_SI);
}

void EEPROM_Read(void)
{
    /* Send start */
    I2C_START(I2C);                             // S
    I2C_WAIT_READY(I2C);                        // (INT), S

    /* Send control byte */
    I2C_SET_DATA(I2C, EEPROM_WRITE_ADDR);       // (DATA), ControlByte-Write
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_SET_DATA(I2C, (0x00 >> 8) & 0xFFUL);    // (DATA), Add-H
    I2C_WAIT_READY(I2C);                        // (INT), Con-W

    I2C_SET_DATA(I2C, 0x01 & 0xFFUL);           // (DATA), Add-L
    I2C_SET_CONTROL_REG(I2C, I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), Add-H

    I2C_SET_CONTROL_REG(I2C, I2C_STA | I2C_SI); // Sr
    I2C_WAIT_READY(I2C);                        // (INT), ADD-L

    I2C_SET_DATA(I2C, EEPROM_READ_ADDR);        // (DATA), ControlByte-Read
    I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), Sr

    I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), ConrtolByte-Read

    I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), data0
    RBuf[0] = I2C_GET_DATA(I2C);                // (DATA), data0

    I2C_SET_CONTROL_REG(I2C, I2C_AA | I2C_SI);
    I2C_WAIT_READY(I2C);                        // (INT), data1
    RBuf[1] = I2C_GET_DATA(I2C);                // (DATA), data1

    I2C_SET_CONTROL_REG(I2C, I2C_STO | I2C_SI); // STOP
    I2C_WAIT_READY(I2C);                        // (INT), data2
    RBuf[2] = I2C_GET_DATA(I2C);                // (DATA), data2

    I2C_SET_CONTROL_REG(I2C, I2C_SI);
}

int main(void)
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

    printf("+-------------------------------------------------------+\n");
    printf("|    Mini58 I2C Driver Sample Code with EEPROM 24LC64   |\n");
    printf("+-------------------------------------------------------+\n");

    /* Setup write buffer */
    WBuf[0] = 0x11;
    WBuf[1] = 0x22;
    WBuf[2] = 0x33;

    /* Open I2C and set to 100k */
    I2C_Open(I2C, 100000);

    printf("I2C Speed:%dkHz\n", I2C_GetBusClockFreq(I2C)/1000);

    /* Enable FIFO mode */
    I2C_ENABLE_FIFO(I2C);

    /* Write data to EEPROM */
    EEPROM_Write();

    /* Polling ACK from EEPROM */
    ACK_Polling();

    /* Read data from EEPROM*/
    EEPROM_Read();

    /* Check receive buffer */
    for(i=0; i<3; i++) {
        if(WBuf[i] != RBuf[i])
            printf("Data-%d Error!\n", i);
        else
            printf("Data-%d OK!\n", i);
    }

    while(1);
}
