/******************************************************************************
 * @file     main.c
 * @version  V0.10
 * $Revision: 5 $
 * $Date: 15/05/28 11:34a $
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
    int32_t i32TimeOutCnt;

    /* Unlock protected registers */
    while(SYS->REGLCTL != 1) {
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
            (CLK_STATUS_XTLSTB_Msk | CLK_STATUS_HIRCSTB_Msk)) {
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

void ACK_Polling(void)
{
    uint32_t u32Status;

    /* Disable FIFO mode , don't need FIFO here */
    I2C->CTL1 &= ~I2C_CTL1_TWOLVFIFO_Msk;

    do {
        /* Send start */
        I2C->CTL = (I2C->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_STA_Msk;         // S
        while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), S

        /* Send control byte */
        I2C->DAT = EEPROM_WRITE_ADDR;       // ConByte(W)
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
        while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), ConByte(W)

        u32Status = I2C_GET_STATUS(I2C);
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI; // STOP
    } while( u32Status!= 0x18);

    /* Enable FIFO mode again */
    I2C->CTL1 |= I2C_CTL1_TWOLVFIFO_Msk;
}

void EEPROM_Write(void)
{
    /* Send start */
    I2C->CTL = (I2C->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_STA_Msk;   // S
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), S

    /* Send control byte */
    I2C->DAT = EEPROM_WRITE_ADDR;                                // (DATA), ConByte(W)
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    I2C->DAT = (0x00 >> 8) & 0xFFUL;                             // (DATA), Add-H
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), ConByte(W)

    I2C->DAT = 0x01 & 0xFFUL;                                    // (DATA), Add-L
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), Add-H

    I2C->DAT = WBuf[0];                                          // (DATA), data0
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), Add-L

    I2C->DAT = WBuf[1];                                          // (DATA), data1
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), data0

    I2C->DAT = WBuf[2];                                          // (DATA), data2
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), data1


    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;             // STOP
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                         // (INT), data2

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
}

void EEPROM_Read(void)
{
    /* Send start */
    I2C->CTL = (I2C->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_STA_Msk;   // S
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), S

    /* Send control byte */
    I2C->DAT = EEPROM_WRITE_ADDR;                                // (DATA), ConByte(W)
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    I2C->DAT = (0x00 >> 8) & 0xFFUL;                             // (DATA), Add-H
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), ConByte(W)

    I2C->DAT = 0x01 & 0xFFUL;                                    // (DATA), Add-L
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), Add-H

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STA | I2C_SI;            // Sr
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), ADD-L

    I2C->DAT = EEPROM_READ_ADDR;                                 // (DATA), ControlByte-Read
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_AA | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), Sr

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_AA | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), ConrtolByte-Read

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_AA | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), data0
    RBuf[0] = I2C->DAT;                                          // (DATA), data0

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_AA | I2C_SI;
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), data1
    RBuf[1] = I2C->DAT;                                          // (DATA), data1

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;            // STOP
    while(!(I2C->CTL & I2C_CTL_SI_Msk));                        // (INT), data2
    RBuf[2] = I2C->DAT;                                          // (DATA), data2

    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
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

void I2C_Init(void)
{
    /* Reset I2C */
    SYS->IPRST1 |=  SYS_IPRST1_I2C0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;

    /* Enable I2C Controller */
    I2C0->CTL |= I2C_CTL_I2CEN_Msk;

    /* I2C clock divider, I2C Bus Clock = 100kHz */
    I2C0->CLKDIV = 0x1D;

    /* Set I2C 4 Slave Addresses */
    I2C0->ADDR0 = (0x15 << 1);
    I2C0->ADDR1 = (0x35 << 1);
    I2C0->ADDR2 = (0x55 << 1);
    I2C0->ADDR3 = (0x75 << 1);
}

int main(void)
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("+-------------------------------------------------------+\n");
    printf("|    Mini58 I2C Driver Sample Code with EEPROM 24LC64   |\n");
    printf("+-------------------------------------------------------+\n");

    /* Setup write buffer */
    WBuf[0] = 0x11;
    WBuf[1] = 0x22;
    WBuf[2] = 0x33;

    /* Open I2C and set to 100k */
    I2C_Init();

    /* Enable FIFO mode */
    I2C->CTL1 |= I2C_CTL1_TWOLVFIFO_Msk;

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
