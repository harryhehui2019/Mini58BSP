/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 5 $
 * $Date: 15/05/28 11:34a $
 * @brief    Read/write EEPROM via I2C interface using interrupt mode.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr;
uint8_t g_au8TxData[3];
uint8_t g_u8RxData;
uint8_t g_u8DataLen;
uint8_t __IO g_u8EndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

I2C_FUNC __IO s_I2CHandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C0->STATUS;

    if (I2C0->TOCTL & I2C_TOCTL_TOIF_Msk)
    {
        /* Clear I2C Timeout Flag */
        I2C0->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if (s_I2CHandlerFn != NULL)
            s_I2CHandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
    {
        I2C0->DAT = g_u8DeviceAddr << 1;         /* Write SLA+W to Register I2CDAT */
        I2C0->CTL |= I2C_CTL_SI_Msk;
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C0->DAT = g_au8TxData[g_u8DataLen++];
        I2C0->CTL |= I2C_CTL_SI_Msk;
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C0->CTL |= I2C_CTL_STO_Msk | I2C_CTL_SI_Msk;
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen != 2)
        {
            I2C0->DAT = g_au8TxData[g_u8DataLen++];
            I2C0->CTL |= I2C_CTL_SI_Msk;
        }
        else
        {
            I2C0->CTL |= I2C_CTL_STA_Msk | I2C_CTL_SI_Msk;
        }
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C0->DAT = ((g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register DAT */
        I2C0->CTL |= I2C_CTL_SI_Msk;
    }
    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
    {
        I2C0->CTL |= I2C_CTL_SI_Msk;
    }
    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
    {
        g_u8RxData = I2C0->DAT;
        I2C0->CTL |= I2C_CTL_STO_Msk | I2C_CTL_SI_Msk;
        g_u8EndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        I2C0->DAT = g_u8DeviceAddr << 1;         /* Write SLA+W to Register I2CDAT */
        I2C0->CTL |= I2C_CTL_SI_Msk;
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C0->DAT = g_au8TxData[g_u8DataLen++];
        I2C0->CTL |= I2C_CTL_SI_Msk;
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C0->CTL |= I2C_CTL_STO_Msk | I2C_CTL_SI_Msk;
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen != 3)
        {
            I2C0->DAT = g_au8TxData[g_u8DataLen++];
            I2C0->CTL |= I2C_CTL_SI_Msk;
        }
        else
        {
            I2C0->CTL |= I2C_CTL_STO_Msk | I2C_CTL_SI_Msk;
            g_u8EndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void SYS_Init(void)
{
    int32_t i32TimeOutCnt;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P5 multi-function pins for XTAL1 and XTAL2 */
    SYS->P5_MFP &= ~(SYS_MFP_P50_Msk | SYS_MFP_P51_Msk);
    SYS->P5_MFP |= (SYS_MFP_P50_XT1_IN | SYS_MFP_P51_XT1_OUT);

    /* Unlock protected registers */
    while(SYS->REGLCTL != 1)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

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

    /* Enable I2C interrupt */
    I2C0->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, accesses EEPROM 24LC64 with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|    Mini58 I2C Driver Sample Code with EEPROM 24LC64  |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C to access EEPROM */
    I2C_Init();

    g_u8DeviceAddr = 0x50;

    for (i = 0; i < 0x100; i++)
    {
        g_au8TxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(i & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);

        g_u8DataLen = 0;
        g_u8EndFlag = 0;

        /* I2C function to write data to slave */
        s_I2CHandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C as master sends START signal */
        I2C0->CTL |= I2C_CTL_STA_Msk;

        /* Wait I2C Tx Finish */
        while (g_u8EndFlag == 0);
        while(I2C0->CTL & I2C_CTL_STO_Msk);
        g_u8EndFlag = 0;

        /* I2C function to read data from slave */
        s_I2CHandlerFn = (I2C_FUNC)I2C_MasterRx;

        g_u8DataLen = 0;
        g_u8DeviceAddr = 0x50;

        /* Wait write finished */
        SysTick->LOAD = 5000 * CyclesPerUs;
        SysTick->VAL  =  (0x00);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        /* Waiting for down-count to zero */
        while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

        /* I2C as master sends START signal */
        I2C0->CTL |= I2C_CTL_STA_Msk;

        /* Wait I2C Rx Finish */
        while (g_u8EndFlag == 0);
        while(I2C0->CTL & I2C_CTL_STO_Msk);

        /* Compare data */
        if (g_u8RxData != g_au8TxData[2])
        {
            printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8RxData);
            return -1;
        }
    }
    printf("I2C Access EEPROM Test OK\n");

    while(1);

    // return 0;
}



