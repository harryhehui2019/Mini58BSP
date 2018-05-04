/**************************************************************************//**
 * @file     i2c_hardware.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/01 11:35a $
 * @brief    MINI58 series hardware I2C driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Mini58Series.h"
#include "i2c_hardware.h"

uint8_t Tx_Data[4]= {0xaa,0x22,0x33,0x44};
uint8_t Rx_Data[5];
uint8_t DataLen;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C);

    switch(u32Status)
    {
    /* Slave Transmitter Mode */
    case 0xC0:                        /* DATA has been transmitted and NACK has been returned */
    case 0xC8:                        /* DATA has been transmitted and ACK has been returned */
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI | I2C_AA;
        printf("Slave Transmitter Success\n");
        break;

    case 0xA8:                        /* SLA+R has been received and ACK has been returned */
    case 0xB0:
        DataLen = 0;
    case 0xB8:                        /* DATA has been transmitted and ACK has been returned */
        I2C->DAT = Tx_Data[DataLen++];
        if(DataLen<sizeof(Tx_Data))
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI | I2C_AA;
        else
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
        break;

    /* Slave Receiver Mode*/
    case 0x68:                        /* SLA+W has been received and ACK has been returned */
    case 0x60:
        DataLen = 0;
        Rx_Data[0] = 0;
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI | I2C_AA;
        break;
    case 0x80:                        /* DATA has been received and ACK has been returned */
        Rx_Data[DataLen++] = I2C->DAT;
        if(DataLen<(sizeof(Rx_Data)-1))
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI | I2C_AA;
        else
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
        break;
    case 0x88:                        /* DATA has been received and NACK has been returned */
        Rx_Data[DataLen++] = I2C->DAT;
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI | I2C_AA;
        break;

    case 0xA0:                      /* STOP or Repeat START has been received */
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI | I2C_AA;
        printf("Slave Receiver Success\n");
        break;
    }
}

void InitI2C_HW(void)
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
    I2C->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);

    /* I2C as slave */
    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI | I2C_AA;
}

