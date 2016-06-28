/**************************************************************************//**
 * @file     EEPROM_24LC64.c
 * @version  V0.10
 * $Revision: 3 $
 * $Date: 15/06/01 11:34a $
 * @brief    MINI58 series 24LC64 EEPROM library source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "Mini58Series.h"
#include "EEPROM_24LC64.h"

#define EEPROM_READ_ADDR      0xA1 /* Address of slave for read  */
#define EEPROM_WRITE_ADDR     0xA0 /* Address of slave for write */

/**
  * @brief Open I2C interface to access EEPROM
  * @param None
  * @return None
  */
void EEPROM_Init(void)
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

/**
  * @brief Write data to EEPROM
  * @param u32Addr Address for writing
  * @param u8Data Data for writing
  * @return None
  */
void EEPROM_Write(uint32_t u32Addr, uint8_t u8Data)
{
    int32_t i32Err;

    do {
        i32Err = 0;

        /* Send start */
        I2C->CTL = (I2C->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_STA_Msk;
        while(!(I2C->CTL & I2C_CTL_SI_Msk));

        /* Send control byte */
        I2C->DAT = EEPROM_WRITE_ADDR;
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
        while(!(I2C->CTL & I2C_CTL_SI_Msk));

        if(I2C->STATUS == 0x18) {
            /* ACK */

            /* Send high address */
            I2C->DAT = (u32Addr >> 8) & 0xFFUL;     // high address
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
            while(!(I2C->CTL & I2C_CTL_SI_Msk));
            if(I2C->STATUS == 0x28) {
                /* ACK */

                /* Send low address */
                I2C->DAT = u32Addr & 0xFFUL;        // low address
                I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
                while(!(I2C->CTL & I2C_CTL_SI_Msk));

                if(I2C->STATUS == 0x28) {
                    /* ACK */

                    /* Send data */
                    I2C->DAT = u8Data;              // data
                    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
                    while(!(I2C->CTL & I2C_CTL_SI_Msk));
                    if(I2C->STATUS == 0x28) {
                        /* ACK */

                        /* Send stop */
                        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;
                    } else {
                        /* NACK */

                        /* Send data error */
                        i32Err = 4;
                    }
                } else {
                    /* NACK */

                    /* Send low address error */
                    i32Err = 3;
                }
            } else {
                /* NACK */

                /* Send high address error */
                i32Err = 2;
            }
        } else {
            /* NACK */

            /* Send control error */
            i32Err = 1;
        }

        if(i32Err) {
            /* Send stop */
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;

            CLK_SysTickDelay(100);
        }

    } while(i32Err);
}

/**
  * @brief Read data from EEPROM
  * @param u32Addr Address for reading
  * @return Data
  */
uint8_t EEPROM_Read(uint32_t u32Addr)
{
    int32_t i32Err;
    uint8_t u8Data;

    u8Data = 0;
    do {
        i32Err = 0;

        /* Send start */
        I2C->CTL = (I2C->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_STA_Msk;
        while(!(I2C->CTL & I2C_CTL_SI_Msk));

        /* Send control byte */
        I2C->DAT = EEPROM_WRITE_ADDR;
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
        while(!(I2C->CTL & I2C_CTL_SI_Msk));
        if(I2C->STATUS == 0x18) {
            /* ACK */

            /* Send high address */
            I2C->DAT = (u32Addr >> 8) & 0xFFUL;     // high address
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
            while(!(I2C->CTL & I2C_CTL_SI_Msk));
            if(I2C->STATUS == 0x28) {
                /* ACK */

                /* Send low address */
                I2C->DAT = u32Addr & 0xFFUL;        // low address
                I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
                while(!(I2C->CTL & I2C_CTL_SI_Msk));
                if(I2C->STATUS == 0x28) {
                    /* ACK */

                    /* Send data */
                    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STA | I2C_SI;
                    while(!(I2C->CTL & I2C_CTL_SI_Msk));
                    if(I2C->STATUS == 0x10) {
                        /* ACK */

                        /* Send control byte */
                        I2C->DAT = EEPROM_READ_ADDR;
                        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
                        while(!(I2C->CTL & I2C_CTL_SI_Msk));
                        if(I2C->STATUS == 0x40) {
                            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
                            while(!(I2C->CTL & I2C_CTL_SI_Msk));

                            /* Read data */
                            u8Data = I2C->DAT;
                            if(I2C->STATUS == 0x58) {
                                /* NACK */
                                /* Send stop */
                                I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;
                            } else {
                                /* ACK */

                                /* read data error */
                                i32Err = 6;
                            }
                        } else {
                            /* NACK */

                            /* Send control read error */
                            i32Err = 5;
                        }
                    } else {
                        /* NACK */

                        /* Send start error */
                        i32Err = 4;
                    }
                } else {
                    /* NACK */

                    /* Send low address error */
                    i32Err = 3;
                }
            } else {
                /* NACK */

                /* Send high address error */
                i32Err = 2;
            }
        } else {
            /* NACK */

            /* Send control write error */
            i32Err = 1;

        }

        if(i32Err) {
            /* Send stop */
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;

            CLK_SysTickDelay(10);
        }

    } while(i32Err);

    return u8Data;
}

/**
  * @brief Read data from EEPROM using sequential read method
  * @param u32Addr Address for reading
  * @param pu8Buf The pointer of buffer that data will be put.
  * @param u32Size Read size
  * @return Actual size that read from EEPROM
  */
uint8_t EEPROM_SequentialRead(uint32_t u32Addr, uint8_t *pu8Buf, uint32_t u32Size)
{
    int32_t i32Err;
    int32_t i;

    do {
        i32Err = 0;

        /* Send start */
        I2C->CTL = (I2C->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_STA_Msk;
        while(!(I2C->CTL & I2C_CTL_SI_Msk));

        /* Send control byte */
        I2C->DAT = EEPROM_WRITE_ADDR;
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
        while(!(I2C->CTL & I2C_CTL_SI_Msk));
        if(I2C->STATUS == 0x18) {
            /* ACK */

            /* Send high address */
            I2C->DAT = (u32Addr >> 8) & 0xFFUL;     // high address
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
            while(!(I2C->CTL & I2C_CTL_SI_Msk));
            if(I2C->STATUS == 0x28) {
                /* ACK */

                /* Send low address */
                I2C->DAT = u32Addr & 0xFFUL;        // low address
                I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
                while(!(I2C->CTL & I2C_CTL_SI_Msk));
                if(I2C->STATUS == 0x28) {
                    /* ACK */

                    /* Send data */
                    I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STA | I2C_SI;
                    while(!(I2C->CTL & I2C_CTL_SI_Msk));
                    if(I2C->STATUS == 0x10) {
                        /* ACK */

                        /* Send control byte */
                        I2C->DAT = EEPROM_READ_ADDR;
                        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
                        while(!(I2C->CTL & I2C_CTL_SI_Msk));
                        if(I2C->STATUS == 0x40) {
                            for(i=0; i<u32Size-1; i++) {
                                I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI | I2C_AA;
                                while(!(I2C->CTL & I2C_CTL_SI_Msk));

                                /* Read data */
                                pu8Buf[i] = I2C->DAT;
                            }

                            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
                            while(!(I2C->CTL & I2C_CTL_SI_Msk));
                            pu8Buf[i] = I2C->DAT;

                            /* Send stop */
                            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;
                        } else {
                            /* NACK */

                            /* Send control read error */
                            i32Err = 5;
                        }
                    } else {
                        /* NACK */

                        /* Send start error */
                        i32Err = 4;
                    }
                } else {
                    /* NACK */

                    /* Send low address error */
                    i32Err = 3;
                }
            } else {
                /* NACK */

                /* Send high address error */
                i32Err = 2;
            }
        } else {
            /* NACK */

            /* Send control write error */
            i32Err = 1;

        }

        if(i32Err) {
            /* Send stop */
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;

            CLK_SysTickDelay(100);
        }

    } while(i32Err);

    return u32Size;
}

/**
  * @brief Write page data to EEPROM
  * @param u32Addr Address for reading
  * @param pu8Buf The pointer of buffer that data will be written.
  * @return None
  */
void EEPROM_PageWrite(uint32_t u32Addr, uint8_t *pu8Buf)
{
    int32_t i32Err;
    int32_t i;

    do {
        i32Err = 0;

        /* Send start */
        I2C->CTL = (I2C->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_STA_Msk;
        while(!(I2C->CTL & I2C_CTL_SI_Msk));

        /* Send control byte */
        I2C->DAT = EEPROM_WRITE_ADDR;
        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
        while(!(I2C->CTL & I2C_CTL_SI_Msk));
        if(I2C->STATUS == 0x18) {
            /* ACK */

            /* Send high address */
            I2C->DAT = (u32Addr >> 8) & 0xFFUL;     // high address
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
            while(!(I2C->CTL & I2C_CTL_SI_Msk));
            if(I2C->STATUS == 0x28) {
                /* ACK */

                /* Send low address */
                I2C->DAT = u32Addr & 0xFFUL;        // low address
                I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
                while(!(I2C->CTL & I2C_CTL_SI_Msk));
                if(I2C->STATUS == 0x28) {
                    /* ACK */

                    for(i=0; i<32; i++) {
                        /* Send data */
                        I2C->DAT = pu8Buf[i]; // data
                        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_SI;
                        while(!(I2C->CTL & I2C_CTL_SI_Msk));
                        if(I2C->STATUS == 0x30) {
                            /* NACK */

                            /* Send data error */
                            i32Err = 4;
                        }
                    }

                    /* Send stop when no any error */
                    if(i32Err == 0) {
                        /* Send stop */
                        I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;
                    }
                } else {
                    /* NACK */

                    /* Send low address error */
                    i32Err = 3;
                }
            } else {
                /* NACK */

                /* Send high address error */
                i32Err = 2;
            }
        } else {
            /* NACK */

            /* Send control error */
            i32Err = 1;
        }

        if(i32Err) {
            /* Send stop */
            I2C->CTL = (I2C->CTL & ~0x3c) | I2C_STO | I2C_SI;

            CLK_SysTickDelay(100);
        }

    } while(i32Err);

}

