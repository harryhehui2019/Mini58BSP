/**************************************************************************//**
 * @file     main.c
 * @version  V1.0
 * $Date: 15/06/02 8:44p $
 * @brief    Use GPIO driver to control the GPIO pin direction, control their
 *           high/low state, and how to use GPIO interrupts.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini58Series.h"
#include "GPIO.h"

/**
 * @brief       Port0/Port1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port0/Port1 default IRQ, declared in startup_Mini58.s.
 */
void GPIO01_IRQHandler(void)
{
    uint32_t reg;
    /* To check if P1.5 interrupt occurred */
    if (P1->INTSRC & BIT5)
    {
        P1->INTSRC = BIT5;
        P30 = P30 ^ 1;
        printf("P1.5 INT occurred. \n");

    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT0, PORT1 interrupts */
        reg = P0->INTSRC;
        P0->INTSRC = reg;
        reg = P1->INTSRC;
        P1->INTSRC = reg;
        printf("Un-expected interrupts. \n");
    }
}


/**
 * @brief       Port2/Port3/Port4 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Port2/Port3/Port4 default IRQ, declared in startup_Mini58.s.
 */
void GPIO234_IRQHandler(void)
{
    uint32_t reg;
    /* To check if P2.2 interrupt occurred */
    if (P2->INTSRC & BIT2)
    {
        P2->INTSRC = BIT2;
        P30 = P30 ^ 1;
        printf("P2.2 INT occurred. \n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORT2, PORT3 and PORT4 interrupts */
        reg = P2->INTSRC;
        P2->INTSRC = reg;
        reg = P3->INTSRC;
        P3->INTSRC = reg;
        reg = P4->INTSRC;
        P4->INTSRC = reg;
        printf("Un-expected interrupts. \n");
    }
}


/**
 * @brief       External INT0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT0(P3.2) default IRQ, declared in startup_Mini58.s.
 */
void EINT0_IRQHandler(void)
{
    /* For P3.2, clear the INT flag */
    P3->INTSRC = BIT2;
    P30 = P30 ^ 1;
    printf("P3.2 EINT0 occurred. \n");
}


/**
 * @brief       External INT1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1(P5.2) default IRQ, declared in startup_Mini58.s.
 */
void EINT1_IRQHandler(void)
{
    /* For P5.2, clear the INT flag */
    P5->INTSRC = BIT2;
    P30 = P30 ^ 1;
    printf("P5.2 EINT1 occurred. \n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    while(SYS->REGLCTL != 1)
    {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    /* Enable HIRC, and LIRC (fro WDT) */
    CLK->PWRCTL = CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk;

    /* Waiting for clock ready */
    while((CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk)) !=
            (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LIRCSTB_Msk));


    /* Enable UART and WDT clock */
    CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk | CLK_APBCLK_WDTCKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P1 multi-function pins for UART RXD, TXD */
    SYS->P1_MFP = SYS_MFP_P12_UART0_RXD | SYS_MFP_P13_UART0_TXD;

    /* Lock protected registers */
    SYS->REGLCTL = 0;
}

void UART_Init(void)
{
    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART0->LINE = UART_LINE_WLS_Msk;
    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk | (0xBE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    int32_t i32Err;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------+ \n");
    printf("|    Mini58 GPIO Driver Sample Code  | \n");
    printf("+-------------------------------------+ \n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect P1.0 and P3.4 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Configure P1.0 as Output mode and P3.4 as Input mode then close it */
    P1->MODE = (P1->MODE & ~(GP_MODE_MODE0_Msk)) | GPIO_MODE_OUTPUT;
    P3->MODE = (P3->MODE & ~(GP_MODE_MODE4_Msk)) | (GPIO_MODE_INPUT << GP_MODE_MODE4_Pos);

    i32Err = 0;
    printf("  GPIO Output/Input test ...... \n");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    P10 = 0;
    if (P34 != 0)
    {
        i32Err = 1;
    }

    P10 = 1;
    if (P34 != 1)
    {
        i32Err = 1;
    }

    if ( i32Err )
    {
        printf("  [FAIL] --- Please make sure P1.0 and P3.4 are connected. \n");
    }
    else
    {
        printf("  [OK] \n");
    }

    /* Configure P1.0 and P3.4 to default Quasi-bidirectional mode */
    P1->MODE = (P1->MODE & ~(GP_MODE_MODE0_Msk)) | GPIO_MODE_QUASI;
    P3->MODE = (P3->MODE & ~(GP_MODE_MODE4_Msk)) | (GPIO_MODE_QUASI << GP_MODE_MODE4_Pos);

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("\n  P15, P22, P32(INT0) and P52(INT1) are used to test interrupt\n  and control LEDs(P30)\n");

    /*Configure P30 for LED control */
    P3->MODE = (P3->MODE & ~(GP_MODE_MODE0_Msk)) | GPIO_MODE_OUTPUT;

    /* Configure P1.5 as Input mode and enable interrupt by rising edge trigger */
    P1->MODE = (P1->MODE & ~(GP_MODE_MODE5_Msk)) | (GPIO_MODE_INPUT << GP_MODE_MODE5_Pos);
    P1->INTTYPE |= (GPIO_INTTYPE_EDGE << 5);
    P1->INTEN |= (BIT5 << GP_INTEN_RHIEN0_Pos);
    NVIC_EnableIRQ(GPIO01_IRQn);

    /*  Configure P2.2 as Quasi-bidirectional mode and enable interrupt by falling edge trigger */
    P2->MODE = (P2->MODE & ~(GP_MODE_MODE2_Msk)) | (GPIO_MODE_QUASI << GP_MODE_MODE2_Pos);
    P2->INTTYPE |= (GPIO_INTTYPE_EDGE << 2);
    P2->INTEN |= (BIT2 << GP_INTEN_FLIEN0_Pos);
    NVIC_EnableIRQ(GPIO234_IRQn);

    /* Configure P3.2 as EINT0 pin and enable interrupt by falling edge trigger */
    P3->MODE = (P3->MODE & ~(GP_MODE_MODE2_Msk)) | (GPIO_MODE_INPUT << GP_MODE_MODE2_Pos);
    P3->INTTYPE |= (GPIO_INTTYPE_EDGE << 2);
    P3->INTEN |= (BIT2 << GP_INTEN_FLIEN0_Pos);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Configure P5.2 as EINT1 pin and enable interrupt by rising and falling edge trigger */
    P5->MODE = (P5->MODE & ~(GP_MODE_MODE2_Msk)) | (GPIO_MODE_INPUT << GP_MODE_MODE2_Pos);
    P5->INTTYPE |= (GPIO_INTTYPE_EDGE << 2);
    P5->INTEN = (BIT2 << GP_INTEN_FLIEN0_Pos) | (BIT2 << GP_INTEN_RHIEN0_Pos);
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO->DBCTL = (GP_DBCTL_ICLKON_Msk | GPIO_DBCTL_DBCLKSRC_HCLK | GPIO_DBCTL_DBCLKSEL_1);
    P1->DBEN |= BIT5;
    P2->DBEN |= BIT2;
    P3->DBEN |= BIT2;
    P5->DBEN |= BIT2;

    /* Waiting for interrupts */
    while (1);

}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


