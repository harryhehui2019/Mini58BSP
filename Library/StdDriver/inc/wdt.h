/**************************************************************************//**
 * @file     wdt.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/05/28 1:16p $
 * @brief    Mini58 series WDT driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __WDT_H__
#define __WDT_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_WDT_Driver WDT Driver
  @{
*/

/** @addtogroup Mini58_WDT_EXPORTED_CONSTANTS WDT Exported Constants
  @{
*/
#define WDT_TIMEOUT_2POW4          (0UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^4 * WDT clocks */
#define WDT_TIMEOUT_2POW6          (1UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^6 * WDT clocks */
#define WDT_TIMEOUT_2POW8          (2UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^8 * WDT clocks */
#define WDT_TIMEOUT_2POW10         (3UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^10 * WDT clocks */
#define WDT_TIMEOUT_2POW12         (4UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^12 * WDT clocks */
#define WDT_TIMEOUT_2POW14         (5UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^14 * WDT clocks */
#define WDT_TIMEOUT_2POW16         (6UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^16 * WDT clocks */
#define WDT_TIMEOUT_2POW18         (7UL << WDT_CTL_TOUTSEL_Pos) /*!< WDT setting for timeout interval = 2^18 * WDT clocks */

#define WDT_RESET_DELAY_3CLK        (3UL << WDT_ALTCTL_RSTDSEL_Pos)    /*!< WDT setting reset delay to 3 WDT clocks */
#define WDT_RESET_DELAY_18CLK       (2UL << WDT_ALTCTL_RSTDSEL_Pos)    /*!< WDT setting reset delay to 18 WDT clocks */
#define WDT_RESET_DELAY_130CLK      (1UL << WDT_ALTCTL_RSTDSEL_Pos)    /*!< WDT setting reset delay to 130 WDT clocks */
#define WDT_RESET_DELAY_1026CLK     (0UL << WDT_ALTCTL_RSTDSEL_Pos)    /*!< WDT setting reset delay to 1026 WDT clocks */


/*@}*/ /* end of group Mini58_WDT_EXPORTED_CONSTANTS */


/** @addtogroup Mini58_WDT_EXPORTED_FUNCTIONS WDT Exported Functions
  @{
*/

/**
  * @brief This macro clear WDT time-out reset system flag.
  * \hideinitializer
  */
#define WDT_CLEAR_RESET_FLAG()  (WDT->CTL = (WDT->CTL & ~(WDT_CTL_IF_Msk | WDT_CTL_WKF_Msk)) | WDT_CTL_RSTF_Msk)

/**
  * @brief This macro clear WDT time-out interrupt flag.
  * \hideinitializer
  */
#define WDT_CLEAR_TIMEOUT_INT_FLAG() (WDT->CTL = (WDT->CTL & ~(WDT_CTL_RSTF_Msk | WDT_CTL_WKF_Msk)) | WDT_CTL_IF_Msk)

/**
  * @brief This macro clear WDT time-out wake-up system flag.
  * \hideinitializer
  */
#define WDT_CLEAR_TIMEOUT_WAKEUP_FLAG() (WDT->CTL = (WDT->CTL & ~(WDT_CTL_RSTF_Msk | WDT_CTL_IF_Msk)) | WDT_CTL_WKF_Msk)

/**
  * @brief This macro indicate WDT time-out to reset system or not.
  * @return WDT reset system or not
  * @retval 0 WDT did not cause system reset
  * @retval 1 WDT caused system reset
  * \hideinitializer
  */
#define WDT_GET_RESET_FLAG() (WDT->CTL & WDT_CTL_RSTF_Msk ? 1 : 0)

/**
  * @brief This macro indicate WDT time-out interrupt occurred or not.
  * @return WDT time-out interrupt occurred or not
  * @retval 0 WDT time-out interrupt did not occur
  * @retval 1 WDT time-out interrupt occurred
  * \hideinitializer
  */
#define WDT_GET_TIMEOUT_INT_FLAG() (WDT->CTL & WDT_CTL_IF_Msk ? 1 : 0)

/**
  * @brief This macro indicate WDT time-out waked system up or not
  * @return WDT time-out waked system up or not
  * @retval 0 WDT did not wake up system
  * @retval 1 WDT waked up system
  * \hideinitializer
  */
#define WDT_GET_TIMEOUT_WAKEUP_FLAG() (WDT->CTL & WDT_CTL_WKF_Msk ? 1 : 0)

/**
  * @brief This macro is used to reset 18-bit WDT counter.
  * @details If WDT is activated and enabled to reset system, software must reset WDT counter
  *  before WDT time-out plus reset delay reached. Or WDT generate a reset signal.
  * \hideinitializer
  */
#define WDT_RESET_COUNTER() (WDT->CTL  = (WDT->CTL & ~(WDT_CTL_IF_Msk | WDT_CTL_WKF_Msk | WDT_CTL_RSTF_Msk)) | WDT_CTL_RSTCNT_Msk)


void  WDT_Open(uint32_t u32TimeoutInterval,
               uint32_t u32ResetDelay,
               uint32_t u32EnableReset,
               uint32_t u32EnableWakeup);
void WDT_Close(void);

void WDT_EnableInt(void);
void WDT_DisableInt(void);

/*@}*/ /* end of group Mini58_WDT_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_WDT_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__WDT_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
