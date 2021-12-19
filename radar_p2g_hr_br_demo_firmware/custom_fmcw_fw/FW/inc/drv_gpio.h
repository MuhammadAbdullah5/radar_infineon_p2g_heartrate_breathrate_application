/**
 *
 *  drv_gpio.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_GPIO_H_
#define FW_INC_DRV_GPIO_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/

#include "XMC4700.h"
#include <types.h>
#include <xmc_gpio.h>

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/


/******************************************************************************
   3. TYPES
*******************************************************************************/

typedef enum DIGITAL_IO_STATUS
{
  DIGITAL_IO_STATUS_OK = 0U,/**< 0=Status OK */
  DIGITAL_IO_STATUS_FAILURE = 1U/**< 1=Status Failed */
} DIGITAL_IO_STATUS_t;

typedef struct DIGITAL_IO
{
  XMC_GPIO_PORT_t *const gpio_port;             /**< port number */
  const XMC_GPIO_CONFIG_t gpio_config;          /**< mode, initial output level and pad driver strength / hysteresis */
  const uint8_t gpio_pin;                       /**< pin number */
  const XMC_GPIO_HWCTRL_t hwctrl;               /**< Hardware port control */
} DIGITAL_IO_t;


extern const DIGITAL_IO_t DIGITAL_IO_BGT_POWER_ENABLE;

extern const DIGITAL_IO_t DIGITAL_IO_PLL_CE;

extern const DIGITAL_IO_t DIGITAL_IO_PLL_MOD;

extern const DIGITAL_IO_t DIGITAL_IO_SPI_M_CLK;

extern const DIGITAL_IO_t DIGITAL_IO_SPI_M_CS_BGT24;

extern const DIGITAL_IO_t DIGITAL_IO_SPI_M_CS_PLL;

extern const DIGITAL_IO_t DIGITAL_IO_SPI_M_DATA;

extern const DIGITAL_IO_t DIGITAL_IO_PLL_TRIG1;

extern const DIGITAL_IO_t DIGITAL_IO_PLL_TRIG2;

extern const DIGITAL_IO_t DIGITAL_IO_BGT_LDO_ENABLE;

extern const DIGITAL_IO_t DIGITAL_IO_SPI_DATA_PGA;

extern const DIGITAL_IO_t DIGITAL_IO_SPI_M_CS_PGA;

extern const DIGITAL_IO_t DIGITAL_IO_PGA_LDO_ENABLE;

extern const DIGITAL_IO_t DIGITAL_IO_PLL_MUXIN;

extern const DIGITAL_IO_t DIGITAL_IO_LED1;

/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/

// API function prototypes
DIGITAL_IO_STATUS_t DIGITAL_IO_Init          (const DIGITAL_IO_t *const handler);
void                DIGITAL_IO_SetOutputHigh (const DIGITAL_IO_t *const handler);
void                DIGITAL_IO_SetOutputLow  (const DIGITAL_IO_t *const handler);
void                DIGITAL_IO_ToggleOutput  (const DIGITAL_IO_t *const handler);
uint32_t            DIGITAL_IO_GetInput      (const DIGITAL_IO_t *const handler);

/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_GPIO_H_ */

/* --- End of File --- */
