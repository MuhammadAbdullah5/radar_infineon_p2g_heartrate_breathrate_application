/**
 *
 *  drv_gpio.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */
 
/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/drv_gpio.h"
#include "stdbool.h"

/******************************************************************************
   2. DATA
*******************************************************************************/

const DIGITAL_IO_t DIGITAL_IO_BGT_POWER_ENABLE =
{
  .gpio_port = XMC_GPIO_PORT2,
  .gpio_pin = 3U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_PLL_CE =
{
  .gpio_port = XMC_GPIO_PORT2,
  .gpio_pin = 2U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
    .output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_MEDIUM_EDGE,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_PLL_MOD =
{
  .gpio_port = XMC_GPIO_PORT1,
  .gpio_pin = 2U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_INPUT_TRISTATE,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_SPI_M_CLK =
{
  .gpio_port = XMC_GPIO_PORT8,
  .gpio_pin = 5U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_SPI_M_CS_BGT24 =
{
  .gpio_port = XMC_GPIO_PORT8,
  .gpio_pin = 6U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_SPI_M_CS_PLL =
{
  .gpio_port = XMC_GPIO_PORT1,
  .gpio_pin = 8U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_SPI_M_DATA =
{
  .gpio_port = XMC_GPIO_PORT8,
  .gpio_pin = 7U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_PLL_TRIG1 =
{
  .gpio_port = XMC_GPIO_PORT1,
  .gpio_pin = 3U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_INPUT_TRISTATE,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_PLL_TRIG2 =
{
  .gpio_port = XMC_GPIO_PORT1,
  .gpio_pin = 1U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_BGT_LDO_ENABLE =
{
  .gpio_port = XMC_GPIO_PORT4,
  .gpio_pin = 3U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_SPI_DATA_PGA =
{
  .gpio_port = XMC_GPIO_PORT5,
  .gpio_pin = 1U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_SPI_M_CS_PGA =
{
  .gpio_port = XMC_GPIO_PORT8,
  .gpio_pin = 4U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_PGA_LDO_ENABLE =
{
  .gpio_port = XMC_GPIO_PORT7,
  .gpio_pin = 11U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_PLL_MUXIN =
{
  .gpio_port = XMC_GPIO_PORT1,
  .gpio_pin = 0U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_INPUT_TRISTATE,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};

const DIGITAL_IO_t DIGITAL_IO_LED1 =
{
  .gpio_port = XMC_GPIO_PORT7,
  .gpio_pin = 8U,
  .gpio_config = {
    .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,

  },
  .hwctrl = XMC_GPIO_HWCTRL_DISABLED
};


/******************************************************************************
   3. LOCAL FUNCTIONS
*******************************************************************************/

/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/

DIGITAL_IO_STATUS_t DIGITAL_IO_Init(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_Init: handler null pointer", handler != NULL);

  /* Initializes input / output characteristics */
  XMC_GPIO_Init(handler->gpio_port, handler->gpio_pin, &handler->gpio_config);

  /*Configure hardware port control*/
  XMC_GPIO_SetHardwareControl(handler->gpio_port, handler->gpio_pin, handler->hwctrl);

  return (DIGITAL_IO_STATUS_OK);
}

void DIGITAL_IO_SetOutputHigh(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_SetOutputHigh: handler null pointer", handler != NULL);
  XMC_GPIO_SetOutputHigh(handler->gpio_port, handler->gpio_pin);
}

void DIGITAL_IO_SetOutputLow(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_SetOutputLow: handler null pointer", handler != NULL);
  XMC_GPIO_SetOutputLow(handler->gpio_port,handler->gpio_pin);
}

void DIGITAL_IO_ToggleOutput(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_ToggleOutput: handler null pointer", handler != NULL);
  XMC_GPIO_ToggleOutput(handler->gpio_port, handler->gpio_pin);
}

uint32_t DIGITAL_IO_GetInput(const DIGITAL_IO_t *const handler)
{
  XMC_ASSERT("DIGITAL_IO_GetInput: handler null pointer", handler != NULL);
  return XMC_GPIO_GetInput(handler->gpio_port, handler->gpio_pin);
}

/* --- End of File --- */

