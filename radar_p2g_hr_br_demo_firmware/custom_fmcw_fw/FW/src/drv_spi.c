/**
 *
 *  drv_spi.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

/******************************************************************************
   1. INCLUDES
*******************************************************************************/
#include "FW/inc/drv_spi.h"

/******************************************************************************
   2. DATA
*******************************************************************************/

/******************************************************************************
   3. LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/

void spi_transmit_data(uint8_t* data_ptr, uint8_t num_of_bytes, void* peripheral_ptr)
{
  SPI_DIGITAL_IOs_t spi_pins = *(SPI_DIGITAL_IOs_t*)peripheral_ptr;
  uint8_t index, data;

  DIGITAL_IO_SetOutputLow((DIGITAL_IO_t*)spi_pins.gpio_cs_pin);
  __NOP(); __NOP(); __NOP(); __NOP();

  for (uint8_t count = 0U; count < num_of_bytes; count++)
  {
    data = *data_ptr++;

    for (index =0 ; index < 8U; index++)
    {
      if (data & 0x80)
      {
        DIGITAL_IO_SetOutputHigh((DIGITAL_IO_t*)spi_pins.gpio_data_pin);
      }
      else
      {
        DIGITAL_IO_SetOutputLow((DIGITAL_IO_t*)spi_pins.gpio_data_pin);
      }
      __NOP(); __NOP();

      DIGITAL_IO_SetOutputHigh((DIGITAL_IO_t*)spi_pins.gpio_clk_pin);
      __NOP(); __NOP(); __NOP(); __NOP();

      DIGITAL_IO_SetOutputLow((DIGITAL_IO_t*)spi_pins.gpio_clk_pin);
      __NOP(); __NOP();

      data <<= 1;
    }
  }

  DIGITAL_IO_SetOutputHigh((DIGITAL_IO_t*)spi_pins.gpio_cs_pin);
  __NOP(); __NOP(); __NOP(); __NOP();
}

//============================================================================

void spi_read_register(uint16_t address, uint8_t num_of_bytes, void* peripheral_ptr, uint8_t* out_data_ptr)
{
  uint8_t i;
  uint8_t out_data;
  uint16_t address_mask;
  uint16_t delay = 0;
  SPI_DIGITAL_IOs_t spi_pins = *(SPI_DIGITAL_IOs_t*)peripheral_ptr;

  DIGITAL_IO_SetOutputLow((DIGITAL_IO_t*)spi_pins.gpio_cs_pin);

  for(delay = 0; delay < 12; delay++)

  /* Write R/W = 1, Address 15Bit */
  address |= 0x8000;
  address_mask = 0x8000;

  for(i = 0 ; i < (16U); i++)
  {
    if (address & address_mask)
    {
      DIGITAL_IO_SetOutputHigh((DIGITAL_IO_t*)spi_pins.gpio_data_pin);
    }
    else
    {
      DIGITAL_IO_SetOutputLow((DIGITAL_IO_t*)spi_pins.gpio_data_pin);
    }

    for(delay =0; delay < 12; delay++)
    __NOP();

    DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_SPI_M_CLK);

    for(delay =0; delay < 12; delay++)
    __NOP();

    DIGITAL_IO_SetOutputLow((DIGITAL_IO_t*)spi_pins.gpio_clk_pin);

    address <<= 1;
  }
  /* Keep data pin low */
  DIGITAL_IO_SetOutputLow((DIGITAL_IO_t*)spi_pins.gpio_data_pin);

  for(uint8_t count = 0; count < num_of_bytes; count++)
  {
    out_data = 0;
    for(i = 0; i < 8; i++)
    {
      /* Read PLL register value (MSB first)*/
      DIGITAL_IO_SetOutputHigh((DIGITAL_IO_t*)spi_pins.gpio_clk_pin);

      for(delay =0; delay < 4; delay++)
      {
        __NOP();
      }

      if(spi_pins.spi_device == LMX_DEVICE)
      {
    	if(DIGITAL_IO_GetInput(&DIGITAL_IO_PLL_MUXIN))
        {
          /* MSB bit first */
          out_data |= 1<<(7-i);
        }
      }

      for(delay =0; delay < 4; delay++)
      {
        __NOP();
      }

      DIGITAL_IO_SetOutputLow((DIGITAL_IO_t*)spi_pins.gpio_clk_pin);

      for(delay =0; delay < 12; delay++)
      {
        __NOP();
      }
    }

    *out_data_ptr = out_data;
    out_data_ptr++;
  }

  DIGITAL_IO_SetOutputHigh((DIGITAL_IO_t*)spi_pins.gpio_cs_pin);
}

/******************************************************************************
   5. LOCAL FUNCTIONS
*******************************************************************************/



/* --- End of File --- */

