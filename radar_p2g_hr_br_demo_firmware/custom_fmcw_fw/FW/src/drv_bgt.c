/**
 *
 *  drv_bgt.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */
 
/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/drv_bgt.h"
#include <FW/inc/drv_spi.h>

/******************************************************************************
   2. DATA
*******************************************************************************/
static SPI_DIGITAL_IOs_t bgt24mtr1x_spi_pins; /**< SPI GPIOs used to configure BGT24MTR1x device */
static volatile uint16_t bgt24mtr1x_ana_command = BGT24MTR1X_ANA_CMD_TX_POWER_REF; /**< Current index of the ANA output Command */
static volatile uint16_t bgt24mtr1x_global_config = BGT24MTR1x_BASE_CONF; /**< Current BGT24MTR1x configuration */
static SPI_DIGITAL_IOs_t bgt24mtr1x_spi_pins; /**< SPI GPIOs used to configure BGT24MTR1x device */

/******************************************************************************
   3. LOCAL FUNCTIONS
*******************************************************************************/
void bgt24mtr1x_set_config(volatile uint16_t config_val)
{
  uint16_t shuffled_data = 0;

  /* Lower byte goes to upper byte position */
  shuffled_data  = (config_val << 8);

  /* Upper byte goes to lower byte position */
  shuffled_data |= (config_val >> 8);

  /* Transmit configuration data to BGT24MTR1x through SPI protocol */
  spi_transmit_data((uint8_t *)&shuffled_data , 2U, (void*) &bgt24mtr1x_spi_pins);
}

static void bgt24mtr1x_ana_vout_tx(void)
{
  bgt24mtr1x_ana_command = BGT24MTR1X_ANA_CMD_TX_POWER;

  bgt24mtr1x_global_config &= BGT24MTR1X_AMUX_VOUT_TX;
}

static void bgt24mtr1x_ana_vref_tx(void)
{
  bgt24mtr1x_ana_command = BGT24MTR1X_ANA_CMD_TX_POWER_REF;

  bgt24mtr1x_global_config &= BGT24MTR1X_AMUX_VOUT_TX;

  bgt24mtr1x_global_config |= BGT24MTR1X_AMUX_0;
}

static void bgt24mtr1x_set_tx_power(Bgt24mtr1x_Power_t power_level)
{
  /* Clear the last 3-bits */
  bgt24mtr1x_global_config &= 0xFFF8;

  switch (power_level)
  {
  case BGT24MTR1X_TX_MIN:
    /* Reduction by 9dBm */
    bgt24mtr1x_global_config |= BGT24MTR1X_PC_PA_7;
    break;

  case BGT24MTR1X_TX_LEVEL_1:
    /* Reduction by 6dBm */
    bgt24mtr1x_global_config |= BGT24MTR1X_PC_PA_6;
    break;

  case BGT24MTR1X_TX_LEVEL_2:
    /* Reduction by 4dBm */
    bgt24mtr1x_global_config |= BGT24MTR1X_PC_PA_5;
    break;

  case BGT24MTR1X_TX_MID:
    /* Reduction by 2.5dBm */
    bgt24mtr1x_global_config |= BGT24MTR1X_PC_PA_4;
    break;

  case BGT24MTR1X_TX_LEVEL_4:
    /* Reduction by 1.4dBm */
    bgt24mtr1x_global_config |= BGT24MTR1X_PC_PA_3;
    break;

  case BGT24MTR1X_TX_LEVEL_5:
    /* Reduction by 0.8dBm */
    bgt24mtr1x_global_config |= BGT24MTR1X_PC_PA_2;
    break;

  case BGT24MTR1X_TX_LEVEL_6:
    /* Reduction by 0.4dBm */
    bgt24mtr1x_global_config |= BGT24MTR1X_PC_PA_1;
    break;

  case BGT24MTR1X_TX_MAX:
  default:
    /* TX on with maximum power */
    bgt24mtr1x_global_config |= BGT24MTR1X_PC_PA_0;
  }

  bgt24mtr1x_ana_vout_tx();
}

static void bgt24mtr1x_init(Bgt24mtr1x_LNAgain_t lna_gain, Bgt24mtr1x_Power_t power_level)
{
  bgt24mtr1x_global_config = BGT24MTR1x_BASE_CONF;

  /* Set the digital IOs used by SPI interface */
  bgt24mtr1x_spi_pins.spi_device    = BGT_DEVICE;
  bgt24mtr1x_spi_pins.gpio_cs_pin 	= (DIGITAL_IO_t*)&DIGITAL_IO_SPI_M_CS_BGT24;
  bgt24mtr1x_spi_pins.gpio_data_pin = (DIGITAL_IO_t*)&DIGITAL_IO_SPI_M_DATA;
  bgt24mtr1x_spi_pins.gpio_clk_pin 	= (DIGITAL_IO_t*)&DIGITAL_IO_SPI_M_CLK;

  if (lna_gain)
  {
	  // if enable, set 15 bit to zero
    bgt24mtr1x_global_config &= BGT24MTR1X_ENA_LNA_MASK;
  }
  else
  {
	  // if disabled, set 15 bit to 1
    bgt24mtr1x_global_config |= BGT24MTR1X_DIS_LNA_MASK;
  }

  /* Configure BGT24MTR1x */
  bgt24mtr1x_set_tx_power(power_level);

  bgt24mtr1x_ana_vref_tx();
}

static void bgt24mtr1x_start_tx(void)
{
  bgt24mtr1x_global_config &= BGT24MTR1X_ENA_PA_MASK;
  bgt24mtr1x_set_config(bgt24mtr1x_global_config);
}

static void bgt24mtr1x_stop_tx(void)
{
  bgt24mtr1x_global_config |= BGT24MTR1X_DIS_PA_MASK;
  bgt24mtr1x_set_config(bgt24mtr1x_global_config);
}

static uint8_t bgt24mtr1x_get_tx_power(void)
{
  /* lower byte contains the Tx power levels info */
  return (uint8_t)(bgt24mtr1x_global_config & 0x07);
}

static void bgt24mtr1x_lna_gain_enable(void)
{
  bgt24mtr1x_global_config &= BGT24MTR1X_ENA_LNA_MASK;
}


static void bgt24mtr1x_lna_gain_disable(void)
{
  bgt24mtr1x_global_config |= BGT24MTR1X_DIS_LNA_MASK;
}

static uint8_t bgt24mtr1x_lna_gain_is_enable(void)
{
  return (uint8_t)((bgt24mtr1x_global_config & BGT24MTR1X_DIS_LNA_MASK) == 0 ? true : false);
}

static uint16_t bgt24mtr1x_get_config(void)
{
  return (bgt24mtr1x_global_config);
}

static void bgt24mtr1x_ana_temp(void)
{
  bgt24mtr1x_ana_command = BGT24MTR1X_ANA_CMD_TEMPERATURE;

  bgt24mtr1x_global_config &= BGT24MTR1X_AMUX_VOUT_TX;

  bgt24mtr1x_global_config |= BGT24MTR1X_AMUX_2;
}


/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/
void bgt_init(uint32_t lna_gain, uint32_t power_level)
{
  /* Set the digital IOs used by SPI interface */
  bgt24mtr1x_spi_pins.spi_device    = BGT_DEVICE;
  bgt24mtr1x_spi_pins.gpio_cs_pin   = (DIGITAL_IO_t*)&DIGITAL_IO_SPI_M_CS_BGT24;
  bgt24mtr1x_spi_pins.gpio_data_pin = (DIGITAL_IO_t*)&DIGITAL_IO_SPI_M_DATA;
  bgt24mtr1x_spi_pins.gpio_clk_pin  = (DIGITAL_IO_t*)&DIGITAL_IO_SPI_M_CLK;

  bgt24mtr1x_init((Bgt24mtr1x_LNAgain_t)lna_gain, (Bgt24mtr1x_Power_t)power_level);

  bgt24mtr1x_ana_vref_tx();
}

//============================================================================

void bgt_start_tx(void)
{
  bgt24mtr1x_start_tx();
}

//============================================================================

void bgt_stop_tx(void)
{
  bgt24mtr1x_stop_tx();
}

//============================================================================

void bgt_power_up(void)
{
  /* Before turning on BGT, we should keep CE pin high.
   * CE pin is active low, so it should keep high until it is activated.
   */
  DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_SPI_M_CS_BGT24);
  DIGITAL_IO_SetOutputLow(&DIGITAL_IO_BGT_POWER_ENABLE);
}

//============================================================================

void bgt_power_down(void)
{
  /* After turning off BGT, we should keep SPI's signals low
   * to avoid offset voltage at BGT's VCC.
   *
   * If they are above 0.3V which is bias voltage inside BGT, then inside BGT turns on.
   * It makes offset voltage at BGT's VCC.
   */
  DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_BGT_POWER_ENABLE);
  DIGITAL_IO_SetOutputLow(&DIGITAL_IO_SPI_M_CS_BGT24);
  DIGITAL_IO_SetOutputLow(&DIGITAL_IO_SPI_M_DATA);
}

//============================================================================

void bgt_set_tx_power(uint8_t power_level)
{
  bgt24mtr1x_set_tx_power((Bgt24mtr1x_Power_t)power_level);
}

//============================================================================

uint8_t bgt_get_tx_power(void)
{
  return bgt24mtr1x_get_tx_power();
}

//============================================================================

void bgt_lna_gain_enable(void)
{
  bgt24mtr1x_lna_gain_enable();
}

//============================================================================

void bgt_lna_gain_disable(void)
{
  bgt24mtr1x_lna_gain_disable();
}

//============================================================================

uint8_t bgt_lna_gain_is_enable(void)
{
  return bgt24mtr1x_lna_gain_is_enable();
}

//============================================================================

void bgt_set_config(uint16_t config_val)
{
  bgt24mtr1x_set_config(config_val);
}

//============================================================================

uint16_t bgt_get_config(void)
{
  return bgt24mtr1x_get_config();
}

//============================================================================

void bgt_ana_temp(void)
{
  bgt24mtr1x_ana_temp();
}

//============================================================================

void bgt_ana_vout_tx(void)
{
  bgt24mtr1x_ana_vout_tx();
}

//============================================================================

void bgt_ana_vref_tx(void)
{
  bgt24mtr1x_ana_vref_tx();
}

//============================================================================

uint16_t bgt_get_ana_config(void)
{
	return (bgt24mtr1x_ana_command);
}

//============================================================================

void bgt_lowest_power_with_q2_disable(void)
{
  bgt24mtr1x_set_config((uint16_t)BGT24MTR1x_POWER_CONF);
}

//============================================================================

void bgt_ldo_enable(void)
{
  DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_BGT_LDO_ENABLE);
}

//============================================================================

void bgt_ldo_disable(void)
{
  DIGITAL_IO_SetOutputLow(&DIGITAL_IO_BGT_LDO_ENABLE);
}


/* --- End of File --- */

