/**
 *
 *  drv_spi.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_SPI_H_
#define FW_INC_DRV_SPI_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
#include <types.h>
#include <FW/inc/drv_gpio.h>

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/

/******************************************************************************
   3. TYPES
*******************************************************************************/

typedef struct
{
  SPI_Device_Type_t spi_device;    /**< SPI device type, refer to SPI_Device_Type_t */
  DIGITAL_IO_t* gpio_cs_pin;       /**< Data structure for SPI_CS GPIO pin */
  DIGITAL_IO_t* gpio_data_pin;     /**< Data structure for SPI_DATA GPIO pin */
  DIGITAL_IO_t* gpio_clk_pin;      /**< Data structure for SPI_CLK GPIO pin */
} SPI_DIGITAL_IOs_t;

/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/

/**
 * This function is used to transmit the data to the BGT, PGA and LMX devices through SPI protocol.
 *
 * param[in]	*data_ptr		Pointer to unsigned 8-bit memory, which is to be transmitted via SPI protocol
 * param[in]	num_of_bytes	Unsigned 8-bit value, representing the number of data bytes to be transmitted via SPI protocol
 * param[in]	*peripheral_ptr	Void pointer pointing to the target SPI peripheral
 *
 */
void spi_transmit_data(uint8_t* data_ptr, uint8_t num_of_bytes, void* peripheral_ptr);

/**
 * This function reads out LMX PLL registers.
 *
 * param[in]  address         16-bit address value, pointing to the first register to start reading out
 * param[in]  num_of_bytes    Number of consecutive bytes to be read out starting form the given address
 * param[in]  *peripheral_ptr Void pointer pointing to the target SPI peripheral
 * param[out] *out_data_ptr   Pointer of type uint8_t to a memory location where output bytes to be written
 *
 */
void spi_read_register(uint16_t address, uint8_t num_of_bytes, void* peripheral_ptr, uint8_t* out_data_ptr);

/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_SPI_H_ */

/* --- End of File --- */
