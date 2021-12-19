/**
 *
 *  drv_acq_buffer.h
 *
 *     Created on: Jul 18, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_ACQ_BUFFER_H_
#define FW_INC_DRV_ACQ_BUFFER_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
#include <types.h>
#include <stdint.h>
#include <stddef.h>

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/
#define USE_MALLOC_FOR_BUFFER           (0)
#define MAX_SUPPORTED_RX_SIGNALS_INT    (2)

/******************************************************************************
   3. TYPES
*******************************************************************************/
typedef enum
{
  data_type_unknown = 0,
  uint16_1x_I,
  uint16_1x_IandQ,
  uint16_2x_I,
  uint16_2x_IandQ, // means two RX antenna, with I (real) and Q (imag) sampled, equal to 4 ADC channels, standard on Position2Go, ADC with 12 bit resolution
} acq_data_type;

typedef struct
{
  uint16_t num_of_samples_per_chirp;
  uint16_t num_of_chirps_per_frame;
  uint16_t num_of_rx_values;
  uint16_t size_of_value_in_bytes;
} acq_buf_radar_params_t;

typedef enum
{
  I_Sample = 0,
  Q_Sample = 1,
} signal_type;

typedef enum
{
  allocation_ok = 1,
  allocation_fails,            // This needs to raise a fatal error, if it occurs
  allocation_buffer_too_small, // This needs to raise a fatal error, if it occurs
  allocation_size_is_zero,
  num_of_RX_not_supported,
  allocation_is_freed,
  not_initialized,
} acq_buf_status;

typedef struct
{
  uint32_t size_of_acq_buffer; // holding the size of this data the acq_buffer points too, important for serialization of this structure
  size_t chirp_inc;
  size_t channel_inc;
  uint32_t is_dynamic_allocated;
} acq_buf_obj_intern;

typedef struct
{
  acq_data_type data_type; // an id, that identifies the data type in the acq_buffer, t.b.d. only the 4 LSBs identify the data_type, the other flags are free for future use!
                           // if the MSB (bit 31) is set, it means that the provided data is invalid, e.t.c. due to updated configuration or timing violations
  acq_buf_radar_params_t params;
  acq_buf_obj_intern internals;

  uint32_t used_size_of_acq_buffer;
  uint32_t acq_timestime; // acquisition time stamp, question, if the clock is maintained during sleep?
  volatile uint32_t access_lock; // is used as access_lock to the data buffer and could guard the access on resource limited systems, where only a few number of acq_buffers are available
  uint8_t *p_acq_buf;
  acq_buf_status status;
} acq_buf_obj;

/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/
/**
 * \brief The address of the internal buffer for the requested data type (I1, Q1, I2, Q2, ...) could be requested:
 *        a) to setup the ADC DMA target buffer address,
 *        b) to get the sample buffer from signal processing view.
 *        This mechanism, allows to be a bit more stable on the interface, while allowing certain flexibility, if needed.
 *
 * \param[in] *p_obj         A pointer to an acquisition buffer object
 * \param[in] channel_index  Channel index
 * \param[in] chirpCnt       Chirp counter
 *
 * \return Buffer address by chirp
 *
 */
uint8_t *get_buffer_address_by_chirp(acq_buf_obj *p_obj, uint32_t channel_index, uint32_t chirpCnt);

/**
 * \brief This function creates an acquisition buffer object.
 *
 * \param[in] *p_allocated_buffer    If NULL && if allocated_buffer_size is 0, the memory could be allocated dynamically by this function (see define USE_MALLOC_FOR_BUFFER)
 *                                   otherwise, if defined the location of the pre-allocated bufferA double point value for the PLL lower frequency for FMCW modulation
 * \param[in] allocated_buffer_size  If the buffer is pre-allocated, this buffer should have the size of the pre-allocated buffer.
 * \param[in] max_num_of_I_values    The number of used I values, if there are two RX channels, both sampling I and Q, this value would be 2, if only the real part is used the value is 1.
 * \param[in] max_num_of_Q_values    The number of used Q values, if there are two RX channels, both sampling I and Q, this value would be 2, if only the real part is used the value is 0.
 * \param[in] adc_resolution         The ADC resolution that is used. That defines the number of bytes required for an acquisition.
 *
 * \return An acquisition buffer object
 *
 */
acq_buf_obj create_acq_data_buffer_obj(uint8_t *p_allocated_buffer, size_t allocated_buffer_size,
		                               uint16_t max_num_of_chirps_per_frame, uint16_t max_num_of_samples_per_chirp, uint16_t max_num_of_I_values, uint16_t max_num_of_Q_values, uint16_t adc_resolution);

/**
 * \brief This function deletes the acquisition buffer object
 *
 * \param[in] *p_obj  A pointer to an acquisition buffer object
 *
 */
void delete_acq_buffer_obj(acq_buf_obj *p_obj);


/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_WATCHDOG_H_ */

/* --- End of File --- */
