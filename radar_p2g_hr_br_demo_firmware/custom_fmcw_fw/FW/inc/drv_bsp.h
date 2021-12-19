/**
 *
 *  drv_bsp.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_BSP_H_
#define FW_INC_DRV_BSP_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
#include <types.h>
#include "drv_pll.h"
#include "drv_pga.h"
#include "drv_bgt.h"
#include "FW/inc/drv_acq_buffer.h"

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/
#define DOPPLER_MODULATION	(0U)
#define FMCW_MODULATION		(1U)

#define USE_POSITION2GO_BOARD

#define BSP_NUM_TX_ANTENNAS        BGT24MTR1X_NUM_TX_ANTENNAS      /**< TX antennas in BGT24MTR12 */

#define BSP_NUM_RX_ANTENNAS        BGT24MTR1X_NUM_RX_ANTENNAS      /**< RX antennas in BGT24MTR12 */

#define BSP_NUM_TEMP_SENSORS       BGT24MTR1X_NUM_TEMP_SENSORS     /**< Number of temperature sensors in BGT24MTR12 */

#define BSP_MAX_TX_POWER_LEVEL     BGT24MTR1X_MAX_TX_POWER_LEVEL   /**< Maximum BGT TX output power SPI input value range is [1 - 7] supported by BGT24MTR1x. */

#define BSP_MIN_RF_FREQUENCY_KHZ   BGT24MTR1X_MIN_RF_FREQUENCY_KHZ /**< Minimum RF frequency supported by BGT24MTR1x in kHz */

#define BSP_MAX_RF_FREQUENCY_KHZ   BGT24MTR1X_MAX_RF_FREQUENCY_KHZ /**< Maximum RF frequency supported by BGT24MTR1x in kHz */

#define BSP_TGT_WAVE_LENGTH_MM                  (12.4f)       /**< Wave length at center frequency of 24.13GHz */

#define BSP_TGT_ANTENNA_SPACING_MM   		(6.22f)		/**< Distance between RX1 and RX2 [6.22mm] in Position2Go hardware V1.2 */

#define	BSP_NUM_OF_ADC_CHANNELS                 (4U)          /**< Number of ADC channels to be stored in the Flash */

#define BSP_NUM_OF_AVAILABLE_RADAR_DEVICES      (1U)          /**< The number of available radar devices */

#define BSP_DOPPLER_BASE_FREQ_MHZ               (24120U)      /**< Base frequency for Doppler */

#define BSP_REFERENCE_OSC_FREQ_HZ               (40000000U)   /**< Reference Oscillator frequency for PLL */

#define BSP_MIN_BANDWIDTH_MHZ                   (1.0f)        /**< Minimum bandwidth supported by the firmware */

#define BSP_MAX_BANDWIDTH_MHZ                   (200.0f)      /**< Maximum bandwidth supported by the firmware */

#define BSP_MIN_CHIRP_TIME_USEC                 (50U)         /**< Minimum chirp time, supported by the firmware (units in us) */

#define BSP_MAX_CHIRP_TIME_USEC                 (3000U)       /**< Maximum chirp time, supported by the firmware (units in us) */

#define BSP_MAX_ADC_BUFFER_SIZE_BYTES           (32768U)      /**< Maximum size of buffer to hold the raw ADC samples in bytes */

#define BSP_MAX_NUM_CHIRPS_PER_FRAME            (200U)         /**< Maximum supports up to 16 ramps, but a single cycle uses up to 2 ramp sections */

#define BSP_MIN_NUM_SAMPLES_PER_CHIRP           (16U)         /**< Minimum Samples per chirp supported */

#define BSP_MAX_NUM_SAMPLES_PER_CHIRP           (256U)        /**< Maximum Samples per chirp supported */

#define BSP_RX_ANTENNA_MASK                     (0x03U)       /**< Selection of RX Antenna */

#define BSP_ADC_RESOLUTION                      (12U)         /**< Number of bits per ADC data sample */

#define BSP_MAX_ADC_FREQ_HZ                     (1500000U)    /**< Maximum sampling frequency of the XMC ADC */

#define BSP_MAX_DMA_BLOCK_SIZE                  (4095U)       /**< Maximum block size defined by the XMC DMA */

#define BSP_MAX_ANGLE_OFFSET_DEG                (30)          /**< Maximum angle offset supported by BGT24MTR1x (units in degree), data type is int16_t */

#define BSP_MIN_ANGLE_OFFSET_DEG                (-30)         /**< Minimum angle offset supported by BGT24MTR1x (units in degree), data type is int16_t */

#define BSP_MAX_RANGE_OFFSET_CM                 (100U)        /**< Maximum range offset supported by BGT24MTR1x (units in cm) */

#define BSP_MAX_RANGE_CM                        (5000U)       /**< Maximum range supported (units in cm), 50m */

#define BSP_MAX_SPEED_KMH                       (20U)         /**< Maximum speed supported (units in km/h), 20km/h */

#define BSP_MAX_RANGE_THRESHOLD                 (1000U)       /**< Maximum range detection threshold in terms of absolute amplitude FFT spectrum */

#define BSP_MAX_SPEED_THRESHOLD                 (1000U)       /**< Maximum speed detection threshold in terms of absolute amplitude FFT spectrum */

#define BSP_MAX_NUM_OF_TRACKS                   (5U)		  /**< Maximum number detected targets to be tracked */

#define	BSP_MAX_NUM_OF_TARGETS                  (5U)          /**< Maximum number of targets to be detected, [1 - 5] */

#define BSP_MAX_MTI_FILTER_LEN                  (1000U)       /**< Maximum MTI Filter weight */

#define BSP_HOST_TIMEOUT_USEC                   (10*1000000U) 	/**< 10s timeout period for Host Application (in units of us) */

#define BSP_USB_TIMEOUT                         (0x0FFFU)       /**< Maximum attempt count to connect with the Host USB */

/******************************************************************************
   3. TYPES
*******************************************************************************/
typedef enum
{
  BSP_ACQUISITION_STARTED_CB_ID   = 1U,    /**< Start of data acquisition callback ID */
  BSP_ACQUISITION_DONE_CB_ID      = 2U     /**< End of data acquisition callback ID */

} Bsp_Callback_ID_t;

/**
 * This is the callback types for BSP layer.
 *
 * This callback is used to inform upper layers about the data acquisition status,
 * this information will be used by radar control and algorithm to start data processing.
 *
 * The user must register the callback functions after calling BSP_init.
 *
 * Whenever a callback is issued, the following parameters are passed to the callback:
 *
 */
typedef struct
{
  /* User callback functions */
  void (*bsp_acq_started_cb)(void *parameter);          /**< Callback executed when starting data acquisition process,
                                                             it is used to inform upper layers about the beginning of data acquisition process */

  void (*bsp_acq_done_cb)(void *parameter);             /**< Callback executed at the end of data acquisition,
                                                             it is used to inform upper layers that data sampling is done and data is ready in user buffer */

} Bsp_callback_Struct_t;

/*
==============================================================================
   4. DATA
==============================================================================
 */

extern TIMER_t TIMER_ADC_TRIG;

/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/

/**
 * \brief   This function configures BGT24MTR1x, LMX249x and PGA112 devices.
 *
 * \return  Returns "0" if successful else "1" for failure.
 */
uint8_t bsp_init(void);

/**
 * \brief   This function is used to trigger a new frame acquisition.
 */
void bsp_trigger_new_frame(void);

/**
 * \brief   This function defines the BGT and PLL power-up protocol with their respective delays during duty cycle mode
 */
void bsp_components_power_up(void);

/**
 * \brief   This function defines the BGT, PLL and PGA power-down protocol with their respective delays during duty cycle mode
 */
void bsp_components_power_down(void);

/**
 * \brief This function returns the current temperature info of BGT24MTR1x chip in C°.
 *
 * \return The current temperature info of BGT24MTR1x chip in C°
 */
float32_t bsp_bgt_get_temp_value(void);

/**
 * \brief This function returns the current TX power of BGT24MTR1x chip in dbm.
 *
 * \return The current Tx power of BGT24MTR1x chip in dbm
 */
float32_t bsp_bgt_get_txpower_value(void);

/**
 * \brief   This function is used to write an array of data to a SRAM buffer, mapped for the EEPROM contents.
 *
 * It will overwrite the contents of SRAM. Need to call the \ref bsp_flash_flush_data() to push the contents of SRAM to the EEPROM emulated on the flash
 *
 * \param[in]  offset_address  Offset Address in terms of bytes from where to starts writing, reference is 0
 * \param[in]  data_ptr  Pointer to unsigned 8-bit memory, which is to be written on the EEPROM
 * \param[in]  num_of_bytes  Unsigned 8-bit value, representing the number of data bytes to be flashed on the EEPROM
 */
void bsp_flash_write_data(const uint16_t offset_address, const uint8_t* data_ptr, uint16_t num_of_bytes);

/**
 * \brief   This function is used to read data from EEPROM emulated on the flash to an array.
 *
 * \param[in]  offset_address  Offset Address in terms of bytes from where to starts reading, reference is 0
 * \param[in]  num_of_bytes  Unsigned 8-bit value, representing the number of data bytes to be read from EEPROM
 * \param[out]  *data_ptr  1 Pointer to unsigned 8-bit memory, which read data from EEPROM to be copied
 */
void bsp_flash_read_data(const uint16_t offset_address, const uint8_t* data_ptr, uint16_t num_of_bytes);

/**
 * \brief   This function is used to push data to physical flash section emulated as EEPROM from SRAM.
 */
void bsp_flash_flush_data(void);

/**
 * \brief   This function returns the status of EEPROM emulated on the physical flash memory, either empty or not.
 *
 * \return  Boolean value showing the current status of the EEPROM
 */
bool bsp_flash_is_empty(void);

/**
 * \brief   This function is used to set the timer initialization flag to true, it is needed for Doppler mode.
 */
void bsp_timer_init(void);

/**
 * \brief   This function is used stop timer and clear its register.
 *
 * \param[in]  handle_timer  Timer handle pointer of type \ref TIMER_t
 *
 * \return  TIMER_STATUS_t  success or failure will be return
 */
TIMER_STATUS_t bsp_timer_stop_clear(TIMER_t* const handle_timer);

/**
 * \brief   This function is used start a timer.
 *
 * \param[in]  handle_timer  Timer handle pointer of type \ref TIMER_t
 *
 * \return  TIMER_STATUS_t  success or failure will be return
 */
TIMER_STATUS_t bsp_timer_start(TIMER_t* const handle_timer);

/**
 * \brief   This function is used create a timer delay in units of microseconds.
 *
 * \param[in]  delay_val_usec  Unsigned 32-bit value, representing the timer delay value in units of microseconds (ms)
 */
void bsp_time_delay_usec(uint32_t delay_val_usec);

/**
 * \brief   This function assigns memory sections to the source pointers defined in the DMA APP.
 */
void bsp_dma_set_source_addr(void);

/**
 * \brief   This function sets the destination address pointers for each DMA transfer channel I1, Q1, I2 and Q2.
 *
 * \param[in]  DMA_dst_I1  Unsigned 32-bit pointer for DMA destination address for ADC_I1 channel
 * \param[in]  DMA_dst_Q1  Unsigned 32-bit pointer for DMA destination address for ADC_Q1 channel
 * \param[in]  DMA_dst_I2  Unsigned 32-bit pointer for DMA destination address for ADC_I2 channel
 * \param[in]  DMA_dst_Q2  Unsigned 32-bit pointer for DMA destination address for ADC_Q2 channel
 */
void bsp_dma_set_destination_addr(uint32_t* DMA_dst_I1, uint32_t* DMA_dst_Q1, uint32_t* DMA_dst_I2, uint32_t* DMA_dst_Q2);

#if PORTING_COMPLETE
void bsp_dma_set_destination_addr_from_acq_buf_obj(acq_buf_obj *acq_buf, uint32_t chirp_cnt);
#endif

/**
 * \brief   This function initializes the USB low layer driver and starts USB data communication.
 *
 * \param[in]  timeout_delay  Unsigned 32-bit counter value, representing number of attempts to be made to connect to the VCOM at the Host
 *
 * \return  Returns "0" if successful else "1" for failure.
 */
uint8_t bsp_usb_init(uint32_t timeout_delay);

/**
 * \brief   This function returns the number of available BGT24 radar devices.
 *
 * \return  Returns "0" if no devices otherwise >= 1.
 */
uint8_t bsp_get_number_of_available_radar_devices(void);

/**
 * \brief This function sets the number of samples to captured during each chirp
 *        This value will be used to configure the DMA block size and destination address
 *
 * \param[in]  num_samples_per_chirp   The number of samples to be captured during each chirp
 */
void bsp_set_num_samples_per_dma_transfer(uint32_t num_samples_per_dma_transfer);

/**
 * \brief This function sets the number of chirps count per frame.
 *
 * \param[in]  num_chirps_per_frame  The number of chirps count per frame
 */
void bsp_set_num_chirps_per_frame(uint32_t num_chirps_per_frame);

/**
 * \brief This function returns the frame count with current frame interval.
 *
 * \return The current frame counter
 */
uint16_t bsp_get_frame_counter(void);

/**
 * \brief This function sets the frame count to default value 0.
 */
void bsp_reset_frame_counter(void);

/**
 * \brief This function sets the radar duty cycle flag to enable or disable status.
 *
 * \param[in]	flag 	True to enable duty cycle else false (zero value).
 */
void bsp_set_duty_cycle_enable_flag(const uint8_t flag);

/**
 * \brief	This function registers user callback functions in BSP callback structure \ref Bsp_callback_Struct_t.
 *          Note: This function must be called after bsp_init() function call
 *
 * \param[in] 	cb_id		BSP callback ID \ref Bsp_Callback_ID_t
 * \param[in] 	pcallback	pointer to BSP callback function, can be one of \ref Bsp_callback_Struct_t
 *
 * \return  Returns "0" if successful else "1" for failure.
 */
uint8_t bsp_register_callback(Bsp_Callback_ID_t cb_id, void (*pcallback)(void *parameter));

/**
 * \brief	This function unregisters user callback functions from radar object structure \ref Bsp_callback_Struct_t.
 *
 * \param[in] 	cb_id		BSP callback ID \ref Bsp_Callback_ID_t
 *
 * \return  Returns "0" if successful else "1" for failure.
 */
uint8_t bsp_unregister_callback(Bsp_Callback_ID_t cb_id);

/**
 * \brief	This function turns On the LED D8 mounted on Position2Go board.
 *          The D8 LED is connected to P7_8 IO pin
 */
void bsp_led_on(void);

/**
 * \brief	This function turns Off the LED D8 mounted on Position2Go board.
 *          The D8 LED is connected to P7_8 IO pin
 */
void bsp_led_off(void);

/**
 * \brief This function toggles On/Off the LED D8 mounted on Position2Go board.
 */
void bsp_led_toggle(void);

/**
 * \brief   This function is used stop and clear timer register.
 *
 * \param[in]	handle_timer	 Timer handle pointer of type \ref TIMER_t
 *
 * \return  TIMER_STATUS_t	success or failure will be return
 *
 */
TIMER_STATUS_t timer_stop_clear(TIMER_t* const handle_timer);


/**
 * \brief This function initializes the stream interface.
 *
 * This function does everything needed to prepare the communication interface
 * for operation. This function is called once during protocol initialization.
 */
void com_init();

/**
 * \brief This function sends a buffer of data through the communication
 *        interface.
 *
 * The function sends the data in the provided buffer to the host through the
 * communication interface. This function may use an internal transmit buffer
 * and copy the data to that buffer instead of sending it directly. By using
 * an internal transmit buffer data from consecutive calls to this function
 * can be combined to a single transaction to reduce overhead. The function
 * \ref com_flush can be used to send all data from the internal buffer
 * immediately.
 *
 * \param[in] data       A pointer to the buffer that contains the data to be
 *                       sent.
 * \param[in] num_bytes  The number of bytes in the buffer to be sent.
 */
void com_send_data(const void* data, uint16_t num_bytes);

/**
 * \brief This function retrieves data that has been received through the
 *        communication interface.
 *
 * This function tries to retrieve the requested number of bytes from the
 * internal receive buffer and copies them to the provided buffer. If the
 * internal receive buffer does not contain enough data this function will not
 * wait but copy all data from the receive buffer and return the number of
 * bytes that have been copied to the provided buffer.
 *
 * \param[in] data       A pointer to the buffer where the received data will
 *                       be copied to.
 * \param[in] num_bytes  The number of bytes that should be copied to data.
 *                       It is assumed that the capacity of data is
 *                       sufficient.
 *
 * \return The function returns the number of bytes that have been copied to
 *         the buffer.
 */
uint16_t com_get_data(void* data, uint16_t num_bytes);

/**
 * \brief This function sends all data from the internal transmit buffer.
 *
 * If an implementation of the communication interfaces contain an internal
 * transmit buffer, this functions sends all data from that buffer to the
 * host. After a call to this function the transmit buffer is empty.
 */
void com_flush();



/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_BSP_H_ */

/* --- End of File --- */
