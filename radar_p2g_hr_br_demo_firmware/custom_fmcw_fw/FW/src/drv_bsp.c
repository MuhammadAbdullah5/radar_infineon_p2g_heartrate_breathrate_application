/**
 *
 *  drv_bsp.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/drv_bsp.h"
#include "FW/inc/drv_timer.h"
#include "USB/USBD/usbd.h"
#include "FW/inc/radar_control.h"
#include "FW/inc/data_store.h"

#define BSP_TGT_ANTENNA_SPACING_MM   		(6.22f)		/**< Distance between RX1 and RX2 [6.22mm] in Position2Go hardware V1.2 */

/******************************************************************************
   2. DATA
*******************************************************************************/
/**
 * @brief Used to store the DMA source addresses.
 */
uint32_t* pDMA_src_I1 = NULL;
uint32_t* pDMA_src_Q1 = NULL;
uint32_t* pDMA_src_I2 = NULL;
uint32_t* pDMA_src_Q2 = NULL;

/**
 * @brief Used to store the DMA destination addresses.
 */
uint32_t* pDMA_dst_I1 = NULL;
uint32_t* pDMA_dst_Q1 = NULL;
uint32_t* pDMA_dst_I2 = NULL;
uint32_t* pDMA_dst_Q2 = NULL;

extern const XMC_CCU4_SLICE_COMPARE_CONFIG_t CCU4_SLICE_CONFIG_ADC_TRIG_compare_config;

/**
 * @brief The Radar duty cycle state enabled/disabled.
 */
static uint8_t bsp_duty_cycle_enable = DUTY_CYCLE_ENABLE;

/**
 * @brief The frame count with current frame interval.
 */
static volatile uint16_t bsp_frame_counter = 0;

/**
 * @brief The number of samples captured during each chirp, it holds the DMA block size per ADC channel
 */
static uint32_t bsp_num_samples_per_dma_transfer = 0;

/**
 * @brief The current temperature info of BGT chip in C°.
 */
static float32_t bsp_bgt_temperature_C = 0;

/**
 * @brief The current output power by BGT chip in dBm.
 */
static float32_t bsp_bgt_vout_tx_power_dBm = 0;

/**
 * @brief This buffer is used to measure the current output power and temperature of BGT.
 */
static uint16_t bsp_bgt_ana_out[3];

/**
 * @brief  Handle used to control all PLL settings.
 */
Lmx249x_Object_t  	Lmx249x_pll;
void* pll_handle = &Lmx249x_pll;

/**
 * \brief  Assign an ADC timer to CCU4 slice3, to trigger ADC at sampling period based on PLL trigger output signals
 *         Rising  edge of PLL output signal is used to start of ADC timer
 *         Falling edge of PLL output signal is used to stop of ADC timer
 *
 */
TIMER_t TIMER_ADC_TRIG =
{
  .ccu4_slice_ptr         = (XMC_CCU4_SLICE_t*) CCU43_CC43,
  .ccu4_slice_number      = 3U,
  .time_interval_value_us = 100000U,
  .timer_max_value_us     = 1533893500U,
  .timer_min_value_us     = 10U,
  .global_ccu4_handler    = (GLOBAL_CCU4_t*)&GLOBAL_CCU4_0,
  .ccu4_slice_config_ptr  = (XMC_CCU4_SLICE_COMPARE_CONFIG_t*)&CCU4_SLICE_CONFIG_ADC_TRIG_compare_config,
  .shadow_mask            = (uint32_t)((uint32_t)XMC_CCU4_SHADOW_TRANSFER_SLICE_3 |
                                       (uint32_t)XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_3),
  .ccu4_period_match_node = XMC_CCU4_SLICE_SR_ID_3,
  .timer_module           = TIMER_MODULE_CCU4,
  .period_value           = 34999U,
  .start_control          = false,
  .period_match_enable    = true,
  .initialized            = false
};

/**
 * @brief Global structure to store all data acquisition functions callback \ref Bsp_callback_Struct_t.
 */
static Bsp_callback_Struct_t bsp_cb =
{
  .bsp_acq_started_cb = NULL,
  .bsp_acq_done_cb = NULL
};

/**
 * @brief  The current watchdog status.
 */
static uint8_t watchdog_status = 0;

/**
 * @brief  The current chrip counter.
 */
static uint32_t chirp_count = 0;

/******************************************************************************
   3. LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
/**
 * \brief  This function is used to get the raw ADC data for Doppler or FMCW.
 *
 * This is called by \ref radar_get_frame() or by automatic trigger set by \ref radar_set_automatic_frame_trigger().
 * First Doppler data is acquired and then afterwards FMCW data is acquired.
 * It checks if ADC timer is stopped and the time interval for ADC timer for Doppler data acquisition is set successfully.
 */
static void get_raw_data(void);

/**
 * \brief  This function is called after data acquisition is finished to set global flags and put device to power-down mode
 */
static void done_sampling(void);

/**
 * \brief  This function sets the block size for each DMA transfer according to the number of samples per chirp
 *
 * \param[in]  block_size   If samples per chirp changes, then new block size in DMA
 */
static void dma_set_block_size(uint32_t block_size);

/**
 * \brief  This function enables the DMA to get further DMA data transactions.
 *         Need to call after reloading the DMA configurations.
 */
static void dma_enable(void);

/**
 * \brief  This function disables the DMA to avoid further DMA data transactions.
 *	   Need to call before reloading the DMA configurations or when data acquisition is finished.
 */
static void dma_disable(void);

/**
 * \brief  This function resets the DMA channel DLR to kill the last over sampled sample.
 */
static void dma_reset_dlr(void);

/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/
uint8_t bsp_init(void)
{
  Pga_Status_t status;

  /* Set all data acquisition functions callback to NULL */
  bsp_cb.bsp_acq_started_cb = NULL;
  bsp_cb.bsp_acq_done_cb = NULL;

#if (FW_MODULATION_TYPE == 0U) /* Doppler modulation */
  bsp_timer_init();
#endif

  /* Set memory sections to DMA source pointers */
  bsp_dma_set_source_addr();

  /* Enable LDO for BGT */
  bgt_ldo_enable();
  bsp_time_delay_usec(1000U);

  /* Enable LDO for PGA */
  pga_ldo_enable();
  bsp_time_delay_usec(1000U);

  /* Power-up BGT */
  bgt_power_up();
  bsp_time_delay_usec(100U); /* Delay for BGT power-up */

  bgt_lowest_power_with_q2_disable(); /* To avoid out of band spurs */
  bsp_time_delay_usec(50U); /* Delay for BGT Power down SPI settings */

  /* Enable PLL power supply */
  pll_power_up();
  bsp_time_delay_usec(100U); /* Delay for PLL power-up */

  /* Initialize BGT */
  bgt_init(LNA_GAIN_ENABLE, BGT_TX_POWER);

  /* Initialize PLL */
  pll_init(pll_handle);
  bsp_time_delay_usec(100U);

  pll_update_configuration(pll_handle, FMCW_MODULATION);

  bsp_time_delay_usec(100U);

  /* Initialize PGA and set its gain */
  status = pga_init(PGA_GAIN);
  bsp_time_delay_usec(50U);

  /* Disable DMA */
  dma_disable();

  /* Check if duty cycle is enabled */
  if(bsp_duty_cycle_enable == ENABLED)
  {
    bsp_components_power_down();
  }
  else	/* duty cycle disabled */
  {
    bgt_start_tx();
  }

  /* Reset the current frame counter */
  bsp_reset_frame_counter();

  return (status);
}

//============================================================================

void triggerFrameISR(void)
{
  bsp_trigger_new_frame();
}

//============================================================================

void bsp_trigger_new_frame(void)
{
  /* Stop ADC timer if ADC is still running */
  if (TIMER_GetTimerStatus(&TIMER_ADC_TRIG))
  {
    bsp_timer_stop_clear(&TIMER_ADC_TRIG);
  }

  /* Get the raw ADC data */
  get_raw_data();

  /* This callback is used to inform upper layers about the beginning of data acquisition process */
  if(bsp_cb.bsp_acq_started_cb != NULL)
  {
    bsp_cb.bsp_acq_started_cb(NULL);
  }
}

//============================================================================

void DMA_Event_Handler(XMC_DMA_CH_EVENT_t dma_event)
{
  if((dma_event & XMC_DMA_CH_EVENT_BLOCK_TRANSFER_COMPLETE) != 0)
  {
    /* Once DMA issue interrupts, we'd better stop the ADC Trigger Timer,
    * we don't need to care which interrupts are triggered by DMA.
    * this timer is source peripheral's trigger to request DMA transaction.
    * Even though DMA error interrupt, we'd better stop the this timer. */
    bsp_timer_stop_clear(&TIMER_ADC_TRIG);

    chirp_count++;

    dma_disable();

    /* Clear the request from source peripheral even though DMA was disabled by itself.
    * We should consider the pending request from external peripheral. */
    dma_reset_dlr();

#if (FW_MODULATION_TYPE == 1U) /* FMCW modulation */

#if PORTING_COMPLETE
    acq_buf_obj *p_acq_buf = ds_get_active_acq_buf();

    if (chirp_count < p_acq_buf->params.num_of_chirps_per_frame)
#endif
    {

#if PORTING_COMPLETE
      /* Reconfiguration of DMA destination offset */
      bsp_dma_set_destination_addr_from_acq_buf_obj(p_acq_buf, chirp_count);
#endif
      /* Enable DMA */
      dma_enable();
    }
#if PORTING_COMPLETE
    else
#endif
#endif
    {
      ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_SCAN);

      bsp_frame_counter++;

      chirp_count = 0;

      done_sampling();
    }
  }
}

//============================================================================

void bsp_components_power_up(void)
{
  /* Power-up BGT */
  bgt_power_up();

  /* Delay for BGT power-up */
  bsp_time_delay_usec(100U);

  bgt_lowest_power_with_q2_disable();
  bsp_time_delay_usec(400U); /* 400us based on PLL pre-charging slope, dependent on board loop filter */

  /* Enable PLL */
  pll_power_up();

  /* Delay for PLL power-up.
   * This delay depends upon the Charge pump voltage for locking time (linked with first BGT power-up delay) */
  bsp_time_delay_usec(100U);

  if (pll_get_update_config_flag() == ENABLED)
  {
	  pll_update_configuration(pll_handle, FMCW_MODULATION);
  }

#if (FW_MODULATION_TYPE == 1U) /* FMCW modulation */
  else
  {
    pll_enable_ramps(pll_handle); /* Enable PLL's ramps */
  }
#endif

  bsp_time_delay_usec(500U);

#if PORTING_COMPLETE
  /* Set the new PGA gain level */
  if(ds_device_get_settings()->isGainlevelUpdated)
  {
    pga_set_gain(ds_device_get_settings()->pga_rx_gain_level);
    ds_device_get_settings()->isGainlevelUpdated = 0;
  }

  bsp_time_delay_usec(PLL_LOCK_TIME_USEC); /* Delay for PLL Lock */

  /* Enable BGT TX Power amplifier */
  bgt_start_tx();

  /* Delay for BGT Q2 divider to be zero and VCO to be settled after SPI setting transmitted */
  bsp_time_delay_usec(BGT_VCO_LOCK_TIME_USEC); /* 1ms delay for IQ voltage at ADC close to reference 1.65V */
#endif
}

//============================================================================

void bsp_components_power_down(void)
{
  pll_disable_ramps(pll_handle);

  /* To avoid out of band spurs */
  bgt_lowest_power_with_q2_disable();
  bsp_time_delay_usec(50U);           /* Delay for BGT Power down SPI settings */

  /* Power-down BGT */
  bgt_power_down();

  /* Disable PLL */
  pll_power_down();
}

//============================================================================

void bsp_flash_write_data(const uint16_t offset_address, const uint8_t* data_ptr, uint16_t num_of_bytes)
{
  E_EEPROM_XMC4_WriteArray(offset_address, data_ptr, num_of_bytes);
}

//============================================================================

void bsp_flash_read_data(const uint16_t offset_address, const uint8_t* data_ptr, uint16_t num_of_bytes)
{
  E_EEPROM_XMC4_ReadArray(offset_address, (uint8_t*)data_ptr, num_of_bytes);
}

//============================================================================

void bsp_flash_flush_data(void)
{
  E_EEPROM_XMC4_UpdateFlashContents();
}

//============================================================================

bool bsp_flash_is_empty(void)
{
  return (E_EEPROM_XMC4_IsFlashEmpty());
}

//============================================================================

void bsp_timer_init(void)
{
  TIMER_ADC_TRIG.initialized = true;
}

//============================================================================

TIMER_STATUS_t bsp_timer_stop_clear(TIMER_t* const handle_timer)
{
  TIMER_STATUS_t  status;

  status = TIMER_Stop(handle_timer);

  status |= TIMER_Clear(handle_timer);

  return (status);
}

TIMER_STATUS_t bsp_timer_start(TIMER_t* const handle_timer)
{
  TIMER_STATUS_t  status;

  status = TIMER_Start(handle_timer);

  return (status);
}

//============================================================================

void bsp_time_delay_usec(uint32_t delay_val_usec)
{
  uint32_t delay_cnt;

  TIMER_ClearEvent(&TIMER_DELAY);

  /* time in in units of microseconds */
  delay_cnt = delay_val_usec * 100U;

  TIMER_SetTimeInterval(&TIMER_DELAY,delay_cnt);

  bsp_timer_start(&TIMER_DELAY);

  while(!TIMER_GetInterruptStatus(&TIMER_DELAY));

  TIMER_Stop(&TIMER_DELAY);

  TIMER_Clear(&TIMER_DELAY);
}

//============================================================================

void bsp_dma_set_source_addr(void)
{
  /* DMA Source/Destination Address setup */
  pDMA_src_I1 = (uint32_t*)&ADC_MEASUREMENT_ADV_G2_IF1_Q_RES;
  pDMA_src_Q1 = (uint32_t*)&ADC_MEASUREMENT_ADV_G1_IF1_I_RES;
  pDMA_src_I2 = (uint32_t*)&ADC_MEASUREMENT_ADV_G4_IF2_Q_RES;
  pDMA_src_Q2 = (uint32_t*)&ADC_MEASUREMENT_ADV_G3_IF2_I_RES;

  /* Set the source address pointers for each DMA transfer */
  XMC_DMA_CH_SetSourceAddress(DMA_CH_I1.dma_global->dma, DMA_CH_I1.ch_num, (uint32_t)pDMA_src_I1);
  XMC_DMA_CH_SetSourceAddress(DMA_CH_Q1.dma_global->dma, DMA_CH_Q1.ch_num, (uint32_t)pDMA_src_Q1);
  XMC_DMA_CH_SetSourceAddress(DMA_CH_I2.dma_global->dma, DMA_CH_I2.ch_num, (uint32_t)pDMA_src_I2);
  XMC_DMA_CH_SetSourceAddress(DMA_CH_Q2.dma_global->dma, DMA_CH_Q2.ch_num, (uint32_t)pDMA_src_Q2);
}

//============================================================================

void bsp_dma_set_destination_addr_from_acq_buf_obj(
#if PORTING_COMPLETE
		acq_buf_obj *acq_buf,
#endif
		uint32_t chirp_cnt
		)
{
#if PORTING_COMPLETE
  pDMA_dst_I1 = (uint32_t*) get_buffer_address_by_chirp(acq_buf, 0, chirp_cnt);
  pDMA_dst_Q1 = (uint32_t*) get_buffer_address_by_chirp(acq_buf, 1, chirp_cnt);
  pDMA_dst_I2 = (uint32_t*) get_buffer_address_by_chirp(acq_buf, 2, chirp_cnt);
  pDMA_dst_Q2 = (uint32_t*) get_buffer_address_by_chirp(acq_buf, 3, chirp_cnt);

  XMC_DMA_CH_SetDestinationAddress(DMA_CH_I1.dma_global->dma, DMA_CH_I1.ch_num, (uint32_t) pDMA_dst_I1);
  XMC_DMA_CH_SetDestinationAddress(DMA_CH_Q1.dma_global->dma, DMA_CH_Q1.ch_num, (uint32_t) pDMA_dst_Q1);
  XMC_DMA_CH_SetDestinationAddress(DMA_CH_I2.dma_global->dma, DMA_CH_I2.ch_num, (uint32_t) pDMA_dst_I2);
  XMC_DMA_CH_SetDestinationAddress(DMA_CH_Q2.dma_global->dma, DMA_CH_Q2.ch_num, (uint32_t) pDMA_dst_Q2);
#endif
}

//============================================================================

uint8_t bsp_usb_init(uint32_t timeout_delay)
{
  uint32_t time_out_cnt = 0;

  if (USBD_VCOM_Connect() != USBD_VCOM_STATUS_SUCCESS)
  {
    return (1); /* Fail */
  }

  while (time_out_cnt < timeout_delay)
  {
    if (USBD_VCOM_IsEnumDone())
    {
      return (0); /* Success */
    }

    time_out_cnt++;
  }

  return (1); /* Fail */
}

//============================================================================

uint8_t bsp_get_number_of_available_radar_devices(void)
{
  /* The number of available radar devices is defined in the BSP layer */
  return (BSP_NUM_OF_AVAILABLE_RADAR_DEVICES);
}

//============================================================================

float32_t bsp_bgt_get_temp_value(void)
{
  return (bsp_bgt_temperature_C);
}

//============================================================================

float32_t bsp_bgt_get_txpower_value(void)
{
  return (bsp_bgt_vout_tx_power_dBm);
}

//============================================================================

void bsp_set_num_samples_per_dma_transfer(uint32_t num_samples_per_dma_transfer)
{
  bsp_num_samples_per_dma_transfer = num_samples_per_dma_transfer;
}

//============================================================================

void bsp_set_num_chirps_per_frame(uint32_t num_chirps_per_frame)
{
  pll_set_num_chirps_per_frame(num_chirps_per_frame);
}

//============================================================================

uint16_t bsp_get_frame_counter(void)
{
  return (bsp_frame_counter);
}

//============================================================================

void bsp_reset_frame_counter(void)
{
  bsp_frame_counter = 0;
}

//============================================================================

void bsp_set_duty_cycle_enable_flag(const uint8_t flag)
{
  bsp_duty_cycle_enable = flag;
}

//============================================================================

uint8_t bsp_register_callback(Bsp_Callback_ID_t cb_id, void (*pcallback)(void *parameter))
{
  uint8_t status = 0;

  /* Check if he callback function is valid */
  if(pcallback == NULL)
  {
    return(1);
  }

  /* Register user callback function based on callback ID */
  switch (cb_id)
  {
  case BSP_ACQUISITION_STARTED_CB_ID:
    bsp_cb.bsp_acq_started_cb = pcallback;
    break;

  case BSP_ACQUISITION_DONE_CB_ID:
    bsp_cb.bsp_acq_done_cb = pcallback;
    break;

  default:
    /* Return error status */
    status =  1;
    break;
  }

  return (status);
}

//============================================================================

uint8_t bsp_unregister_callback(Bsp_Callback_ID_t cb_id)
{
  uint8_t status = 0;

  /* Unregister user callback function based on callback ID */
  switch (cb_id)
  {
  case BSP_ACQUISITION_STARTED_CB_ID:
    bsp_cb.bsp_acq_started_cb = NULL;
    break;

  case BSP_ACQUISITION_DONE_CB_ID:
    bsp_cb.bsp_acq_done_cb = NULL;
    break;

  default:
    /* Return error status */
    status =  1;
    break;
  }

  return (status);
}

//============================================================================

void bsp_led_on(void)
{
  DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_LED1);
}

//============================================================================

void bsp_led_off(void)
{
  DIGITAL_IO_SetOutputLow(&DIGITAL_IO_LED1);
}

//============================================================================

void bsp_led_toggle(void)
{
  DIGITAL_IO_ToggleOutput(&DIGITAL_IO_LED1);
}

/******************************************************************************
   5. LOCAL FUNCTIONS
*******************************************************************************/

static void get_raw_data(void)
{
  watchdog_status = 0;

  /* DMA Reconfiguration */
  dma_disable();
  dma_set_block_size(bsp_num_samples_per_dma_transfer);
#if PORTING_COMPLETE
  acq_buf_obj *p_acq_buf = ds_get_active_acq_buf();

  if (chirp_count == 0)
  {
	  bsp_dma_set_destination_addr_from_acq_buf_obj(p_acq_buf, 0 );
  }
  else
  {
	  bsp_dma_set_destination_addr_from_acq_buf_obj(p_acq_buf, chirp_count );
  }
#endif

  dma_enable();

  /* BGT and PLL Power-up */
  if(bsp_duty_cycle_enable == ENABLED)
  {
    bsp_components_power_up();
  }
  else
  {
    if (pll_get_update_config_flag() == ENABLED)
    {
      pll_update_configuration(pll_handle, FMCW_MODULATION);
    }

#if (FW_MODULATION_TYPE == 1U) /* FMCW modulation */
    else
    {
      pll_enable_ramps(pll_handle); /* Enable PLL's ramps */
    }
#endif /* FMCW modulation */

    bgt_start_tx();
    bsp_time_delay_usec(50);
  }

#if PORTING_COMPLETE
  /* Set WatchDog Timer Period */
  TIMER_SetTimeInterval(&TIMER_WATCHDOG, ds_device_get_settings()->pll_num_of_chirps_per_frame * (ds_device_get_settings()->pll_chirp_time_usec + PLL_RAMP_DOWM_TIME_USEC + 5 * PLL_STEADY_STATE_USEC) * 100U);
#endif

  /* PLL Chirp Trigger */
#if (FW_MODULATION_TYPE == 1U) /* FMCW modulation */
  pll_trigger_ramp();
#endif /* FMCW modulation */

  pll_release_ramp_trigger();

  /* Start WatchDog timer to check the timings of chirps generated by PLL */
  bsp_timer_start(&TIMER_WATCHDOG);
}

//============================================================================

static void done_sampling(void)
{
  uint16_t  lbgt_ana_command = 0;

  if(bsp_duty_cycle_enable == ENABLED)
  {
    bsp_components_power_down();
  }

  lbgt_ana_command = bgt_get_ana_config();
  bsp_bgt_ana_out[lbgt_ana_command] = (ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_SCAN_BGT24_ANA_handle));

  if (bsp_bgt_ana_out[2] == 0)
  {
    bgt_ana_vref_tx();
  }
  else if (lbgt_ana_command == 2)
  {
    bgt_ana_vout_tx();
  }
  else
  {
    bgt_ana_temp();
  }

  /* Calculate the BGT temperature and output power based on the BGT ANA MUX pin ADC sampled values */
  bsp_bgt_temperature_C = (0.2629f * (1000 * 3.3f * bsp_bgt_ana_out[0] / 4095.0f) - 364.32f);

  if((bsp_bgt_ana_out[1] - bsp_bgt_ana_out[2]) >= 0)
  {
    bsp_bgt_vout_tx_power_dBm = 5.24f * logf(1000 * 3.3f / 4095.0f * (bsp_bgt_ana_out[1] - bsp_bgt_ana_out[2])) - 23.74f;
  }

  /* This callback is used to inform upper layers that a valid data is available and ADC sampling is completed */
  if((bsp_cb.bsp_acq_done_cb != NULL) & (watchdog_status == 0))
  {
    bsp_cb.bsp_acq_done_cb(NULL);
  }
}

void com_init(void)
{
  /* Don't do anything here as the init must have been done already when calling dave_init */
}

//============================================================================

void com_send_data(const void* pData, uint16_t uNumBytes)
{
  USBD_VCOM_SendData(pData,uNumBytes);
}

//============================================================================

uint16_t com_get_data(void* pData, uint16_t uNumBytes)
{
  uint16_t bytes = 0;

  bytes = USBD_VCOM_BytesReceived();

  bytes = (bytes > uNumBytes) ? uNumBytes : bytes;

  if (bytes)
  {
    USBD_VCOM_ReceiveData((int8_t*)pData, bytes);
  }

  return (bytes);
}

//============================================================================

void com_flush(void)
{
  CDC_Device_USBTask(&USBD_VCOM_cdc_interface);	/* Do nothing as USBD VCOM already flushes the data that is sent */
}

//============================================================================

static void dma_set_block_size(uint32_t block_size)
{
  /* The maximum block size defined by DMA is 4095 */
  block_size = (block_size > BSP_MAX_DMA_BLOCK_SIZE) ? BSP_MAX_DMA_BLOCK_SIZE : block_size;

  /* Set block size */
  DMA_CH_SetBlockSize(&DMA_CH_I1, block_size);
  DMA_CH_SetBlockSize(&DMA_CH_Q1, block_size);
  DMA_CH_SetBlockSize(&DMA_CH_I2, block_size);
  DMA_CH_SetBlockSize(&DMA_CH_Q2, block_size);
}

//============================================================================

static void dma_enable(void)
{
  DMA_CH_Enable(&DMA_CH_I1);
  DMA_CH_Enable(&DMA_CH_Q1);
  DMA_CH_Enable(&DMA_CH_I2);
  DMA_CH_Enable(&DMA_CH_Q2);
}

//============================================================================

static void dma_disable(void)
{
  DMA_CH_Disable(&DMA_CH_I1);
  DMA_CH_Disable(&DMA_CH_Q1);
  DMA_CH_Disable(&DMA_CH_I2);
  DMA_CH_Disable(&DMA_CH_Q2);
}

//============================================================================

static void dma_reset_dlr(void)
{
  XMC_DMA_CH_ClearSourcePeripheralRequest(DMA_CH_I1.dma_global->dma, DMA_CH_I1.ch_num);
  XMC_DMA_CH_ClearSourcePeripheralRequest(DMA_CH_Q1.dma_global->dma, DMA_CH_Q1.ch_num);
  XMC_DMA_CH_ClearSourcePeripheralRequest(DMA_CH_I2.dma_global->dma, DMA_CH_I2.ch_num);
  XMC_DMA_CH_ClearSourcePeripheralRequest(DMA_CH_Q2.dma_global->dma, DMA_CH_Q2.ch_num);
}

//============================================================================

void watchdog_isr(void)
{
#if PORTING_COMPLETE
  if(chirp_count < (ds_device_get_settings()->pll_num_of_chirps_per_frame-1))
  {
    watchdog_status = 1;
  }

  timer_stop_clear(&TIMER_WATCHDOG);
#endif
}

//============================================================================

TIMER_STATUS_t timer_stop_clear(TIMER_t* const handle_timer)
{
  TIMER_STATUS_t  status;

  status = TIMER_Stop(handle_timer);

  status |= TIMER_Clear(handle_timer);

  return status;
}

/* --- End of File --- */

