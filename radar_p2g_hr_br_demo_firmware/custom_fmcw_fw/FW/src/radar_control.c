/**
 *
 *  radar_control.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/radar_control.h"
#include "FW/inc/data_store.h"
#include "FW/inc/drv_bsp.h"

/******************************************************************************
   2. DATA
*******************************************************************************/
extern float result_range_detection_threshold;

/* Global constant structure to store the device info, this only holds compile time defines, so no need to store it in RAM! */
const Device_Info_t cg_device_info_struct = {
  .description           = DEMO_DESCRIPTION,
  .major_version_hw      = HW_VERSION_MAJOR,
  .minor_version_hw      = HW_VERSION_MINOR,
  .num_tx_antennas       = BSP_NUM_TX_ANTENNAS,
  .num_rx_antennas       = BSP_NUM_RX_ANTENNAS,
  .num_temp_sensors      = BSP_NUM_TEMP_SENSORS,
  .max_tx_power          = BSP_MAX_TX_POWER_LEVEL,
  .min_rf_frequency_kHz  = BSP_MIN_RF_FREQUENCY_KHZ,
  .max_rf_frequency_kHz  = BSP_MAX_RF_FREQUENCY_KHZ,
  .interleaved_rx        = 0,
  .data_format 	         = RADAR_RX_DATA_COMPLEX
};


volatile  uint32_t    g_adc_sampling_completed = false;
volatile  uint32_t    g_do_processing = true;

Radar_Data_Acq_Mode_t  s_data_acq_mode = RADAR_MANUAL_TRIGGER;
uint16_t   adc_i1_calibration[BSP_NUM_OF_ADC_CHANNELS * MAX_CALIB_SAMPLES_PER_CHANNEL];
uint16_t*  adc_q1_calibration = &adc_i1_calibration[MAX_CALIB_SAMPLES_PER_CHANNEL * 1];
uint16_t*  adc_i2_calibration = &adc_i1_calibration[MAX_CALIB_SAMPLES_PER_CHANNEL * 2];
uint16_t*  adc_q2_calibration = &adc_i1_calibration[MAX_CALIB_SAMPLES_PER_CHANNEL * 3];
Algo_Calibrations_t  algo_calibration_s = {0,0};

size_t frame_size = 0;

/* Global Data Buffer used to store the raw IQ data samples for FMCW chirps,
   first (SAMPLES_PER_CHIRP * 2) samples are for I&Q data for RX1,
   followed by (SAMPLES_PER_CHIRP * 2) samples for I&Q data for RX2 */
uint32_t data_fifo[BSP_MAX_ADC_BUFFER_SIZE_BYTES/sizeof(uint32_t)];

/* Global buffer to have a copy of fresh ADC data to avoid frame corruption at Host */
uint32_t transport_buffer[BSP_MAX_ADC_BUFFER_SIZE_BYTES/sizeof(uint32_t)];

static const Driver_Version_t driver_version_s =
{
  /*.uMajor    =*/ FW_VERSION_MAJOR,
  /*.uMinor    =*/ FW_VERSION_MINOR,
  /*.uRevision =*/ FW_VERSION_REVISION
};


extern Radar_Handle_t  h_radar_device;
extern Radar_Handle_t h_radar_device;

/******************************************************************************
   3. LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
/**
 * \brief  This function saves the calibration data to the SRAM or Flash based on the target memory defined by argument \ref Calibration_Target_t.
 *
 * \param[in]	target	Target of the calibration defined by \ref Calibration_Target_t.
 */
static void save_adc_calibration(Calibration_Target_t target);

/**
 * \brief  This function reads the calibration data from the SRAM or Flash based on the target memory defined by argument \ref Calibration_Target_t.
 *
 * \param[in]	target	Target of the calibration defined by \ref Calibration_Target_t.
 *
 * \return		Number of bytes occupied by calibration samples for all ADC channels
 */
static uint32_t read_adc_calibration(Calibration_Target_t target);

/**
 * \brief  This function erase calibration data from the SRAM or Flash based on the target memory defined by argument \ref Calibration_Target_t.
 *
 * \param[in]	target	Target of the calibration defined by \ref Calibration_Target_t.
 */
static void clear_adc_calibration(Calibration_Target_t target);

/**
 * \brief  This function saves the algorithm related calibrations in the  SRAM/Flash memory based on target argument type.
 *
 * \param[in]	target					Target of the calibration defined by \ref Calibration_Target_t.
 * \param[in]	*algo_calibration_ptr	A const pointer to the structure defined by \ref Algo_Calibrations_t.
 */
static void save_algo_calibration(Calibration_Target_t target, const Algo_Calibrations_t* algo_calibration_ptr);

/**
 * \brief  This function reads the algorithm related calibrations from SRAM/Flash memory based on target argument type.
 *
 * \param[in]	target	Target memory of the calibration defined by \ref Calibration_Target_t.
 *
 * \return		Number of bytes used in algo calibration structure
 */
static uint32_t read_algo_calibration(Calibration_Target_t target);

/**
 * \brief  This function clears the algorithm related calibrations from SRAM/Flash memory based on target argument type.
 *
 * \param[in]	target	Target memory of the calibration defined by \ref Calibration_Target_t.
 */
static void clear_algo_calibration(Calibration_Target_t target);

/**
 * \brief  This function writes the calibration data to the Flash.
 */
static void write_calibration_flash_data(void);

/**
 * \brief  This is a helper function to sets the angle offset compensation to get zero angle for target in front of the radar.
 *
 * \param[in]	offset_deg	Signed integer 16-bit value between [-30, +30] in unit of degrees.
 */
static void set_angle_offset(int16_t offset_deg);

/**
 * \brief  This is a helper function to sets the range offset compensation.
 *
 * \param[in]	offset_cm	Unsigned int16 signed value between [0, 100] in units of cm.
 */
static void set_range_offset(uint16_t offset_cm);

/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/

uint16_t radar_set_dsp_settings(Radar_Handle_t device, const DSP_Settings_t* configuration)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */
  algo_settings_t new_settings;
  memset(&new_settings, 0, sizeof(algo_settings_t));

  /* Fetch the complete existing data, to update only partially the new data. */
  ds_algo_fetch_settings(&new_settings);

#ifdef FMCW_SUPPORTED
  new_settings.tracking_enable 		  	 = configuration->enable_tracking;
  new_settings.mti_filter_enable	 	 = configuration->enable_mti_filter;
  new_settings.num_of_tracks			 = configuration->num_of_tracks;
  new_settings.mti_filter_len			 = configuration->mti_filter_length;
  new_settings.median_filter_len		 = configuration->median_filter_length;
  new_settings.mvg_avg_len				 = configuration->range_mvg_avg_length;
  new_settings.min_distance_cm			 = configuration->min_range_cm;
  new_settings.max_distance_cm			 = configuration->max_range_cm;
  new_settings.range_detection_threshold = configuration->range_threshold;
#endif

#if defined(DOPPLER_SUPPORTED) || defined(FMCW_SUPPORTED)
  new_settings.min_speed_kmh = configuration->min_speed_kmh;
  new_settings.max_speed_kmh = configuration->max_speed_kmh;
  new_settings.speed_detection_threshold = configuration->speed_threshold;
#endif

  /* Check for new settings */
  if (ds_algo_check_settings(&new_settings) == 0)
  {
    /* Apply new settings */
    ds_algo_store_settings(&new_settings);
    return (RADAR_ERR_OK);
  }
  else
  {
    return (RADAR_ERR_PARAMETER_OUT_OF_RANGE);
  }
}

//============================================================================

uint16_t radar_get_dsp_settings(Radar_Handle_t device, DSP_Settings_t* configuration)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */
  algo_settings_t *p_ds_config = ds_algo_get_settings();

  configuration->enable_tracking      = p_ds_config->tracking_enable;
  configuration->enable_mti_filter    = p_ds_config->mti_filter_enable;
  configuration->num_of_tracks        = p_ds_config->num_of_tracks;
  configuration->mti_filter_length    = p_ds_config->mti_filter_len;
  configuration->median_filter_length = p_ds_config->median_filter_len;
  configuration->range_mvg_avg_length = p_ds_config->mvg_avg_len;
  configuration->min_range_cm         = p_ds_config->min_distance_cm;
  configuration->max_range_cm         = p_ds_config->max_distance_cm;
  configuration->range_threshold      = p_ds_config->range_detection_threshold;

  configuration->min_speed_kmh        = p_ds_config->min_speed_kmh;
  configuration->max_speed_kmh        = p_ds_config->max_speed_kmh;
  configuration->speed_threshold      = p_ds_config->speed_detection_threshold;

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_target_info(Radar_Handle_t device, Target_Info_t* target_info, uint8_t* target_count)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  ds_ep_fetch_target_list(target_info, target_count);
  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_range_detection_threshold(Radar_Handle_t device, uint16_t *threshold)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  ds_ep_fetch_range_detection_threshold(threshold);
  return (RADAR_ERR_OK);
}

uint16_t radar_set_gain_level(Radar_Handle_t device, uint16_t gain_level)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  if (gain_level > MAX_PGA_GAIN_LEVEL)
  {
    return (RADAR_ERR_UNSUPPORTED_PGA_GAIN);
  }

  /* Update the PGA Gain level */
  ds_device_get_settings()->pga_rx_gain_level = gain_level;
  ds_device_get_settings()->isGainlevelUpdated = 1;

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_gain_level(Radar_Handle_t device, uint16_t* gain_level)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  /* Get current PGA Gain level from driver */
  *gain_level = ds_device_get_settings()->pga_rx_gain_level;

  return (RADAR_ERR_OK);
}

uint16_t radar_set_duty_cycle(Radar_Handle_t device, const uint8_t flag)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  if (flag != ds_device_get_settings()->power_duty_cycle_enable_flag)
  {
    ds_device_get_settings()->is_duty_cycle_enable_updated = 1;
    ds_device_get_settings()->power_duty_cycle_enable_flag = flag;
  }

  return (RADAR_ERR_OK);
}

//============================================================================

void radar_get_duty_cycle(Radar_Handle_t device, uint8_t* flag)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  *flag = ds_device_get_settings()->power_duty_cycle_enable_flag;
}

//============================================================================

void radar_disable_lna_gain(Radar_Handle_t device)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  ds_device_get_settings()->bgt_rx_lna_gain_enable_flag = false;
}

//============================================================================

void radar_enable_lna_gain(Radar_Handle_t device)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  ds_device_get_settings()->bgt_rx_lna_gain_enable_flag = true;
}

//============================================================================

uint8_t radar_get_lna_gain_enable_status(Radar_Handle_t device)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  return (ds_device_get_settings()->bgt_rx_lna_gain_enable_flag);
}

uint16_t radar_set_fmcw_configuration(Radar_Handle_t device,
                                      const Fmcw_Configuration_t* configuration)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  float tmp_bandwidth_mhz;

  /* Configuration change is only allowed while automatic trigger is not working */
  if (radar_get_data_acq_mode() != RADAR_MANUAL_TRIGGER)
  {
    return (RADAR_ERR_BUSY);
  }

  /* Check if configuration is valid */
  /* ------------------------------- */
  if ((configuration->lower_frequency_kHz < BSP_MIN_RF_FREQUENCY_KHZ) ||
      (configuration->lower_frequency_kHz > BSP_MAX_RF_FREQUENCY_KHZ) ||
        (configuration->upper_frequency_kHz < BSP_MIN_RF_FREQUENCY_KHZ) ||
          (configuration->upper_frequency_kHz > BSP_MAX_RF_FREQUENCY_KHZ) ||
            (configuration->lower_frequency_kHz > configuration->upper_frequency_kHz))
  {
    return (RADAR_ERR_FREQUENCY_OUT_OF_RANGE);
  }

  tmp_bandwidth_mhz = ((float)(configuration->upper_frequency_kHz - configuration->lower_frequency_kHz)) / 1000.0f;

  if ((tmp_bandwidth_mhz < BSP_MIN_BANDWIDTH_MHZ) || (tmp_bandwidth_mhz > BSP_MAX_BANDWIDTH_MHZ))
  {
    return (RADAR_ERR_FREQUENCY_OUT_OF_RANGE);
  }

  if (configuration->direction != RADAR_DIR_UPCHIRP_ONLY)
  {
    return (RADAR_ERR_UNSUPPORTED_DIRECTION);
  }

  if (configuration->tx_power > BSP_MAX_TX_POWER_LEVEL)
  {
    return (RADAR_ERR_POWER_OUT_OF_RANGE);
  }

  ds_ep_store_fmcw_config(configuration);

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_fmcw_configuration(Radar_Handle_t device,
                                      Fmcw_Configuration_t* configuration)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  /* Get the current FMCW configuration from data store */
  ds_ep_fetch_fmcw_config(configuration);
  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_bandwidth_per_second(Radar_Handle_t device,
                                        uint32_t* bandwidth_per_second_MHz_s)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  uint32_t remainder;
  uint32_t divisor;
  Frame_Format_t frame_format;

  /* Get the current FMCW configuration from data store */
  ds_ep_fetch_frame_format(&frame_format);

  /* Unit is MHz/s. */
  /*
  * Actual calculation "uBandwidthPerSecond = (uUpperFMCWFrequency - uLowerFMCWFrequency) * 1000000 / chirp_duration"
  * is split up to avoid overflow of 32 bit words
  */
  remainder = (uint32_t)((ds_device_get_settings()->pll_upper_frequency_kHz - ds_device_get_settings()->pll_lower_frequency_kHz) *
              (BSP_REFERENCE_OSC_FREQ_HZ / 1000000));

  uint32_t sample_rate_divider = BSP_REFERENCE_OSC_FREQ_HZ / ds_device_get_settings()->adc_sampling_freq_Hz;

  divisor = frame_format.num_samples_per_chirp * sample_rate_divider;

  *bandwidth_per_second_MHz_s = remainder / divisor;

  remainder -= *bandwidth_per_second_MHz_s * divisor;

  *bandwidth_per_second_MHz_s *= 1000;

  *bandwidth_per_second_MHz_s += (remainder * 1000) / divisor;

  return (RADAR_ERR_OK);
}

uint16_t radar_set_doppler_configuration(Radar_Handle_t device, const Doppler_Configuration_t* const configuration)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  /* Configuration change is only allowed while automatic trigger is not working */
  if (radar_get_data_acq_mode() != RADAR_MANUAL_TRIGGER)
  {
    return (RADAR_ERR_BUSY);
  }

  /* Check if configuration is valid */
  /* ------------------------------- */
  if((configuration->frequency_kHz < BSP_MIN_RF_FREQUENCY_KHZ) ||
     (configuration->frequency_kHz > BSP_MAX_RF_FREQUENCY_KHZ))
  {
    return (RADAR_ERR_FREQUENCY_OUT_OF_RANGE);
  }

  if (configuration->tx_power > BSP_MAX_TX_POWER_LEVEL)
  {
    return (RADAR_ERR_POWER_OUT_OF_RANGE);
  }

  /* Copy the new Doppler configuration to the data store*/
  ds_ep_store_doppler_config(configuration);

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_doppler_configuration(Radar_Handle_t device, Doppler_Configuration_t* configuration)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  /* Get the current Doppler configuration from data store */
  ds_ep_fetch_doppler_config(configuration);
  return (RADAR_ERR_OK);
}

uint16_t radar_init(Radar_Handle_t device)
{
  uint16_t status = RADAR_ERR_BUSY;
  uint16_t num_of_bytes;
  void* calibration_data_ptr = NULL;

  /* PLL parameters setup */
  /* -------------------- */
  float temp_pll_lower_freq_MHz = ds_device_get_settings()->pll_lower_frequency_kHz / 1000.0f;
  float temp_pll_upper_freq_MHz = ds_device_get_settings()->pll_upper_frequency_kHz / 1000.0f;

  /* Set PLL upper and lower frequencies for FMCW modulation */
  pll_set_upper_lower_frequency(temp_pll_lower_freq_MHz, temp_pll_upper_freq_MHz);

#if FW_MODULATION_TYPE == 1U

  ds_device_get_settings()->pll_modulation_mode = MODULATION_FMCW;
  status = radar_set_pll_frequency(device, temp_pll_upper_freq_MHz);

#else

  ds_device_get_settings()->pll_modulation_mode = MODULATION_DOPPLER;
  status = radar_set_pll_frequency(device, BSP_DOPPLER_BASE_FREQ_MHZ);

#endif

  /* BGT, PLL, PGA, USB and DMA Setup */
  /* -------------------------------- */

  acq_buf_obj *p_acq_buf = ds_get_active_acq_buf();

  bsp_dma_set_destination_addr_from_acq_buf_obj(p_acq_buf, 0);

  status |= (uint16_t) bsp_init();

  /* Load calibration data & Timers setup */
  /* ------------------------------------ */
  radar_init_calibration(device);

  /* Read ADC calibration data */
  status |= radar_read_calibration(device, CALIBRATION_TARGET_FLASH, CALIBRATION_DATA_ADC, &calibration_data_ptr, &num_of_bytes);

  /* Read algorithm calibration data */
  status |= radar_read_calibration(device, CALIBRATION_TARGET_FLASH, CALIBRATION_DATA_ALGO, &calibration_data_ptr, &num_of_bytes);

  status |= radar_set_sampling_freq(device, ds_device_get_settings()->pll_modulation_mode, ds_device_get_settings()->adc_sampling_freq_Hz);

  return (status);
}

//============================================================================

uint16_t radar_start_acquisition(Radar_Handle_t device)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */
  uint16_t status = RADAR_ERR_BUSY;

  if (ds_device_get_settings()->frame_period_usec > 0)
  {
    status = radar_set_automatic_frame_trigger(h_radar_device, ds_device_get_settings()->frame_period_usec);
  }

  return (status);
}

//============================================================================

uint16_t radar_stop_acquisition(Radar_Handle_t device)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */
  uint16_t status = RADAR_ERR_BUSY;

  while(g_adc_sampling_completed != true);

  status = radar_set_automatic_frame_trigger(h_radar_device , 0);

  return (status);
}

//============================================================================

uint16_t radar_set_bandwidth(Radar_Handle_t device, const float bandwidth_MHz)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  if ((bandwidth_MHz > 0) && (bandwidth_MHz <= BSP_MAX_BANDWIDTH_MHZ))
  {
    ds_device_get_settings()->pll_bandwidth_MHz = bandwidth_MHz;

    pll_set_bandwidth(bandwidth_MHz);
    pll_set_update_config_flag(true);

    return (RADAR_ERR_OK);
  }
  else
  {
    return (RADAR_ERR_PARAMETER_OUT_OF_RANGE);
  }
}

//============================================================================

void radar_get_bandwidth(Radar_Handle_t device, float* bandwidth_MHz)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  *bandwidth_MHz = ds_device_get_settings()->pll_bandwidth_MHz;
}

//============================================================================
uint16_t radar_set_pll_frequency(Radar_Handle_t device, float freq_MHz)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  if ((freq_MHz >= BSP_MIN_RF_FREQUENCY_KHZ/1000.0f) && (freq_MHz <= BSP_MAX_RF_FREQUENCY_KHZ/1000.0f))
  {
    pll_set_frequency(freq_MHz);
    pll_set_update_config_flag(true);

    return (RADAR_ERR_OK);
  }
  else
  {
    return (RADAR_ERR_PARAMETER_OUT_OF_RANGE);
  }
}

//============================================================================

void radar_get_pll_frequency(Radar_Handle_t device, float* freq_MHz)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  *freq_MHz = pll_get_frequency();
}

//============================================================================

static uint16_t radar_check_chirp_time(const uint32_t chirp_time_usec)
{
  if ((ds_device_get_settings()->pll_modulation_mode == MODULATION_FMCW) &&
      (chirp_time_usec >= BSP_MIN_CHIRP_TIME_USEC) && (chirp_time_usec <= BSP_MAX_CHIRP_TIME_USEC))
  {
    if ((chirp_time_usec < 300 && ds_device_get_settings()->num_samples_per_chirp > 256) ||
        (chirp_time_usec < 150 && ds_device_get_settings()->num_samples_per_chirp > 128) ||
        (chirp_time_usec < 75  && ds_device_get_settings()->num_samples_per_chirp > 64))
    {
      return (RADAR_ERR_PARAMETER_OUT_OF_RANGE);
    }
  }
  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_set_chirp_time(Radar_Handle_t device, const uint32_t chirp_time_usec)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  uint16_t retValue = radar_check_chirp_time(chirp_time_usec);

  if(retValue)
  {
    return retValue;
  }

  /* ---- chirp time was verified being o.k. */
  ds_device_get_settings()->pll_chirp_time_usec = chirp_time_usec;

  /* Set the current PLL chirp-time, in units of usec */
  pll_set_chirp_time(ds_device_get_settings()->pll_chirp_time_usec);

  bsp_timer_stop_clear(&TIMER_ADC_TRIG);

  uint32_t sampling_period_usec = (uint32_t)(((float)ds_device_get_settings()->pll_chirp_time_usec * 100.0f) / (float)ds_device_get_settings()->num_samples_per_chirp);

  if (TIMER_SetTimeInterval(&TIMER_ADC_TRIG, sampling_period_usec) != TIMER_STATUS_SUCCESS)
  {
    return (RADAR_ERR_PARAMETER_OUT_OF_RANGE);
  }

  /* Update the PLL Configuration */
  pll_set_update_config_flag(true);

  return (RADAR_ERR_OK);
}

//============================================================================

void radar_get_chirp_time(Radar_Handle_t device, uint32_t* chirp_time_usec)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  *chirp_time_usec = ds_device_get_settings()->pll_chirp_time_usec;
}

//============================================================================

uint16_t radar_set_frame_period(const uint32_t time_ms)
{
  return (radar_set_automatic_frame_trigger(h_radar_device, time_ms * 1000U));
}

//============================================================================

void radar_get_frame_period(Radar_Handle_t device, uint32_t* time_msec)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  *time_msec = (ds_device_get_settings()->frame_period_usec / 1000);
}

//============================================================================

uint16_t radar_set_sampling_freq(Radar_Handle_t device, Modulation_Type_t modulation_type, const float freq_Hz)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */
  TIMER_STATUS_t status = TIMER_STATUS_FAILURE;
  uint32_t sampling_period_usec = 0;

  if (((uint32_t)freq_Hz <= 0) || ((uint32_t)freq_Hz > BSP_MAX_ADC_FREQ_HZ))
  {
    return (RADAR_ERR_BUSY);
  }

  bsp_timer_stop_clear(&TIMER_ADC_TRIG);

  if (modulation_type == MODULATION_FMCW)
  {
    sampling_period_usec = (uint32_t)(((float) ds_device_get_settings()->pll_chirp_time_usec * 100.0f) / (float)ds_device_get_settings()->num_samples_per_chirp);
  }
  else
  {
    sampling_period_usec = (uint32_t)(1000.0f * 1000.0f * 100.0f / freq_Hz);
  }

  status = TIMER_SetTimeInterval(&TIMER_ADC_TRIG, sampling_period_usec);

  if (status == TIMER_STATUS_SUCCESS)
  {
    return (RADAR_ERR_OK);
  }
  else
  {
    return (RADAR_ERR_BUSY);
  }
}

//============================================================================

void radar_get_sampling_freq(Radar_Handle_t device, float* freq_Hz)
{
  XMC_UNUSED_ARG(device); ; /* Suppress compiler warnings of unused variable */

  *freq_Hz = (1000.0f * 1000.0f) * 100.0f / ds_device_get_settings()->adc_sampling_freq_Hz;
}

//============================================================================

uint16_t radar_register_callback(Radar_Callback_ID_t cb_id, void (*pcallback)(void *parameter))
{
  uint16_t status = RADAR_ERR_OK;

  /* Check if the callback function is valid */
  if(pcallback == NULL)
  {
    return(RADAR_ERR_BUSY);
  }

  /* Register user callback function based on callback ID */
  switch (cb_id)
  {
  case RADAR_ACQUISITION_STARTED_CB_ID:
    /* Callback executed by BSP layer at the beginning of data acquisition */
    bsp_register_callback(BSP_ACQUISITION_STARTED_CB_ID, pcallback);
    break;

  case RADAR_ACQUISITION_DONE_CB_ID:
    /* Callback executed by BSP layer at the end of data acquisition process */
    bsp_register_callback(BSP_ACQUISITION_DONE_CB_ID, pcallback);
    break;

  default:
    /* Return error status */
    status =  RADAR_ERR_BUSY;
    break;
  }

  return (status);
}

//============================================================================

uint16_t radar_unregister_callback(Radar_Callback_ID_t cb_id)
{
  uint16_t status = RADAR_ERR_OK;

  /* Unregister user callback function based on callback ID */
  switch (cb_id)
  {
  case RADAR_ACQUISITION_STARTED_CB_ID:
    bsp_unregister_callback(BSP_ACQUISITION_STARTED_CB_ID);
    break;

  case RADAR_ACQUISITION_DONE_CB_ID:
    bsp_unregister_callback(BSP_ACQUISITION_DONE_CB_ID);
    break;

  default:
    /* Return error status */
    status = RADAR_ERR_BUSY;
    break;
  }

  return (status);
}

//============================================================================

static uint32_t radar_apply_new_hw_setting(Radar_Handle_t device, device_settings_t *p_new)
{
  bsp_components_power_down();

  device_settings_t *p_curr = ds_device_get_shadow_settings();

  /* Set BGT RX LNA gain */
  if (p_new->bgt_rx_lna_gain_enable_flag == true)
  {
    bgt_lna_gain_enable();
  }
  else
  {
    bgt_lna_gain_disable();
  }

  if (p_curr->pll_chirp_time_usec != p_new->pll_chirp_time_usec || (p_curr->num_samples_per_chirp != p_new->num_samples_per_chirp))
  {
    if (ds_device_get_settings()->pll_modulation_mode == MODULATION_FMCW)
	{
      pll_set_chirp_time(p_new->pll_chirp_time_usec);
      pll_set_update_config_flag(true);

      uint32_t sampling_period_usec = (uint32_t)(((float) p_new->pll_chirp_time_usec * 100.0f) / (float)p_new->num_samples_per_chirp);

      if (TIMER_SetTimeInterval(&TIMER_ADC_TRIG, sampling_period_usec) != TIMER_STATUS_SUCCESS)
      {
        return (RADAR_ERR_PARAMETER_OUT_OF_RANGE);
      }
	}
  }

  if (p_curr->pll_num_of_chirps_per_frame != p_new->pll_num_of_chirps_per_frame)
  {
    /* Update the number of chirps count per frame */
    bsp_set_num_chirps_per_frame(p_new->pll_num_of_chirps_per_frame);
    pll_set_update_config_flag(true);
  }

  if (p_curr->num_samples_per_chirp != p_new->num_samples_per_chirp)
  {
    bsp_set_num_samples_per_dma_transfer(p_new->num_samples_per_chirp);
  }

  if (p_new->isUpdated_doppler_config)
  {
    float freq_MHz;

    /* Apply RF configuration */
    /* ---------------------- */
    bgt_set_tx_power(p_new->bgt_tx_power_level);

    freq_MHz = (float)(p_new->pll_frequency_kHz / 1000.0f); // to bring into the MHz

    if (ds_device_get_settings()->pll_modulation_mode == MODULATION_DOPPLER)
    {
      radar_set_pll_frequency(device, freq_MHz);
    }

    p_new->isUpdated_doppler_config = 0;
  }

  if(p_new->isUpdated_fmcw_config)
  {
    float bandwidth_mhz;
    float pll_base_frequency_mhz;

    /* Apply RF configuration */
    /* ---------------------- */
    bandwidth_mhz = ((float)(p_new->pll_upper_frequency_kHz - p_new->pll_lower_frequency_kHz)) / 1000.0f;

    pll_base_frequency_mhz = (float)(p_new->pll_upper_frequency_kHz) / 1000.0f;

    // This is redundant, but we leave it for now, needs to be tested!
    ds_device_get_settings()->pll_bandwidth_MHz = bandwidth_mhz;

    pll_set_frequency(pll_base_frequency_mhz);
    pll_set_bandwidth(bandwidth_mhz);
    pll_set_upper_lower_frequency(p_new->pll_lower_frequency_kHz / 1000.0f, p_new->pll_upper_frequency_kHz / 1000.0f);
    pll_set_update_config_flag(true);

    bgt_set_tx_power(p_new->bgt_tx_power_level);

    p_new->isUpdated_fmcw_config = 0;
  }

  /* Set the Radar duty cycle */
  if(p_new->is_duty_cycle_enable_updated)
  {
    bsp_set_duty_cycle_enable_flag(p_new->power_duty_cycle_enable_flag);
    p_new->is_duty_cycle_enable_updated = 0;
  }

  if ((p_curr->pll_num_of_chirps_per_frame != p_new->pll_num_of_chirps_per_frame) || (p_curr->num_samples_per_chirp != p_new->num_samples_per_chirp))
  {
    /* Recreate acq-buffer object! */
    /* The buffer is statically allocated to the max supported buffer size, so it is save to re-use it!
     * checks should have been applied earlier! */
    acq_buf_obj *p_acq_buf = ds_get_active_acq_buf();

    uint8_t *p_temp_buf = p_acq_buf->p_acq_buf;
    uint32_t reserved_size = p_acq_buf->internals.size_of_acq_buffer;

    acq_buf_obj acq_buf = create_acq_data_buffer_obj(p_temp_buf, reserved_size,
                                                     p_new->pll_num_of_chirps_per_frame,
                                                     p_new->num_samples_per_chirp, 2, 2, 12);  // 2 --> 2 I channels and two Q-channels, from two RX-antennas, 12-bit ADC resolution

    ds_set_active_acq_buf(acq_buf);
  }

  bsp_components_power_up();

  return 0; // confirm that everything was o.k.
}

//============================================================================

uint32_t radar_apply_hw_settings(Radar_Handle_t device, hw_state_setting_t state, device_settings_t *p_hw_settings)
{
  uint32_t retValue = RADAR_ERR_OK;

  switch(state)
  {
  case INIT:
    retValue = radar_apply_new_hw_setting(device, p_hw_settings);
    break;

  case ENTER_POWERSAVING:
    //bsp_... dear down
    break;

  case LEAVE_POWERSAVING:
    break;

  case POWER_DOWN:
    bgt_stop_tx();
    break;

  case UPDATE:
    if(ds_is_device_settings_updated())
    {
      /* stop the timer for ADC acquisition, while updating the values! */
      bsp_timer_stop_clear(&TIMER_ADC_TRIG);

      retValue = radar_apply_new_hw_setting(device, p_hw_settings);
    }
    break;

  case START_MANUAL_ACQ:
    {
      acq_buf_obj *p_acq_buf = ds_get_active_acq_buf();

      bsp_dma_set_destination_addr_from_acq_buf_obj(p_acq_buf, 0);

      bsp_reset_frame_counter();

      g_adc_sampling_completed = false;
    }
    break;

  case START_AUTO_ACQ:
    break;

  case STOP_ACQ:
    break;

  default:
    break;
  }

  return retValue;
}

//============================================================================

Radar_Data_Acq_Mode_t radar_get_data_acq_mode(void)
{
  return s_data_acq_mode;
}

//============================================================================

void radar_set_data_acq_mode(Radar_Data_Acq_Mode_t new_mode)
{
  s_data_acq_mode = new_mode;
}

void radar_init_calibration(Radar_Handle_t device)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  memset(adc_i1_calibration, DEFAULT_CALIBRATION_VALUE, ADC_CALIB_LEN_BYTES);
}

//============================================================================

uint16_t radar_save_calibration(Radar_Handle_t device, Calibration_Target_t target, Calibration_Data_t calib_data_type, const void* calibration_data_ptr)
{
  if (calib_data_type == CALIBRATION_DATA_ADC)
  {
    uint16_t status  = 1;
    Frame_Info_t frame_info;

    g_do_processing = false;

    while(status)
    {
      status = radar_get_frame(device, &frame_info, true);
    }

    save_adc_calibration(target);
    g_do_processing = true;
  }
  else
  {

    save_algo_calibration(target, (const Algo_Calibrations_t*)calibration_data_ptr);
  }

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_read_calibration(Radar_Handle_t device, Calibration_Target_t target, Calibration_Data_t calib_data_type, void** data_ptr, uint16_t* num_of_bytes)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  if (calib_data_type == CALIBRATION_DATA_ADC)
  {
    *num_of_bytes = read_adc_calibration(target);
    *data_ptr = adc_i1_calibration;
  }
  else
  {
    *num_of_bytes = read_algo_calibration(target);
    *data_ptr = &algo_calibration_s;
  }

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_clear_calibration(Radar_Handle_t device, Calibration_Target_t target, Calibration_Data_t calib_data_type)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  if (calib_data_type == CALIBRATION_DATA_ADC)
  {
    clear_adc_calibration(target);
  }
  else
  {
    clear_algo_calibration(target);
  }

  return (RADAR_ERR_OK);
}

const Driver_Version_t* radar_get_driver_version(void)
{
  return (&driver_version_s);
}

//============================================================================

uint8_t radar_get_number_of_available_devices(void)
{
  return bsp_get_number_of_available_radar_devices();
}

//============================================================================

uint16_t radar_open_device(int32_t device_number, Radar_Handle_t* device)
{
  XMC_UNUSED_ARG(device_number); /* Suppress compiler warnings of unused variable */

  Frame_Format_t frame_format;
  uint16_t retValue = RADAR_ERR_OK;
  int32_t status = 0;
  uint16_t num_of_bytes;
  void* calibration_data_ptr = NULL;
  acq_buf_obj acq_buf = { .status = not_initialized } ;

  /* Initialize the driver instance with NULL, until it was successfully initialized */
  *device = NULL;

  /* Initialize driver data structure */
  /* -------------------------------- */
  status = ds_init();
  status |= radar_read_calibration(device, CALIBRATION_TARGET_FLASH, CALIBRATION_DATA_ADC, &calibration_data_ptr, &num_of_bytes);
  if(status > 0)
  {
    return (RADAR_ERR_UNSUPPORTED_FRAME_FORMAT);
  }

  //========================================================================
  //------------------------- Frame Format configurations ------------------
  //========================================================================
  ds_ep_fetch_frame_format(&frame_format);

  //=========================================================================
  //------------------------- General configurations ------------------------
  //=========================================================================
  radar_set_data_acq_mode(RADAR_MANUAL_TRIGGER);

  /* If the frame format contains a 0, this makes no sense. */
  if ((frame_format.rx_mask == 0) ||
      (frame_format.num_chirps_per_frame  == 0) ||
        (frame_format.num_samples_per_chirp == 0) ||
          (frame_format.num_chirps_per_frame  > BSP_MAX_NUM_CHIRPS_PER_FRAME) ||
            (frame_format.num_samples_per_chirp > BSP_MAX_NUM_SAMPLES_PER_CHIRP))
  {
    return (RADAR_ERR_UNSUPPORTED_FRAME_FORMAT);
  }

  /* Check frame memory limits */
  /* ------------------------- */
  frame_size = frame_format.num_chirps_per_frame * frame_format.num_samples_per_chirp *
    ds_ep_get_device_info()->num_rx_antennas * 2U * sizeof(uint16_t); // complex IQ samples per antenna

  if (frame_size > BSP_MAX_ADC_BUFFER_SIZE_BYTES)
  {
    return (RADAR_ERR_UNSUPPORTED_FRAME_FORMAT);
  }

  /* Initialize sensor hardware */
  /* -------------------------- */
  acq_buf = create_acq_data_buffer_obj((uint8_t*) data_fifo, BSP_MAX_ADC_BUFFER_SIZE_BYTES,
                                       frame_format.num_chirps_per_frame,
                                       frame_format.num_samples_per_chirp, 2, 2, 12);

  ds_set_active_acq_buf(acq_buf);

  ds_device_get_settings()->num_samples_per_chirp = frame_format.num_samples_per_chirp;

  /* Set the number of samples to captured during each chirp
  This value will be used in BSP to configure the DMA block size and destination address */
  bsp_set_num_samples_per_dma_transfer(ds_device_get_settings()->num_samples_per_chirp);

  ds_device_get_settings()->pll_num_of_chirps_per_frame = frame_format.num_chirps_per_frame;

  /* Set the number of chirps count per frame */
  bsp_set_num_chirps_per_frame(frame_format.num_chirps_per_frame);

  /* BGT, PLL, DMA, Timers & Calibration initialization */
  /* -------------------------------------------------- */
  retValue = radar_init(device);

  g_adc_sampling_completed = false;

  return (retValue);
}

//============================================================================

void radar_close_device(Radar_Handle_t device)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  bgt_stop_tx();
}

//============================================================================

uint16_t radar_get_device_info(Radar_Handle_t device, Device_Info_t* device_info)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  /* Get device information:
      - Sensor description
      - Hardware Version information
      - Sensor features
      - Data format
  */
  *device_info = *ds_ep_get_device_info();

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_temperature(Radar_Handle_t device, uint8_t temp_sensor, int32_t* temperature_001C)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  /* Check temperature sensor, only single sensor supported */
  if (temp_sensor == 0)
  {
    *temperature_001C = (int32_t) (bsp_bgt_get_temp_value() * 1000.0f);	// (expected value is actual temperature in C° x 1000)

    return (RADAR_ERR_OK);
  }
  else
  {
    return (RADAR_ERR_SENSOR_DOES_NOT_EXIST);
  }
}

//============================================================================

uint16_t radar_get_tx_power(Radar_Handle_t device, uint8_t tx_antenna, int32_t* tx_power_001dBm)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  /* Check number of TX antenna */
  if (tx_antenna == 0)
  {
    *tx_power_001dBm = (int32_t)(bsp_bgt_get_txpower_value() * 1000.0f); // return TX power

    return (RADAR_ERR_OK);
  }

  return (RADAR_ERR_ANTENNA_DOES_NOT_EXIST);
}

//============================================================================

uint16_t radar_get_chirp_duration(Radar_Handle_t device, uint32_t* chirp_duration_nsec)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  /* Calculate chirp timing from current settings */
  /* -------------------------------------------- */
  /* Unit is ns. (factor 10^9 is distributed to denominator and denominator for fixed point precision) */

  *chirp_duration_nsec = (ds_device_get_settings()->pll_chirp_time_usec * 1000U);

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_min_frame_interval(Radar_Handle_t device, uint32_t* min_frame_interval_usec)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */
  uint32_t bgt_duty_cycle_delay_usec;

  if(ds_device_get_settings()->power_duty_cycle_enable_flag == true)
  {
    bgt_duty_cycle_delay_usec = 3000U; /* approximate value calculated from bgt power up sequence 100 + 400 + 100 + update configuration time + PLL_LOCK_TIME_us + BGT_VCO_LOCK_TIME_us */
  }
  else
  {
    bgt_duty_cycle_delay_usec = 1000U; /* approximate delay to accommodate processing time */
  }

#if defined (FMCW_SUPPORTED) && defined (DOPPLER_SUPPORTED)

  *min_frame_interval_usec = ds_device_get_settings()->pll_num_of_chirps_per_frame * (ds_device_get_settings()->pll_chirp_time_usec + PLL_RAMP_DOWM_TIME_USEC + PLL_STEADY_STATE_USEC) + bgt_duty_cycle_delay_usec +
    ((SAMPLES_PER_CHIRP * 1000U / DOPPLER_SAMPLING_FREQ_HZ) * 1000U) + 2 * ALGO_PROCESS_TIME_USEC; // units in us

#elif defined (FMCW_SUPPORTED)
  *min_frame_interval_usec = ds_device_get_settings()->pll_num_of_chirps_per_frame * (ds_device_get_settings()->pll_chirp_time_usec + PLL_RAMP_DOWM_TIME_USEC + 5*PLL_STEADY_STATE_USEC) + bgt_duty_cycle_delay_usec /*+ ALGO_PROCESS_TIME_USEC*/; // units in us

#elif defined (DOPPLER_SUPPORTED)
  *min_frame_interval_usec = ds_device_get_settings()->pll_num_of_chirps_per_frame * ((ds_device_get_settings()->num_samples_per_chirp * 1000U * 1000U) / DOPPLER_SAMPLING_FREQ_HZ) + bgt_duty_cycle_delay_usec + ALGO_PROCESS_TIME_USEC; // units in us

#else

  if(ds_device_get_settings()->pll_modulation_mode == MODULATION_FMCW)
  {
    *min_frame_interval_usec = ds_device_get_settings()->pll_num_of_chirps_per_frame * (ds_device_get_settings()->pll_chirp_time_usec + PLL_RAMP_DOWM_TIME_USEC + PLL_STEADY_STATE_USEC) + bgt_duty_cycle_delay_usec; // units in us
  }
  else /* Doppler modulation */
  {
    /* min_frame_interval_usec = Sampling period x No. of chirps x No. of samples per chirp x 1000000 (for microseconds)
    Sampling period =  1 / DOPPLER_SAMPLING_FREQ_Hz */
    *min_frame_interval_usec = ds_device_get_settings()->pll_num_of_chirps_per_frame * ((ds_device_get_settings()->num_samples_per_chirp * 1000U * 1000U) / DOPPLER_SAMPLING_FREQ_HZ) + bgt_duty_cycle_delay_usec; // units in us
  }

#endif

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_set_frame_format(Radar_Handle_t device, const Frame_Format_t* frame_format)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  /* Configuration change is only allowed while automatic trigger is not working */
  if (radar_get_data_acq_mode() != RADAR_MANUAL_TRIGGER)
  {
    return (RADAR_ERR_BUSY);
  }

  /* Check if configuration is valid */
  /* ------------------------------- */

  /* This sensor can only acquire real signal data */
  if (frame_format->signal_part != RADAR_SIGNAL_I_AND_Q)
  {
    return (RADAR_ERR_UNAVAILABLE_SIGNAL_PART);
  }

  /* This sensor can only support both RX antennas enabled */
  if (frame_format->rx_mask != BSP_RX_ANTENNA_MASK)
  {
    return (RADAR_ERR_ANTENNA_DOES_NOT_EXIST);
  }

  /* LMX2491 supports only a limited number of ramps before auto turn off. */
  if ((frame_format->num_chirps_per_frame == 0) || (frame_format->num_chirps_per_frame > BSP_MAX_NUM_CHIRPS_PER_FRAME))
  {
    return (RADAR_ERR_UNSUPPORTED_FRAME_FORMAT);
  }

  /* BGT can store only a limited number of samples per chirp */
  if ((frame_format->num_samples_per_chirp == 0) || (frame_format->num_samples_per_chirp > BSP_MAX_NUM_SAMPLES_PER_CHIRP))
  {
    return (RADAR_ERR_UNSUPPORTED_FRAME_FORMAT);
  }

  /* For Doppler modulation mode the number of chirp must be 1 */
  if ((frame_format->num_chirps_per_frame != 1) && (ds_device_get_settings()->pll_modulation_mode == MODULATION_DOPPLER))
  {
    return (RADAR_ERR_UNSUPPORTED_FRAME_FORMAT);
  }

  /* Check if samples per chirp are in [32,64,128,256] range */
  if ((frame_format->num_samples_per_chirp <= 10) ||
      (frame_format->num_samples_per_chirp >= 400))
  {
    return (RADAR_ERR_SAMPLERATE_OUT_OF_RANGE);
  }

  /* Check frame memory limits */
  /* ------------------------- */
  frame_size = frame_format->num_chirps_per_frame * frame_format->num_samples_per_chirp * 2U *
    ds_ep_get_device_info()->num_rx_antennas * sizeof(uint16_t); // complex IQ samples per antenna

  if (frame_size > BSP_MAX_ADC_BUFFER_SIZE_BYTES)
  {
    return (RADAR_ERR_UNSUPPORTED_FRAME_FORMAT);
  }

  /* Store new frame format in store */
  ds_ep_store_frame_format(frame_format);

  g_adc_sampling_completed = false;

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_frame_format(Radar_Handle_t device, Frame_Format_t* p_frame_format)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  /* Return current frame format of driver object */
  ds_ep_fetch_frame_format(p_frame_format);

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_frame(Radar_Handle_t device, Frame_Info_t* frame_info, uint8_t wait_for_data)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  Frame_Format_t frame_format;

  ds_ep_fetch_frame_format(&frame_format);

  acq_buf_obj *p_acq_buf = ds_get_active_acq_buf();

  if (radar_get_data_acq_mode() != RADAR_AUTO_TRIGGER)	// without automaticFrameTrigger
  {
    bsp_trigger_new_frame();
  }
  else // with automaticFrameTrigger
  {
    if ((!wait_for_data) && (!g_adc_sampling_completed))
    {
      return (RADAR_ERR_TIMEOUT);
    }
  }

  while (g_adc_sampling_completed == false); // block GetFrame if data is not ready

  /* Data acquired */
  /* ------------- */
  memcpy(transport_buffer, p_acq_buf->p_acq_buf, frame_size);	// copy ADC buffer to be send to avoid data corruption by Automatic trigger

  frame_info->sample_data           = transport_buffer;
  frame_info->num_rx_antennas       = ds_ep_get_device_info()->num_rx_antennas;
  frame_info->num_chirps            = frame_format.num_chirps_per_frame;
  frame_info->rx_mask               = frame_format.rx_mask;
  frame_info->adc_resolution        = ds_device_get_settings()->adc_resolution;
  frame_info->interleaved_rx        = 0;
  frame_info->frame_number          = bsp_get_frame_counter();
  frame_info->data_format           = RADAR_RX_DATA_COMPLEX;
  frame_info->temperature_001C      = (int32_t) (bsp_bgt_get_temp_value() * 1000);	// (expected value is actual temperature in C° x 1000)
  frame_info->num_samples_per_chirp = frame_format.num_samples_per_chirp;

  g_adc_sampling_completed          = false;

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_set_automatic_frame_trigger(Radar_Handle_t device, uint32_t frame_interval_usec)
{
  /* Stop trigger timer first  */
  /* ------------------------- */
  bsp_timer_stop_clear(&TIMER_FRAME_TRIG);

  radar_set_data_acq_mode(RADAR_MANUAL_TRIGGER);

  /* Restart trigger timer */
  /* --------------------- */
  if (frame_interval_usec > 0)
  {
    uint32_t min_frame_interval_usec;

    radar_get_min_frame_interval(device, &min_frame_interval_usec);
    //frame_interval_usec = min_frame_interval_usec;

    if (frame_interval_usec < min_frame_interval_usec)
    {
      return (RADAR_ERR_UNSUPPORTED_FRAME_INTERVAL);
    }

    /* Update time period of Automatic Trigger */
    if (TIMER_SetTimeInterval(&TIMER_FRAME_TRIG, frame_interval_usec * 100U) != TIMER_STATUS_SUCCESS)
    {
      return (RADAR_ERR_UNSUPPORTED_FRAME_INTERVAL);
    }

    ds_device_get_settings()->frame_period_usec = frame_interval_usec;

    /* Reset the current frame counter */
    bsp_reset_frame_counter();

    g_adc_sampling_completed = false;

    radar_set_data_acq_mode(RADAR_AUTO_TRIGGER);

    bsp_timer_start(&TIMER_FRAME_TRIG);
  }

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_test_antennas(Radar_Handle_t device, uint8_t tx_antenna_mask, uint8_t rx_antenna_mask, uint32_t frequency_kHz, uint8_t tx_power)
{
  /* Suppress compiler warnings of unused variables */
  XMC_UNUSED_ARG(device);
  XMC_UNUSED_ARG(tx_antenna_mask);
  XMC_UNUSED_ARG(rx_antenna_mask);
  XMC_UNUSED_ARG(frequency_kHz);
  XMC_UNUSED_ARG(tx_power);

  return (RADAR_ERR_FEATURE_UNAVAILABLE);
}

uint16_t radar_set_adc_configuration(Radar_Handle_t device, const Adc_Configuration_t* configuration)
{
  XMC_UNUSED_ARG(device); /* Suppress compiler warnings of unused variable */

  uint32_t chirp_time_usec;

  /* Configuration change is only allowed while automatic trigger is not working */
  if (radar_get_data_acq_mode() != RADAR_MANUAL_TRIGGER)
  {
    return (RADAR_ERR_BUSY);
  }

  /* Check if configuration is valid */
  /* ------------------------------- */
  if ((configuration->samplerate_Hz == 0) || (configuration->samplerate_Hz > BSP_MAX_ADC_FREQ_HZ))
  {
    return (RADAR_ERR_SAMPLERATE_OUT_OF_RANGE);
  }

  if (configuration->resolution != BSP_ADC_RESOLUTION)
  {
    return (RADAR_ERR_UNSUPPORTED_RESOLUTION);
  }

  /* Apply ADC configuration */
  /* ----------------------- */
  //sample_rate_divider = ((REFERENCE_OSC_FREQ_HZ << 1) / configuration->samplerate_Hz + 1) >> 1;

  if (ds_device_get_settings()->pll_modulation_mode == MODULATION_FMCW)
  {
    chirp_time_usec = (uint32_t) (((float)ds_device_get_settings()->num_samples_per_chirp * 1000.0f ) / configuration->samplerate_Hz * 1000.0f);

    if (radar_set_chirp_time(device, chirp_time_usec) != RADAR_ERR_OK)
    {
      return (RADAR_ERR_SAMPLERATE_OUT_OF_RANGE);
    }
  }
  else
  {
    /* Apply ADC configuration */
    /* ----------------------- */
    if (radar_set_sampling_freq(device, MODULATION_DOPPLER, ds_device_get_settings()->adc_sampling_freq_Hz) != RADAR_ERR_OK)
    {
      return (RADAR_ERR_SAMPLERATE_OUT_OF_RANGE);
    }
  }

  /* Remember configuration (adjust frequency to rounded value) */
  /* ---------------------------------------------------------- */
  /* Copy the new adc configuration to the data store*/
  ds_ep_store_adc_config(configuration);

  return (RADAR_ERR_OK);
}

//============================================================================

uint16_t radar_get_adc_configuration(Radar_Handle_t device, Adc_Configuration_t* configuration)
{
  XMC_UNUSED_ARG(device);  /* Suppress compiler warnings of unused variable */

  if(configuration != NULL)
  {
    ds_ep_fetch_adc_config(configuration);
    return (RADAR_ERR_OK);
  }
  return (RADAR_ERR_FEATURE_UNAVAILABLE);
}

/******************************************************************************
   5. LOCAL FUNCTIONS
*******************************************************************************/

static void save_adc_calibration(Calibration_Target_t target)
{
  uint32_t cal_idx = 0;
  uint32_t interp_factor = MAX_CALIB_SAMPLES_PER_CHANNEL / ds_device_get_settings()->num_samples_per_chirp;	// interpolation factor

  acq_buf_obj *p_acq_buf = ds_get_active_acq_buf();

  uint16_t *adc_temp[4];
  adc_temp[0] = (uint16_t *)get_buffer_address_by_chirp(p_acq_buf, 0, 0);
  adc_temp[1] = (uint16_t *)get_buffer_address_by_chirp(p_acq_buf, 1, 0);
  adc_temp[2] = (uint16_t *)get_buffer_address_by_chirp(p_acq_buf, 2, 0);
  adc_temp[3] = (uint16_t *)get_buffer_address_by_chirp(p_acq_buf, 3, 0);

  /* Fill the calibration buffer with raw ADC data */
  for (uint32_t i = 0; i < ds_device_get_settings()->num_samples_per_chirp; i++)
  {
    adc_i1_calibration[cal_idx] = (adc_temp[0])[i];
    adc_q1_calibration[cal_idx] = (adc_temp[1])[i];
    adc_i2_calibration[cal_idx] = (adc_temp[2])[i];
    adc_q2_calibration[cal_idx] = (adc_temp[3])[i];

    cal_idx += interp_factor;
  }

  /* Permanently write the calibration data in to the Flash */
  if (target == CALIBRATION_TARGET_FLASH)
  {
    /* Read the flashed algorithm parameters */
    bsp_flash_read_data(ADC_CALIB_LEN_BYTES, (uint8_t*)&algo_calibration_s, sizeof(algo_calibration_s));

    write_calibration_flash_data();
  }
}

//============================================================================

static void clear_adc_calibration(Calibration_Target_t target)
{
  Radar_Handle_t device = NULL;

  /* Reset SRAM buffer to a default value */
  radar_init_calibration(device);

  if (target == CALIBRATION_TARGET_FLASH)
  {
    bsp_flash_read_data(ADC_CALIB_LEN_BYTES, (uint8_t*)&algo_calibration_s, sizeof(algo_calibration_s));

    write_calibration_flash_data();
  }
}

//============================================================================

static uint32_t read_adc_calibration(Calibration_Target_t target)
{
	Radar_Handle_t device = NULL;

	if (target == CALIBRATION_TARGET_FLASH)
	{
		if (bsp_flash_is_empty())
		{
			/* Flash memory is empty, set the initial data to default value */
			radar_init_calibration(device);
		}
		else
		{
			/* Read calibration stored in flash */
			bsp_flash_read_data(0U, (uint8_t*)adc_i1_calibration, ADC_CALIB_LEN_BYTES);
		}
	}

	return (ADC_CALIB_LEN_BYTES);
}

//============================================================================

static void save_algo_calibration(Calibration_Target_t target, const Algo_Calibrations_t*  algo_calibration_ptr)
{
	algo_calibration_s = *algo_calibration_ptr;

	if (target == CALIBRATION_TARGET_FLASH)
	{
		read_adc_calibration(CALIBRATION_TARGET_FLASH);

		write_calibration_flash_data();
	}

	set_angle_offset(algo_calibration_s.angle_offset_deg);
	set_range_offset(algo_calibration_s.distance_offset_cm);
}

//============================================================================

static uint32_t read_algo_calibration(Calibration_Target_t target)
{
	size_t algo_size = sizeof(algo_calibration_s);

	if (target == CALIBRATION_TARGET_FLASH)
	{
		if (bsp_flash_is_empty())
		{
			memset(&algo_calibration_s, DEFAULT_CALIBRATION_VALUE, algo_size);
		}
		else
		{
			bsp_flash_read_data(ADC_CALIB_LEN_BYTES, (uint8_t*)&algo_calibration_s, algo_size);
		}
	}

	set_angle_offset(algo_calibration_s.angle_offset_deg);
	set_range_offset(algo_calibration_s.distance_offset_cm);

	return (algo_size);
}

//============================================================================

static void clear_algo_calibration(Calibration_Target_t target)
{
	memset(&algo_calibration_s, 0, sizeof(algo_calibration_s));

	if (target == CALIBRATION_TARGET_FLASH)
	{
		read_adc_calibration(CALIBRATION_TARGET_FLASH);

		write_calibration_flash_data();
	}

	set_angle_offset(algo_calibration_s.angle_offset_deg);
	set_range_offset(algo_calibration_s.distance_offset_cm);
}

//============================================================================

static void write_calibration_flash_data(void)
{
	bsp_flash_write_data(0U, (uint8_t*)adc_i1_calibration, ADC_CALIB_LEN_BYTES);

	bsp_flash_write_data(ADC_CALIB_LEN_BYTES, (uint8_t*)&algo_calibration_s, sizeof(algo_calibration_s));

	bsp_flash_flush_data();
}

//============================================================================

static void set_angle_offset(int16_t offset)
{
	if ((offset > BSP_MIN_ANGLE_OFFSET_DEG) || (offset < BSP_MAX_ANGLE_OFFSET_DEG))
	{
		ds_algo_get_settings()->angle_offset_deg = offset;
	}
}

//============================================================================

static void set_range_offset(uint16_t offset)
{
	if (offset < BSP_MAX_RANGE_OFFSET_CM)
	{
		ds_algo_get_settings()->range_offset_cm = offset;
	}
}

void ds_ep_fetch_adc_config(Adc_Configuration_t *ptr)
{
	device_settings_t *p_dev   = ds_device_get_settings();
	ptr->samplerate_Hz 		   = p_dev->adc_sampling_freq_Hz;
	ptr->resolution    		   = p_dev->adc_resolution;
	ptr->use_post_calibration  = p_dev->adc_use_post_calibration;
}

//============================================================================

uint16_t ds_ep_store_adc_config(const Adc_Configuration_t *p_config)
{
  uint16_t status = RADAR_ERR_BUSY;

  if(p_config != NULL)
  {
	  device_settings_t *p_dev		   = ds_device_get_settings();
	  p_dev->adc_sampling_freq_Hz 	   = p_config->samplerate_Hz;
	  p_dev->adc_resolution    		   = p_config->resolution;
	  p_dev->adc_use_post_calibration  = p_config->use_post_calibration;

	  status = RADAR_ERR_OK;
  }
  return (status);
}

//============================================================================

void ds_ep_fetch_doppler_config(Doppler_Configuration_t *config)
{
	device_settings_t *p_dev  = ds_device_get_settings();
    config->frequency_kHz 	  = p_dev->pll_frequency_kHz;
    config->tx_power 		  = p_dev->bgt_tx_power_level;
}

//============================================================================

uint16_t ds_ep_store_doppler_config(const Doppler_Configuration_t *config)
{
  uint16_t status = RADAR_ERR_BUSY;

  if(config != NULL)
  {
	  device_settings_t *p_dev		   = ds_device_get_settings();
	  p_dev->pll_frequency_kHz 		   = config->frequency_kHz;
	  p_dev->bgt_tx_power_level 	   = config->tx_power;
	  p_dev->isUpdated_doppler_config  = 1;

	  status = RADAR_ERR_OK;
  }
  return (status);
}

//============================================================================

void ds_ep_fetch_fmcw_config(Fmcw_Configuration_t *config)
{
  device_settings_t *p_dev	   = ds_device_get_settings();
  config->lower_frequency_kHz  = p_dev->pll_lower_frequency_kHz;
  config->upper_frequency_kHz  = p_dev->pll_upper_frequency_kHz;
  config->direction 		   = RADAR_DIR_UPCHIRP_ONLY;
  config->tx_power 			   = p_dev->bgt_tx_power_level;
}

//============================================================================

uint16_t ds_ep_store_fmcw_config(const Fmcw_Configuration_t *config)
{
  uint16_t status = RADAR_ERR_BUSY;

  if(config != NULL)
  {
	  if(config->direction != RADAR_DIR_UPCHIRP_ONLY)
	  {
		  status = RADAR_ERR_UNSUPPORTED_DIRECTION;
	  }
	  else
	  {
		  device_settings_t *p_dev		  = ds_device_get_settings();
		  p_dev->pll_lower_frequency_kHz  = config->lower_frequency_kHz;
		  p_dev->pll_upper_frequency_kHz  = config->upper_frequency_kHz;
		  p_dev->bgt_tx_power_level 	  = config->tx_power;
		  p_dev->isUpdated_fmcw_config 	  = 1;

		  status = RADAR_ERR_OK;
	  }
  }
  return (status);
}

//============================================================================

const Device_Info_t *ds_ep_get_device_info(void)
{
  return (&cg_device_info_struct);
}

//============================================================================

void ds_ep_fetch_frame_format(Frame_Format_t *p_frame_format)
{
	device_settings_t *p_dev			  = ds_device_get_settings();
	p_frame_format->num_samples_per_chirp = p_dev->num_samples_per_chirp;
	p_frame_format->num_chirps_per_frame  = p_dev->pll_num_of_chirps_per_frame;
	p_frame_format->rx_mask 		      = p_dev->rx_antenna_mask;
	p_frame_format->signal_part 	      = RADAR_SIGNAL_I_AND_Q;
}

//============================================================================

uint16_t ds_ep_store_frame_format(const Frame_Format_t *p_new_frame_format)
{
  uint16_t status = RADAR_ERR_BUSY;

  if(p_new_frame_format != NULL)
  {
	  if(p_new_frame_format->signal_part != RADAR_SIGNAL_I_AND_Q)
	  {
		  status = RADAR_ERR_UNAVAILABLE_SIGNAL_PART;
	  }
	  else
	  {
		  device_settings_t *p_dev				= ds_device_get_settings();
		  p_dev->num_samples_per_chirp 	        = p_new_frame_format->num_samples_per_chirp;
		  p_dev->pll_num_of_chirps_per_frame 	= p_new_frame_format->num_chirps_per_frame;
		  p_dev->rx_antenna_mask	            = p_new_frame_format->rx_mask;

		  status = RADAR_ERR_OK;
	  }
  }
  return (status);
}

//============================================================================

void ds_ep_fetch_target_list(Target_Info_t* target_info_ptr, uint8_t* target_count)
{
  uint8_t num_targets = 0;

  if (ds_device_get_settings()->pll_modulation_mode == MODULATION_FMCW)
  {
	ds_target_list_t *p_target_list = ds_get_target_list();
    for (uint8_t idx = 0; idx < p_target_list->max_num_of_targets; idx++)
    {
      if (p_target_list->elems[idx].target_id != 0)
      {
        target_info_ptr[num_targets].target_id 	   = idx + 1;
    	target_info_ptr[num_targets].level 		   = p_target_list->elems[idx].strength;
        target_info_ptr[num_targets].radius 	   = p_target_list->elems[idx].range;
        target_info_ptr[num_targets].radial_speed  = p_target_list->elems[idx].speed * 3.6f; // m/s => km/h
        target_info_ptr[num_targets].azimuth 	   = p_target_list->elems[idx].angle;

        num_targets++;
      }
    }
  }

  *target_count = num_targets;
}

//============================================================================

void ds_ep_fetch_range_detection_threshold(uint16_t* threshold)
{
  uint16_t temp = (uint16_t) result_range_detection_threshold;

  *threshold = temp;
}



/* --- End of File --- */

