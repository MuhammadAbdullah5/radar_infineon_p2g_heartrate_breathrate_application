/**
 *
 *  data_store.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

/******************************************************************************
   1. INCLUDES
*******************************************************************************/
#include "types.h"
#include "FW/inc/data_store.h"
#include "FW/inc/drv_bsp.h"

/******************************************************************************
   2. DATA
*******************************************************************************/

/*----------------------------------------------------------------------------
 * NOTE: It is assumed that the storage and the consumption of the data is
 *       executed in the same execution context. (=Thread or task) otherwise
 *       counter measures against data corruption need to be considered!
 */

/* This would be the data structure that is persistent in flash memory! (done via scatter load linker file, once implemented! */
const algo_settings_t ds_default_algo_settings = {
  .max_number_of_targets          = MAX_NUM_OF_TARGETS,
  .max_number_of_tracks           = CURRENT_NUM_OF_TRACKS,
  .isChecked		              = 0,
  .isUpdated                      = 0,

  .tracking_enable                = TRACKING_ENABLE,
  .num_of_tracks                  = CURRENT_NUM_OF_TRACKS,
  .mti_filter_len                 = MTI_FILTER_LEN,
  .mti_filter_enable              = MTI_FILTER_ENABLE,
  .median_filter_len              = MEDIAN_FILTER_LEN,
  .mvg_avg_len                    = MVG_AVG_LEN,
  .range_detection_threshold      = RANGE_DETECTION_THRESHOLD,
  .min_distance_cm                = MINIMUM_RANGE_CM,
  .max_distance_cm                = MAXIMUM_RANGE_CM,
  .speed_detection_threshold      = SPEED_DETECTION_THRESHOLD,
  .min_speed_kmh                  = MINIMUM_SPEED_KMH,
  .max_speed_kmh                  = MAXIMUM_SPEED_KMH,
  .wave_length_ant_spacing_ratio  = (BSP_TGT_WAVE_LENGTH_MM / BSP_TGT_ANTENNA_SPACING_MM),
  .min_angle_for_track_assignment = MIN_ANGLE_FOR_ASSIGNMENT,
};

/* Global structure used to fill the target information e.g. range, speed, angle etc. in FMCW */
ds_target_list_t  ds_target_list = { .max_num_of_targets = MAX_NUM_OF_TARGETS };

static algo_settings_t ds_algo_settings;

float result_range_detection_threshold = 0;

static device_settings_t ds_default_device_settings = {
  /* PLL Settings */
  .pll_chirp_time_usec             = CHIRP_TIME_USEC,
  .pll_frequency_kHz               = BSP_DOPPLER_BASE_FREQ_MHZ * 1000U,
  .pll_lower_frequency_kHz         = BSP_MIN_RF_FREQUENCY_KHZ,
  .pll_upper_frequency_kHz         = BSP_MIN_RF_FREQUENCY_KHZ + (BANDWIDTH_MHZ * 1000U),
  .pll_bandwidth_MHz               = BANDWIDTH_MHZ,
  .pll_num_of_chirps_per_frame     = NUM_OF_CHIRPS,
  .pll_modulation_mode			   = 1, /* 0: Constant frequency (Doppler), 1: FMCW-modulation */

  /* ADC Settings */
#if FW_MODULATION_TYPE == 1U /* FMCW modulation */
  .adc_sampling_freq_Hz            = FMCW_SAMPLING_FREQ_HZ,
#else /* Doppler modulation */
  .adc_sampling_freq_Hz            = DOPPLER_SAMPLING_FREQ_HZ,
#endif
  .adc_resolution                  = 12,
  .adc_use_post_calibration        = 0,

  /* BGT and PGA Settings */
  .pga_rx_gain_level               = PGA_GAIN,
  .power_duty_cycle_enable_flag    = DUTY_CYCLE_ENABLE,
  .bgt_tx_power_level              = BGT_TX_POWER,
  .bgt_rx_lna_gain_enable_flag     = true,

  /* Frame Settings */
  .frame_period_usec               = FRAME_PERIOD_MSEC *1000,
  .num_samples_per_chirp           = SAMPLES_PER_CHIRP,
  .rx_antenna_mask                 = BSP_RX_ANTENNA_MASK,

  .isGainlevelUpdated              = 0,
  .is_duty_cycle_enable_updated    = 0,
  .isUpdated_doppler_config        = 0,
  .isUpdated_fmcw_config           = 0
};

static device_settings_t ds_shadow_device_settings;

/* The new device settings will be applied for the next frame  */
static device_settings_t ds_device_settings;

static acq_buf_obj s_acq_buf = { .status = not_initialized } ;

/******************************************************************************
   3. LOCAL FUNCTIONS
*******************************************************************************/
static void copy_target_elem(ds_target_elem_t *p_dst, target_data_t *p_src)
{
  p_dst->target_id  = p_src->idx;
  p_dst->strength   = p_src->strength;
  p_dst->speed      = p_src->speed;
  p_dst->range      = p_src->range;
  p_dst->angle      = p_src->angle;
}

/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/
void store_algo_result(algo_result_t *p_result)
{
  algo_target_list_t *p_curr_target_list = p_result->p_target_list;
  uint32_t index;

  for(index = 0; index < p_curr_target_list->max_num_of_targets; index++)
  {
    copy_target_elem(&ds_target_list.elems[index], &p_curr_target_list->elems[index]);
  }
  ds_target_list.max_num_of_targets = p_curr_target_list->max_num_of_targets;
  ds_target_list.used_num_of_targets = p_curr_target_list->num_of_targets;

  result_range_detection_threshold = p_result->range_detection_threshold;
}

//============================================================================

ds_target_list_t *ds_get_target_list(void)
{
	return &ds_target_list;
}

//============================================================================

void ds_algo_store_settings(algo_settings_t *p_algo_settings)
{
  memcpy(&ds_algo_settings, p_algo_settings, sizeof(ds_algo_settings));
}

//============================================================================

void ds_algo_fetch_settings(algo_settings_t *p_algo_settings)
{
  /* Data structures are aligned, so no need to do a assignment by member */
  memcpy(p_algo_settings, &ds_algo_settings, sizeof(ds_algo_settings));
}

//============================================================================

algo_settings_t *ds_algo_get_settings(void)
{
  return &ds_algo_settings;
}

//============================================================================

/* If staged Algo settings are with-in range 0 is return, otherwise non-zero
 * the return value could be interpreted as bit field to identify the failure, if needed!
 */
uint32_t ds_algo_check_settings(algo_settings_t *ptr)
{
  uint32_t retValue = 0;

  if((ptr->tracking_enable != 0) && (ptr->tracking_enable != 1))
  {
    retValue += 2;
  }
  if(ptr->mti_filter_len > BSP_MAX_MTI_FILTER_LEN)
  {
    retValue += 4;
  }
  if(!((ptr->num_of_tracks > 0) && (ptr->num_of_tracks <= BSP_MAX_NUM_OF_TRACKS)))
  {
    retValue += 8;
  }
  if(ptr->mti_filter_enable > 1)
  {
    retValue += 32;
  }
  if(!((ptr->mvg_avg_len != 0) && (ptr->mvg_avg_len <= MAX_MVG_AVG_LEN)))
  {
    retValue += 128;
  }
  if( ptr->speed_detection_threshold > BSP_MAX_SPEED_THRESHOLD )
  {
    retValue += 256;
  }
  if(!(ptr->max_speed_kmh > ptr->min_speed_kmh) && (ptr->max_speed_kmh <= BSP_MAX_SPEED_KMH))
  {
    retValue += 512;
  }
  if(ptr->min_speed_kmh > ptr->max_speed_kmh)
  {
    retValue += 1024;
  }
  if(ptr->range_detection_threshold > BSP_MAX_RANGE_THRESHOLD)
  {
    retValue += 2048;
  }
  if((ptr->max_distance_cm < ptr->min_distance_cm) || (ptr->max_distance_cm > BSP_MAX_RANGE_CM))
  {
    retValue += 4096;
  }
  if(ptr->min_distance_cm >= ptr->max_distance_cm)
  {
    retValue += 8192;
  }
  if(retValue == 0)
  {
    ptr->isChecked = 1;
  }

  return retValue;
}

//============================================================================

device_settings_t *ds_device_get_settings(void)
{
  return &ds_device_settings;
}

//============================================================================

device_settings_t *ds_device_get_shadow_settings(void)
{
  return &ds_shadow_device_settings;
}

//============================================================================

/* Handling for staged setting */
uint32_t ds_is_device_settings_updated(void)
{
  int32_t retValue = 0;

  /* If both structs are identical, retValue is 0, otherwise != 0 */
  retValue = memcmp(&ds_shadow_device_settings,&ds_device_settings, sizeof(ds_shadow_device_settings));

  /* Result of memcmp could be negative! */
  return (retValue == 0) ? 0U : 1U;
}

//============================================================================

void ds_device_settings_copy_to_shadow(void)
{
  memcpy(&ds_shadow_device_settings, &ds_device_settings, sizeof(ds_shadow_device_settings));
}

//============================================================================

/* This function could be used to pre-init the settings store
 *  e.g. with persistent value, that are stored in some persistent memory region.
 */
uint32_t ds_init(void)
{
  uint32_t retValue = 0;

  ds_algo_settings = ds_default_algo_settings;

  ds_device_settings = ds_default_device_settings;

  ds_device_settings_copy_to_shadow();

  retValue += ds_algo_check_settings(&ds_algo_settings);

  return retValue;
}

//============================================================================

acq_buf_obj *ds_get_active_acq_buf(void)
{
  return &s_acq_buf;
}

//============================================================================

void ds_set_active_acq_buf(const acq_buf_obj acq_buf)
{
  if(s_acq_buf.p_acq_buf != NULL)
  {
    delete_acq_buffer_obj(&s_acq_buf);
  }

  memcpy(&s_acq_buf, &acq_buf, sizeof(acq_buf_obj));
}




/* --- End of File --- */

