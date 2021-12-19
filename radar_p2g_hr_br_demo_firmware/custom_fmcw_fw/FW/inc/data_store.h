/**
 *
 *  data_store.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_DATA_STORE_H_
#define FW_INC_DRV_DATA_STORE_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/

#include "FW/inc/drv_acq_buffer.h"

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/

/******************************************************************************
   3. TYPES
*******************************************************************************/
/*
 *  Hardware configuration structure
 */
typedef struct device_settings_TAG
{
  /* PLL Settings */
  uint32_t    pll_chirp_time_usec;
  float       pll_frequency_kHz;
  float       pll_lower_frequency_kHz;
  float       pll_upper_frequency_kHz;
  float       pll_bandwidth_MHz;
  uint32_t	  pll_num_of_chirps_per_frame;
  uint32_t 	  pll_modulation_mode; /* 0: Constant frequency (Doppler), 1: FMCW-modulation */

  /* ADC Settings */
  uint32_t    adc_sampling_freq_Hz;
  uint8_t 	  adc_resolution;
  uint8_t 	  adc_use_post_calibration;

  /* BGT and PGA Settings */
  uint16_t    pga_rx_gain_level;
  uint8_t     power_duty_cycle_enable_flag;
  uint8_t     bgt_tx_power_level;
  uint8_t     bgt_rx_lna_gain_enable_flag;

  /* Frame Settings */
  uint32_t 	  frame_period_usec;
  uint32_t 	  num_samples_per_chirp;
  uint8_t     rx_antenna_mask;

  uint32_t 	  isGainlevelUpdated;
  uint32_t	  is_duty_cycle_enable_updated;
  uint32_t	  isUpdated_fmcw_config;
  uint32_t    isUpdated_doppler_config;
} device_settings_t;

typedef struct targetData
{
  uint32_t idx;
  float strength;
  float range;
  float speed;
  float angle;
} target_data_t;

typedef enum
{
  unknown,
  target_mode,
  tracking_mode
} e_target_mode_t;

typedef struct algo_target_list_TAG
{
  uint32_t num_of_targets;
  uint32_t max_num_of_targets;
  e_target_mode_t mode;
  target_data_t elems[MAX_NUM_OF_TARGETS];
} algo_target_list_t;

typedef struct target_elem
{
  uint32_t  target_id; /**< Non-zero value if target detected else zero */
  float     range;     /**< Range of target detected in units of cm */
  float     speed;     /**< Speed of target detected in units of km/h */
  float     angle;     /**< Angle of target detected in units of degrees */
  float     strength;  /**< Absolute magnitude FFT spectrum values calculated by RangeDoppler algorithm */
} ds_target_elem_t;

typedef struct
{
  uint32_t max_num_of_targets;
  uint32_t used_num_of_targets;
  ds_target_elem_t elems[MAX_NUM_OF_TARGETS];
} ds_target_list_t;

typedef struct
{
  float  range_detection_threshold;
  algo_target_list_t *p_target_list;
} algo_result_t;

/*
 * Algorithm settings structure
 */
typedef struct algo_settings_TAG
{
  uint8_t    isUpdated;
  uint8_t    isChecked;

  uint32_t   max_number_of_targets;
  uint32_t   max_number_of_tracks;

  uint32_t   tracking_enable;
  uint32_t   num_of_tracks;
  uint32_t   mvg_avg_len;
  uint32_t   median_filter_len;
  uint32_t   mti_filter_len;
  uint32_t   mti_filter_enable;
  uint32_t   range_offset_cm;    // provided via calibration endpoint/struct!
  int16_t    angle_offset_deg;   // provided via calibration endpoint/struct!

  uint32_t   min_distance_cm;
  uint32_t   max_distance_cm;
  uint32_t   range_detection_threshold;

  uint32_t   speed_detection_threshold;
  uint32_t   min_speed_kmh;
  uint32_t   max_speed_kmh;
  float      wave_length_ant_spacing_ratio;
  float	     min_angle_for_track_assignment;
} algo_settings_t;

/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/
void ds_algo_store_settings(algo_settings_t *p_algo_settings);

void ds_algo_fetch_settings(algo_settings_t *p_algo_settings);

uint8_t ds_algo_apply_staged_settings(void);

algo_settings_t *ds_algo_get_settings(void);

uint32_t ds_algo_check_settings(algo_settings_t *ptr);

uint32_t ds_init(void);

device_settings_t *ds_device_get_settings(void);

device_settings_t *ds_device_get_shadow_settings(void);

uint8_t ds_device_apply_staged_settings(void);

uint32_t ds_is_device_settings_updated(void);

void ds_device_settings_copy_to_shadow(void);

acq_buf_obj *ds_get_active_acq_buf(void);

void ds_set_active_acq_buf(const acq_buf_obj acq_buf);

void store_algo_result(algo_result_t *p_curr_target_list);

void fetch_range_detection_threshold(uint16_t* threshold);

ds_target_list_t *ds_get_target_list(void);



/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_DATASTORE_H_ */

/* --- End of File --- */
