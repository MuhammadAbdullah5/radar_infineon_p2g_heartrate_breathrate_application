/**
 *
 *  algorithm.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_ALGORITHM_H_
#define FW_INC_DRV_ALGORITHM_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
#include "FW/inc/data_store.h"

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/
#define  MAX_MVG_AVG_LEN		(10U)		/**<  Maximum length of moving average filter */

/******************************************************************************
   3. TYPES
*******************************************************************************/
/**
 * \brief Data structure for Median filtering.
 * @{
 */
typedef struct
{
  uint8_t  is_full;
  uint32_t median_filter_len;
  float    buffer[MAX_MEDIAN_FILTER_LEN];
} Median_Filtering_t;

/**
 * \brief Data structure for current measurements used in data association.
 * @{
 */
typedef struct
{
  uint16_t is_associated;
  float    strength;
  float    range;
  float    speed;
  float    rx1_angle_arg_re;
  float    rx1_angle_arg_im;
  float    rx2_angle_arg_re;
  float    rx2_angle_arg_im;
} Measurement_elem_t;

/**
 * \brief Data structure for Track parameters used in tracking.
 * @{
 */
typedef struct
{
  uint8_t  track_id;
  uint8_t  is_alived;
  uint16_t speed_count;
  uint16_t range_change_flag;
  uint16_t lifetime_counter;
  uint32_t measurement_counter;
  float    strength;
  float    range;
  float    speed;
  float    angle;
  float    rx1_angle_arg_re[NUM_OF_CHIRPS];
  float    rx1_angle_arg_im[NUM_OF_CHIRPS];
  float    rx2_angle_arg_re[NUM_OF_CHIRPS];
  float    rx2_angle_arg_im[NUM_OF_CHIRPS];
  float    d_phi;
} Tracking_Params_t;

/**
 * \brief Data structure for Tracking List.
 * @{
 */
typedef struct
{
  uint32_t num_of_tracks;
  uint32_t max_num_of_tracks;
  Tracking_Params_t elems[CURRENT_NUM_OF_TRACKS];
} tracking_list_t;


/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/
/**
 * \brief  Top-level wrapper function for FMCW algorithm.
 *
 * Internally, this function do the following sub tasks;
 * 	- Subtract calibration data from raw ADC IQ
 * 	- Calls standard FMCW core functions to compute targets based on range
 */
void range_doppler_do(acq_buf_obj *p_acq_buf, const algo_settings_t *cp_algo_settings, const device_settings_t *cp_dev_settings, algo_result_t *p_algo_result);

/**
 * \brief  Assigns new measurements to the tracks and initializes new tracks.
 */

void data_association(tracking_list_t *p_tracks, Measurement_elem_t* target_measurements,
                      uint16_t num_of_targets, const algo_settings_t *cp_algo_settings, uint32_t frame_period_usec, uint32_t num_of_chirps);

/**
 * \brief  Resets the Median filter for all tracks in case Median filter length changes.
 */
void median_filter_init(uint32_t median_filter_len);


/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_ALGORITHM_H_ */

/* --- End of File --- */
