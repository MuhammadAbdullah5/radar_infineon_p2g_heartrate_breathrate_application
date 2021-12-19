/**
 *
 *  algorithm.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/algorithm.h"
#include "FW/inc/drv_acq_buffer.h"
#include "FW/inc/radar_control.h"
#include "FW/inc/dsp_lib.h"

/******************************************************************************
   2. DATA
*******************************************************************************/

#define  LIFE_TIME_COUNT	(20U)			/**< Number of frames after which track is killed */

#define  GHOST_LIFE_TIME	(5U)			/**< Number of frames after which ghost track is killed */

#define  DELTA_PX_CM		(120.0f)		/**< Size of prediction window, 3 bins */

#define  COHERENT_INTEG_LEN             (NUM_OF_CHIRPS) /**< Half of Chirps to be averaged in Fast Time FFT */

#define  HEADER_LEN                     (4U) /**< Number of parameters per target computed */

#define  RANGE_FFT_SIGNAL_ARRAY_LEN     (2 * RANGE_FFT_SIZE * NUM_OF_CHIRPS)

#define  RANGE_FFT_SIGNAL_LEN           (2 * RANGE_FFT_SIZE)

#define  DOPPLER_FFT_SIGNAL_ARRAY_LEN   (2 * DOPPLER_FFT_SIZE)

#define  RANGE_FFT_SPECT_ARRAY_LEN      (RANGE_FFT_SIZE / 2)

#define  DOPPLER_FFT_SPECT_ARRAY_LEN    (DOPPLER_FFT_SIZE)

#define  Hz_2_MPS                       (0.0124f * 0.5f) /**< wave_len_meters / 2 */

static Median_Filtering_t median_angle_arr[CURRENT_NUM_OF_TRACKS];

#if 0
static float   fft_input_i1[BSP_MAX_NUM_SAMPLES_PER_CHIRP];
static float   fft_input_q1[BSP_MAX_NUM_SAMPLES_PER_CHIRP];

static float   fft_input_i2[BSP_MAX_NUM_SAMPLES_PER_CHIRP];
static float   fft_input_q2[BSP_MAX_NUM_SAMPLES_PER_CHIRP];

static float  range_fft_signal_rx1[RANGE_FFT_SIGNAL_ARRAY_LEN]; /** Complex array with IQ interleaved */
static float  range_fft_signal_rx2[RANGE_FFT_SIGNAL_ARRAY_LEN];

static float  range_fft_spectrum[RANGE_FFT_SIZE/2]; /**	For Range FFT Magnitude Spectrum for target detection */

static float  range_fft_spectrum_temp[RANGE_FFT_SIZE/2]; /** For MTI filtering */
static float  range_fft_spectrum_hist[RANGE_FFT_SIZE/2];

static float  doppler_fft_signal_rx1[DOPPLER_FFT_SIGNAL_ARRAY_LEN];
static float  doppler_fft_signal_rx2[DOPPLER_FFT_SIGNAL_ARRAY_LEN];

static float  doppler_fft_spectrum[DOPPLER_FFT_SPECT_ARRAY_LEN];

static tracking_list_t s_tracking_list = { .max_num_of_tracks = CURRENT_NUM_OF_TRACKS, .num_of_tracks = 0};

static float  if_scale = (16 * 3.3f / 4095.0f);

static float  alpha_mti, beta_mti;

static uint32_t	samples_per_channel = 0;

#if (FFT_INPUT_TYPE == 0)
static FFT_Input_t	fft_type = FFT_INPUT_REAL_I;
#elif (FFT_INPUT_TYPE == 1)
static FFT_Input_t	fft_type = FFT_INPUT_REAL_Q;
#else
static FFT_Input_t	fft_type = FFT_INPUT_COMPLEX;
#endif

static Measurement_elem_t target_measurements[CURRENT_NUM_OF_TRACKS];

static float  distance_per_bin = 0; // distance/bin (meters) = c /(2B * Nfft/Ns);


static float rx_angle_fft[2*DOPPLER_FFT_SIZE];

static float rx_angle_fft_spectrum[DOPPLER_FFT_SIZE];


/******************************************************************************
   3. LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

static void range_doppler_init(const algo_settings_t *cp_algo_settings);

static uint8_t target_detection(const algo_settings_t *p_algo_settings, float* fft_spectrum, target_data_t* target_info, uint32_t search_length, float threshold);

static void target_sorting(algo_result_t *algo_result);

static void update_tracking_list(tracking_list_t* p_tracking_list, Measurement_elem_t* measurement_ptr, const algo_settings_t *cp_algo_Settings, uint32_t num_of_targets, uint32_t frame_period_usec, uint32_t num_of_chirps );

static void update_target_list_from_tracks(algo_target_list_t *p_targets, tracking_list_t *p_tracks, uint32_t reduced_num_of_tracks);

static uint8_t get_next_free_trackID(tracking_list_t *p_tracks);

static void clear_track_elem(Tracking_Params_t* track_ptr);

static uint32_t assign_track(tracking_list_t *p_tracks, Measurement_elem_t* target_measurement, uint8_t id, uint16_t angle_offset_deg, float wave_length_ant_spacing_ratio, float min_angle);

static void update_track(Tracking_Params_t* track_ptr, Measurement_elem_t* target_measurement, uint8_t track_id,
		float px_track_predict, float range_detection_threshold, int16_t angle_offset_deg, uint32_t num_of_chirps,
		float wave_length_ant_spacing_ratio);

static int compare_float(const void* a, const void* b);

static float median_filtering(Median_Filtering_t *track_median_arr, float new_input);

/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/

void range_doppler_do(acq_buf_obj *p_acq_buf, const algo_settings_t *cp_algo_settings, const device_settings_t *cp_dev_settings, algo_result_t *p_algo_result)
{
  algo_target_list_t *p_targets = p_algo_result->p_target_list;

  uint32_t Nc = p_acq_buf->params.num_of_chirps_per_frame;
  uint32_t Ns = p_acq_buf->params.num_of_samples_per_chirp;

  uint32_t i, j;
  uint32_t max_idx;
  uint32_t max_idx_arr;
  uint32_t cal_idx;
  uint32_t interp_factor = MAX_CALIB_SAMPLES_PER_CHANNEL / Ns;
  uint32_t target_idx;

  float if1_real, if2_real;
  float if1_imag, if2_imag;

  float max_value;

  float PulseRepTime_sec = (float)(cp_dev_settings->pll_chirp_time_usec + PLL_RAMP_DOWM_TIME_USEC + 5 * PLL_STEADY_STATE_USEC) * 1e-6;  // Chirp repetition time in seconds

  float range_detection_threshold = cp_algo_settings->range_detection_threshold;

  uint32_t peak_search_max_idx;

  float Fd_per_bin = 1.0f/(float)(PulseRepTime_sec * DOPPLER_FFT_SIZE);  // Doppler Per bin

  //--------------------------------------------------------------------------

  alpha_mti = 1.0f / cp_algo_settings->mti_filter_len;

  beta_mti = (1.0f - alpha_mti);

  if (samples_per_channel != Ns)
  {
    range_doppler_init(cp_algo_settings);

    samples_per_channel = Ns;
  }

  distance_per_bin = (float)(300.0f / (cp_dev_settings->pll_bandwidth_MHz * 2 * (RANGE_FFT_SIZE / Ns)));  // distance/bin (meters) = c /(2B * Nfft/Ns)

  peak_search_max_idx = (uint32_t)(cp_algo_settings->max_distance_cm / (100.0f * distance_per_bin) + 2);

  peak_search_max_idx = (peak_search_max_idx > RANGE_FFT_SIZE/2-1) ? RANGE_FFT_SIZE/2-1 : peak_search_max_idx;

  // Nc is set via "p_acq_buf->params.num_of_chirps_per_frame" and comes from the buffer, so the following statement is obsolete, to some extent. If violated, it should raise an fatal error!
  Nc = (Nc > NUM_OF_CHIRPS) ? NUM_OF_CHIRPS : Nc; // to avoid out of memory flow in case more number of chirps selected

  //==================== (1) Fast FFT on Rx1 & Rx2 ===========================

  for (j = 0; j < Nc; j++)		// Loop over number of chirps
  {
    uint16_t *p_temp[4];
    p_temp[0] = (uint16_t *)get_buffer_address_by_chirp(p_acq_buf, 0, j );
    p_temp[1] = (uint16_t *)get_buffer_address_by_chirp(p_acq_buf, 1, j );
    p_temp[2] = (uint16_t *)get_buffer_address_by_chirp(p_acq_buf, 2, j );
    p_temp[3] = (uint16_t *)get_buffer_address_by_chirp(p_acq_buf, 3, j );
    cal_idx = 0;

    for (i = 0; i < Ns; i++)	// Loop over number of samples per chirp
    {
      fft_input_i1[i]  = (float)((p_temp[0])[i] - adc_i1_calibration[cal_idx]);
      fft_input_q1[i]  = (float)((p_temp[1])[i] - adc_q1_calibration[cal_idx]);

      fft_input_i2[i]  = (float)((p_temp[2])[i] - adc_i2_calibration[cal_idx]);
      fft_input_q2[i]  = (float)((p_temp[3])[i] - adc_q2_calibration[cal_idx]);
      cal_idx += interp_factor;
    }

    //---------------------------------------- Rx1 ---------------------------
    compute_fft_signal(fft_input_i1, fft_input_q1, Ns, RANGE_FFT_SIZE, if_scale,
                       fft_type, FFT_FAST_TIME, &if1_real, &if1_imag, &range_fft_signal_rx1[(j * RANGE_FFT_SIZE * 2)]);

    //---------------------------------------- Rx2 ---------------------------
    compute_fft_signal(fft_input_i2, fft_input_q2, Ns, RANGE_FFT_SIZE, if_scale,
                       fft_type, FFT_FAST_TIME, &if2_real, &if2_imag, &range_fft_signal_rx2[(j * RANGE_FFT_SIZE * 2)]);
  }

  //==================== (2) FFT spectrum on chirp with maximum FFT absolute magnitude =====================

  float last_max_value = 0;

  max_idx_arr = 0;

  for (j = 0; j < Nc; j++)  // Loop over number of chirps
  {
    uint32_t idx = (2 * RANGE_FFT_SIZE) * j;

#if (RX_ANTENNA_SELECTION == 1U)
    compute_fft_spectrum(&range_fft_signal_rx1[idx], RANGE_FFT_SIZE/2, range_fft_spectrum);
#else
    compute_fft_spectrum(&range_fft_signal_rx2[idx], RANGE_FFT_SIZE/2, range_fft_spectrum);
#endif

    arm_max_f32(range_fft_spectrum, (uint32_t)RANGE_FFT_SIZE/2, &max_value, &max_idx);

    if(last_max_value < max_value)
    {
      last_max_value = max_value;

      max_idx_arr = j;
    }
  }

  max_idx = (2 * RANGE_FFT_SIZE) * max_idx_arr;

#if (RX_ANTENNA_SELECTION == 1U)
  compute_fft_spectrum(&range_fft_signal_rx1[max_idx], RANGE_FFT_SIZE/2, range_fft_spectrum);
#else
  compute_fft_spectrum(&range_fft_signal_rx2[max_idx], RANGE_FFT_SIZE/2, range_fft_spectrum);
#endif

  //==================== (3) MTI Filter update =======================

  if (cp_algo_settings->mti_filter_enable == 1U)
  {
    memcpy(range_fft_spectrum_temp, range_fft_spectrum, (RANGE_FFT_SIZE/2) * sizeof(float));

    for (i = 0; i < (RANGE_FFT_SIZE/2); i++)
    {
      range_fft_spectrum[i] = fabsf(range_fft_spectrum[i] - range_fft_spectrum_hist[i]);

      range_fft_spectrum_hist[i] = alpha_mti * range_fft_spectrum_temp[i] + beta_mti * range_fft_spectrum_hist[i];
    }
  }
  else
  {
    memset(range_fft_spectrum_hist, 0, RANGE_FFT_SIZE/2 * sizeof(float));
  }

  //=========== (4) Target detection on Rx1 ==================================

  memset(&p_targets->elems[0], 0, sizeof(p_targets->elems)); // clear target data

  memset(&target_measurements[0], 0, sizeof(target_measurements));

  p_targets->num_of_targets = target_detection(cp_algo_settings, range_fft_spectrum, &p_targets->elems[0], peak_search_max_idx, range_detection_threshold);

  //=========== (5) Slow FFT on Rx1 & Rx2 ====================================

  if ((p_targets->num_of_targets > 0) && (Nc > 1))
  {
    for (j = 0; j < p_targets->num_of_targets; j++)
    {
      uint32_t idx_max = 0;
      target_idx = 2 * p_targets->elems[j].idx; // Get the index of Fast FFT spectrum where target is detected

      for (i = 0; i < Nc; i++)	// Loop over number of chirps
      {
        uint32_t idx = target_idx + (2 * RANGE_FFT_SIZE) * i;

        fft_input_i1[i]  = range_fft_signal_rx1[idx];
        fft_input_q1[i]  = range_fft_signal_rx1[idx + 1];

        fft_input_i2[i]  = range_fft_signal_rx2[idx];
        fft_input_q2[i]  = range_fft_signal_rx2[idx + 1];
      }

      //-------------------------------- Rx1 ---------------------------------
      compute_fft_signal(fft_input_i1, fft_input_q1, Nc, DOPPLER_FFT_SIZE, 1.0,
                         FFT_INPUT_COMPLEX, FFT_SLOW_TIME, &if1_real, &if1_imag, doppler_fft_signal_rx1);

      //-------------------------------- Rx2 ---------------------------------

      compute_fft_signal(fft_input_i2, fft_input_q2, Nc, DOPPLER_FFT_SIZE, 1.0,
                         FFT_INPUT_COMPLEX, FFT_SLOW_TIME, &if2_real, &if2_imag, doppler_fft_signal_rx2);

#if (RX_ANTENNA_SELECTION == 1U)
      compute_fft_spectrum(doppler_fft_signal_rx1, DOPPLER_FFT_SIZE, doppler_fft_spectrum);
#else
      compute_fft_spectrum(doppler_fft_signal_rx2, DOPPLER_FFT_SIZE, doppler_fft_spectrum);
#endif

      arm_max_f32(doppler_fft_spectrum, (uint32_t)DOPPLER_FFT_SIZE, &max_value, &idx_max);

      if (max_value >= cp_algo_settings->speed_detection_threshold && idx_max != 0)
      {
        if (idx_max < DOPPLER_FFT_SIZE/2) // target departing
        {
          p_targets->elems[j].speed = (float) ((int32_t)idx_max * -Fd_per_bin) * (float)Hz_2_MPS;
        }
        else // target approching
        {
          p_targets->elems[j].speed = (float)((DOPPLER_FFT_SIZE - 1 - idx_max) * Fd_per_bin) * (float)Hz_2_MPS;
        }

        //=========== (6) Angle Calculation on Rx1 & Rx2 =====================

        if1_real = doppler_fft_signal_rx1[2*idx_max];
        if1_imag = doppler_fft_signal_rx1[2*idx_max + 1];

        if2_real = doppler_fft_signal_rx2[2*idx_max];
        if2_imag = doppler_fft_signal_rx2[2*idx_max + 1];
      }
      else
      {
        p_targets->elems[j].speed = 0;
      }

      if (cp_algo_settings->tracking_enable == 1U)
      {
        target_measurements[j].strength = p_targets->elems[j].strength;
        target_measurements[j].range    = p_targets->elems[j].range;
        target_measurements[j].speed    = p_targets->elems[j].speed;

        target_measurements[j].rx1_angle_arg_re = if1_real;
        target_measurements[j].rx1_angle_arg_im = if1_imag;
        target_measurements[j].rx2_angle_arg_re = if2_real;
        target_measurements[j].rx2_angle_arg_im = if2_imag;
      }
      else
      {
        target_angle_data T;

        float wave_length_ant_spacing_ratio = (BSP_TGT_WAVE_LENGTH_MM / BSP_TGT_ANTENNA_SPACING_MM);

        T= compute_angle(if1_real, if1_imag, if2_real, if2_imag, IGNORE_NAN, cp_algo_settings->angle_offset_deg, wave_length_ant_spacing_ratio);

        p_targets->elems[j].angle = T.target_angle;
      }
    }
  }

  //=========== (7) Measurement update =======================================

  p_algo_result->p_target_list->mode = target_mode;
  if (cp_algo_settings->tracking_enable == 1U)
  {
    update_tracking_list(&s_tracking_list, target_measurements, cp_algo_settings, p_targets->num_of_targets, cp_dev_settings->frame_period_usec, cp_dev_settings->pll_num_of_chirps_per_frame );
    update_target_list_from_tracks(p_algo_result->p_target_list, &s_tracking_list, cp_algo_settings->num_of_tracks);

    p_algo_result->p_target_list->mode = tracking_mode;
  }
  else
  {
    target_sorting(p_algo_result);
  }
  p_algo_result->range_detection_threshold = range_detection_threshold;

}


/******************************************************************************
   5. LOCAL FUNCTIONS
*******************************************************************************/

static void range_doppler_init(const algo_settings_t *cp_algo_settings)
{
  /* Clear FFT arrays */
  memset(range_fft_signal_rx1, 0, RANGE_FFT_SIGNAL_ARRAY_LEN * sizeof(float));
  memset(range_fft_signal_rx2, 0, RANGE_FFT_SIGNAL_ARRAY_LEN * sizeof(float));

  memset(doppler_fft_signal_rx1, 0, DOPPLER_FFT_SIGNAL_ARRAY_LEN * sizeof(float));
  memset(doppler_fft_signal_rx2, 0, DOPPLER_FFT_SIGNAL_ARRAY_LEN * sizeof(float));

  memset(range_fft_spectrum_hist, 0, RANGE_FFT_SIZE/2 * sizeof(float));

  memset(doppler_fft_spectrum, 0, DOPPLER_FFT_SPECT_ARRAY_LEN * sizeof(float));

  memset(&target_measurements[0], 0, sizeof(target_measurements));

  memset(&s_tracking_list, 0, sizeof(s_tracking_list));
  s_tracking_list.max_num_of_tracks = CURRENT_NUM_OF_TRACKS;

  fft_init();  // To init the internal FFT functions
}

//==========================================================================

static uint8_t target_detection(const algo_settings_t *p_algo_settings, float* fft_spectrum, target_data_t* target_info, uint32_t search_length, float threshold)
{
  uint32_t target_range;
  uint32_t n = 2;
  uint32_t  fl2_bin, fl_bin, fp_bin, fr_bin, fr2_bin;
  float fl2,fl, fp, fr,fr2;
  float peak_idx = 0;
  uint8_t num_of_targets = 0;

  while (n <= (search_length - 2))
  {
    fp_bin  = n;
    fl_bin  = fp_bin - 1;
    fl2_bin = fp_bin - 2;
    fr_bin  = fp_bin + 1;
    fr2_bin = fp_bin + 2;

    fp  = fft_spectrum[fp_bin];
    fl  = fft_spectrum[fl_bin];
    fl2 = fft_spectrum[fl2_bin];
    fr  = fft_spectrum[fr_bin];
    fr2 = fft_spectrum[fr2_bin];

    if((fp >= threshold) && (fp >= fl2) && (fp >= fl) && (fp > fr) && (fp > fr2))
    {
      peak_idx = (fl2_bin * fl2 + fl_bin * fl + fp_bin * fp + fr_bin * fr + fr2_bin * fr2) / (fl2 + fl + fp + fr + fr2);

      target_range = (uint32_t)(peak_idx * 100.0f * distance_per_bin);

      if ((target_range - p_algo_settings->range_offset_cm) > 0)
      {
        target_range = target_range - p_algo_settings->range_offset_cm;
      }

      if ((target_range >= p_algo_settings->min_distance_cm) && (target_range <= p_algo_settings->max_distance_cm))
      {
        float fp_new;

        if(peak_idx > fp_bin)
          fp_new = fp +(fr - fp) * (peak_idx - fp_bin) / (fr_bin - fp_bin);
        else
          fp_new= fl + (fp - fl) * (fp_bin - peak_idx) / (fp_bin - fl_bin);

        target_info[num_of_targets].strength = fp_new;  // FFT magnitude level

        target_info[num_of_targets].range = target_range; // Range in centimeters (cm)

        target_info[num_of_targets].idx = (uint32_t)(peak_idx + 0.5);  // index of FFT where target is detected (rounded)

        num_of_targets++;
      }

      if (num_of_targets >= MAX_NUM_OF_TARGETS)
      {
        break;
      }
    }
    n += 1;
  }

  return num_of_targets;
}

//===========================================================================

static void target_sorting(algo_result_t *algo_result)
{
  target_data_t *p_target = algo_result->p_target_list->elems;
  target_data_t tmp;

  if(algo_result->p_target_list->num_of_targets < 2)
    return; // nothing to sort

  /* Sorting of the targets header */
  for (uint8_t m = 0; m < (algo_result->p_target_list->num_of_targets - 1); m++)
  {
    for (uint8_t n = 0; n < (algo_result->p_target_list->num_of_targets - m - 1); n++)
    {
      if (p_target[n].strength < p_target[n+1].strength) // Compare FFT amplitude levels
      {
        tmp = p_target[n];
        p_target[n] = p_target[n+1];
        p_target[n+1] = tmp;
      }
    }
  }
}

//==========================================================================

static void update_tracking_list(tracking_list_t* p_tracking_list, Measurement_elem_t* measurement_ptr, const algo_settings_t *cp_algo_Settings, uint32_t num_of_targets, uint32_t frame_period_usec, uint32_t num_of_chirps)
{
  data_association(p_tracking_list, measurement_ptr, num_of_targets, cp_algo_Settings, frame_period_usec, num_of_chirps);
}

//==========================================================================

static void update_target_list_from_tracks(algo_target_list_t *p_targets, tracking_list_t *p_tracks, uint32_t reduced_num_of_tracks)
{
  if (p_tracks->num_of_tracks > CURRENT_NUM_OF_TRACKS )
  {
    p_tracks->num_of_tracks = CURRENT_NUM_OF_TRACKS;
  }

  p_targets->num_of_targets = 0;

  for (uint8_t i = 0; i < p_tracks->max_num_of_tracks; i++)
  {
    p_targets->elems[i].idx = 0;

    if (p_tracks->elems[i].is_alived == 1 && p_tracks->elems[i].measurement_counter > 2 &&
        p_tracks->elems[i].range_change_flag > 0)
    {
      p_targets->elems[i].idx           = (uint32_t)(p_tracks->elems[i].track_id + 1);
      p_targets->elems[i].strength      = p_tracks->elems[i].strength;
      p_targets->elems[i].range         = p_tracks->elems[i].range;
      p_targets->elems[i].speed         = p_tracks->elems[i].speed;
      p_targets->elems[i].angle         = p_tracks->elems[i].angle;

      p_targets->num_of_targets         += 1;

      if (p_targets->num_of_targets >= reduced_num_of_tracks)
        break;
    }
  }
}

void data_association(tracking_list_t *p_tracks, Measurement_elem_t* target_measurements, uint16_t num_of_targets,
		const algo_settings_t *cp_algo_settings, uint32_t frame_period_usec, uint32_t num_of_chirps )
{

	float px_track_predict = 0;
	float px_measured = 0;
	float px_min, px_max;
	float curr_neighbor, last_neighbor;

	//========= CASE 1: all tracks are empty ===========
	if(p_tracks->num_of_tracks == 0 && num_of_targets != 0)
	{
		// clear all tracks:
		for (uint8_t i = 0; i < p_tracks->max_num_of_tracks; i++)
		{
			memset(&p_tracks->elems[i], 0 ,sizeof(p_tracks->elems[i]) );
		}

		for (uint8_t j = 0; j < num_of_targets; j++)
		{
			if(target_measurements[j].speed != 0)		// only moving targets
			{
				uint8_t next_free_id = get_next_free_trackID( p_tracks );

				(void) assign_track(p_tracks, &target_measurements[j], next_free_id, cp_algo_settings->angle_offset_deg,
						cp_algo_settings->wave_length_ant_spacing_ratio, cp_algo_settings->min_angle_for_track_assignment);

				if (p_tracks->num_of_tracks >= p_tracks->max_num_of_tracks)
				{
					break;
				}
			}
		}
	}

	//========= CASE 2: update tracks already assigned ===========
	else
	{
		int8_t tgt_idx;

		for (uint8_t i = 0; i < p_tracks->max_num_of_tracks; i++)
		{
			if (p_tracks->elems[i].is_alived == 0)
			{
				continue;
			}

			px_track_predict = p_tracks->elems[i].range - (p_tracks->elems[i].speed * ((frame_period_usec/1000) * 0.1f) );	// range is in cm, thus speed is cm/sec

			px_max = px_track_predict + DELTA_PX_CM;

			px_min = px_track_predict - DELTA_PX_CM;

			if (px_min < 0)
			{
				px_min = 0;
			}

			//=========== SNN loop =============

			tgt_idx = -1;

			curr_neighbor = 0;

			last_neighbor = 10000;

			for (uint8_t j = 0; j < num_of_targets; j++)
			{
				if(target_measurements[j].is_associated == 0)
				{
					px_measured = target_measurements[j].range;

					if((px_min <= px_measured) && (px_measured <= px_max))
					{
						curr_neighbor = fabsf(px_measured - px_track_predict);

						if (curr_neighbor < last_neighbor)
						{
							last_neighbor = px_measured;

							tgt_idx = j;
						}
					}
				}
			}

			if (tgt_idx == -1)
			{
				p_tracks->elems[i].lifetime_counter += 1;

				p_tracks->elems[i].range = 0.5 * (p_tracks->elems[i].range + px_track_predict);
			}
			else
			{
				p_tracks->elems[i].lifetime_counter /= 2;

				p_tracks->elems[i].measurement_counter += 1;

				update_track(&p_tracks->elems[i], &target_measurements[tgt_idx], i, px_track_predict * 0.5,
								cp_algo_settings->range_detection_threshold , cp_algo_settings->angle_offset_deg, num_of_chirps,
								cp_algo_settings->wave_length_ant_spacing_ratio);

				target_measurements[tgt_idx].is_associated = 1;
			}
		}

		//========= CASE 3: Kill expired tracks ===========

		int32_t ghost_target_diff = 0;

		for (uint32_t j = 0; j < p_tracks->max_num_of_tracks; j++)
		{
			ghost_target_diff = p_tracks->elems[j].lifetime_counter - p_tracks->elems[j].measurement_counter - GHOST_LIFE_TIME;

			if (p_tracks->elems[j].lifetime_counter > LIFE_TIME_COUNT || ghost_target_diff > 0  ||
					p_tracks->elems[j].speed_count > 2*LIFE_TIME_COUNT )
			{
				clear_track_elem(&p_tracks->elems[j]);  // init track data
				p_tracks->num_of_tracks -= 1;

				median_angle_arr[j].is_full = 0;
			}
		}

		//========= CASE 4: assign new tracks  ===========

		if (p_tracks->num_of_tracks < p_tracks->max_num_of_tracks)
		{
			for (uint8_t j = 0; j < num_of_targets; j++)
			{
				if(target_measurements[j].is_associated == 0 && target_measurements[j].speed != 0)
				{
					uint8_t next_free_id = get_next_free_trackID( p_tracks );

					(void) assign_track(p_tracks, &target_measurements[j], next_free_id, cp_algo_settings->angle_offset_deg,
							cp_algo_settings->wave_length_ant_spacing_ratio, cp_algo_settings->min_angle_for_track_assignment );

					if (p_tracks->num_of_tracks >= p_tracks->max_num_of_tracks)
					{
						break;
					}
				}
			}
		}
	}
}

static void clear_track_elem(Tracking_Params_t* track_ptr)
{
	memset(track_ptr, 0, sizeof(Tracking_Params_t));
	track_ptr->d_phi = IGNORE_NAN;
}

//===========================================================================

static void update_track(Tracking_Params_t* track_ptr, Measurement_elem_t* target_measurement, uint8_t track_id,
		float px_track_predict, float range_detection_threshold, int16_t angle_offset_deg, uint32_t num_of_chirps,
		float wave_length_ant_spacing_ratio)
{

	float last_range = track_ptr->range;

	float i1_mean, q1_mean;

	float i2_mean, q2_mean;

	float max_value;

	uint32_t idx;

	float alpha_range ;

	if (fabs(target_measurement->speed) <= 0.3)
	{
		alpha_range = 0.25;
	}
	else if (fabs(target_measurement->speed) > 0.3 && fabs(target_measurement->speed) < 0.5)
	{
		alpha_range = 0.50;
	}
	else if (fabs(target_measurement->speed) >= 0.5 && fabs(target_measurement->speed) < 1.0)
	{
		alpha_range = 0.8;
	}
	else if (fabs(target_measurement->speed) >= 1.0)
	{
		alpha_range = 0.99;
	}
	else
	{
		alpha_range = 1.0/16.0;
	}

	//------------------------- update range -----------------------------------

	track_ptr->range = (1.0f - alpha_range) * track_ptr->range +
			alpha_range * (target_measurement->range * 0.5f + px_track_predict);

	if (track_ptr->range_change_flag == 0)
	{
		if ((track_ptr->range < last_range - 40) || (track_ptr->range > last_range + 40))
		{
			track_ptr->range_change_flag = 1;
		}
	}

	//------------------------- update speed -----------------------------------

	track_ptr->speed = target_measurement->speed;

	if (track_ptr->speed == 0)
	{
		track_ptr->speed_count += 1;
	}
	else
	{
		track_ptr->speed_count = 0;
	}

	//------------------------- update angle -----------------------------------

	track_ptr->strength = target_measurement->strength;

	if (track_ptr->strength > 1.0 * range_detection_threshold)
	{
		if (track_ptr->speed_count == 0)
		{
			track_ptr->rx1_angle_arg_re[0] = target_measurement->rx1_angle_arg_re;

			track_ptr->rx1_angle_arg_im[0] = target_measurement->rx1_angle_arg_im;

			track_ptr->rx2_angle_arg_re[0] = target_measurement->rx2_angle_arg_re;

			track_ptr->rx2_angle_arg_im[0] = target_measurement->rx2_angle_arg_im;

			target_angle_data T;

			T = compute_angle(track_ptr->rx1_angle_arg_re[0], track_ptr->rx1_angle_arg_im[0],
					track_ptr->rx2_angle_arg_re[0], track_ptr->rx2_angle_arg_im[0],track_ptr->d_phi, angle_offset_deg, wave_length_ant_spacing_ratio);

			track_ptr->angle = T.target_angle;
			track_ptr->d_phi = T.d_phi;

			track_ptr->angle = median_filtering(&median_angle_arr[track_id], track_ptr->angle);
		}
		else if ((track_ptr->speed_count % (num_of_chirps + 1)) != 0)
		{

			idx = (track_ptr->speed_count % (num_of_chirps + 1)) - 1;

			track_ptr->rx1_angle_arg_re[idx] = target_measurement->rx1_angle_arg_re;

			track_ptr->rx1_angle_arg_im[idx] = target_measurement->rx1_angle_arg_im;

			track_ptr->rx2_angle_arg_re[idx] = target_measurement->rx2_angle_arg_re;

			track_ptr->rx2_angle_arg_im[idx] = target_measurement->rx2_angle_arg_im;

			target_angle_data T;

			T = compute_angle(track_ptr->rx1_angle_arg_re[idx], track_ptr->rx1_angle_arg_im[idx],
					track_ptr->rx2_angle_arg_re[idx], track_ptr->rx2_angle_arg_im[idx],track_ptr->d_phi, angle_offset_deg, wave_length_ant_spacing_ratio);

			track_ptr->angle = T.target_angle;
			track_ptr->d_phi = T.d_phi;

			track_ptr->angle = median_filtering(&median_angle_arr[track_id], track_ptr->angle);
		}
		else
		{
			//---------------------------------------- Rx1 ------------------------------------------

			compute_fft_signal(track_ptr->rx1_angle_arg_re, track_ptr->rx1_angle_arg_im, num_of_chirps, DOPPLER_FFT_SIZE, 1.0,
					FFT_INPUT_COMPLEX, FFT_SLOW_TIME, &i1_mean, &q1_mean, rx_angle_fft);

			compute_fft_spectrum(rx_angle_fft, DOPPLER_FFT_SIZE, rx_angle_fft_spectrum);

			arm_max_f32(rx_angle_fft_spectrum, (uint32_t)DOPPLER_FFT_SIZE, &max_value, &idx);

			if (idx != 0)
			{
				i1_mean = rx_angle_fft[2*idx + 0];
				q1_mean = rx_angle_fft[2*idx + 1];
			}

			//---------------------------------------- Rx2 ------------------------------------------

			compute_fft_signal(track_ptr->rx2_angle_arg_re, track_ptr->rx2_angle_arg_im, num_of_chirps, DOPPLER_FFT_SIZE, 1.0,
					FFT_INPUT_COMPLEX, FFT_SLOW_TIME, &i2_mean, &q2_mean, rx_angle_fft);

			compute_fft_spectrum(rx_angle_fft, DOPPLER_FFT_SIZE, rx_angle_fft_spectrum);

			arm_max_f32(rx_angle_fft_spectrum, (uint32_t)DOPPLER_FFT_SIZE, &max_value, &idx);

			if (idx != 0)
			{
				i2_mean = rx_angle_fft[2*idx + 0];
				q2_mean = rx_angle_fft[2*idx + 1];
			}

			target_angle_data T;

			T = compute_angle(i1_mean, q1_mean, i2_mean, q2_mean,track_ptr->d_phi, angle_offset_deg, wave_length_ant_spacing_ratio);

			track_ptr->angle = T.target_angle;
			track_ptr->d_phi = T.d_phi;

			track_ptr->angle = median_filtering(&median_angle_arr[track_id], track_ptr->angle);
		}
	}
}

//===========================================================================

static uint32_t assign_track(tracking_list_t *p_tracks, Measurement_elem_t* target_measurement, uint8_t id, uint16_t angle_offset_deg, float wave_length_ant_spacing_ratio, float min_angle)
{

	uint32_t retValue = 0;

	target_angle_data T;
	T = compute_angle(target_measurement->rx1_angle_arg_re, target_measurement->rx1_angle_arg_im,
			target_measurement->rx2_angle_arg_re, target_measurement->rx2_angle_arg_im, IGNORE_NAN, angle_offset_deg, wave_length_ant_spacing_ratio);

	if( fabs(T.target_angle) < min_angle)
	{

		Tracking_Params_t *p_track = &p_tracks->elems[id];

		p_track->track_id = id;
		p_track->is_alived = 1;

		p_track->measurement_counter = 1;

		p_track->range_change_flag = 1;
		p_track->speed_count = 1;

		p_track->strength = target_measurement->strength;
		p_track->range    = target_measurement->range;
		p_track->speed    = target_measurement->speed;

		p_track->rx1_angle_arg_re[0] = target_measurement->rx1_angle_arg_re;
		p_track->rx1_angle_arg_im[0] = target_measurement->rx1_angle_arg_im;
		p_track->rx2_angle_arg_re[0] = target_measurement->rx2_angle_arg_re;
		p_track->rx2_angle_arg_im[0] = target_measurement->rx2_angle_arg_im;

		p_track->angle  = T.target_angle;
		p_track->d_phi  = T.d_phi;

		// calculate the medium filter for the track angle
		median_angle_arr[id].is_full = 0;
		median_filtering(&median_angle_arr[id], p_track->angle);

		target_measurement->is_associated = 1;

		// Inc the number of valid tracks:
		p_tracks->num_of_tracks += 1;
		retValue = 1;
	}
	else
	{
		clear_track_elem(&p_tracks->elems[id]);
		retValue = 0;
	}

	return retValue;
}

//===========================================================================

static uint8_t get_next_free_trackID(tracking_list_t *p_tracks )
{
	uint8_t i = 0;

	while(i < p_tracks->max_num_of_tracks)
	{
		if(p_tracks->elems[i].is_alived == 0)
		{
			break;
		}
		i++;
	}
	return i;
}

//===========================================================================

static int compare_float(const void *a, const void *b)
{
	int retval = 0;

	float a_f = *(float*)a;
	float b_f = *(float*)b;

	if (a_f > b_f)
	{
		retval = 1;
	}
	else if (a_f < b_f)
	{
		retval = -1;
	}

	return retval;
}

//===========================================================================

float median_filtering(Median_Filtering_t *track_median_arr, float new_input)
{

	if( track_median_arr->median_filter_len > MAX_MEDIAN_FILTER_LEN)
		track_median_arr->median_filter_len = MAX_MEDIAN_FILTER_LEN;

	if (track_median_arr->is_full == 0)
	{
		for (uint32_t j = 0; j < track_median_arr->median_filter_len; j++)
		{
			track_median_arr->buffer[j] = new_input;
		}

		track_median_arr->is_full = 1;

		return new_input;
	}
	else
	{
		float sorting_arr[MAX_MEDIAN_FILTER_LEN];

		uint32_t len = MAX_MEDIAN_FILTER_LEN-1;
		if( len > track_median_arr->median_filter_len-1)
			len = track_median_arr->median_filter_len-1;

		for (uint32_t j = 0; j < len; j++)
		{

			sorting_arr[j] = track_median_arr->buffer[j+1];		// shift the array left in order to add new value

			track_median_arr->buffer[j] = track_median_arr->buffer[j+1];

		}

		track_median_arr->buffer[track_median_arr->median_filter_len-1] = new_input;

		sorting_arr[track_median_arr->median_filter_len-1] = new_input;

		qsort(sorting_arr, track_median_arr->median_filter_len, sizeof(float), compare_float);

		return sorting_arr[track_median_arr->median_filter_len / 2];
	}
}
#endif
//===========================================================================

void median_filter_init( uint32_t median_filter_len )
{
	for (uint32_t j = 0; j < sizeof(median_angle_arr)/sizeof(median_angle_arr[0]); j++)
	{
		median_angle_arr[j].is_full = 0;
		median_angle_arr[j].median_filter_len = MAX_MEDIAN_FILTER_LEN;
		if( median_filter_len < MAX_MEDIAN_FILTER_LEN)
			median_angle_arr[j].median_filter_len = median_filter_len;
		// The buffer values of the structure is not cleared. The buffer values are initalized, when then filter is used the first time!
	}

}

/* --- End of File --- */


