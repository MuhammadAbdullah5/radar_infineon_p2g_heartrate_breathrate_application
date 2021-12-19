/**
 *
 *  dsp_lib.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/dsp_lib.h"

/******************************************************************************
   2. DATA
*******************************************************************************/

#define FAST_TIME_FFT_WIN_LEN			(256U)		/**< Max Window length for FFT windowing over fast time */

#define SLOW_TIME_FFT_WIN_LEN			(16U)		/**< Max Window length for FFT windowing over slow time */

/******************************************************************************
   3. DATA
*******************************************************************************/
const arm_cfft_instance_f32 *fast_time_twiddle_table;
const arm_cfft_instance_f32 *slow_time_twiddle_table;

// Blackman Window has 3dB loss, so additional x 2 factor added in the window

static float FastTime_Window[] = {
		0.000000,0.000109,0.000438,0.000985,0.001755,0.002747,0.003965,0.005412,0.007091,0.009008,0.011165,0.013569,0.016225,0.019139,0.022318,0.025769,
		0.029499,0.033515,0.037827,0.042441,0.047366,0.052612,0.058187,0.064101,0.070362,0.076980,0.083964,0.091325,0.099070,0.107210,0.115754,0.124711,
		0.134089,0.143897,0.154145,0.164839,0.175988,0.187599,0.199678,0.212233,0.225269,0.238792,0.252807,0.267318,0.282328,0.297840,0.313857,0.330381,
		0.347411,0.364949,0.382992,0.401540,0.420590,0.440139,0.460181,0.480712,0.501727,0.523217,0.545174,0.567591,0.590456,0.613759,0.637488,0.661630,
		0.686172,0.711099,0.736394,0.762042,0.788025,0.814324,0.840921,0.867795,0.894924,0.922289,0.949864,0.977628,1.005556,1.033624,1.061805,1.090074,
		1.118404,1.146768,1.175138,1.203486,1.231783,1.260000,1.288108,1.316076,1.343876,1.371477,1.398849,1.425961,1.452783,1.479285,1.505436,1.531207,
		1.556567,1.581486,1.605936,1.629887,1.653310,1.676177,1.698460,1.720131,1.741165,1.761535,1.781216,1.800182,1.818411,1.835879,1.852564,1.868445,
		1.883501,1.897713,1.911063,1.923534,1.935108,1.945773,1.955512,1.964315,1.972169,1.979063,1.984990,1.989941,1.993910,1.996891,1.998880,1.999876,
		1.999876,1.998880,1.996891,1.993910,1.989941,1.984990,1.979063,1.972169,1.964315,1.955512,1.945773,1.935108,1.923534,1.911063,1.897713,1.883501,
		1.868445,1.852564,1.835879,1.818411,1.800182,1.781216,1.761535,1.741165,1.720131,1.698460,1.676177,1.653310,1.629887,1.605936,1.581486,1.556567,
		1.531207,1.505436,1.479285,1.452783,1.425961,1.398849,1.371477,1.343876,1.316076,1.288108,1.260000,1.231783,1.203486,1.175138,1.146768,1.118404,
		1.090074,1.061805,1.033624,1.005556,0.977628,0.949864,0.922289,0.894924,0.867795,0.840921,0.814324,0.788025,0.762042,0.736394,0.711099,0.686172,
		0.661630,0.637488,0.613759,0.590456,0.567591,0.545174,0.523217,0.501727,0.480712,0.460181,0.440139,0.420590,0.401540,0.382992,0.364949,0.347411,
		0.330381,0.313857,0.297840,0.282328,0.267318,0.252807,0.238792,0.225269,0.212233,0.199678,0.187599,0.175988,0.164839,0.154145,0.143897,0.134089,
		0.124711,0.115754,0.107210,0.099070,0.091325,0.083964,0.076980,0.070362,0.064101,0.058187,0.052612,0.047366,0.042441,0.037827,0.033515,0.029499,
		0.025769,0.022318,0.019139,0.016225,0.013569,0.011165,0.009008,0.007091,0.005412,0.003965,0.002747,0.001755,0.000985,0.000438,0.000109,0.000000};

static float SlowTime_Window[] = {
		0.005613,0.037983,0.140790,0.368172,0.747438,1.235470,1.706994,2.000000,2.000000,1.706994,1.235470,0.747438,0.368172,0.140790,0.037983,0.005613};

/******************************************************************************
   4. LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

/******************************************************************************
   5. EXPORTED FUNCTIONS
*******************************************************************************/
void fft_init(void)
{
	/* Fast Time FFT */
#if (RANGE_FFT_SIZE == 1024U)
	fast_time_twiddle_table = &arm_cfft_sR_f32_len1024;		// as FFT_SIZE = 1024 points
#elif (RANGE_FFT_SIZE == 512U)
	fast_time_twiddle_table = &arm_cfft_sR_f32_len512;		// as FFT_SIZE = 512 points
#elif (RANGE_FFT_SIZE == 256U)
	fast_time_twiddle_table = &arm_cfft_sR_f32_len256;		// as FFT_SIZE = 256 points
#elif (RANGE_FFT_SIZE == 128U)
	fast_time_twiddle_table = &arm_cfft_sR_f32_len128;		// as FFT_SIZE = 128 points
#else
	fast_time_twiddle_table = &arm_cfft_sR_f32_len64;		// as FFT_SIZE = 64 points
#endif

	/* Slow Time FFT */
#if (DOPPLER_FFT_SIZE == 64U)
	slow_time_twiddle_table = &arm_cfft_sR_f32_len64;		// as FFT_SIZE = 64 points
#else
	slow_time_twiddle_table = &arm_cfft_sR_f32_len32;		// as FFT_SIZE = 32 points
#endif
}

//============================================================================

void compute_fft_signal(float* i_data, float* q_data, uint16_t Nd, uint16_t Nf, float if_scale,
						FFT_Input_t fft_type, FFT_Direction_t fft_direction,
						float* i_mean, float* q_mean, float* complex_fft_signal)
{
	uint32_t idx;
	uint32_t dec_idx;   // decimation index of FFT window in case selected data points are less than Window length
	float *fft_window;
	const arm_cfft_instance_f32 *cfft_twiddle_table;


	if (fft_direction == FFT_FAST_TIME)
	{
		fft_window = FastTime_Window;

		cfft_twiddle_table = fast_time_twiddle_table;

		dec_idx = FAST_TIME_FFT_WIN_LEN / Nd;
	}
	else /* fft_direction == FFT_SLOW_TIME */
	{
		fft_window = SlowTime_Window;

		cfft_twiddle_table = slow_time_twiddle_table;

		dec_idx = SLOW_TIME_FFT_WIN_LEN / Nd;
	}

	if_scale *= dec_idx;

	/* Find the mean in i_data */
	arm_mean_f32(i_data, Nd, i_mean);

	/* Find the mean in q_data */
	arm_mean_f32(q_data, Nd, q_mean);

	if (fft_type == FFT_INPUT_REAL_I)
	{
		/* Interleaved (re = I & im = 0) samples as {re[0], im[0], re[1], im[1], ...} */
		for (idx = 0; idx < Nd; idx++)
		{
			complex_fft_signal[2 * idx + 0] = 2.0 * (float)(i_data[idx] - *i_mean) * if_scale * fft_window[idx*dec_idx];	// additional scaling by 2 for real input FFT
			complex_fft_signal[2 * idx + 1] = 0;
		}
	}
	else if (fft_type == FFT_INPUT_REAL_Q)
	{
		/* Interleaved (re = Q & im = 0) samples as {re[0], im[0], re[1], im[1], ...} */
		for (idx = 0; idx < Nd; idx++)
		{
			complex_fft_signal[2 * idx + 0] = 2.0 * (float)(q_data[idx] - *q_mean) * if_scale * fft_window[idx*dec_idx];	// additional scaling by 2 for real input FFT
			complex_fft_signal[2 * idx + 1] = 0;
		}
	}
	else
	{
		/* Interleaved (re = I & im = Q) samples as {re[0], im[0], re[1], im[1], ...} */
		for (idx = 0; idx < Nd; idx++)
		{
			complex_fft_signal[2 * idx + 0] = (float)(i_data[idx] - *i_mean) * if_scale * fft_window[idx*dec_idx];
			complex_fft_signal[2 * idx + 1] = (float)(q_data[idx] - *q_mean) * if_scale * fft_window[idx*dec_idx];
		}
	}

	/* Zero Padding */
	for (uint16_t idx = Nd; idx < Nf; idx++)
	{
		complex_fft_signal[2 * idx + 0] = 0;
		complex_fft_signal[2 * idx + 1] = 0;
	}

	/* Processing the floating-point complex FFT. */
	arm_cfft_f32(cfft_twiddle_table, complex_fft_signal, 0, 1);
}

//============================================================================

void compute_fft_spectrum(float* fft_input_signal, uint32_t Nf, float* fft_output_spectrum)
{
	/* Convert to real magnitude data */
	arm_cmplx_mag_f32(fft_input_signal, fft_output_spectrum, Nf);
}
//============================================================================

target_angle_data compute_angle(float if1_i, float if1_q, float if2_i, float if2_q, double d_old, int16_t angle_offset_deg, float wave_length_ant_spacing_ratio)
{

	//float wave_length_ant_spacing_ratio = (TGT_WAVE_LENGTH_MM / TGT_ANTENNA_SPACING_MM);

	target_angle_data temp;

	float rx1_ang, rx2_ang;

	float d_phi;

	float delta_angle;

	float target_angle;

	rx1_ang = get_phase(if1_i, if1_q); //- (double)(0.13 * PI);

	rx2_ang = get_phase(if2_i, if2_q);

	d_phi = (rx1_ang - rx2_ang);

	if (d_phi <= 0)
	{
		d_phi += 2*PI;
	}
	d_phi -= PI;

	if ((uint32_t)d_old == IGNORE_NAN)
	{
		target_angle = 0;
	}
	else if (d_phi > d_old + 0.9* PI || d_phi < d_old - 0.9* PI)
	{
	   d_phi = d_old;
	}

	/* Arcus sinus (-PI/2 to PI/2), input= -1..1 */
	target_angle = asin(d_phi * wave_length_ant_spacing_ratio / (2*PI));

	target_angle = target_angle * 180 / PI;	// Angle (-90...90°)

	target_angle = target_angle + (double)((int32_t) angle_offset_deg + ANGLE_QUANTIZATION * 0.5);

	delta_angle  = fmodf(target_angle , (double)ANGLE_QUANTIZATION);

	target_angle -= delta_angle;

	temp.d_phi = d_phi;

	temp.target_angle = target_angle;

	return temp;
}

//============================================================================

double get_phase(float real, float imag)
{
	double phi;

	/* Phase angle (0 to 2Pi) */
	if((real > 0) && (imag >= 0))		// 1st quadrant
	{
		phi = atan((double)imag / (double)real);
	}
	else if((real < 0) && (imag >= 0))	// 2nd quadrant
	{
		phi = atan((double)imag / (double)real) + PI;
	}
	else if((real < 0) && (imag <= 0)) 	// 3rd quadrant
	{
		phi = atan((double)imag / (double)real) + PI;
	}
	else if((real > 0) && (imag <= 0)) 	// 4th quadrant
	{
		phi = atan((double)imag / (double)real) + 2*PI;
	}
	else if((real == 0) && (imag > 0))
	{
		phi = PI/2;
	}
	else if((real == 0) && (imag < 0))
	{
		phi = 3*PI/2;
	}
	else
	{
		phi = 0;	// Indeterminate
	}

	return(phi);
}

/******************************************************************************
   6. LOCAL FUNCTIONS
*******************************************************************************/



/* --- End of File --- */

