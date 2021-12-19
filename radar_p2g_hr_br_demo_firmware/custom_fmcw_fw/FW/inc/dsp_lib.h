/**
 *
 *  dsp_lib.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_DSP_LIB_H_
#define FW_INC_DRV_DSP_LIBH_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
#include "CMSIS_DSP/arm_const_structs.h"
#include "types.h"

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/
#define IGNORE_NAN		(555U)

/******************************************************************************
   3. TYPES
*******************************************************************************/
/**
 * \brief Defines supported FFT input options. Use type FFT_Input_t for this enum.
 * @{
 */
typedef enum
{
   FFT_INPUT_REAL_I    = 0U,  	/**< Only real input I channel for Complex FFT */
   FFT_INPUT_REAL_Q    = 1U,  	/**< Only real input Q channel for Complex FFT */
   FFT_INPUT_COMPLEX   = 2U		/**< Complex input IQ channels for Complex FFT */
}  FFT_Input_t;

/** @} */

/**
 * \brief Defines supported FFT directions. Use type FFT_Direction_t for this enum.
 * @{
 */
typedef enum
{
   FFT_FAST_TIME   = 0U,  	/**< Complex FFT on raw ADC data for Range Spectrum = Fast Time */
   FFT_SLOW_TIME   = 1U  	/**< Complex FFT on fast FFT output signal for Doppler Spectrum = Slow Time */
}  FFT_Direction_t;

typedef struct
{
	double d_phi;

	float target_angle;

}target_angle_data;

/******************************************************************************
   4. EXPORTED DATA
*******************************************************************************/

extern uint16_t g_range_offset_cm;  /**< Compensate range offset in cm */

extern int16_t  g_angle_offset_deg;  /**< Compensate angle offset in degree */

/******************************************************************************
   5. FUNCTION PROTOTYPES
*******************************************************************************/

/**
 * \brief  Performs the initialization of Fast and Slow FFT complex structure used by CMSIS library for FFT calculation.
 */
void fft_init(void);

/**
 * \brief  This function computes the FFt signal out of raw ADC samples.
 *
 *  Internally it computes mean of respective I & Q signal and subtract it before applying IF scaling and Windowing.
 *  Afterwards computes the FFT signal and returns the Nf number of complex samples.
 *
 * \param[in]	*i_data		Pointer of type signed 16-bit integer, containing the address of the I data buffer
 * \param[in]	*q_data		Pointer of type signed 16-bit integer, containing the address of the Q data buffer
 * \param[in]	Nd			Unsigned 16-bit integer, containing the size of raw ADC IQ data buffer
 * \param[in]	Nf			Unsigned 16-bit integer, containing the size of FFT complex values array
 * \param[in]	if_scale	Floating point scale applied to the FFT spectrum to enhance the visibility of targets
 * \param[in]	fft_type	Complex or Real input FFT to be computed defined by \ref FFT_Input_t
 * \param[in]	fft_direction	Fast or Slow FFT to be computed defined by \ref FFT_Direction_t
 *
 * \param[out]  *i_mean		Pointer to a floating point value, containing the mean of the I channel
 * \param[out]  *q_mean		Pointer to a floating point value, containing the mean of the Q channel
 * \param[out]  *complex_fft_signal		Pointer to a floating point array, to return the complex FFT signal in interleaved I&Q format.
 *
 */
void compute_fft_signal(float* i_data, float* q_data, uint16_t Nd, uint16_t Nf, float if_scale,
						FFT_Input_t fft_type, FFT_Direction_t fft_direction,
						float* i_mean, float* q_mean, float* complex_fft_signal);

/**
 * \brief  This function computes the FFt spectrum out of raw ADC samples.
 *
 *  Internally it computes mean of respective I & Q signal and subtract it before applying IF scaling and Windowing.
 *  Afterwards computes the FFT signal and returns the Nf number of real samples as FFT spectrum.
 *
 * \param[in]	*fft_input_signal		Pointer of type float, containing the address of the Complex FFT signal with interleaved IQ
 * \param[in]	Nf						Unsigned 32-bit integer, containing the size of FFT complex values array
 *
 * \param[out]  *fft_output_spectrum	Pointer to a floating point array, to return the real valued FFT spectrum.
 *
 */
void compute_fft_spectrum(float* fft_input_signal, uint32_t Nf, float32_t* fft_output_spectrum);

/**
 * \brief  This function computes the angle (in degrees) using two receive antennas IQ signals.
 *
 * The distance between the two RX-antennas (7.5mm) results in a maximum measurable angle of +-30°
 * Can be calculated as:  +-phi = arcsin((+-Pi*lambda)/(2*Pi*distanceRX))
 *
 * \param[in]	if1_i				Real component of FFT signal for the desired detected target bin for receive antenna 1.
 * \param[in]	if1_q				Imaginary component of FFT signal for the desired detected target bin for receive antenna 1.
 * \param[in]	if2_i				Real component of FFT signal for the desired detected target bin for receive antenna 2.
 * \param[in]	if2_q				Imaginary component of FFT signal for the desired detected target bin for receive antenna 2.
 * \param[in]	angle_offset_deg	correction offset of angle, from calibration data.
 * \param[in]	wave_length_ant_spacing_ratio	ratio bewetween wave_length and rx antenna spacing
 *
 * \return	Angle of respective detected target in units of degrees with quadrant adjusted.
 *
 */
target_angle_data compute_angle(float if1_i, float if1_q, float if2_i, float if2_q, double d_old, int16_t angle_offset_deg, float wave_length_ant_spacing_ratio);

/**
 * \brief  This function computes the phase (in radian) from real and imaginary part of FFT signal.
 *
 * \param[in]	real		Real component of FFT signal for the desired detected target bin.
 * \param[in]	imag		Imaginary component of FFT signal for the desired detected target bin.
 *
 * \return	Phase in units of radians between (0 , 2Pi).
 *
 */
double get_phase(float real, float imag);

/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* DSP_LIB */

/* --- End of File --- */
