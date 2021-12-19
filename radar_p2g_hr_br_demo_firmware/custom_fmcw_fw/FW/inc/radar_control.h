/**
 *
 *  radar_control.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_RADAR_CONTROL_H_
#define FW_INC_DRV_RADAR_CONTROL_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
#include "FW/inc/drv_acq_buffer.h"
#include "FW/inc/data_store.h"
#include "FW/inc/drv_bsp.h"


/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/
#define 	PLL_LOCK_TIME_USEC		(50U)		/**< This is the minimum time in microseconds (usec) to program the PLL */

#define 	BGT_VCO_LOCK_TIME_USEC	(1000U) 	/**< This is the minimum time in microseconds (usec) to program after BGT and PLL
													 power-up in duty cycle mode. After 1500us BGT Q2 divider settled to zero */

#define 	DEFAULT_CALIBRATION_VALUE		(0U)		/**<  Default calibration value to be written in to the Flash */

#define		MAX_CALIB_SAMPLES_PER_CHANNEL	(256U)		/**< Number of data samples stored in the calibration EEPROM per ADC channel */

#define		ADC_CALIB_LEN_BYTES				(BSP_NUM_OF_ADC_CHANNELS * MAX_CALIB_SAMPLES_PER_CHANNEL * 2U)

/**
 * \defgroup ErrorCodes
 *
 * \brief Most functions of the radar driver API return an error code defined
 *        below.
 *
 * @{
 */
#define RADAR_ERR_OK                          0x0000  /**< No error has
                                                           occurred. */
#define RADAR_ERR_DEVICE_DOES_NOT_EXIST       0x0001  /**< The device number
                                                           requested to open
                                                           does not exist. */
#define RADAR_ERR_BUSY                        0x0002  /**< The requested
                                                           operation can't be
                                                           executed. A
                                                           possible reason is
                                                           that a certain test
                                                           mode is activated
                                                           or the automatic
                                                           trigger is active.
                                                           */
#define RADAR_ERR_INCOMPATIBLE_MODE           0x0003  /**< The requested
                                                           operation is not
                                                           supported by the
                                                           currently active
                                                           mode of operation.
                                                           */
#define RADAR_ERR_TIMEOUT                     0x0004  /**< A timeout has
                                                           occurred while
                                                           waiting for
                                                           a data frame to be
                                                           acquired. */
#define RADAR_ERR_UNSUPPORTED_FRAME_INTERVAL  0x0005  /**< The requested time
                                                           interval between
                                                           two frames is out
                                                           of range. */
#define RADAR_ERR_ANTENNA_DOES_NOT_EXIST      0x0006  /**< One or more of the
                                                           selected RX or TX
                                                           antennas is not
                                                           present on the
                                                           device. */
#define RADAR_ERR_SENSOR_DOES_NOT_EXIST       0x0007  /**< The requested
                                                           temperature sensor
                                                           does not exist. */
#define RADAR_ERR_UNSUPPORTED_FRAME_FORMAT    0x0008  /**< The combination of
                                                           chirps per frame,
                                                           samples per chirp
                                                           and number of
                                                           antennas is not
                                                           supported by the
                                                           driver. A possible
                                                           reason is the limit
                                                           of the driver
                                                           internal data
                                                           memory. */
#define RADAR_ERR_FREQUENCY_OUT_OF_RANGE      0x0009  /**< The specified RF
                                                           frequency is not in
                                                           the supported range
                                                           of the device. */
#define RADAR_ERR_POWER_OUT_OF_RANGE          0x000A  /**< The specified
                                                           transmission power
                                                           is not in the valid
                                                           range of
                                                           0...max_tx_power
                                                           (see \ref
                                                           Device_Info_t). */
#define RADAR_ERR_UNAVAILABLE_SIGNAL_PART     0x000B  /**< The device is not
                                                           capable to capture
                                                           the requested part
                                                           of the complex
                                                           signal (see \ref
                                                           Device_Info_t). */

#define RADAR_ERR_PARAMETER_OUT_OF_RANGE      0x000C  /**< The specified
                                                           parameter is out of
                                                           range */

#define RADAR_ERR_FEATURE_UNAVAILABLE     	  0x8000  /**< If feature is currently
                                                           not available in the API. */

#define RADAR_ERR_UNSUPPORTED_DIRECTION       0x0020  /**< The specified FMCW ramp
                                                           direction is not
                                                           supported by the device. */

#define RADAR_ERR_SAMPLERATE_OUT_OF_RANGE     0x0050  /**< The specified sample
                                                            rate is out of range. */

#define RADAR_ERR_UNSUPPORTED_RESOLUTION      0x0051  /**< The specified sample
                                                           resolution is out of
                                                           range. */

#define RADAR_ERR_UNSUPPORTED_PGA_GAIN        0x0052  /**< The specified PGA gain
                                                           level is out of
                                                           range. */


/******************************************************************************
   3. TYPES
*******************************************************************************/
/**
 * \brief Defines radar function callback ID.
 * @{
 */
typedef enum
{
  RADAR_ACQUISITION_STARTED_CB_ID   = 1U,   /**< Start of data acquisition callback ID */
  RADAR_ACQUISITION_DONE_CB_ID      = 2U    /**< End of data acquisition callback ID   */
} Radar_Callback_ID_t;
/** @} */

/**
 * \brief Defines radar data acquisition modes: Manual, Automatic or Test mode.
 * @{
 */
typedef enum
{
  RADAR_MANUAL_TRIGGER    = 0U,   /**< Manual triggering data acquisition */
  RADAR_AUTO_TRIGGER      = 1U,   /**< Automatic triggering data acquisition */
  RADAR_TEST_MODE         = 2U    /**< Test mode, not used yet */
} Radar_Data_Acq_Mode_t;
/** @} */

/**
 * \brief Defines radar hardware state settings
 * @{
 */
typedef enum
{
 	  INIT,
	  ENTER_POWERSAVING,
	  LEAVE_POWERSAVING,
	  POWER_DOWN,
	  UPDATE,
	  START_MANUAL_ACQ,
	  START_AUTO_ACQ,
	  STOP_ACQ,
} hw_state_setting_t;


/**
 * \brief This struct holds the parameters to configure the Analog/Digital
 *        Converter.
 *
 * A struct of this type must be provided to \ref radar_set_adc_configuration
 * and \ref radar_get_adc_configuration.
 */
typedef struct
{
    uint32_t samplerate_Hz;        /**< The sampling rate at with the IF
                                        signals are captured. The value is
                                        specified in Hz.  This value affects
                                        the chirp duration. */
    uint8_t resolution;            /**< The number of bits for each signal
                                        sample. The value may affect the
                                        highest possible sampling rate */
    uint8_t use_post_calibration;  /**< If this is non-zero the post
                                        calibration of the ADC is used. */
} Adc_Configuration_t;

/** @} */


/**
 * \defgroup FMCWConfiguration
 *
 * \brief Functions and types to read and modify configuration of FMCW radar
 *        operation.
 *
 * @{
 */

/**
 * \brief This enumeration type lists all possible chirp directions in FMCW
 *        radar mode.
 *
 * A value of this type is part of the struct \ref Fmcw_Configuration_t.
 *
 * To use the modes RADAR_DIR_ALTERNATING_FIRST_UP and
 * RADAR_DIR_ALTERNATING_FIRST_DOWN, the number of chirp per frame should be
 * at least 2, otherwise these modes behave the same way as
 * RADAR_DIR_UPCHIRP_ONLY and RADAR_DIR_DOWNCHIRP_ONLY.
 */
typedef enum
{
    RADAR_DIR_UPCHIRP_ONLY           = 0, /**< All chirps of a frame start at
                                               the lower FMCW frequency and
                                               ramp up to the upper FMCW
                                               frequency. */
    RADAR_DIR_DOWNCHIRP_ONLY         = 1, /**< All chirps of a frame start at
                                               the upper FMCW frequency and
                                               ramp down to the lower FMCW
                                               frequency. */
    RADAR_DIR_ALTERNATING_FIRST_UP   = 2, /**< Only the odd chirps of a frame
                                               start at the lower RF frequency
                                               and ramp up to the upper FMCW
                                               frequency. The even chirps of a
                                               frame go from the upper to the
                                               lower FMCW frequency. */
    RADAR_DIR_ALTERNATING_FIRST_DOWN = 3  /**< Only the even chirps of a frame
                                               start at the lower RF frequency
                                               and ramp up to the upper FMCW
                                               frequency. The odd chirps of a
                                               frame go from the upper to the
                                               lower RF frequency. */
} Chirp_Direction_t;

/**
 * \brief This struct holds the parameters to configure the RF behaviour in
 *        FMCW radar mode.
 *
 * A struct of this type must be provided to \ref radar_set_fmcw_configuration
 * and \ref radar_get_fmcw_configuration.
 */
typedef struct
{
    uint32_t lower_frequency_kHz;  /**< The lower frequency of an FMCW chirp.
                                        The value is specified in kHz. */
    uint32_t upper_frequency_kHz;  /**< The upper frequency of an FMCW chirp.
                                        The value is specified in kHz. */
    Chirp_Direction_t direction;   /**< The direction of the frequency ramp
                                        during FMCW operation. */
    uint8_t tx_power;              /**< The transmission power the emitted
                                        FMCW chirps. The value is expected to
                                        be in the range of 0...max_tx_power
                                        (see \ref Device_Info_t). */
} Fmcw_Configuration_t;

/** @} */


/**
 * \defgroup DopplerConfiguration
 *
 * \brief Functions and types to read and modify configuration of Doppler
 *        radar operation.
 *
 * @{
 */

/**
 * \brief This struct holds the parameters to configure the RF behaviour in
 *        Doppler radar mode.
 *
 * A struct of this type must be provided to
 * \ref radar_set_doppler_configuration and
 * \ref radar_get_doppler_configuration.
 */
typedef struct
{
    uint32_t frequency_kHz;  /**< The lower frequency of a FMCW chirp. The
                                  value is specified in kHz. */
    uint8_t  tx_power;       /**< The transmission power the emitted FMCW
                                  chirps. The value is expected to be in the
                                  range of 0...max_tx_power (see
                                  \ref Device_Info_t). */
} Doppler_Configuration_t;

/** @} */


/**
 * \defgroup TargetDetection
 *
 * \brief Functions, structures and types used for target detection radar operation.
 *
 * @{
 */

/**
 *
 * \brief Data structure for DSP settings for the target detection processing.
 */
typedef struct
{
	uint8_t  range_mvg_avg_length;

	uint16_t min_range_cm;
	uint16_t max_range_cm;
	uint16_t min_speed_kmh;
	uint16_t max_speed_kmh;
	uint16_t min_angle_degree;
	uint16_t max_angle_degree;
	uint16_t range_threshold;
	uint16_t speed_threshold;

	uint8_t  enable_tracking;
	uint8_t  num_of_tracks;

	uint8_t  median_filter_length;
	uint8_t  enable_mti_filter;
	uint16_t mti_filter_length;
} DSP_Settings_t;

/**
 * \brief This struct lists the content of target data
 *
 * Target data is provided by the driver and can be retrieved
 * through this struct by calling \ref ep_radar_targetdetect_get_targets.
 */
typedef struct
{
	uint32_t target_id;		/**< An unique ID of that target. */
	float level;			/**< The Level at the peak relative to Threshold. */
	float radius;			/**< The Distance of the target from the sensor. */
	float azimuth;			/**< The azimuth angle of the target. Positive values in right direction from the sensing board perspective. */
	float elevation;		/**< The elevation angle of the target. Positive values in up direction from the sensing board perspective. */
	float radial_speed;		/**< The change of radius per second. */
	float azimuth_speed;	/**< The change of azimuth angle per second. */
	float elevation_speed;	/**< The change of elevation angle per second. */
} Target_Info_t;

/** @} */


/**
 * \defgroup Calibration
 *
 * \brief Functions and types used for radar and algorithm calibration operation.
 *
 * @{
 */

/**
 * \brief Defines supported Calibration targets
 */
typedef enum
{
	CALIBRATION_TARGET_FLASH   = 0U,  	/**< EEPMROM emulated on Flash memory is used as calibration target */
    CALIBRATION_TARGET_SRAM    = 1U  	/**< SRAM is used as calibration target, not to save calibration permanently */
} Calibration_Target_t;

/**
 * \brief Defines supported Calibration data.
 */
typedef enum
{
	CALIBRATION_DATA_ADC   = 0U,  	/**< Raw ADC IQ data will be stored as calibration data */
	CALIBRATION_DATA_ALGO  = 1U     /**< Algorithm offset (Angle, Range) will be stored as calibration data */
} Calibration_Data_t;

/** @} */


/**
 * \defgroup DriverInformation
 *
 * \brief Functions and types to read driver version information.
 *
 * @{
 */

/**
 * \brief This struct holds the version of the radar driver.
 *
 * An instance of this struct must be provided to the function
 * \ref radar_get_driver_version which fills the fields of the struct.
 */
typedef struct
{
	uint8_t major;      /**< The major version number of the driver. */
    uint8_t minor;      /**< The minor version number of the driver. */
    uint8_t revision;   /**< The revision number of the driver. */
} Driver_Version_t;

/** @} */

/**
 * \defgroup DeviceInfo
 *
 * \brief Functions and types to read information about the radar device.
 * @{
 */

/**
 * \brief This enumeration type lists the possible formats of sampled radar
 *        data.
 *
 * The format of sampled radar data acquired by the driver and retrieved
 * through a struct of type \ref Frame_Info_t by calling \ref radar_get_frame
 * depends on the radar sensor hardware.
 *
 * The radar front end comprises a mixer that demodulates the RF signal
 * received through the connected antennas down to the base band, where it is
 * captured by an Analog/Digital Converter (ADC). A RF mixer can output two
 * signals with a phase shift of 90 degrees, called I-signal and Q-signal. These two
 * signals can be considered to be real and imaginary parts of a complex valued
 * signal. Depending on the sensor hardware, the sensor may be able to capture
 * only one or both signals with its ADCs.
 *
 * If only one signal is captured, the radar frame data is real valued, so a
 * block of N samples consists of N values. If both signals are captured, a
 * block of N samples consists of 2*N values, which may be interleaved (each
 * real value is followed by the according imaginary value) or straight (N real
 * samples followed by N imaginary samples).
 *
 * A value of this type is part of the struct \ref Device_Info_t.
 */
typedef enum
{
    RADAR_RX_DATA_REAL                					= 0, 			/**< The frame data contains only I or Q signal
    																		 with non-continuous chirps. */
    RADAR_RX_DATA_COMPLEX             					= 1, 			/**< The frame data contains I and Q signals in
    																		 separate data blocks with non-continuous chirps. */
    RADAR_RX_DATA_COMPLEX_INTERLEAVED 					= 2,  			/**< The frame data contains I and Q signals in
    																		 one interleaved data block with non-continuous chirps. */
	RADAR_RX_DATA_REAL_CONTINUOUS_CHIRPS    			= (0x8 | 0x0),  /**< The frame data contains only I or Q signal
																			 with continuous chirps */
	RADAR_RX_DATA_COMPLEX_CONTINUOUS_CHIRPS 			= (0x8 | 0x1), 	/**< The frame data contains continuous chirps with
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 I and Q signals in separate data blocks. */
	RADAR_RX_DATA_COMPLEX_INTERLEAVED_CONTINUOUS_CHIRPS = (0x8 | 0x2)  	/**< The frame data contains continuous chirps with
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 I and Q signals in one interleaved data block. */
} Rx_Data_Format_t;

/**
 * \brief This struct holds information about a device and its capabilities.
 *
 * A struct of this type must be provided to \ref radar_get_device_info when
 * reading the device information.
 */
typedef struct
{
    const char*      description;          /**< A pointer to a null terminated
                                                string holding a human
                                                readable description of the
                                                device. */
    uint32_t         min_rf_frequency_kHz; /**< The minimum RF frequency the
                                                the sensor device can emit. */
    uint32_t         max_rf_frequency_kHz; /**< The maximum RF frequency the
                                                sensor device can emit. */
    uint8_t          num_tx_antennas;      /**< The number of RF antennas used
                                                for transmission. */
    uint8_t          num_rx_antennas;      /**< The number of RF antennas used
                                                for reception. */
    uint8_t          max_tx_power;         /**< The amount of RF transmission
                                                power can be controlled in the
                                                range of 0 ... max_tx_power.
                                                */
    uint8_t          num_temp_sensors;     /**< The number of available
                                                temperature sensors. */
    uint8_t          major_version_hw;     /**< The major version number of
                                                the sensor hardware. */
    uint8_t          minor_version_hw;     /**< The minor version number of
                                                the sensor hardware. */
    uint8_t          interleaved_rx;       /**< If this is 0, the radar data
                                                of multiple RX antennas is
                                                stored in consecutive data
                                                blocks, where each block holds
                                                data of one antenna. If this
                                                is non-zero, the radar data of
                                                multiple RX antennas is stored
                                                in one data block, where for
                                                each point in time the samples
                                                from all RX antennas are
                                                stored consecutively before
                                                the data of the next point in
                                                time follows. */
    Rx_Data_Format_t data_format;          /**< If the device can acquire
                                                complex data, this indicates
                                                if real and imaginary part are
                                                interleaved or not. */
} Device_Info_t;

/** @} */


/**
 * \defgroup FrameConfiguration
 *
 * \brief Functions and types to read and modify configuration of radar data
 *        frames.
 *
 * @{
 */

/**
 * \brief This enumeration type lists the possible signal parts' formats of
 *        sampled radar data.
 *
 * If the device is capable to capture complex signal data (I and Q), the
 * values of this enumeration allow to chose between the full complex signal,
 * or just the real or imaginary signal part.
 *
 * A value of this type is part of the struct \ref Frame_Format_t.
 */
typedef enum
{
    RADAR_SIGNAL_ONLY_I  = 0, /**< Only the I signal is captured during radar
                                   data frame acquisition. */
    RADAR_SIGNAL_ONLY_Q  = 1, /**< Only the Q signal is captured during radar
                                   data frame acquisition. */
    RADAR_SIGNAL_I_AND_Q = 2  /**< Both, I and Q signal are captured as a
                                   complex signal during radar data frame
                                   acquisition. */
} Signal_Part_t;

/**
 * \brief This struct holds the parameters that define the structure of a
 *        radar data frame.
 *
 * A struct of this type must be provided to \ref radar_set_frame_format
 * and \ref radar_get_frame_format.
 */
typedef struct
{
    uint32_t      num_samples_per_chirp;  /**< The number of samples captured
                                               during each chirp. This value
                                               affects the duration of a
                                               single chirp. Must not be 0. */
    uint32_t      num_chirps_per_frame;   /**< The number of chirps that are
                                               processed consecutively in a
                                               frame. Must not be 0. */
    uint8_t       rx_mask;                /**< Each available RX antenna is
                                               represented by a bit in this
                                               mask. If a bit is set, the IF
                                               signal received through the
                                               according RX antenna is
                                               captured during chirp
                                               processing. */
    Signal_Part_t signal_part;            /**< The part of a complex signal to
                                               be captured. */
} Frame_Format_t;

/** @} */

/**
 * \defgroup DataAcquisition
 *
 * \brief Functions and types to acquire radar data frames.
 *
 * @{
 */

/**
 * \brief This struct holds all information about a single frame of radar
 *        data.
 *
 * A struct of this type must be provided to \ref radar_get_frame.
 *
 * The data type of samples in sample_data depends on the ADC resolution. If
 * adc_resolution <= 8, sample_data points to an array of uint8_t. If
 * 8 < adc_resolution <= 16, sample_data points to an array of uint16_t.
 * Higher ADC resolutions are currently not supported.
 *
 * If the frame contains more than one chirp, the chirps are stored in
 * consecutive data blocks. The  start of each chirp can be calculated by the
 * following formula.
 * \code
 * frame_start = &sample_data[CHIRP_NUMBER *
 *                            num_rx_antennas * num_samples_per_chirp *
 *                            ((data_format == RADAR_RX_DATA_REAL)? 1 : 2)];
 * \endcode
 *
 * The index calculation of a certain data sample captured from a certain RX
 * antenna depends on the data interleaving and on real/complex data format.
 * The following code examples demonstrate how to access a data sample.
 * \code
 * // interleaved_rx = 0, data_format = RADAR_RX_DATA_REAL
 * // --------------------------------------------------------------------
 * value = frame_start[ANTENNA_NUMBER * num_samples_per_chirp +
 *                     SAMPLE_NUMBER];
 *
 * // interleaved_rx = 0, data_format = RADAR_RX_DATA_COMPLEX
 * // --------------------------------------------------------------------
 * value_real = frame_start[(2 * ANTENNA_NUMBER    ) * num_samples_per_chirp +
 *                          SAMPLE_NUMBER];
 * value_imag = frame_start[(2 * ANTENNA_NUMBER + 1) * num_samples_per_chirp +
 *                          SAMPLE_NUMBER];
 *
 * // interleaved_rx = 0, data_format = RADAR_RX_DATA_COMPLEX_INTERLEAVED
 * // --------------------------------------------------------------------
 * value_real = frame_start[2 * ANTENNA_NUMBER * num_samples_per_chirp +
 *                          SAMPLE_NUMBER];
 * value_imag = frame_start[2 * ANTENNA_NUMBER * num_samples_per_chirp +
 *                          SAMPLE_NUMBER + 1];
 *
 * // interleaved_rx = 1, data_format = RADAR_RX_DATA_REAL
 * // --------------------------------------------------------------------
 * value = pFrameStart[SAMPLE_NUMBER * num_rx_antennas + ANTENNA_NUMBER];
 *
 * // interleaved_rx = 1, data_format = RADAR_RX_DATA_COMPLEX
 * // --------------------------------------------------------------------
 * value_real = frame_start[SAMPLE_NUMBER * num_rx_antennas + ANTENNA_NUMBER];
 * value_imag = frame_start[(num_samples_per_chirp + SAMPLE_NUMBER) *
 *                          num_rx_antennas + ANTENNA_NUMBER];
 *
 * // interleaved_rx = 1, data_format = RADAR_RX_DATA_COMPLEX_INTERLEAVED
 * // --------------------------------------------------------------------
 * value_real = frame_start[2 * SAMPLE_NUMBER * num_rx_antennas +
 *                          ANTENNA_NUMBER];
 * value_imag = frame_start[2 * SAMPLE_NUMBER * num_rx_antennas +
 *                          ANTENNA_NUMBER + 1];
 * \endcode
 *
 * In the code examples above, ANTENNA_NUMBER does not refer to the physical
 * Antennae number, but the n-th enabled antenna. If for example rx_mask = 0xA,
 * ANTENNA_NUMBER=0 refers to antenna 0010b and ANTENNA_NUMBER=1 refers to
 * antenna 1000b. Furthermore it is assumed that pDataBuffer and frame_start
 * are casted to (uint8_t*) or (uint16_t*) according to adc_resolution.
 *
 * In total the data buffer contains
 * uNumChirps * uNumRXAntennas * uNumSamplesPerChirp values. If radar data is
 * complex, that value is multiplied by 2.
 */
typedef struct
{
    const void*      sample_data;            /**< The buffer containing the
                                                  radar data */
    uint32_t         frame_number;           /**< The running number of the
                                                  data frame. The frame
                                                  counter is, reset every time
                                                  \ref radar_set_automatic_frame_trigger
                                                  is called. If automatic
                                                  frame trigger is not active,
                                                  the frame counter may not
                                                  work, and this could be 0.
                                                  */
    uint32_t         num_chirps;             /**< The number of chirps in this
                                                  frame. */
    uint8_t          num_rx_antennas;        /**< The number of RX signals
                                                  that have been acquired with
                                                  each chirp. */
    uint32_t         num_samples_per_chirp;  /**< The number of samples
                                                  acquired in each chirp for
                                                  each enabled RX antenna. */
    int32_t          temperature_001C;       /**< The temperature during chirp
                                                  acquisition. This value is
                                                  always measured using the
                                                  first temperature sensor. */
    uint8_t          rx_mask;                /**< Each antenna is represented
                                                  by a bit in this mask. If
                                                  the bit is set, the
                                                  according RX antenna was
                                                  used to capture data in this
                                                  frame. */
    uint8_t          adc_resolution;         /**< The ADC resolution of the
                                                  data in pDataBuffer. */
    uint8_t          interleaved_rx;         /**< If this is 0, the radar data
                                                  of multiple RX antennas is
                                                  stored in consecutive data
                                                  blocks, where each block
                                                  holds data of one antenna.
                                                  If this is non-zero, the
                                                  radar data of multiple RX
                                                  antennas is stored in one
                                                  data block, where for each
                                                  point in time the samples
                                                  from all RX antennas are
                                                  stored consecutively before
                                                  the data of the next point
                                                  in time follows. */
    Rx_Data_Format_t data_format;            /**< This indicates if the data
                                                  is pDataBuffer is real or
                                                  complex, and if complex data
                                                  is interleaved. */
} Frame_Info_t;

/**
 * \brief Defines supported modulation types. Use type Modulation_Type_t for this enum.
 */
typedef enum
{
	MODULATION_DOPPLER	= 0U,  	/**< Doppler Modulation for speed calculation */
	MODULATION_FMCW		= 1U	/**< FMCW Modulation for range calculation*/
} Modulation_Type_t;

/**
 * \brief Data structure for Algorithm related Calibrations values.
 * @{
 */
typedef struct
{
	uint16_t 	distance_offset_cm;		/**< Distance offset in units of cm */
	int16_t		angle_offset_deg;		/**< Angle offset in units of degrees */
} 	Algo_Calibrations_t;

typedef void* Radar_Handle_t;

typedef void(*algorithm)(acq_buf_obj *p_acq_buf, const algo_settings_t *cp_algo_settings, const device_settings_t *cp_dev_settings, algo_result_t *p_algo_result);

/*
==============================================================================
   4. DATA
==============================================================================
 */

extern volatile uint32_t   g_adc_sampling_completed;   /**< Global variable to inform about the sampling done status, used by algorithm to start processing */
extern volatile uint32_t   g_do_processing;            /**< Global variable to enable/disable post processing after data acquisition, valid range [0 - 1] */

/* Data memory section in the SRAM containing the current calibration contents of EEPROM */
extern uint16_t    adc_i1_calibration[MAX_CALIB_SAMPLES_PER_CHANNEL * BSP_NUM_OF_ADC_CHANNELS];
extern uint16_t*   adc_q1_calibration;
extern uint16_t*   adc_i2_calibration;
extern uint16_t*   adc_q2_calibration;

/******************************************************************************
   5. FUNCTION PROTOTYPES
*******************************************************************************/

/**
 * \brief	This function registers user callback functions in BSP layer.
 *          It delegates user callback to the BSP low layer driver.
 *
 * \param[in] 	cb_id		 Radar callback ID \ref Radar_Callback_ID_t
 * \param[in] 	*pcallback   A pointer to user callback function
 *
 * \return One of the following status can be return:
 *   	   - \ref RADAR_STATUS_SUCCESS    Success
 *   	   - \ref RADAR_STATUS_FAIL       Fail
 *   	   - \ref RADAR_STATUS_NA         Feature not available
 */
uint16_t radar_register_callback(Radar_Callback_ID_t cb_id, void (*pcallback)(void *parameter));

/**
 * \brief	This function unregisters user callback functions from BSP layer.
 *
 * \param[in] 	cb_id		Radar callback ID \ref Radar_Callback_ID_t
 *
 * \return One of the following status can be return:
 *   	   - \ref RADAR_STATUS_SUCCESS    Success
 *   	   - \ref RADAR_STATUS_FAIL       Fail
 *   	   - \ref RADAR_STATUS_NA         Feature not available
 */
uint16_t radar_unregister_callback(Radar_Callback_ID_t cb_id);

/**
 * \brief	This function applies radar hardware settings.
 *
 * \param[in] 	device           A handle to the radar sensor device.
 * \param[in] 	state		     Radar hardware state \ref hw_state_setting_t
 * \param[in] 	*p_hw_settings   A pointer to hardware configuration structure \ref device_settings_t
 *
 * \return One of the following status can be return:
 *   	   - \ref RADAR_STATUS_SUCCESS    Success
 *   	   - \ref RADAR_STATUS_FAIL       Fail
 *   	   - \ref RADAR_STATUS_NA         Feature not available
 */
uint32_t radar_apply_hw_settings(Radar_Handle_t device, hw_state_setting_t state, device_settings_t *p_hw_settings);

/**
 * \brief	This function returns the radar data acquisition mode.
 *
 * \return One of the following modes can be return \ref Radar_Data_Acq_Mode_t:
 *   	   - \ref RADAR_MANUAL_TRIGGER    Manual trigger
 *   	   - \ref RADAR_AUTO_TRIGGER      Automatic trigger
 *   	   - \ref RADAR_TEST_MODE         Test mode
 */
Radar_Data_Acq_Mode_t radar_get_data_acq_mode(void);

/**
 * \brief	This function sets the radar data acquisition mode: Manual, Automatic or Test mode.
 *
 * \param[in] 	new_mode		Radar data acquisition mode \ref Radar_Data_Acq_Mode_t
 */
void radar_set_data_acq_mode(Radar_Data_Acq_Mode_t new_mode);


/**
 * \brief This function modifies the current ADC configuration.
 *
 * The function changes the ADC parameters of the XMC's built in
 * Analog/Digital Converter according to the provided structure of type
 * \ref Adc_Configuration_t.
 *
 * A value check is applied to each field of pConfiguration. If any value or
 * the combination of the values is out of range, the ADC parameters are not
 * changed and an error is returned.
 *
 * If the field uSamplingRate in pConfiguration is 0 the sampling rate will be
 * set to the highest possible value.
 *
 * The configuration can not be changed while the automatic trigger is active.
 *
 * \param[in] device         A handle to the radar sensor device.
 * \param[in] configuration  A pointer to the new ADC parameters to be applied.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                      if the ADC parameters
 *                                                  could be applied
 *         - \ref RADAR_ERR_BUSY                    if the automatic trigger
 *                                                  is active and the device
 *                                                  is busy with acquiring
 *                                                  radar data
 *         - \ref RADAR_ERR_SAMPLERATE_OUT_OF_RANGE if the specified sampling
 *                                                  rate is not supported by
 *                                                  the ADC
 *         - \ref RADAR_ERR_UNSUPPORTED_RESOLUTION  if the specified ADC
 *                                                  resolution is not
 *                                                  supported by the ADC
 */
uint16_t radar_set_adc_configuration(Radar_Handle_t device,
                                     const Adc_Configuration_t* configuration);

/**
 * \brief This function returns the current ADC configuration.
 *
 * This function returns the current ADC parameters of the XMC's built in
 * Analog/Digital Converter through a struct of type
 * \ref Adc_Configuration_t. The struct must be provided by the user and
 * may be uninitialized.
 *
 * While automatic trigger is active, reading the current frame format may
 * fail.
 *
 * \param[in]  device        A handle to the radar sensor device.
 * \param[out] configuration A pointer to a location, where the current ADC
 *                           configuration will be stored.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                      if the ADC parameters
 *                                                  could be applied
 *         - \ref RADAR_ERR_BUSY                    if the automatic trigger
 *                                                  is active and the device
 *                                                  is busy with acquiring
 *                                                  radar data
 */
uint16_t radar_get_adc_configuration(Radar_Handle_t device,
                                     Adc_Configuration_t* configuration);

/** @} */


/**
 * \addtogroup FMCWConfiguration
 *
 * @{
 */

/**
 * \brief This function modifies the RF parameters in FMCW radar mode.
 *
 * The function changes the RF parameters for FMCW radar mode according to the
 * provided structure of type \ref Fmcw_Configuration_t. After calling this
 * function, the device will be in FMCW radar mode.
 *
 * A value check is applied to each field of configuration. If any value or
 * the combination of the values is out of range, the FMCW parameters are not
 * changed and an error is returned.
 *
 * The configuration can not be changed while the automatic trigger is active.
 *
 * \param[in] device         A handle to the radar sensor device.
 * \param[in] configuration  A pointer to the new FMCW parameters to be
 *                           applied.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                      if the FMCW parameters
 *                                                  could be applied
 *         - \ref RADAR_ERR_BUSY                    if the automatic trigger
 *                                                  is active and the device
 *                                                  is busy with acquiring
 *                                                  radar data
 *         - \ref RADAR_ERR_FREQUENCY_OUT_OF_RANGE  if the upper and/or lower
 *                                                  FMCW frequency is not in
 *                                                  the radar sensor's RF
 *                                                  range
 *         - \ref RADAR_ERR_POWER_OUT_OF_RANGE      if the specified TX power
 *                                                  value exceeds the maximum
 *                                                  value
 *         - \ref RADAR_ERR_UNSUPPORTED_DIRECTION   if the device does not
 *                                                  support the requested
 *                                                  chirp direction
 */
uint16_t radar_set_fmcw_configuration(Radar_Handle_t device,
                                      const Fmcw_Configuration_t* configuration);

/**
 * \brief This function returns the current RF parameters in FMCW radar mode.
 *
 * This function returns the current RF parameters in FMCW mode through a
 * struct of type \ref Fmcw_Configuration_t. The struct must be provided by
 * the user and may be uninitialized.
 *
 * While automatic trigger is active, reading the current frame format may
 * fail.
 *
 * \param[in]  device         A handle to the radar sensor device.
 * \param[out] configuration  A pointer to a location, where the current FMCW
 *                            configuration will be stored.
 */
uint16_t radar_get_fmcw_configuration(Radar_Handle_t device,
                                      Fmcw_Configuration_t* configuration);

/**
 * \brief This function returns the FMCW ramp speed in in current
 *        configuration.
 *
 * This function writes the RF frequency change per second to an integer
 * variable. The variable must be provided by the user and may be
 * uninitialized.
 *
 * The unit of chirp duration value is MHz/s.
 *
 * While automatic trigger is active, reading the chirp duration may fail.
 *
 * \param[in]  device                      A handle to the radar sensor
 *                                         device.
 * \param[out] bandwidth_per_second_MHz_s  A pointer to a location, where the
 *                                         FMCW ramp speed value will be
 *                                         stored.
 */
uint16_t radar_get_bandwidth_per_second(Radar_Handle_t device,
                                        uint32_t* bandwidth_per_second_MHz_s);

/** @} */


/**
 * \addtogroup DopplerConfiguration
 *
 * @{
 */

/**
 * \brief This function modifies the RF parameters in Doppler radar mode.
 *
 * The function changes the RF parameters for Doppler radar mode according to
 * the provided structure of type \ref Doppler_Configuration_t. After calling
 * this function, the device will be in Doppler radar mode.
 *
 * A value check is applied to each field of pConfiguration. If any value or
 * the combination of the values is out of range, the FMCW parameters are not
 * changed and an error is returned.
 *
 * The configuration can not be changed while the automatic trigger is active.
 *
 * \param[in] device         A handle to the radar sensor device.
 * \param[in] configuration  A pointer to the new Doppler parameters to be
 *                           applied.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                      if the FMCW parameters
 *                                                  could be applied
 *         - \ref RADAR_ERR_BUSY                    if the automatic trigger
 *                                                  is active and the device
 *                                                  is busy with acquiring
 *                                                  radar data
 *         - \ref RADAR_ERR_FREQUENCY_OUT_OF_RANGE  if the upper and/or lower
 *                                                  FMCW frequency is not in
 *                                                  the radar sensor's RF
 *                                                  range
 *         - \ref RADAR_ERR_POWER_OUT_OF_RANGE      if the specified TX power
 *                                                  value exceeds the maximum
 *                                                  value
 */
uint16_t radar_set_doppler_configuration(Radar_Handle_t device,
                                         const Doppler_Configuration_t* configuration);

/**
 * \brief This function returns the current RF parameters in Doppler radar
 *        mode.
 *
 * This function returns the current RF parameters in Doppler mode through a
 * struct of type \ref Doppler_Configuration_t. The struct must be provided by
 * the user and may be uninitialized.
 *
 * While automatic trigger is active, reading the current frame format may
 * fail.
 *
 * \param[in]  device         A handle to the radar sensor device.
 * \param[out] configuration  A pointer to a location, where the current
 *                            Doppler configuration will be stored.
 */
uint16_t radar_get_doppler_configuration(Radar_Handle_t device,
                                         Doppler_Configuration_t* configuration);

/** @} */


/**
 * \addtogroup TargetDetection
 *
 * @{
 */

/**
 * \brief This function modifies DSP settings for target detection with FMCW and Doppler modulation.
 *
 * A value check is applied to each field of configuration. If any value or
 * the combination of the values is out of range, the FMCW parameters are not
 * changed and an error is returned.
 *
 * The configuration can not be changed while the automatic trigger is active.
 *
 * \param[in] device          A handle to the radar sensor device.
 * \param[in] *configuration  A pointer of type \ref DSP_Settings_t, containing the new DSP settings.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                      if the DSP settings
 *                                                  could be applied
 *         - \ref RADAR_ERR_BUSY                    if the automatic trigger
 *                                                  is active and the device
 *                                                  is busy with acquiring
 *                                                  radar data
 */
uint16_t radar_set_dsp_settings(Radar_Handle_t device,
		                        const DSP_Settings_t* configuration);

/**
 * \brief This function returns DSP settings for target detection with FMCW and Doppler modulation.
 *
 * \param[in]  device          A handle to the radar sensor device.
 * \param[out] *configuration  A pointer of type \ref DSP_Settings_t, containing the current DSP settings.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                      if the DSP settings
 *                                                  could be applied
 *         - \ref RADAR_ERR_BUSY                    if the automatic trigger
 *                                                  is active and the device
 *                                                  is busy with acquiring
 *                                                  radar data
 */
uint16_t radar_get_dsp_settings(Radar_Handle_t device,
		                        DSP_Settings_t* configuration);

/**
 * \brief This function returns target info defined in \ref Target_Info_t for detected targets with FMCW or Doppler.
 *
 * \param[in]  device        A handle to the radar sensor device.
 * \param[out] *target_info  A pointer of type \ref Target_Info_t, containing the detected targets information,
 * 							 e.g. target_id, target FFT amplitude spectrum level, its radial speed and radial distance etc.
 * \param[out] target_count	 Count of the detected targets using FMCW as for Doppler it is always One.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                      if the DSP settings
 *                                                  could be applied
 *         - \ref RADAR_ERR_BUSY                    if the automatic trigger
 *                                                  is active and the device
 *                                                  is busy with acquiring
 *                                                  radar data
 */
uint16_t radar_get_target_info(Radar_Handle_t device,
		                       Target_Info_t* target_info, uint8_t* target_count);

/**
 * \brief This function returns the detection threshold (in terms of FFT absolute
 *        amplitude spectrum) for distance calculation in FMCW.
 *
 * \param[in]  device      A handle to the radar sensor device.
 * \param[out] *threshold  A pointer to floating point, to be filled with
 *                         the current FMCW range detection threshold.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                      if the DSP settings
 *                                                  could be applied
 */
uint16_t radar_get_range_detection_threshold(Radar_Handle_t device, uint16_t *threshold);

/** @} */


/**
 * \addtogroup DriverInformation
 *
 * @{
 */

/**
 * \brief This function returns the version of the driver implementation.
 *
 * The function returns a pointer to a struct of type \ref Driver_Version_t.
 * That pointer points to driver internal memory, so the struct content can't
 * be changed by the user.
 *
 * \return The version of the radar driver.
 */
const Driver_Version_t* radar_get_driver_version(void);

/** @} */

/**
 * \addtogroup OpenClose
 *
 * @{
 */

/**
 * \brief This function retrieves the number of available radar devices.
 *
 * \return The number of available interfaces.
 */
uint8_t radar_get_number_of_available_devices(void);

/**
 * \brief This function opens an available radar device.
 *
 * Before an device can be used, it must be opened with this function. If the
 * function succeeds a handle of type Radar_Handle_t is returned that must be
 * used in future function calls. When the device is not used any more, it
 * must be closed by calling the function \ref radar_close_device.
 *
 * A driver may support multiple sensor devices at a time. The number of
 * available devices can be retrieved by calling
 * \ref radar_get_number_of_available_devices.
 *
 * \param[in]  device_number  The number of the device to open. The first
 *                            interface is 0.
 * \param[out] *device        A pointer to a handle variable. If the device
 *                            could be opened, the handle is stored to that
 *                            variable.
 *
 * \note Due to performance issues, the API does not check provided device
 *       handles. Each function assumes that the provided device handle has
 *       been opened properly and that handles are not used any more after
 *       closing. If these assumptions are false this will lead to undefined
 *       behaviour.
 *
 * \return An error code indicating if the device could be opened. The
 *         following error codes can occur:
 *         - \ref RADAR_ERR_OK                    if the device could be
 *                                                initialized successfully
 *         - \ref RADAR_ERR_DEVICE_DOES_NOT_EXIST if the device does not exist
 *         - \ref RADAR_ERR_BUSY                  if the device has already
 *                                                been opened
 */
uint16_t radar_open_device(int32_t device_number, Radar_Handle_t* device);

/**
 * \brief This function closes an open device
 *
 * When a device is not used any more, it must be closed by calling this
 * function.
 *
 * \note After a device has been closed, the handle must not be used any more.
 *       The result will be undefined.
 *
 * \param[in] device  The handle to the device to be closed.
 *
 * \see radar_open_device
 */
void radar_close_device(Radar_Handle_t device);

/** @} */


/**
 * \addtogroup DeviceInfo
 *
 * @{
 */

/**
 * \brief This function returns information about a device and its
 *        capabilities.
 *
 * This function returns device information through a struct of type
 * \ref Device_Info_t. The struct must be provided by the user and may be
 * uninitialized.
 *
 * \param[in]  device       A handle to the device of whose information should
 *                          be read.
 * \param[out] device_info  A pointer to a location, where the device
 *                          information will be stored.
 */
uint16_t radar_get_device_info(Radar_Handle_t device,
                               Device_Info_t* device_info);

/**
 * \brief This function returns the current temperature of the sensor device.
 *
 * This function writes the current temperature value to an integer variable.
 * The variable must be provided by the user and may be uninitialized.
 *
 * The unit of the temperature value is 0.001 degree Celsius.
 *
 * A radar sensor device may contain more than one temperature sensors, so the
 * number of the sensor to be used for measuring must be specified. The number
 * of available temperature sensors is part of the \ref Device_Info_t structure
 * returned by \ref radar_get_device_info. The number of the first temperature
 * sensor is 0.
 *
 * A call to this function not necessarily triggers a temperature measurement.
 * The function may return the last known temperature of the specified sensor.
 *
 * \param[in]  device            A handle to the radar sensor device.
 * \param[in]  temp_sensor       The number of the temperature sensor to be
 *                               used.
 * \param[out] temperature_001C  A pointer to a location, where the
 *                               temperature value will be stored.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                     if a valid temperature
 *                                                 could be returned
 *         - \ref RADAR_ERR_SENSOR_DOES_NOT_EXIST  if the specified
 *                                                 temperature sensor does not
 *                                                 exist
 */
uint16_t radar_get_temperature(Radar_Handle_t device, uint8_t temp_sensor,
                               int32_t* temperature_001C);

/**
 * \brief This function returns the RF transmission power of the specified TX
 *        antenna.
 *
 * This function writes the current RF transmission power value to an integer
 * variable. The variable must be provided by the user and may be
 * uninitialized.
 *
 * The unit of the RF transmission power value is 0.001 dBm.
 *
 * Typically a radar sensor device contains a power sensor for each
 * transmission antenna. The number of transmission antennas is part of the
 * \ref Device_Info_t structure returned by \ref radar_get_device_info. The
 * number of the first antenna (and TX power sensor) is 0.
 *
 * A call to this function not necessarily triggers a power measurement. The
 * function may return the last known transmission power of the specified TX
 * antenna.
 *
 * \param[in]  device           A handle to the radar sensor device.
 * \param[in]  tx_antenna       The number of the TX antenna whose transmission
 *                              power is requested.
 * \param[out] tx_power_001dBm  A pointer to a location, where the
 *                              transmission power value will be stored.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                     if a valid power value
 *                                                 could be returned
 *         - \ref RADAR_ERR_SENSOR_DOES_NOT_EXIST  if the specified
 *                                                 transmission power sensor
 *                                                 does not exist
 */
uint16_t radar_get_tx_power(Radar_Handle_t device, uint8_t tx_antenna,
                            int32_t* tx_power_001dBm);

/**
 * \brief This function returns the duration for a single chirp in current
 *        configuration.
 *
 * This function writes the duration, a chirp with current configuration will
 * take, to an integer variable. The variable must be provided by the user and
 * may be uninitialized.
 *
 * The unit of chirp duration value is nanoseconds.
 *
 * While automatic trigger is active, reading the chirp duration may fail.
 *
 * \param[in]  device             	A handle to the radar sensor device.
 * \param[out] chirp_duration_nsec  A pointer to a location, where the chirp
 *                                	duration value will be stored
 *                                	in units of nanoseconds.
 */
uint16_t radar_get_chirp_duration(Radar_Handle_t device,
                                 uint32_t* chirp_duration_nsec);

/**
 * \brief This function returns the minimum frame interval possible in current
 *        configuration.
 *
 * This function writes the minimum frame interval currently possible to an
 * integer variable. The variable must be provided by the user and may be
 * uninitialized.
 *
 * The unit of frame interval value is microseconds.
 *
 * Passing a value to \ref radar_set_automatic_frame_trigger that is smaller
 * than the returned one will result in an error.
 *
 * While automatic trigger is active, reading the minimum frame interval may
 * fail.
 *
 * \param[in]  device                 	A handle to the radar sensor device.
 * \param[out] min_frame_interval_usec  A pointer to a location, where the chirp
 *                                    	duration value will be stored
 *                                    	in units of microseconds.
 */
uint16_t radar_get_min_frame_interval(Radar_Handle_t device,
                                      uint32_t* min_frame_interval_usec);

/** @} */


/**
 * \addtogroup FrameConfiguration
 *
 * @{
 */

/**
 * \brief This function modifies the current radar data frame format.
 *
 * The function changes the radar data format according to the provided
 * structure of type \ref Frame_Format_t.
 *
 * A value check is applied to each field of configuration. If any value or
 * the combination of the values is out of range, the frame format is not
 * changed and an error is returned.
 *
 * The configuration can not be changed while the automatic trigger is active.
 *
 * \param[in] device        A handle to the radar sensor device.
 * \param[in] frame_format  A pointer to the new frame configuration to be
 *                          applied.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                        if the frame
 *                                                    configuration could be
 *                                                    applied
 *         - \ref RADAR_ERR_BUSY                      if the automatic trigger
 *                                                    is active and the device
 *                                                    is busy with acquiring
 *                                                    radar data
 *         - \ref RADAR_ERR_ANTENNA_DOES_NOT_EXIST    if one ore more of the
 *                                                    antennas requested by
 *                                                    rx_mask does not exist
 *         - \ref RADAR_ERR_UNSUPPORTED_FRAME_FORMAT  if a radar data frame
 *                                                    with the specified
 *                                                    configuration would
 *                                                    exceed the internal
 *                                                    memory
 *         - \ref RADAR_ERR_UNAVAILABLE_SIGNAL_PART   if the selected signal
 *                                                    part can't be captured
 *                                                    by the device.
 */
uint16_t radar_set_frame_format(Radar_Handle_t device,
                                  const Frame_Format_t* frame_format);

/**
 * \brief This function returns the current frame configuration.
 *
 * This function returns the current frame configuration through a structure of
 * type\ref Frame_Format_t. The structure must be provided by the user and may be
 * uninitialized.
 *
 * While automatic trigger is active, reading the current frame format may
 * fail.
 *
 * \param[in]  device         A handle to the radar sensor device.
 * \param[out] frame_format   A pointer to a location, where the current frame
 *                            configuration will be stored.
 */
uint16_t radar_get_frame_format(Radar_Handle_t device,
                                      Frame_Format_t* frame_format);

/** @} */


/**
 * \addtogroup DataAcquisition
 *
 * @{
 */

/**
 * \brief This function returns the next frame of radar data.
 *
 * If the automatic frame trigger is active, this function returns the next
 * frame of radar data from internal FIFO memory. If no data is available and
 * wait_for_data is non-zero, the function will wait until a new frame of
 * radar data becomes available. If bWaitForData is zero, the function will
 * return immediately with error code RADAR_ERR_TIMEOUT if no data is
 * available.
 *
 * If the automatic frame trigger is not active, this function triggers the
 * acquisition of a single frame and waits until it has completed.
 * wait_for_data will be ignored.
 *
 * Without automatic frame trigger or with wait_for_data != 0, this function
 * is always expected to return a valid frame of data. Anyway the function may
 * also return with error code RADAR_ERR_TIMEOUT, if the acquisition of a
 * data frame takes too long. If this happens a malfunction has occurred.
 *
 * The radar data is returned through a structure of type \ref Frame_Info_t. The
 * structure must be provided by the user and may be uninitialized.
 *
 * \param[in]  device         A handle to the radar sensor device.
 * \param[out] frame_info     A pointer to a location, where the data frame
 *                            information will be stored.
 * \param[in]  wait_for_data  If automatic trigger is active a non zero value
 *                            let's the function wait until valid data is
 *                            available.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK        if the a frame of radar data could be
 *                                    returned
 *         - \ref RADAR_ERR_TIMEOUT   if no radar data is available (either
 *                                    because wait_for_data = 0 and the
 *                                    internal data FIFO memory is empty or a
 *                                    malfunction has occurred)
 *         - \ref RADAR_ERR_BUSY      if the device is not ready for data
 *                                    acquisition (maybe it's in test mode)
 */
uint16_t radar_get_frame(Radar_Handle_t device, Frame_Info_t* frame_info,
                        uint8_t wait_for_data);

/**
 * \brief This function starts or stops automatic radar frame trigger.
 *
 * This function starts the automatic frame trigger if uFrameInterval is
 * non-zero. If frame_interval_usec is zero, the automatic trigger is stopped.
 *
 * While the automatic frame trigger active, an internal timer triggers the
 * acquisition of radar data frames with the given interval and stores the
 * acquired data in an internal FIFO memory. The data can be retrieved from
 * the FIFO memory by calling \ref radar_get_frame.
 *
 * While the automatic trigger is active an internal frame counter is
 * increased each time the trigger interval elapses. The sensor may not
 * trigger a frame in case the internal FIFO memory has runs out of capacity.
 * The frame counter is increased anyway, so the user has the chance to detect
 * this data loss. The frame counter is reset each time this function is
 * called.
 *
 * When this function is called while the frame automatic trigger is active,
 * the trigger timer is stopped and restarted with the new frame interval.
 *
 * \param[in] device               A handle to the radar sensor device.
 * \param[in] frame_interval_usec  The time interval between two consecutive
 *                                 frames in units of microseconds.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                          if the a frame of
 *                                                      radar data could be
 *                                                      returned
 *         - \ref RADAR_ERR_UNSUPPORTED_FRAME_INTERVAL  if the internal timer
 *                                                      can't be configured to
 *                                                      the given frame
 *                                                      interval
 *         - \ref RADAR_ERR_BUSY                        if the device is not
 *                                                      ready for data
 *                                                      acquisition (maybe
 *                                                      it's in test mode)
 */
uint16_t radar_set_automatic_frame_trigger(Radar_Handle_t device,
                                          uint32_t frame_interval_usec);

/** @} */

/**
 * \defgroup RadarControl
 *
 * \brief Functions to initialize and control radar device.
 *
 * @{
 */

/**
 * \brief  This function initializes the BGT, PLL, USB and Timers.
 *
 * Must be called in open device method. After initializing the device
 * it puts device in power down mode.
 *
 * \param[in] device  A handle to the radar sensor device.
 *
 * \return One of the following status can be return:
 *   	   - \ref RADAR_STATUS_SUCCESS    Success
 *   	   - \ref RADAR_STATUS_FAIL       Fail
 *   	   - \ref RADAR_STATUS_NA         Feature not available
 */
uint16_t radar_init(Radar_Handle_t device);

/**
 * \brief This function starts the acquisition of data.
 *
 * This function sets the Frame Interval period . Afterwards starts the timer
 * for data acquisition trigger.A valid frame interval must be provided by
 * the user in config.h file.
 *
 * \param[in] device  A handle to the radar sensor device.
 *
 * \return One of the following status can be return:
 *   	   - \ref RADAR_STATUS_SUCCESS	Success
 *   	   - \ref RADAR_STATUS_FAIL    	Fail
 *   	   - \ref RADAR_STATUS_NA      	Feature not available
 */
uint16_t radar_start_acquisition(Radar_Handle_t device);

/**
 * \brief This function stops the acquisition of data.
 *
 * This function sets the Frame Interval period to Zero and stops the
 * timer for data acquisition trigger through the firmware base API method
 * \ref radar_set_automatic_frame_trigger().
 *
 * \param[in] device  A handle to the radar sensor device.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                          if the a frame of
 *                                                      radar data could be
 *                                                      returned
 *         - \ref RADAR_ERR_UNSUPPORTED_FRAME_INTERVAL  if the internal timer
 *                                                      can't be configured to
 *                                                      the given frame
 *                                                      interval
 *         - \ref RADAR_ERR_BUSY                        if the device is not
 *                                                      ready for data
 *                                                      acquisition (maybe
 *                                                      it's in test mode)
 */
uint16_t radar_stop_acquisition(Radar_Handle_t device);

/**
 * \brief This function sets the bandwidth, which is swept during one chirp.
 *
 * \param[in] device         A handle to the radar sensor device.
 * \param[in] bandwidth_MHz  A floating point value which should be within the
 *                           limits of \ref BSP_MIN_BANDWIDTH_MHZ & \ref BSP_MAX_BANDWIDTH_MHZ
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - \ref RADAR_ERR_OK                      if the PLL parameters
 *                                                  could be applied
 *         - \ref RADAR_ERR_SAMPLERATE_OUT_OF_RANGE if the specified bandwidth
 *                                                  is not supported by
 *                                                  the PLL
 */
uint16_t radar_set_bandwidth(Radar_Handle_t device, const float bandwidth_MHz);

/**
 * \brief This function returns the bandwidth, which is swept during one chirp.
 *
 * \param[in]  device          A handle to the radar sensor device.
 * \param[out] *bandwidth_MHz  Pointer of floating type value which should be filled
 *                             with the current bandwidth used by PLL chirps.
 */
void radar_get_bandwidth(Radar_Handle_t device, float* bandwidth_MHz);

/**
 * \brief This function sets the PLL base frequency.
 *
 * \param[in] device    A handle to the radar sensor device.
 * \param[in] freq_MHz  Floating type value for the PLL base frequency,
 *                      which should be within the limits of [24020 , 24240] MHz.
 *
 * \return One of the following status can be return:
 *   	   - \ref RADAR_STATUS_SUCCESS	Success
 *   	   - \ref RADAR_STATUS_FAIL    	Fail
 *   	   - \ref RADAR_STATUS_NA      	Feature not available
 */
uint16_t radar_set_pll_frequency(Radar_Handle_t device, float freq_MHz);

/**
 * \brief This function returns the PLL base frequency.
 *
 * \param[in]  device         A handle to the radar sensor device.
 * \param[out] *freq_MHz      Pointer of floating type value for the PLL base
 *                            frequency, which should be within the limits of [24020 , 24240] MHz.
 */
void radar_get_pll_frequency(Radar_Handle_t device, float* freq_MHz);

/**
 * \brief This function sets the chirp-time, used by PLL for linear FMCW chirp and CW Doppler chirp.
 *
 *  Changing chirp-time also changes the sampling frequency of the FMCW. There are some limits on this chirp time because
 *  of MCU processing capabilities and PLL configuration limitations.
 *
 * \param[in] device             A handle to the radar sensor device.
 * \param[in] chirp_time_usec    An unsigned 32-bit value, in units of microseconds (usec)
 * 								 which should be within the limits of \ref BSP_MIN_CHIRP_TIME_USEC & \ref BSP_MAX_CHIRP_TIME_USEC.
 *
 * \return One of the following status can be return:
 *   	   - \ref RADAR_STATUS_SUCCESS	Success
 *   	   - \ref RADAR_STATUS_FAIL    	Fail
 *   	   - \ref RADAR_STATUS_NA      	Feature not available
 */
uint16_t radar_set_chirp_time(Radar_Handle_t device, const uint32_t chirp_time_usec);

/**
 * \brief This function returns the chirp-time, used by PLL for linear FMCW chirp and CW Doppler chirp.
 *
 * \param[in]  device            A handle to the radar sensor device.
 * \param[out] *chirp_time_usec  Pointer to unsigned 32-bit value, in units of usec,
 *                               used to fill the current chirp-time.
 */
void radar_get_chirp_time(Radar_Handle_t device, uint32_t* chirp_time_usec);

/**
 * \brief This function returns the frame period in units of msec.
 *
 * \param[in]  device       A handle to the radar sensor device.
 * \param[out] *time_msec	Pointer of unsigned integer type value, in units of msec,
 *                          which should be filled with the current frame period.
 */
void radar_get_frame_period(Radar_Handle_t device, uint32_t* time_msec);

/**
 * \brief  This function sets the ADC Sampling frequency for FMCW or Doppler Modulation.
 *
 * \param[in] device           A handle to the radar sensor device.
 * \param[in] modulation_type  For defined modulations in \ref Modulation_Type_t
 * \param[in] freq_Hz	       Float value, representing the ADC Sampling frequency value in Hz
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - RADAR_ERR_OK                      if the test mode could be
 *                                             enabled with the specified
 *                                             settings
 *         - RADAR_ERR_PARAMETER_OUT_OF_RANGE  if the specified parameter is
 *                                             out of range
 */
uint16_t radar_set_sampling_freq(Radar_Handle_t device, Modulation_Type_t modulation_type, const float freq_Hz);

/**
 * \brief  This function returns the ADC Sampling frequency in Hz for current Modulation (FMCW or Doppler).
 *
 * \param[in]  device    A handle to the radar sensor device.
 * \param[out] *freq_Hz	 Pointer to a float type, representing the current ADC Sampling frequency value in Hz
 */
void radar_get_sampling_freq(Radar_Handle_t device, float* freq_Hz);

/** @} */

/**
 * \defgroup RadarPower
 *
 * \brief Functions to control gain levels an power consumption of BGT and PGA devices.
 *
 * @{
 */

/**
 * \brief This function sets the power level of PGA112 at the Rx side.
 *
 * \param[in] device      A handle to the radar sensor device.
 * \param[in] gain_level  Unsigned 16-bit integer, whose values are matched to the Power levels defined by enum \ref PGA_Binary_Gain_t.
 *						  There are eight binary gain levels [0 - 7] that can be set with PGA112.
 */
uint16_t radar_set_gain_level(Radar_Handle_t device, uint16_t gain_level);

/**
 * \brief This function returns the current power level of PGA112 at the Rx side.
 *
 * \param[in]  device       A handle to the radar sensor device.
 * \param[out] *gain_level  Pointer to Unsigned 16-bit integer, whose values are within the range of [1 - 8].
 */
uint16_t radar_get_gain_level(Radar_Handle_t device, uint16_t* gain_level);

/**
 * \brief This function enables/disables the duty cycle by turning PLL/BGT ON/OFF.
 *
 * The duty cycle is enabled in order to save power and reduce board temperature. By disabling the
 * duty cycle, board is kept constantly ON in order to do lab testing and measurements.
 * A valid value [0 - 1] must be provided by the user in config.h file for the DUTY_CYCLE_ENABLE field.
 *
 * \param[in] device    A handle to the radar sensor device.
 * \param[in] flag	    An unsigned 8-bit value, where '0' represents disable and non-zero value means enable duty cycle.
 */
uint16_t radar_set_duty_cycle(Radar_Handle_t device, const uint8_t flag);

/**
 * \brief This function returns the status enabled/disabled of the duty cycle.
 *
 * \param[in]  device  A handle to the radar sensor device.
 * \param[out] *flag   Pointer to unsigned 8-bit value, filled with '0' represents disabled and non-zero value means enabled duty cycle.
 */
void radar_get_duty_cycle(Radar_Handle_t device, uint8_t* flag);

/**
 * \brief This function disables the Receiver LNA gain.
 *
 * \param[in] device  A handle to the radar sensor device.
 */
void radar_disable_lna_gain(Radar_Handle_t device);

/**
 * \brief This function enables the Receiver LNA gain.
 *
 * \param[in] device  A handle to the radar sensor device.
 */
void radar_enable_lna_gain(Radar_Handle_t device);

/**
 * \brief This function returns the Receiver LNA gain enable status (enabled or disabled)
 *
 * \param[in] device  A handle to the radar sensor device.
 *
 * \return	Unsigned 8-bit integer, 0 for false (disabled) and non-zero for true (enabled) case.
 */
uint8_t radar_get_lna_gain_enable_status(Radar_Handle_t device);

/** @} */

/**
 * \addtogroup Calibration
 *
 * @{
 */

/**
 * \brief  This function clears the memory buffer for calibration data.
 *
 * \param[in]  device  A handle to the radar sensor device.
 *
 */
void radar_init_calibration(Radar_Handle_t device);

/**
 * \brief This function saves the calibration data to the SRAM or Flash based on the target memory defined by argument \ref Calibration_Target_t.
 *
 * All boards are pre-calibrated with TX power level, RX LNA gain, chirp-time, bandwidth etc... defined in the config.h file. Only for the
 * change of TX power level, LNA gain disable or different Bandwidth and chirp-time requires new calibration.
 *
 * \param[in] device                     A handle to the radar sensor device.
 * \param[in] target					 Target of the calibration defined by \ref Calibration_Target_t.
 * \param[in] calib_data_type			 Type of calibration data e.g. ADC data or Algorithm offset data defined by \ref Calibration_Data_t.
 * \param[in] *calibration_data_ptr	NULL pointer for ADC data while for Algorithm calibration data
 * 										 need to cast to \ref Algo_Calibrations_t.
 */
uint16_t radar_save_calibration(Radar_Handle_t device, Calibration_Target_t target,
		                         Calibration_Data_t calib_data_type, const void* calibration_data_ptr);

/**
 * \brief This function erase calibration data from the SRAM or Flash based on the target memory defined by argument \ref Calibration_Target_t.
 *
 * After clearing it resets the target memory with the default calibration values.
 *
 * \param[in] device           A handle to the radar sensor device.
 * \param[in] target		   Target of the calibration defined by \ref Calibration_Target_t.
 * \param[in] calib_data_type  Type of calibration data e.g. ADC data or Algo offset data defined by \ref Calibration_Data_t.
 */
uint16_t radar_clear_calibration(Radar_Handle_t device, Calibration_Target_t target,
		                          Calibration_Data_t calib_data_type);

/**
 * \brief This function reads the calibration data from the SRAM or Flash based on the target memory defined by argument \ref Calibration_Target_t.
 *
 * \param[in]  device           A handle to the radar sensor device.
 * \param[in]  target		   	Target of the calibration defined by \ref Calibration_Target_t.
 * \param[in]  calib_data_type  Type of calibration data e.g. ADC data or Algo offset data defined by \ref Calibration_Data_t.
 * \param[out] **data_ptr       Pointer to a pointer to the buffer of 16-bit unsigned integer type, containing the calibration data.
 * \param[out] *num_of_bytes    Pointer to 16-bit unsigned integer type, containing the size in bytes for the calibration data.
 */
uint16_t radar_read_calibration(Radar_Handle_t device, Calibration_Target_t target, Calibration_Data_t calib_data_type,
		                         void** data_ptr, uint16_t* num_of_bytes);

/** @} */


/**
 * \defgroup Testing
 *
 * \brief Functions and types to test the radar sensor device.
 *
 * @{
 */

/**
 * \brief This function enables a test mode for the antennas of the radar
 *        device.
 *
 * The device will emit a constant RF frequency using the specified antenna
 * configuration. The device stays in this mode until \ref radar_test_antennas
 * is called again with tx_antenna_mask = 0 and rx_antenna_mask = 0.
 *
 * If one of the specified parameters is out of range. The function does not
 * change anything.
 *
 * While the device is in test mode, normal radar operation is not possible.
 * While automatic trigger is active, the test mode can not be activated.
 *
 * \warning The device may get hot quickly in test mode. Over-heating and
 *          damage of the radar device is possible.
 *
 * \param[in] device           A handle to the radar sensor device.
 * \param[in] tx_antenna_mask  A mask to enable the antennas for transmission.
 *                             Each bit of the mask represents an available TX
 *                             antenna. If a bit is set, the according antenna
 *                             is activated.
 * \param[in] rx_antenna_mask  A mask to enable RX antennas and demodulation
 *                             circuitry. Each bit of the mask represents an
 *                             available RX antenna. If a bit is set, the
 *                             according bit is activated.
 * \param[in] frequency_kHz    The frequency of the constant tone that is
 *                             transmitted. The value is specified in kHz.
 *                             If tx_antenna_mask = 0, the specified frequency
 *                             will still be fed into the demodulation
 *                             circuitry of the RX chain.
 * \param[in] tx_power         The power value the constant tone is emitted
 *                             with. The value must be in the range of
 *                             0 ... max_tx_power in \ref Device_Info_t
 *                             returned by \ref radar_get_device_info. If
 *                             tx_antenna_mask = 0, this value is ignored.
 *
 * \return An error code indicating if the function succeeded. The following
 *         error codes can occur:
 *         - RADAR_ERR_OK                      if the test mode could be
 *                                             enabled with the specified
 *                                             settings
 *         - RADAR_ERR_ANTENNA_DOES_NOT_EXIST  if an enable bits in
 *                                             tx_antenna_mask and/or
 *                                             rx_antenna_mask for a non
 *                                             existing antenna is set
 *         - RADAR_ERR_FREQUENCY_OUT_OF_RANGE  if the specified RF frequency
 *                                             can't be emitted
 *         - RADAR_ERR_POWER_OUT_OF_RANGE      if the specified TX power value
 *                                             exceeds the maximum
 */
uint16_t radar_test_antennas(Radar_Handle_t device, uint8_t tx_antenna_mask,
                             uint8_t rx_antenna_mask, uint32_t frequency_kHz,
                             uint8_t tx_power);

/* ------------- ADC Endpoint ------------- */
/**
 * \brief	This function is used to get the current ADC configuration from data store.
 */
void ds_ep_fetch_adc_config(Adc_Configuration_t *ptr);

/**
 * \brief	This function is used to store the new ADC configuration.
 */
uint16_t ds_ep_store_adc_config(const Adc_Configuration_t *new_adc_config);

/* ------------- Doppler Endpoint ------------- */
/**
 * \brief	This function is used to get the current Doppler settings.
 */
void ds_ep_fetch_doppler_config(Doppler_Configuration_t *config);

/**
 * \brief	This function is used to set the new Doppler settings.
 */
uint16_t ds_ep_store_doppler_config(const Doppler_Configuration_t *new_doppler_config);

/* ------------- FMCW Endpoint ------------- */
/**
 * \brief	This function is used to get the current FMCW settings.
 */
void ds_ep_fetch_fmcw_config(Fmcw_Configuration_t *config);

/**
 * \brief	This function is used to set the new FMCW settings.
 */
uint16_t ds_ep_store_fmcw_config(const Fmcw_Configuration_t *new_fmcw_config);

/* ------------- Base Endpoint ------------- */
/**
 * \brief	This function is used to get the device information and its capabilities.
 */
const Device_Info_t *ds_ep_get_device_info(void);

/**
 * \brief	This function is used to get the current Frame format.
 */
void ds_ep_fetch_frame_format(Frame_Format_t *p_frame_format);

/**
 * \brief	This function is used to set the new frame format.
 */
uint16_t ds_ep_store_frame_format(const Frame_Format_t *new_frame_format);

/**
 * \brief	This function is used fetch target list.
 */
void ds_ep_fetch_target_list(Target_Info_t* target_info_ptr, uint8_t* target_count);

/**
 * \brief	This function is used to fetch range detection threshold.
 */
void ds_ep_fetch_range_detection_threshold(uint16_t* threshold);


/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_RADAR_CONTROL_H_ */

/* --- End of File --- */
