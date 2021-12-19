/*
 * types.h
 *
 *  Created on: Jul 4, 2020
 *      Author: Abdullah
 */

#ifndef TYPES_H_
#define TYPES_H_

#define PORTING_COMPLETE 1

#include <XMC4700.h>
#include <stdbool.h>
#include <stdint.h>
#include "xmc_common.h"
#include "FW/inc/drv_clk.h"
#include "FW/inc/drv_adc.h"
#include "FW/inc/drv_adc.h"
#include "FW/inc/drv_gpio.h"
#include "FW/inc/drv_mem.h"
#include "FW/inc/drv_timer.h"
#include "FW/inc/drv_eru.h"
#include "CMSIS_DSP/cmsis_dsp.h"
#include "FW/inc/drv_dma.h"
#include "USB/USBD/usbd.h"
#include "USB/USBD_VCOM/usbd_vcom.h"


#define XMC_ASSERT(msg, exp) { ; }
#define __STATIC_INLINE      static inline

#define ENABLED  (1)
#define DISABLED (0)

//============================= GENERAL CONFIG =============================//

#define FW_MODULATION_TYPE 			(1U)		/**< Firmware Modulation Type, FMCW [1] or Doppler [0] */

#define	NUM_OF_CHIRPS				(1U)		/**< Valid range of chirps in relation to the SAMPLES_PER_CHIRP, [1 - 64] */

#define	SAMPLES_PER_CHIRP 			(40U)		/**< Size of IQ raw ADC buffer, [32, 64, 128, 256] */

#define FRAME_PERIOD_MSEC 			(1030U)    	/**< Time period of one frame to capture data, [50 - ..] */

//=========================== BGT / PGA CONFIG =============================//

#define DUTY_CYCLE_ENABLE			(1U)		/**< Enable [1] or Disable [0] duty cycle of Position2Go via BGT & PLL On/Off */

#define BGT_TX_POWER				(7U)		/**< BGT TX Power levels: [1 - 7], Minimum = 1 & Maximum = 7 */

#define LNA_GAIN_ENABLE				(1U)		/**< Enable [1] or Disable [0] LNA Gain in BGT TX */

#define PGA_GAIN					(4U)		/**< PGA112 default gain value, [0 - 7] */

#define RADAR_SIGNAL_PART           (2U)        /**< IQ (complex) signal [2], only Q signal [1] or Only I signal [0], is captured
                                                     during radar data frame acquisition */

//============================ DSP CONFIG ==================================//

#define	RANGE_FFT_SIZE 				(256U)		/**< FFT length for FMCW mode, with zero padding */

#define	DOPPLER_FFT_SIZE 			(64U)		/**< FFT length for Doppler mode, with zero padding */

#define	FFT_INPUT_TYPE				(2U)        /**< FFT input type: Complex input IQ [2], real input Q [1] or real input I [0] */

#define	MAX_NUM_OF_TARGETS			(5U)		/**< Maximum number of targets to be detected, [1 - 5] */

#define RX_ANTENNA_SELECTION		(1U)		/**< RX1 antenna[1] or RX2 antenna[2] */

#define ALGO_PROCESS_TIME_USEC		(25000U)	/**< This is the minimum time(us) used to process raw data for 16 chirps and 64 samples.
                                                     This time varies based on chirps, samples and algorithm being used */

//======================== TRACKING & FILTER CONFIG ========================//

#define TRACKING_ENABLE				(0U)		/**< Enable [1], or Disable [0] Tracking */

#define CURRENT_NUM_OF_TRACKS       (5U)		/**< Maximum number detected targets to be tracked */

#define	MVG_AVG_LEN					(2U)		/**< Moving average window length, [1 - 10] */
#define MAX_MVG_AVG_LEN         	(10U)       /**< Maximum window length of Moving average Window */

#define MEDIAN_FILTER_LEN			(5U) 		/**< Length of the median filter for angle, must be Odd number */
#define MAX_MEDIAN_FILTER_LEN       (13U)       /**< Maximum length of the median filter */

#define ANGLE_QUANTIZATION			(1U)		/**< Enable and set the Number of degrees to be quantized [2 - ..], or Disable quantization [1] */

#define MTI_FILTER_ENABLE			(0U)		/**< Enable [1] or Disable [0] MTI Filter */

#define MTI_FILTER_LEN				(100U)		/**< MTI Filter weight, [1 - 1000] */

#define MIN_ANGLE_FOR_ASSIGNMENT	(50.0)	    /**< Minimum angle to assign a track to a target */

//============================ FMCW CONFIG =================================//

#define FMCW_MODULATION		        (1U)

#define	FMCW_SUPPORTED						    /**< Comment/Uncomment this macro, to disable/enable FMCW support */

#define BANDWIDTH_MHZ 				(200U)    	/**< Bandwidth (units in MHz), [0 - 200MHz] */

#define CHIRP_TIME_USEC				(300U)    /**< Chirp time (units in us), [50us - 3000us].
                                                 Additional ramp down time [100us], and steady state [100us] time are added to it */

#define MINIMUM_RANGE_CM 			(90U)    	/**< Used in FMCW to exclude targets below this range (units in cm) */

#define MAXIMUM_RANGE_CM 			(1000U)    	/**< Used in FMCW to exclude targets beyond this range (units in cm) */

#define RANGE_DETECTION_THRESHOLD 	(100U)   	/**< FFT spectrum threshold to detect a target in FMCW */

#define FMCW_SAMPLING_FREQ_HZ 	    (100000000/ ((CHIRP_TIME_USEC *100) / SAMPLES_PER_CHIRP)) 	/**< Sampling frequency for FMCW (units in Hz) */

//========================== DOPPLER CONFIG ================================//

//#define DOPPLER_SUPPORTED					    /**< Comment/Uncomment this macro, to disable/enable Doppler support */

#define DOPPLER_SAMPLING_FREQ_HZ 	(20000U) 	/**< Sampling frequency for Doppler (units in Hz) */

#define MINIMUM_SPEED_KMH 		 	(0U)    	/**< Used in Doppler to exclude targets below this speed (units in km/h) */

#define MAXIMUM_SPEED_KMH 		 	(4U)    	/**< Used in Doppler to exclude targets above this speed (units in km/h) */

#define SPEED_DETECTION_THRESHOLD	(50U)  	    /**< FFT spectrum threshold to detect a target in Doppler */


#define DEMO_DESCRIPTION            "Position2Go BGT24MTR12 Demo Radar Board"


#define FW_VERSION_MAJOR  			(1U)			/**< Firmware major version */
#define FW_VERSION_MINOR  			(0U)			/**< Firmware minor version */
#define FW_VERSION_REVISION 		(2U)			/**< Firmware patch version */

#define HW_VERSION_MAJOR  			(1U)		/**< Hardware major version */
#define HW_VERSION_MINOR  			(2U)		/**< Hardware minor version */



typedef enum
{
  BGT_DEVICE               = 0U,   /**< Select BGT for SPI read/write operation */
  LMX_DEVICE               = 1U,   /**< Select LMX for SPI read/write operation */
  PGA_DEVICE               = 2U    /**< Select PGA for SPI read/write operation */
} SPI_Device_Type_t;

typedef enum
{
  PGA_STATUS_SUCCESS  = 0U,        /**< Success */
  PGA_STATUS_FAIL     = 1U,        /**< Fail */
  PGA_STATUS_NA       = 2U         /**< Feature not available */
} Pga_Status_t;

typedef enum DAVE_STATUS
{
  DAVE_STATUS_SUCCESS = 0,
  DAVE_STATUS_FAILURE
} DAVE_STATUS_t;


#endif /* TYPES_H_ */
