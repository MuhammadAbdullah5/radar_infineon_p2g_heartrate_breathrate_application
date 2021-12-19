/**
 *
 *  application.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_APPLICATION_H_
#define FW_INC_DRV_APPLICATION_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
#include "types.h"
#include "FW/inc/radar_control.h"
#include "FW/inc/drv_host_comm.h"
#include "FW/inc/algorithm.h"

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/
/**
 * \brief Defines the main radar application states.
 * @{
 */
typedef enum
{
   APP_IDLE                        = 1U, /* The default application state */
   APP_APPLY_DEVICE_SETTINGS,
   APP_RADAR_DATA_ACQUISITION_COMPLETED, /* Data acquisition completed and ADC raw data is available in the buffer */
   APP_CHECK_FOR_SETTINGS_UPDATE,        /* Check if a new hardware settings are required before to start acquisition for the next frame*/
} Radar_App_State_t;
/** @} */

/******************************************************************************
   3. TYPES
*******************************************************************************/

/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/
/**
 * \brief  This function initializes all firmware and algorithm parameters, starts the
 *         radar data acquisition and initiates the communication with Radar GUI.
 *
 * \param[in]  None
 *
 * \return None
 */
void app_init(void);

/**
 * \brief  This function registers algorithm processing function in application layer
 *
 * \param[in] algo_processor  Processing algorithm need to be assigned to this
 *                            function pointer of signature "void *funcptr(void)".
 *
 * \return None
 */
void app_register_algo_process(algorithm algo_processor);

/**
 * \brief  This function is the main application process, called from the main program
 *         loop and it manages the communication with the Radar GUI.
 *
 * \param[in]  None
 *
 * \return None
 */
void app_process(void);

/**
 * \brief  This function is used to de-initialize and free all used
 *         resources by the application.
 *
 * \param[in]  None
 *
 * \return None
 */
void app_destructor(void);

/**
 * \brief  This function is used for software reset.
 *
 * \param[in]  None
 *
 * \return None
 */
void system_reset(void);

/**
 * \brief  The main radar application process.
 *
 * \param[in]  None
 *
 * \return None
 */
void radar_app_process(void);



/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_APPLICATION_H_ */

/* --- End of File --- */
