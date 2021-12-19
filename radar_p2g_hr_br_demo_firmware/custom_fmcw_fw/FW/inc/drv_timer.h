/**
 *
 *  drv_timer.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_TIMER_H_
#define FW_INC_DRV_TIMER_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/

#include <xmc_ccu4.h>
#include <xmc_ccu8.h>
#include <xmc_scu.h>

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/
#define GLOBAL_CCU4_CCUCON_Msk ((uint32_t)XMC_SCU_CCU_TRIGGER_CCU43)
#define GLOBAL_CCU8_CCUCON_Msk ((uint32_t)XMC_SCU_CCU_TRIGGER_CCU80)

/* Moudule and Kernel Pointers */
#define TIMER_DELAY_KERNEL_PTR (XMC_CCU8_MODULE_t*)(void *)CCU80_BASE
#define TIMER_DELAY_SLICE_PTR  (XMC_CCU8_SLICE_t*)(void *)CCU80_CC80
/* Shadow transfer masks */
#define TIMER_DELAY_SLICE_SH_MSK      XMC_CCU8_SHADOW_TRANSFER_SLICE_0
#define TIMER_DELAY_PRESCALER_SH_MSK  XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_0

/* Moudule and Kernel Pointers */
#define TIMER_FRAME_TRIG_KERNEL_PTR (XMC_CCU4_MODULE_t*)(void *)CCU43_BASE
#define TIMER_FRAME_TRIG_SLICE_PTR  (XMC_CCU4_SLICE_t*)(void *)CCU43_CC42
/* Shadow transfer masks */
#define TIMER_FRAME_TRIG_SLICE_SH_MSK      XMC_CCU4_SHADOW_TRANSFER_SLICE_2
#define TIMER_FRAME_TRIG_PRESCALER_SH_MSK  XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_2

/* Moudule and Kernel Pointers */
#define TIMER_WATCHDOG_KERNEL_PTR (XMC_CCU4_MODULE_t*)(void *)CCU43_BASE
#define TIMER_WATCHDOG_SLICE_PTR  (XMC_CCU4_SLICE_t*)(void *)CCU43_CC41
/* Shadow transfer masks */
#define TIMER_WATCHDOG_SLICE_SH_MSK      XMC_CCU4_SHADOW_TRANSFER_SLICE_1
#define TIMER_WATCHDOG_PRESCALER_SH_MSK  XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1

/** This is used to calculate the time in GetTime API */
#define TIMER_CLK_CONST_SCALED (3067833782U)

/** This enables the functionality for the CCU4 timer */
#define TIMER_CCU4_USED

/** This enables the functionality for the CCU8 timer */
#define TIMER_CCU8_USED

/** This enables the Interrupt functionality */
#define TIMER_INTERRUPT

/* System Core clock frequency in Hz multiplied by 1000000
*  changes done to avoid Float operation
*/
#define SYSTIMER_SYSTICK_CLOCK   (140000000U)

/**< SysTick interval in seconds */
#define SYSTIMER_TICK_PERIOD  (0.001F)

/**< SysTick interval in microseconds */
#define SYSTIMER_TICK_PERIOD_US  (1000U)

/**< Maximum No of timer */
#define SYSTIMER_CFG_MAX_TMR  (1U)

#define SYSTIMER_PRIORITY  (63U)
#define SYSTIMER_SUBPRIORITY  (0U)


/******************************************************************************
   3. TYPES
*******************************************************************************/
/**
 *   @brief The type identifies the APP status.
 */
typedef enum CCU4_SLICE_CONFIG_STATUS
{
  /**
  * STATUS SUCCESS
  */
  CCU4_SLICE_CONFIG_STATUS_SUCCESS = 0,

  /**
  * STATUS FAILURE
  */
  CCU4_SLICE_CONFIG_STATUS_FAILURE,

} CCU4_SLICE_CONFIG_STATUS_t;

typedef enum GLOBAL_CCU4_STATUS
{
  GLOBAL_CCU4_STATUS_SUCCESS = 0U, /**< Status success */
  GLOBAL_CCU4_STATUS_FAILURE /**< Status failure */
} GLOBAL_CCU4_STATUS_t;

typedef enum GLOBAL_CCU8_STATUS
{
  GLOBAL_CCU8_STATUS_SUCCESS = 0U, /**< Status success */
  GLOBAL_CCU8_STATUS_FAILURE /**< Status failure */
} GLOBAL_CCU8_STATUS_t;

/**
 * @brief The type identifies the CCU4 or CCU8 timer selected.
 */
typedef enum TIMER_MODULE
{
  TIMER_MODULE_CCU4 = 0U, /**< CCU4 is selected */
  TIMER_MODULE_CCU8       /**< CCU8 is selected */
} TIMER_MODULE_t;

/**
 * @brief status of the TIMER APP
 */
typedef enum TIMER_STATUS{
  TIMER_STATUS_SUCCESS = 0U, /**< Status success */
  TIMER_STATUS_FAILURE       /**< Status failure */
} TIMER_STATUS_t;

/**
 * @brief This enumeration indicates status of SYSTIMER
 */
typedef enum SYSTIMER_STATUS
{
  SYSTIMER_STATUS_SUCCESS = 0U, /**< Status Success if initialization is successful */
  SYSTIMER_STATUS_FAILURE  /**< Status Failure if initialization is failed */
} SYSTIMER_STATUS_t;

/**
 * @brief This enumeration defines possible timer state
 */
typedef enum SYSTIMER_STATE
{
  SYSTIMER_STATE_NOT_INITIALIZED = 0U, /**< The timer is in uninitialized state */
  SYSTIMER_STATE_RUNNING, /**< The timer is in running state */
  SYSTIMER_STATE_STOPPED /**< The timer is in stop state */
} SYSTIMER_STATE_t;

/**
 * @brief Enumeration values which describes timer types
 */
typedef enum SYSTIMER_MODE
{
  SYSTIMER_MODE_ONE_SHOT = 0U, /**< timer type is one shot */
  SYSTIMER_MODE_PERIODIC /**< timer type is periodic */
} SYSTIMER_MODE_t;

typedef struct GLOBAL_CCU4
{
  const uint32_t module_frequency; /**< fccu frequency */
  const XMC_SCU_CCU_TRIGGER_t syncstart_trigger_msk; /**< Mask to start the timers synchronously */
  XMC_CCU4_MODULE_t* const module_ptr;   /**< reference to module handle */
  XMC_CCU4_SLICE_MCMS_ACTION_t const mcs_action; /**< Shadow transfer of selected values in multi-channel mode */
  bool  is_initialized; /**< Indicates initialized state of particular instance of the APP */
} GLOBAL_CCU4_t;

typedef struct CCU4_SLICE_CONFIG
{
  CCU4_SLICE_CONFIG_STATUS_t (*init)(void); /**< Defines the local initialization function for the instance */
  GLOBAL_CCU4_t *global; /**< Pointer to required GLOBAL_CCU4 APP */
  XMC_CCU4_SLICE_t *slice_ptr; /**< CCU4x_CC4y slice pointer */
  uint8_t slice_num; /**< slice number */
} CCU4_SLICE_CONFIG_t;

/**
 * This saves the context of the GLOBAL_CCU8 APP.
 */
typedef struct GLOBAL_CCU8
{
  const uint32_t module_frequency; /**< fccu frequency */
  const XMC_SCU_CCU_TRIGGER_t syncstart_trigger_msk; /**< Mask to start the timers synchronously */
  XMC_CCU8_MODULE_t* const module_ptr;   /**< reference to module handle */
  XMC_CCU8_SLICE_MCMS_ACTION_t const mcs_action; /**< Shadow transfer of selected values in multi-channel mode */
  bool  is_initialized; /**< Indicates initialized state of particular instance of the APP */
} GLOBAL_CCU8_t;

/**
 * @brief Initialization parameters of the TIMER APP
 */
typedef struct TIMER
{
  uint32_t time_interval_value_us; /**< Timer interval value for which event is being generated */
  const uint32_t timer_max_value_us;	/**< Maximum timer value in micro seconds for the available clock */
  const uint32_t timer_min_value_us;  /**< Minimum timer value in micro seconds for the available clock */
  const uint32_t shadow_mask;  /**< shadow transfer mask for the selected timer */
#ifdef  TIMER_CCU4_USED
  GLOBAL_CCU4_t* const global_ccu4_handler; /**< Reference to CCU4GLOBAL APP handler */
  XMC_CCU4_SLICE_t* const ccu4_slice_ptr;  /**< Reference to CCU4-CC4 slice identifier data handler */
  const uint8_t ccu4_slice_number;  /* Timer being used */
  XMC_CCU4_SLICE_COMPARE_CONFIG_t* const ccu4_slice_config_ptr; /**< Reference to initialization data structure of
                                                                           the core timer functionality */
  XMC_CCU4_SLICE_SR_ID_t  const ccu4_period_match_node; /**< Service Request Id for period match event */
#endif
#ifdef  TIMER_CCU8_USED
  GLOBAL_CCU8_t* const global_ccu8_handler; /**< Reference to CCU8GLOBAL APP handler */
  XMC_CCU8_SLICE_t* const ccu8_slice_ptr; /**< Reference to CCU8-CC8 slice identifier data handler */
  const uint8_t ccu8_slice_number;  /* Timer being used */
  XMC_CCU8_SLICE_COMPARE_CONFIG_t* const ccu8_slice_config_ptr; /**< Reference to initialization data structure of
                                                                           the core timer functionality */
  XMC_CCU8_SLICE_SR_ID_t const ccu8_period_match_node; /**< Service Request Id for period match event */
#endif
  TIMER_MODULE_t const timer_module; /**< Indicate which timer module is being used from CCU4 and CCU8 */
  uint16_t period_value; /**< Period value to be loaded into timer for the corresponding time tick */
  bool const start_control; /**< Indicate whether to start the APP during initialization itself */
  bool const period_match_enable; /**< Indicate the generation of period match event */
  bool initialized;  /* flag to indicate the initialization state of the APP instance */
} TIMER_t;


/**
 * @brief timer callback function pointer
 */
typedef void (*SYSTIMER_CALLBACK_t)(void *args);

/**
 * @brief This structure contains pointer which is used to hold CPU instance handle and
 * variables for priority group
 */
typedef struct SYSTIMER
{
  bool init_status; /**< APP initialization status to ensure whether SYSTIMER_Init called or not */
} SYSTIMER_t;


extern const CCU4_SLICE_CONFIG_t CCU4_SLICE_CONFIG_ADC_TRIG;

extern GLOBAL_CCU4_t GLOBAL_CCU4_0; /**< APP handle for handle GLOBAL_CCU4_0*/
extern GLOBAL_CCU8_t GLOBAL_CCU8_0; /**< APP handle for handle GLOBAL_CCU8_0*/
extern SYSTIMER_t SYSTIMER_0;


/* Extern declaration of EVENT_DET APP handler */
extern TIMER_t TIMER_DELAY;

/* Extern declaration of EVENT_DET APP handler */
extern TIMER_t TIMER_FRAME_TRIG;

/* Extern declaration of EVENT_DET APP handler */
extern TIMER_t TIMER_WATCHDOG;


/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/

/**
 * @brief Initializes a GLOBAL_CCU4 with generated configuration.
 *
 * @param handle pointer to the GLOBAL_CCU4 APP handle structure.
 * @return GLOBAL_CCU4_STATUS_t\n  GLOBAL_CCU4_STATUS_SUCCESS : if initialization is successful\n
 *                                 GLOBAL_CCU4_STATUS_FAILURE : if initialization is failed\n
 * <BR>
 * \par<b>Description:</b><br>
 * <ul>
 * <li>Enable the module.</li>
 * <li>Start the prescaler.</li>
 * </ul>
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  DAVE_STATUS_t init_status;
 *  init_status = DAVE_Init();	// GLOBAL_CCU4_Init(&GLOBAL_CCU4_0) will be called from DAVE_Init()
 *
 *  while(1)
 *  {
 *  }
 *  return 1;
 * }
 * @endcode<BR>
 */
GLOBAL_CCU4_STATUS_t GLOBAL_CCU4_Init(GLOBAL_CCU4_t* handle);

/**
 * @brief Start all the timers which are configured to start externally on positive edge.<br>
 * @param ccucon_msk mask for which kernels sync start has to be applied.
 * \par<b>Note:</b><br>
 * This mask has been generated in the APP handle and as a macro in global_ccu4_conf.h file.
 * 1. The variable from the APP handle is useful while starting the specific kernel/s
 * 2. GLOBAL_CCU4_CCUCON_Msk Macro from global_ccu4_conf.h file can be used to start all the selected kernels at a time.
 * @retval none
 *
 * \par<b>Description:</b><br>
 * The top level APPs have to be enabled, to start the timer externally with positive trigger edge.
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *   DAVE_STATUS_t status;
 *
 *   status = DAVE_Init();	// GLOBAL_CCU4_Init() is called from DAVE_Init()
 *
 *  // Below can be used to start the specific kernels, by generating two instance of APP
 *  // GLOBAL_CCU4_SyncStartTriggerHigh((uint32_t)(GLOBAL_CCU4_0.syncstart_trigger_msk | GLOBAL_CCU4_1.syncstart_trigger_msk));
 *  // Below can be used to start all the kernels simultaneously
 *   GLOBAL_CCU4_SyncStartTriggerHigh(GLOBAL_CCU4_CCUCON_Msk);
 *
 *   while(1)
 *   {
 *   }
 *
 *   return 1;
 * }
 * @endcode <BR> </p>
 */
__STATIC_INLINE void GLOBAL_CCU4_SyncStartTriggerHigh(uint32_t ccucon_msk)
{
  XMC_SCU_SetCcuTriggerHigh(ccucon_msk);
}

/**
 * @brief Start all the timers which are configured to start externally on negative edge.<br>
 * @param ccucon_msk mask for which kernels sync start has to be applied.
 * \par<b>Note:</b><br>
 * This mask has been generated in the APP handle and a macro in global_ccu4_conf.h file.
 * 1. The variable from the APP handle is useful while starting the specific kernel/s
 * 2. GLOBAL_CCU4_CCUCON_Msk Macro from global_ccu4_conf.h file can be used to start all the selected kernels at a time.
 * @retval none
 *
 * \par<b>Description:</b><br>
 * The top level APPs have to be enabled, to start the timer externally with negative trigger edge.
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *   DAVE_STATUS_t status;
 *
 *   status = DAVE_Init();	// GLOBAL_CCU4_Init() is called from DAVE_Init()
 *
 *  // Below can be used to start the specific kernels, by generating two instance of APP
 *  // GLOBAL_CCU4_SyncStartTriggerLow((uint32_t)(GLOBAL_CCU4_0.syncstart_trigger_msk | GLOBAL_CCU4_1.syncstart_trigger_msk));
 *  // Below can be used to start all the kernels simultaneously
 *   GLOBAL_CCU4_SyncStartTriggerLow(GLOBAL_CCU4_CCUCON_Msk);
 *
 *   while(1)
 *   {
 *   }
 *
 *   return 1;
 * }
 * @endcode <BR> </p>
 */
__STATIC_INLINE void GLOBAL_CCU4_SyncStartTriggerLow(uint32_t ccucon_msk)
{
  XMC_SCU_SetCcuTriggerLow(ccucon_msk);
}

/**
 * @brief Initializes a GLOBAL_CCU8 with generated configuration.
 *
 * @param handle pointer to the GLOBAL_CCU8 APP handle structure
 * @return GLOBAL_CCU8_STATUS_t\n  GLOBAL_CCU8_STATUS_SUCCESS : if initialization is successful\n
 *                                 GLOBAL_CCU8_STATUS_FAILURE : if initialization is failed\n
 * <BR>
 * \par<b>Description:</b><br>
 * <ul>
 * <li>Enable the module.</li>
 * <li>Start the prescaler.</li>
 * </ul>
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  DAVE_STATUS_t init_status;
 *  init_status = DAVE_Init();	// GLOBAL_CCU8_Init(&GLOBAL_CCU8_0) will be called from DAVE_Init()
 *
 *  while(1)
 *  {
 *  }
 *  return 1;
 * }
 * @endcode<BR>
 */
GLOBAL_CCU8_STATUS_t GLOBAL_CCU8_Init(GLOBAL_CCU8_t* handle);

/**
 * @brief Start all the timers which are configured to start externally on positive edge.<br>
 * @param ccucon_msk mask for which kernels sync start has to be applied.
 * \par<b>Note:</b><br>
 * This mask has been generated in the APP handle and as a macro in global_ccu8_conf.h file.
 * 1. The variable from the APP handle is useful while starting the specific kernel/s
 * 2. GLOBAL_CCU8_CCUCON_Msk Macro from global_ccu8_conf.h file can be used to start all the selected kernels at a time.
 * @retval none
 *
 * \par<b>Description:</b><br>
 *  The top level APPs have to be enabled, to start the timer externally with positive trigger edge.
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *   DAVE_STATUS_t status;
 *
 *   status = DAVE_Init();	// GLOBAL_CCU8_Init() is called from DAVE_Init()
 *
 *  // Below can be used to start the specific kernels, by generating two instance of APP
 *  // GLOBAL_CCU8_SyncStartTriggerHigh((uint32_t)(GLOBAL_CCU8_0.syncstart_trigger_msk | GLOBAL_CCU8_1.syncstart_trigger_msk));
 *  // Below can be used to start all the kernels simultaneously
 *   GLOBAL_CCU8_SyncStartTriggerHigh(GLOBAL_CCU8_CCUCON_Msk);
 *
 *   while(1)
 *   {
 *   }
 *
 *   return 1;
 * }
 * @endcode <BR> </p>
 */
__STATIC_INLINE void GLOBAL_CCU8_SyncStartTriggerHigh(uint32_t ccucon_msk)
{
  XMC_SCU_SetCcuTriggerHigh(ccucon_msk);
}

/**
 * @brief Start all the timers which are configured to start externally on negative edge.<br>
 * @param ccucon_msk mask for which kernels sync start has to be applied.
 * \par<b>Note:</b><br>
 * This mask has been generated in the APP handle and a macro in global_ccu8_conf.h file.
 * 1. The variable from the APP handle is useful while starting the specific kernel/s
 * 2. GLOBAL_CCU8_CCUCON_Msk Macro from global_ccu8_conf.h file can be used to start all the selected kernels at a time.
 * @retval none
 *
 * \par<b>Description:</b><br>
 * The top level APPs have to be enabled, to start the timer externally with negative trigger edge.
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *   DAVE_STATUS_t status;
 *
 *   status = DAVE_Init();	// GLOBAL_CCU8_Init() is called from DAVE_Init()
 *
 *  // Below can be used to start the specific kernels, by generating two instance of APP
 *  // GLOBAL_CCU8_SyncStartTriggerLow((uint32_t)(GLOBAL_CCU8_0.syncstart_trigger_msk | GLOBAL_CCU8_1.syncstart_trigger_msk));
 *  // Below can be used to start all the kernels simultaneously
 *   GLOBAL_CCU8_SyncStartTriggerLow(GLOBAL_CCU8_CCUCON_Msk);
 *
 *   while(1)
 *   {
 *   }
 *
 *   return 1;
 * }
 * @endcode <BR> </p>
 */
__STATIC_INLINE void GLOBAL_CCU8_SyncStartTriggerLow(uint32_t ccucon_msk)
{
  XMC_SCU_SetCcuTriggerLow(ccucon_msk);
}


/**
 * @brief Initializes a TIMER with generated configuration.
 *
 * @param handle_ptr pointer to the TIMER APP configuration.
 * @return TIMER_STATUS_t\n  TIMER_STATUS_SUCCESS : if initialization is successful\n
 *                           TIMER_STATUS_FAILURE : if initialization is failed\n
 * <BR>
 * \par<b>Description:</b><br>
 * <ul>
 * <li>Enable the clock for the slice and invoke the LLD API with generated configuration handle.</li>
 * <li>Load the Period, Compare and Prescaler shadow registers with the generated values and enable the shadow transfer
 * request. This loads the values into the actual registers and start the TIMER based on the configuration.</li>
 * <li>If "Start after initialization" is not enabled, TIMER_Start() can be invoked to start the timer.</li>
 * </ul>
 * <BR>
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  DAVE_STATUS_t init_status;
 *  init_status = DAVE_Init();	// TIMER_Init(&TIMER_0) will be called from DAVE_Init()
 *
 *  while(1)
 *  {
 *  }
 *  return 1;
 * }
 * @endcode<BR>
 */
TIMER_STATUS_t TIMER_Init(TIMER_t  *const handle_ptr);

/**
 * @brief       Starts the timer if the initialization of the APP is successful.
 *
 * @param handle_ptr pointer to the TIMER APP configuration.
 * @return TIMER_STATUS_t\n TIMER_STATUS_SUCCESS : if timer start is successful\n
 *                          TIMER_STATUS_FAILURE : if timer start is failed\n
 * <BR>
 *
 * \par<b>Description:</b><br>
 * If "Start after initialization" is not enabled, TIMER_Start() can be invoked to start the timer. TIMER_Stop() can be
 * used to stop the Timer. No need to reconfigure the timer to start again.
 *
 * <BR>
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  DAVE_STATUS_t init_status;
 *  TIMER_STATUS_t timer_status;
 *  init_status = DAVE_Init();	// TIMER_Init(&TIMER_0) will be called from DAVE_Init()
 *
 *  if(init_status == DAVE_STATUS_SUCCESS)
 *  {
 *    timer_status = TIMER_Start(&TIMER_0);
 *  }
 *  while(1)
 *  {
 *  }
 *  return 1;
 * }
 * @endcode<BR>
 */
TIMER_STATUS_t TIMER_Start(TIMER_t  *const handle_ptr);

/**
 * @brief Stops the TIMER, if it is running.
 *
 * @param handle_ptr pointer to the TIMER APP configuration.
 * @return TIMER_STATUS_t\n TIMER_STATUS_SUCCESS : if timer is running and stop is successful\n
 *                          TIMER_STATUS_FAILURE : if timer is in idle state, and stop is called\n
 *<BR>
 *
 * \par<b>Description:</b><br>
 * Clears the Timer run bit to stop. No further event is generated.
 *
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  DAVE_STATUS_t init_status;
 *  TIMER_STATUS_t timer_status;
 *  init_status = DAVE_Init();	// TIMER_Init(&TIMER_0) will be called from DAVE_Init()
 *
 *  if(init_status == DAVE_STATUS_SUCCESS)
 *  {
 *    timer_status = TIMER_Start(&TIMER_0);
 *  }
 *
 *  if (timer_status == TIMER_STATUS_SUCCESS)
 *  {
 *    while(TIMER_GetInterruptStatus(&TIMER_0));
 *
 *    timer_status = TIMER_Stop(&TIMER_0);
 *  }
 *  while(1)
 *  {
 *  }
 *  return 1;
 * }
 * @endcode<BR>
 */

TIMER_STATUS_t TIMER_Stop(TIMER_t  *const handle_ptr);

/**
 * @brief Returns the current time in micro seconds by scaling with 100.
 *
 * @param handle_ptr pointer to the TIMER APP configuration.
 * @return uint32_t\n time in microseconds
 *<BR>
 *
 * \par<b>Description:</b><br>
 * By using prescaler and frequency and timer register value, this API calculates the current time in micro seconds.
 * Then the value is scaled with 100, before returning.
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  DAVE_STATUS_t init_status;
 *  TIMER_STATUS_t timer_status;
 *  uint32_t elapsed_time;
 *  init_status = DAVE_Init();	// TIMER_Init(&TIMER_0) will be called from DAVE_Init()
 *
 *  if(init_status == DAVE_STATUS_SUCCESS)
 *  {
 *    timer_status = TIMER_Start(&TIMER_0);
 *  }
 *
 *   timer_status = TIMER_Stop(&TIMER_0);
 *
 *   elapsed_time = TIMER_GetTime(&TIMER_0);
 *
 *  while(1)
 *  {
 *  }
 *  return 1;
 * }
 * @endcode<BR>
 */
uint32_t TIMER_GetTime(TIMER_t *const handle_ptr);

/**
 * @brief Clears the timer register.
 *
 * @param handle_ptr pointer to the TIMER APP configuration.
 * @return TIMER_STATUS_t\n TIMER_STATUS_SUCCESS : if clear is successful\n
 *                          TIMER_STATUS_FAILURE : if timer is not initialized and clear is requested\n
 *<BR>
 *
 * \par<b>Description:</b><br>
 * TIMER_Clear() clears the timer register so that next cycle starts from reset value.
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  DAVE_STATUS_t init_status;
 *  TIMER_STATUS_t timer_status;
 *  init_status = DAVE_Init();	// TIMER_Init(&TIMER_0) will be called from DAVE_Init()
 *
 *  if(init_status == DAVE_STATUS_SUCCESS)
 *  {
 *    timer_status = TIMER_Start(&TIMER_0);
 *  }
 *
 *  if (TIMER_GetTimerStatus(&TIMER_0))
 *  {
 *    timer_status = TIMER_Stop(&TIMER_0);
 *  }
 *
 *  timer_status = TIMER_Clear(&TIMER_0);
 *
 *  while(1)
 *  {
 *  }
 *  return 1;
 * }
 * @endcode<BR>
 */
TIMER_STATUS_t TIMER_Clear(TIMER_t *const handle_ptr);

/**
 * @brief Returns the running state of the timer.
 *
 * @param handle_ptr pointer to the TIMER APP configuration.
 * @return bool\n true  : if the timer is running\n
 *                false : if the timer is not running\n
 *<BR>
 *
 * \par<b>Description:</b><br>
 * TIMER_GetTimerStatus() reads the run bit of the timer to indicate the actual state of the TIMER.
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  DAVE_STATUS_t init_status;
 *  TIMER_STATUS_t timer_status;
 *  init_status = DAVE_Init();	// TIMER_Init(&TIMER_0) will be called from DAVE_Init()
 *
 *  if(init_status == DAVE_STATUS_SUCCESS)
 *  {
 *    timer_status = TIMER_Start(&TIMER_0);
 *  }
 *
 *  if (TIMER_GetTimerStatus(&TIMER_0))
 *  {
 *    while(TIMER_GetTimerStatus(&TIMER_0));
 *
 *    timer_status = TIMER_Stop(&TIMER_0);
 *  }
 *  while(1)
 *  {
 *  }
 *  return 1;
 * }
 * @endcode<BR>
 */

bool TIMER_GetTimerStatus(TIMER_t  *const handle_ptr);

/**
 * @brief Set the new time interval for the event generation, by checking with the supported range.
 * @param handle_ptr pointer to the TIMER APP configuration.
 * @param time_interval new time interval value in micro seconds.
 *
 * @return TIMER_STATUS_t\n TIMER_STATUS_SUCCESS : Setting new time interval value is successful\n
 *                          TIMER_STATUS_FAILURE : New time value is not in range of supported time value\n
 *                                                 Timer is in running condition
 *              <BR>
 *
 * \par<b>Description:</b><br>
 * Based on the timer interval, prescaler value is calculated for the CCU timer. By using this prescaler and
 * time interval values  Period value is calculated. The period value is updated into the shadow register and shadow
 * transfer request is enabled. Timer has to be stopped before updating the time interval.<br>\n
 *
 * \par<b>Note:</b><br>
 * Input time interval value has to be scaled by 100 to the actual required value.\n
 * e.g. : required timer interval value = 30.45 micro seconds\n
 *        Input value to the API = 30.45 * 100 = 3045
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * #include <xmc_gpio.h>                   // GPIO LLD header, this contains the interface for Port functionality
 * #define TIMER_GPIO_PORT XMC_GPIO_PORT0  // PORT0 Address
 * #define TIMER_GPIO_PIN  0U              // Pin number
 * #define TIMER_500MS 500000*100U
 *
 * volatile uint32_t count = 0U;          // count variable to change the time tick interval
 * uint32_t shadow_transfer_msk;          // This is to generate the slice specific shadow transfer mask
 *
 * const XMC_GPIO_CONFIG_t GPIO_0_config  =
 * {
 *   .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
 *   .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
 * };
 *
 * int main(void)
 * {
 *   DAVE_STATUS_t status;
 *
 *   XMC_GPIO_Init(TIMER_GPIO_PORT, TIMER_GPIO_PIN, &GPIO_0_config);
 *
 *   status = DAVE_Init();		// Initialization of DAVE APPs
 *
 *   while(1U)
 *   {
 *   }
 *   return 1;
 * }
 *
 * void Timetick_Handler(void)
 * {
 *  count++;
 *
 *  TIMER_ClearEvent(&TIMER_0);
 *
 *  XMC_GPIO_ToggleOutput(TIMER_GPIO_PORT, TIMER_GPIO_PIN);
 *
 *  if(count > 10)
 *  {
 *    count = 0U;
 *    TIMER_Stop(&TIMER_0);
 *    status = TIMER_SetTimeInterval(&TIMER_0, TIMER_500MS);
 *    if (status == TIMER_STATUS_SUCCESS)
 *    {
 *      TIMER_Start(&TIMER_0);
 *    }
 *  }
 * }
 * @endcode<BR>
 */
TIMER_STATUS_t TIMER_SetTimeInterval(TIMER_t  *const handle_ptr, uint32_t time_interval);

/**
 * @brief Indicates the occurrence of time interval event.
 * @param handle_ptr pointer to the TIMER APP configuration.
 * @return bool\n true  : if event set\n
 *                false : if event is not set
 * <BR>
 *
 * \par<b>Description:</b><br>
 * The status returned, can be utilized to generate the delay function.
 *
 * Example Usage:
 * @code
 * #include <DAVE.h>
 * #define TIMER_DELAY_MUL_FACTOR 100000U // Converts micro seconds to milli seconds with multiplication factor for
 *                                        // TIMER_GetInterruptStatus().
 * void TIMER_Delay(uint32_t);
 * int main(void)
 * {
 *  DAVE_STATUS_t init_status;
 *  TIMER_STATUS_t status;
 *  uint32_t delay_val; // delay value in terms milli seconds
 *
 *  init_status = DAVE_Init();	// TIMER_Init(&TIMER_0) will be called from DAVE_Init()
 *
 *  TIMER_ClearEvent(&TIMER_0);
 *
 *  if(init_status == DAVE_STATUS_SUCCESS)
 *  {
 *    delay_val = 1000; // 1000 milli seconds
 *
 *    TIMER_Delay(delay_val);
 *  }
 *
 *  while(1)
 *  {
 *
 *  }
 *  return 1;
 * }
 *
 * void TIMER_Delay(uint32_t delay_val)
 * {
 *   uint32_t delay_cnt;
 *
 *   delay_cnt = delay_val * TIMER_DELAY_MUL_FACTOR;
 *
 *   TIMER_SetTimeInterval(&TIMER_0,delay_cnt);
 *
 *   TIMER_Start(&TIMER_0);
 *
 *   while(!TIMER_GetInterruptStatus(&TIMER_0));
 *
 *   TIMER_Stop(&TIMER_0);
 * }
 * @endcode<BR>
 */

bool TIMER_GetInterruptStatus(TIMER_t * const handle_ptr);

/**
 * @brief Clears the period match interrupt status of the given timer.
 * @param handle_ptr pointer to the TIMER APP configuration.
 * @return None
 * <BR>
 * \par<b>Description:</b><br>
 * For each occurrence of the time interval event, it has to be cleared through software only. So next event is
 * considered as new.
 *
 * <BR><P ALIGN="LEFT"><B>Example Usage:</B>
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *   DAVE_STATUS_t status;
 *
 *   status = DAVE_Init();		// Initialization of DAVE APPs
 *
 *   while(1U)
 *   {
 *   }
 *   return 1;
 * }
 *
 * void Timetick_Handler(void)
 * {
 *  TIMER_ClearEvent(&TIMER_0);
 * }
 * @endcode<BR>
 */
void TIMER_ClearEvent(TIMER_t *const handle_ptr);

/**
 * @brief Initializes SYSTIMER APP
 * @param handle Pointer pointing to SYSTIMER APP data structure. Refer @ref SYSTIMER_t for details.
 * @return SYSTIMER_STATUS_t APP status. Refer @ref SYSTIMER_STATUS_t for details.
 *
 * \par<b>Description: </b><br>
 * Initializes the SysTick counter as per the SysTick interval specified by the
 * user and start the SysTick counter. It also initializes global variables.
 *
 * \par<b>Example Usage:</b><br>
 * @code
 *   #include <DAVE.h>         //Declarations from DAVE Code Generation (includes SFR declaration)
 *
 *   int main(void)
 *   {
 *     SYSTIMER_STATUS_t init_status;
 *
 *     init_status = (SYSTIMER_STATUS_t)SYSTIMER_Init(&SYSTIMER_0); // Initialization of SYSTIMER APP
 *     if (init_status == SYSTIMER_STATUS_SUCCESS)
 *     {
 *       // Add application code here
 *       while(1)
 *       {
 *       }
 *     }
 *     else
 *     {
 *      XMC_DEBUG("main: Application initialization failed");
 *      while(1)
 *      {
 *      }
 *     }
 *      return (1);
 *  }
 *  @endcode
 */
 SYSTIMER_STATUS_t SYSTIMER_Init(SYSTIMER_t *handle);

/**
 * @brief Starts the SysTick timer.
 * @param None.
 *
 * \par<b>Description: </b><br>
 * Starts the SysTick timer.
 */
void SYSTIMER_Start(void);

/**
 * @brief Stops the SysTick timer.
 * @param None.
 *
 * \par<b>Description: </b><br>
 * Stops the SysTick timer therefore no software timer will time out.
 */
void SYSTIMER_Stop(void);

/**
 * @brief Creates a new software timer. This function cannot be called from an ISR. Use SYSTIMER_CreateTimerFromISR() instead.
 * @param period  timer period value in microseconds. Range: (SYSTIMER_TICK_PERIOD_US) to pow(2,32).
 * @param mode  Mode of timer(ONE_SHOT/PERIODIC). Refer @ref SYSTIMER_MODE_t for details.
 * @param callback  Call back function of the timer(No Macros are allowed).
 * @param args  Call back function parameter.
 *
 * @return uint32_t returns timer ID if timer created successfully otherwise returns 0 if timer creation failed.
 *                  Range: 0 to 16, 0: Invalid timer ID, 1-16: Valid timer ID.
 *
 * \par<b>Description: </b><br>
 * API for creating a new software timer instance. This also add created software timer to timer list.<br>
 * <b>Note :</b><br> 1. This APP uses SysTick exception for controlling the timer list.
 *  Call back function registered through this function will be called in
 *  SysTick exception when the software timer is expired i.e the software timers callback is executed in
 *  the interrupt context.<br>
 *  2. Due to time at which software timer creation asked by user will not be in synchronize with Hardware SysTick
 *  timer, the count value used during creation of software timer will not create starting/initial period same as
 *  expected value. It is decided to add one extra count(HW_TIMER_ADDITIONAL_CNT) with Software timer. Impact of this
 *  additional count(HW_TIMER_ADDITIONAL_CNT) is, first SW timer period(Initial one) is always equal to or more than
 *  expected/configured.<br>
 *  3. Callbacks are executed in round robin manner if more than one software timers are created with same period
 *  value. Last created software is having higher priority and its associated callback is executed first.<br>
 *  4. Avoid any call to wait, infinitive while loop, blocking calls or creating software timer in ISR because their
 *  behavior could be corrupted when called from an ISR.<br>
 *  5. Software timers are based on 24-bit Hardware SysTick counters, so maximum counts can achieve is
 *  pow(2,24) *(1/fCPU) * 1E6,  where fCPU is in hertz. Software timers are designed for times between microseconds and
 *  seconds. For longer times, application code need to ensure to take necessary action.<br>
 *  6. Software timer period value must be equal to SysTick Interval or integer multiple of a number with SysTick
 *  interval (i.e. SysTick Interval  * n, where n is integer number, n can be 1,2,3,4... but n should not be fractional
 *  or float number). And also software timer period value should not be 0 or less than Hardware SysTick Interval.
 *
 *
 * \par<b>Example Usage:</b><br>
 *
 * @code
 *  #include <DAVE.h>
 *  #define ONESEC 1000000U
 *  void LED_Toggle_EverySec(void)
 *  {
 *    // Add user code here
 *  }
 *
 *  int main(void)
 *  {
 *    uint32_t TimerId;
 *    // ... Initializes APPs configuration ...
 *    DAVE_Init(); // SYSTIMER APP Initialized during DAVE Initialization
 *    // Create Software timer
 *    TimerId = (uint32_t)SYSTIMER_CreateTimer(ONESEC,SYSTIMER_MODE_PERIODIC,(void*)LED_Toggle_EverySec,NULL);
 *    if (TimerId != 0U)
 *    {
 *      //software timer is created successfully
 *      //Add user code here
 *    }
 *    else
 *    {
 *      // //software timer creation is failed
 *    }
 *    while (1)
 *    {
 *
 *    }
 *    return (1);
 *  }
 * @endcode<BR> </p>
 */
uint32_t SYSTIMER_CreateTimer
(
  uint32_t period,
  SYSTIMER_MODE_t mode,
  SYSTIMER_CALLBACK_t callback,
  void  *args
);

/**
 * @brief A version of SYSTIMER_CreateTimer() that can be called from an ISR.
 * @param period  timer period value in microseconds. Range: (SYSTIMER_TICK_PERIOD_US) to pow(2,32).
 * @param mode  Mode of timer(ONE_SHOT/PERIODIC). Refer @ref SYSTIMER_MODE_t for details.
 * @param callback  Call back function of the timer(No Macros are allowed).
 * @param args  Call back function parameter.
 *
 * @return uint32_t returns timer ID if timer created successfully otherwise returns 0 if timer creation failed.
 *                  Range: 0 to 16, 0: Invalid timer ID, 1-16: Valid timer ID.
 *
 */
uint32_t SYSTIMER_CreateTimerFromISR
(
  uint32_t period,
  SYSTIMER_MODE_t mode,
  SYSTIMER_CALLBACK_t callback,
  void  *args
);

/**
 * @brief Starts the software timer. This function cannot be called from an ISR. Use SYSTIMER_StartTimerFromISR() instead.
 * @param id  timer ID obtained from SYSTIMER_CreateTimer. Range : 1 to 16
 * @return SYSTIMER_STATUS_t APP status. Refer @ref SYSTIMER_STATUS_t for details.
 *
 * \par<b>Description: </b><br>
 * API for starting a software timer instance.<br>
 *<b>Note :</b> This API must be called after software timer is created using SYSTIMER_CreateTimer API with generated
 * ID and enable XMC_ASSERT for better understanding of API behavioral in run time.<br>
 * \par<b>Example Usage:</b><br>
 *
 * @code
 *  #include <DAVE.h>
 *  #define ONESEC 1000000U
 *  void LED_Toggle_EverySec(void)
 *  {
 *    // Add user code here
 *  }
 *
 *  int main(void)
 *  {
 *    uint32_t TimerId;
 *    SYSTIMER_STATUS_t status;
 *    // ... Initializes APPs configuration ...
 *    DAVE_Init(); // SYSTIMER APP Initialized during DAVE Initialization
 *    // Create Software timer
 *    TimerId = (uint32_t)SYSTIMER_CreateTimer(ONESEC,SYSTIMER_MODE_PERIODIC,(void*)LED_Toggle_EverySec,NULL);
 *    if (TimerId != 0U)
 *    {
 *      //timer is created successfully, now start/run software timer
 *      status = SYSTIMER_StartTimer(TimerId);
 *      if (status == SYSTIMER_STATUS_SUCCESS)
 *      {
 *        // Software timer is running
 *        // Add user code here
 *      }
 *      else
 *      {
 *        // Error during software timer start operation
 *      }
 *    }
 *    else
 *    {
 *      // timer ID Can not be zero
 *    }
 *    // ... infinite loop ...
 *    while (1)
 *    {
 *
 *    }
 *    return (1);
 *  }
 * @endcode<BR> </p>
 */
SYSTIMER_STATUS_t SYSTIMER_StartTimer(uint32_t id);

/**
 * @brief A version of SYSTIMER_StartTimer() that can be called from an ISR.
 * @param id  timer ID obtained from SYSTIMER_CreateTimer. Range : 1 to 16
 * @return SYSTIMER_STATUS_t APP status. Refer @ref SYSTIMER_STATUS_t for details.
 *
 */
SYSTIMER_STATUS_t SYSTIMER_StartTimerFromISR(uint32_t id);

/**
 * @brief Stops the software timer. This function cannot be called from an ISR. Use SYSTIMER_StopTimerFromISR() instead.
 * @param id  timer ID obtained from SYSTIMER_CreateTimer. Range : 1 to 16
 * @return SYSTIMER_STATUS_t APP status. Refer @ref SYSTIMER_STATUS_t for details.
 *
 * \par<b>Description: </b><br>
 * API to stop created software timer instance.<br>
 *<b>Note :</b> This API must be called after software timer is created using SYSTIMER_CreateTimer API with generated
 * ID and enable XMC_ASSERT for better understanding of API behavioral in run time.<br>
 *
 * \par<b>Example Usage:</b><br>
 *
 * @code
 *  #include <DAVE.h>
 *  #define ONESEC 1000000U
 *  void LED_Toggle_EverySec(void)
 *  {
 *    // Add user code here
 *  }
 *
 *  int main(void)
 *  {
 *    uint32_t TimerId;
 *    SYSTIMER_STATUS_t status;
 *    // ... Initializes APPs configuration ...
 *    DAVE_Init(); // SYSTIMER APP Initialized during DAVE Initialization
 *    // Create Software timer
 *    TimerId = (uint32_t)SYSTIMER_CreateTimer(ONESEC,SYSTIMER_MODE_PERIODIC,(void*)LED_Toggle_EverySec,NULL);
 *    if (TimerId != 0U)
 *    {
 *      //timer is created successfully, now start/run software timer
 *      status = SYSTIMER_StartTimer(TimerId);
 *      if (status == SYSTIMER_STATUS_SUCCESS)
 *      {
 *        // Software timer is running
 *        // Add user code here
 *
 *
 *
 *        //stop the timer
 *        status = SYSTIMER_StopTimer(TimerId);
 *        if (status == SYSTIMER_STATUS_SUCCESS)
 *        {
 *          //Software timer has stopped
 *        }
 *        else
 *        {
 *           // Error during software timer stop operation
 *        }
 *      }
 *      else
 *      {
 *        // Error during software timer start operation
 *      }
 *    }
 *    else
 *    {
 *      // timer ID Can not be zero
 *    }
 *    // ... infinite loop ...
 *    while (1)
 *    {
 *
 *    }
 *    return (1);
 *  }
 * @endcode<BR> </p>
 */
SYSTIMER_STATUS_t SYSTIMER_StopTimer(uint32_t id);

/**
 * @brief A version of SYSTIMER_StopTimer() that can be called from an ISR.
 * @param id  timer ID obtained from SYSTIMER_CreateTimer. Range : 1 to 16
 * @return SYSTIMER_STATUS_t APP status. Refer @ref SYSTIMER_STATUS_t for details.
 *
 */
SYSTIMER_STATUS_t SYSTIMER_StopTimerFromISR(uint32_t id);

/**
 * @brief Function to modify the time interval and restart the timer for the new time interval. This function cannot be called from an ISR. Use SYSTIMER_RestartTimerFromISR() instead. <br>
 * @param id ID of already created system timer. Range : 1 to 16
 * @param microsec new time interval. Range: (SYSTIMER_TICK_PERIOD_US) to pow(2,32).
 * @return SYSTIMER_STATUS_t APP status. Refer @ref SYSTIMER_STATUS_t for details.
 *
 * \par<b>Description: </b><br>
 * API for restarting the created software timer instance with new time interval.<br>
 *<b>Note :</b> This API must be called after software timer is created using SYSTIMER_CreateTimer API with generated
 * ID and enable XMC_ASSERT for better understanding of API behavioral in run time.<br>
 *
 * \par<b>Example Usage:</b><br>
 * Demonstrate SYSTIMER_RestartTimer API
 *
 * @code
 *  #include <DAVE.h>
 *  #define ONESEC 1000000U
 *  #define NEW_INTERVAL (ONESEC * 10U)
 *  void LED_Toggle_EverySec(void)
 *  {
 *    // Add user code here
 *  }
 *
 *  int main(void)
 *  {
 *    uint32_t TimerId;
 *    SYSTIMER_STATUS_t status;
 *    // ... Initializes APPs configuration ...
 *    DAVE_Init(); // SYSTIMER APP Initialized during DAVE Initialization
 *    // Create Software timer
 *    TimerId = (uint32_t)SYSTIMER_CreateTimer(ONESEC,SYSTIMER_MODE_PERIODIC,(void*)LED_Toggle_EverySec,NULL);
 *    if (TimerId != 0U)
 *    {
 *      //timer is created successfully
 *      // Start/Run Software timer
 *      status = SYSTIMER_StartTimer(TimerId);
 *      if (status == SYSTIMER_STATUS_SUCCESS)
 *      {
 *
 *        // User code
 *
 *
 *        status = SYSTIMER_StopTimer(TimerId);
 *        //User code
 *
 *        if (status == SYSTIMER_STATUS_SUCCESS)
 *        {
 *          //User code
 *
 *
 *          status = SYSTIMER_RestartTimer(TimerId,NEW_INTERVAL);
 *          if (status == SYSTIMER_STATUS_SUCCESS)
 *          {
 *            // timer configured with the new time interval and is running
 *          }
 *          else
 *          {
 *            // Error during software timer restart operation
 *          }
 *        }
 *        else
 *        {
 *           // Error during software timer stop operation
 *        }
 *      }
 *      else
 *      {
 *        // Error during software timer start operation
 *      }
 *    }
 *    else
 *    {
 *      // timer ID can not be zero
 *    }
 *    while (1)
 *    {
 *
 *    }
 *    return (1);
 *  }
 * @endcode<BR> </p>
 */
SYSTIMER_STATUS_t SYSTIMER_RestartTimer(uint32_t id, uint32_t microsec);

/**
 * @brief A version of SYSTIMER_RestartTimer() that can be called from an ISR.
 * @param id ID of already created system timer. Range : 1 to 16
 * @param microsec new time interval. Range: (SYSTIMER_TICK_PERIOD_US) to pow(2,32).
 * @return SYSTIMER_STATUS_t APP status. Refer @ref SYSTIMER_STATUS_t for details.
 *
 */
SYSTIMER_STATUS_t SYSTIMER_RestartTimerFromISR(uint32_t id, uint32_t microsec);

/**
 * @brief Deletes the software timer from the timer list. This function cannot be called from an ISR. Use SYSTIMER_DeleteTimerFromISR() instead.
 * @param id  timer ID obtained from SYSTIMER_CreateTimer. Range : 1 to 16
 * @return SYSTIMER_STATUS_t APP status. Refer @ref SYSTIMER_STATUS_t for details.
 *
 * \par<b>Description: </b><br>
 * API for deleting the created software timer instance from timer list.<br>
 *<b>Note :</b> This API must be called after software timer is created using SYSTIMER_CreateTimer API with generated
 * ID and enable XMC_ASSERT for better understanding of API behavioral in run time.<br>
 *
 *
 * \par<b>Example Usage:</b><br>
 *
 * @code
 *  #include <DAVE.h>
 *  #define ONESEC 1000000U
 *  void LED_Toggle_EverySec(void)
 *  {
 *    // Add user code here
 *  }
 *
 *  int main(void)
 *  {
 *    uint32_t TimerId;
 *    SYSTIMER_STATUS_t status;
 *    // ... Initializes APPs configuration ...
 *    DAVE_Init(); // SYSTIMER APP Initialized during DAVE Initialization
 *    // Create Software timer
 *    TimerId = (uint32_t)SYSTIMER_CreateTimer(ONESEC,SYSTIMER_MODE_PERIODIC,(void*)LED_Toggle_EverySec,NULL);
 *    if (TimerId != 0U)
 *    {
 *      //timer is created successfully, now start/run software timer
 *      status = SYSTIMER_StartTimer(TimerId);
 *      if (status == SYSTIMER_STATUS_SUCCESS)
 *      {
 *
 *      // User code
 *
 *
 *      status = SYSTIMER_StopTimer(TimerId);
 *      //User code
 *
 *        if (status == SYSTIMER_STATUS_SUCCESS)
 *        {
 *          //User code
 *
 *
 *          status = SYSTIMER_DeleteTimer(TimerId);
 *          if (status == SYSTIMER_STATUS_SUCCESS)
 *          {
 *            // Software timer has deleted
 *          }
 *          else
 *          {
 *            // Error during software timer delete operation
 *          }
 *        }
 *        else
 *        {
 *           // Error during software timer stop operation
 *        }
 *      }
 *      else
 *      {
 *        // Error during software timer start operation
 *      }
 *    }
 *    else
 *    {
 *      // timer ID Can not be zero
 *    }
 *    // ... infinite loop ...
 *    while (1)
 *    {
 *
 *    }
 *    return (1);
 *  }
 * @endcode<BR> </p>
 */
SYSTIMER_STATUS_t SYSTIMER_DeleteTimer(uint32_t id);

/**
 * @brief A version of SYSTIMER_DeleteTimer() that can be called from an ISR.
 * @param id  timer ID obtained from SYSTIMER_CreateTimer. Range : 1 to 16
 * @return SYSTIMER_STATUS_t APP status. Refer @ref SYSTIMER_STATUS_t for details.
 *
 */
SYSTIMER_STATUS_t SYSTIMER_DeleteTimerFromISR(uint32_t id);

/**
 * @brief Gives the current hardware SysTick time in microsecond since start of hardware SysTick timer.
 * @return  uint32_t  returns current SysTick time in microsecond. Range: (SYSTIMER_TICK_PERIOD_US) to pow(2,32).
 *
 * \par<b>Description: </b><br>
 * API to get current hardware SysTick time in microsecond since start of hardware SysTick timer.
 *
 * \par<b>Example Usage:</b><br>
 *
 * @code
 *  #include <DAVE.h>
 *  #define ONESEC 1000000U
 *  void LED_Toggle_EverySec(void)
 *  {
 *    // Add user code here
 *  }
 *
 *  int main(void)
 *  {
 *    uint32_t TimerId;
 *    uint32_t SysTick_Time;
 *    SYSTIMER_STATUS_t status;
 *    // ... Initializes APPs configuration ...
 *    DAVE_Init(); // SYSTIMER APP Initialized during DAVE Initialization
 *    // Create Software timer
 *    TimerId = (uint32_t)SYSTIMER_CreateTimer(ONESEC,SYSTIMER_MODE_PERIODIC,(void*)LED_Toggle_EverySec,NULL);
 *    if (TimerId != 0U)
 *    {
 *      //timer is created successfully, now start/run software timer
 *      status = SYSTIMER_StartTimer(TimerId);
 *      if (status == SYSTIMER_STATUS_SUCCESS)
 *      {
 *        // Add user code here
 *
 *
 *         SysTick_Time = SYSTIMER_GetTime();
 *         // Add user code here
 *
 *      }
 *      else
 *      {
 *      // Error during software timer start operation
 *      }
 *    }
 *    else
 *    {
 *      // timer ID Can not be zero
 *    }
 *    // ... infinite loop ...
 *    while (1)
 *    {
 *
 *    }
 *    return (1);
 *  }
 * @endcode<BR> </p>
 */
uint32_t SYSTIMER_GetTime(void);

/**
 * @brief Gives the SysTick count.
 * @return  uint32_t  returns SysTick count. Range: 0 to pow(2,32).
 *
 * \par<b>Description: </b><br>
 * API to get hardware SysTick counts since start of hardware SysTick timer.
 *
 *
 * \par<b>Example Usage:</b><br>
 *
 * @code
 *  #include <DAVE.h>
 *  #include <DAVE.h>
 *  #define ONESEC 1000000U
 *  void LED_Toggle_EverySec(void)
 *  {
 *    // Add user code here
 *  }
 *
 *
 *  int main(void)
 *  {
 *    uint32_t TimerId;
 *    uint32_t SysTick_Count;
 *    SYSTIMER_STATUS_t status;
 *    // ... Initializes APPs configuration ...
 *    DAVE_Init(); // SYSTIMER APP Initialized during DAVE Initialization
 *    // Create Software timer
 *    TimerId = (uint32_t)SYSTIMER_CreateTimer(ONESEC,SYSTIMER_MODE_PERIODIC,(void*)LED_Toggle_EverySec,NULL);
 *    if (TimerId != 0U)
 *    {
 *      //timer is created successfully, now start/run software timer
 *      status = SYSTIMER_StartTimer(TimerId);
 *      if (status == SYSTIMER_STATUS_SUCCESS)
 *      {
 *        // Add user code here
 *
 *
 *         SysTick_Count = SYSTIMER_GetTickCount();
 *         // Add user code here
 *
 *      }
 *      else
 *      {
 *      // Error during software timer start operation
 *      }
 *    }
 *    else
 *    {
 *      // timer ID Can not be zero
 *    }
 *    // ... infinite loop ...
 *    while (1)
 *    {
 *
 *    }
 *    return (1);
 *  }
 * @endcode<BR> </p>
 */
uint32_t SYSTIMER_GetTickCount(void);

/**
 * @brief Gives the current state of software timer.
 * @param id  timer ID obtained from SYSTIMER_CreateTimer. Range : 1 to 16
 * @return SYSTIMER_STATE_t Software timer state. Refer @ref SYSTIMER_STATE_t for details.
 *
 * \par<b>Description: </b><br>
 * API to get current software timer state.
 *
 * \par<b>Example Usage:</b><br>
 *
 * @code
 *  #include <DAVE.h>
 *  #define ONESEC 1000000U
 *  #define NEW_INTERVAL (ONESEC * 10U)
 *  void LED_Toggle_EverySec(void)
 *  {
 *    // Add user code here
 *  }
 *
 *  int main(void)
 *  {
 *    uint32_t TimerId;
 *    SYSTIMER_STATUS_t status;
 *    SYSTIMER_STATE_t timer_state;
 *    // ... Initializes APPs configuration ...
 *    DAVE_Init(); // SYSTIMER APP Initialized during DAVE Initialization
 *    // Create Software timer
 *    TimerId = (uint32_t)SYSTIMER_CreateTimer(ONESEC,SYSTIMER_MODE_PERIODIC,(void*)LED_Toggle_EverySec,NULL);
 *    if (TimerId != 0U)
 *    {
 *      //timer is created successfully, now start/run software timer
 *      status = SYSTIMER_StartTimer(TimerId);
 *      timer_state = SYSTIMER_GetTimerState(TimerId);  // use case scenario 1
 *      if (timer_state == SYSTIMER_STATE_RUNNING)
 *      {
 *        // software timer start operation is successful
 *        // Add user code here
 *      }
 *      else
 *      {
 *        // Error during software timer start operation
 *      }
 *
 *      // Add user code here
 *
 *      // user decided to change software interval, oops but user don't know the timer state
 *
 *      timer_state = SYSTIMER_GetTimerState(TimerId);   // use case scenario 2
 *      if (timer_state == SYSTIMER_STATE_RUNNING)
 *      {
 *        status = SYSTIMER_StopTimer(TimerId);
 *        status = SYSTIMER_RestartTimer(TimerId,NEW_INTERVAL);
 *        // Add user code here
 *      }
 *      else if (timer_state == SYSTIMER_STATE_STOPPED)
 *      {
 *        status = SYSTIMER_RestartTimer(TimerId,NEW_INTERVAL);
 *      }
 *      else if (timer_state == SYSTIMER_STATE_NOT_INITIALIZED)
 *      {
 *       // user has already deleted this software timer but need to recreate
 *       TimerId = (uint32_t)SYSTIMER_CreateTimer(NEW_INTERVAL,SYSTIMER_MODE_PERIODIC,(void*)LED_Toggle_EverySec,NULL);
 *       status = SYSTIMER_StartTimer(TimerId);
 *       // Add user code here
 *
 *      }
 *    }
 *    else
 *    {
 *      // timer ID Can not be zero
 *    }
 *    // ... infinite loop ...
 *    while (1)
 *    {
 *
 *    }
 *    return (1);
 *  }
 * @endcode<BR> </p>
 */
SYSTIMER_STATE_t SYSTIMER_GetTimerState(uint32_t id);

/**
 * @brief Initializes the CCU4_SLICE_CONFIG APP.
 * @param  handle_ptr Instance of CCU4_SLICE_CONFIG_t APP
 * @return CCU4_SLICE_CONFIG_STATUS_t
 *
 * \par<b>Description: </b><br>
 * Configures the CCU4 slice registers with the selected CCU4_SLICE_CONFIG parameters.
 *
 * Example Usage:
 * @code
  #include <DAVE.h>

  int main(void)
  {
   DAVE_Init(); //CCU4_SLICE_CONFIG_Init() is called by DAVE_Init().
    while(1);
    return 0;
  }
 * @endcode
 */
CCU4_SLICE_CONFIG_STATUS_t CCU4_SLICE_CONFIG_Init(const CCU4_SLICE_CONFIG_t *const handle_ptr);


/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_TIMER_H_ */

/* --- End of File --- */
