/**
 *
 *  drv_timer.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */
 
/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/drv_timer.h"

/******************************************************************************
   2. DATA
*******************************************************************************/
#define TIMER_CMP_100_DUTY             ((uint16_t)0) /* Compare value for 100% duty cycle */
#define TIMER_RESOLUTION_SEC_TO_MICRO  (100000000U)  /* Convert the resolution from sec to usec, by dividing with the \
                                                      scale factor */
#define TIMER_PRESCALER_MAX            (15U)      /* Maximum prescaler values allowed */
#define TIMER_PERIOD_16BIT_MAX         (0xFFFFU)  /* Maximum period value */
#define TIMER_PERIOD_MIN               (0x1U)     /* Minimum period value */
#define TIMER_CLK_SCALE_FACTOR         (32U)      /* Scale factor used during calculation of the "TIMER_CLK_CONST_SCALED" */

#define HW_TIMER_ADDITIONAL_CNT (1U)

static CCU4_SLICE_CONFIG_STATUS_t CCU4_SLICE_CONFIG_ADC_TRIG_lInit(void);

const CCU4_SLICE_CONFIG_t CCU4_SLICE_CONFIG_ADC_TRIG =
{
  .init = CCU4_SLICE_CONFIG_ADC_TRIG_lInit,
  .global = &GLOBAL_CCU4_0,
  .slice_ptr = CCU43_CC43,
  .slice_num = (uint8_t)3
};

const XMC_CCU4_SLICE_COMPARE_CONFIG_t CCU4_SLICE_CONFIG_ADC_TRIG_compare_config     =
{
  .timer_mode            = (uint32_t)XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot              = (uint32_t)XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
  .shadow_xfer_clear     = true,
  .dither_timer_period   = false,
  .dither_duty_cycle     = false,
  .prescaler_mode        = (uint32_t)XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable            = false,
  .prescaler_initval     = XMC_CCU4_SLICE_PRESCALER_1,
  .float_limit           = XMC_CCU4_SLICE_PRESCALER_32768,
  .dither_limit          = 0,
  .passive_level         = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .timer_concatenation   = false
};

const XMC_CCU4_SLICE_EVENT_CONFIG_t CCU4_SLICE_CONFIG_ADC_TRIG_event0_config =
{
  .mapped_input        = XMC_CCU4_SLICE_INPUT_D,
  .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE,
  .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
  .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED
 };

const XMC_CCU4_SLICE_EVENT_CONFIG_t CCU4_SLICE_CONFIG_ADC_TRIG_event1_config =
{
  .mapped_input        = XMC_CCU4_SLICE_INPUT_D,
  .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_FALLING_EDGE,
  .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW,
  .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED
 };

const XMC_CCU4_SLICE_EVENT_CONFIG_t CCU4_SLICE_CONFIG_ADC_TRIG_event2_config =
{
  .mapped_input        = XMC_CCU4_SLICE_INPUT_A,
  .edge                = XMC_CCU4_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
  .level               = XMC_CCU4_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_HIGH,
  .duration            = XMC_CCU4_SLICE_EVENT_FILTER_DISABLED
 };

/**< Configuration for HandleGLOBAL_CCU4_0 */
GLOBAL_CCU4_t GLOBAL_CCU4_0 =
{
  .module_frequency = 140000000U,  /**< CCU4 input clock frequency */
  .syncstart_trigger_msk = XMC_SCU_CCU_TRIGGER_CCU43,
  .module_ptr = (XMC_CCU4_MODULE_t*) CCU43,      /**< CCU4 Module Pointer */
  .mcs_action = (XMC_CCU4_SLICE_MCMS_ACTION_t)XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR,
  .is_initialized = false
};

/**< Configuration for HandleGLOBAL_CCU8_0 */
GLOBAL_CCU8_t GLOBAL_CCU8_0 =
{
  .module_frequency = 140000000U,  /**< CCU8 input clock frequency */
  .syncstart_trigger_msk = XMC_SCU_CCU_TRIGGER_CCU80,
  .module_ptr = (XMC_CCU8_MODULE_t*) CCU80,      /**< CCU8 Module Pointer */
  .mcs_action = (XMC_CCU8_SLICE_MCMS_ACTION_t)XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR,
  .is_initialized = false
};


/**
  * @brief Contents entered via GUI
  */
XMC_CCU8_SLICE_COMPARE_CONFIG_t TIMER_DELAY_config =
{
  .timer_mode          = XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot            = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
  .shadow_xfer_clear   = false,
  .dither_timer_period = false,
  .dither_duty_cycle   = false,
  .prescaler_mode      = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_ch1_enable      = false,
  .mcm_ch2_enable      = false,
  .slice_status        = XMC_CCU8_SLICE_STATUS_CHANNEL_1,
  .passive_level_out0  = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .passive_level_out1  = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .passive_level_out2  = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .passive_level_out3  = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .asymmetric_pwm      = false,
  .prescaler_initval   = 0U,
  .float_limit         = 0U,
  .dither_limit        = 0U,
  .timer_concatenation = false

};

TIMER_t TIMER_DELAY =
{
  .ccu8_slice_ptr         = (XMC_CCU8_SLICE_t*) CCU80_CC80,
  .ccu8_slice_number      = 0U,
  .time_interval_value_us = 100U,
  .timer_max_value_us     = 1533893500U,
  .timer_min_value_us     = 10U,
  .global_ccu8_handler    = (GLOBAL_CCU8_t*)&GLOBAL_CCU8_0,
  .ccu8_slice_config_ptr  = (XMC_CCU8_SLICE_COMPARE_CONFIG_t*)&TIMER_DELAY_config,
  .shadow_mask            = (uint32_t)((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
                                       (uint32_t)XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_0),
  .ccu8_period_match_node = XMC_CCU8_SLICE_SR_ID_0,
  .timer_module           = TIMER_MODULE_CCU8,
  .period_value           = 139U,
  .start_control          = false,
  .period_match_enable    = false,
  .initialized            = false
};


/**
 * @brief Contents entered via GUI
 */

XMC_CCU4_SLICE_COMPARE_CONFIG_t TIMER_FRAME_TRIG_config =
{
  .timer_mode          = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot            = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
  .shadow_xfer_clear   = false,
  .dither_timer_period = false,
  .dither_duty_cycle   = false,
  .prescaler_mode      = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable          = false,
  .prescaler_initval   = 6U,
  .float_limit         = 0U,
  .dither_limit        = 0U,
  .passive_level       = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .timer_concatenation = false
};

TIMER_t TIMER_FRAME_TRIG =
{
  .ccu4_slice_ptr         = (XMC_CCU4_SLICE_t*) CCU43_CC42,
  .ccu4_slice_number      = 2U,
  .time_interval_value_us = 2000000U,
  .timer_max_value_us     = 1533893500U,
  .timer_min_value_us     = 10U,
  .global_ccu4_handler    = (GLOBAL_CCU4_t*)&GLOBAL_CCU4_0,
  .ccu4_slice_config_ptr  = (XMC_CCU4_SLICE_COMPARE_CONFIG_t*)&TIMER_FRAME_TRIG_config,
  .shadow_mask            = (uint32_t)((uint32_t)XMC_CCU4_SHADOW_TRANSFER_SLICE_2 |
                                       (uint32_t)XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_2),
  .ccu4_period_match_node = XMC_CCU4_SLICE_SR_ID_0,
  .timer_module           = TIMER_MODULE_CCU4,
  .period_value           = 43749U,
  .start_control          = false,
  .period_match_enable    = true,
  .initialized            = false
};


/**
 * @brief Contents entered via GUI
 */

XMC_CCU4_SLICE_COMPARE_CONFIG_t TIMER_WATCHDOG_config =
{
  .timer_mode          = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot            = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
  .shadow_xfer_clear   = false,
  .dither_timer_period = false,
  .dither_duty_cycle   = false,
  .prescaler_mode      = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable          = false,
  .prescaler_initval   = 5U,
  .float_limit         = 0U,
  .dither_limit        = 0U,
  .passive_level       = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .timer_concatenation = false
};

TIMER_t TIMER_WATCHDOG =
{
  .ccu4_slice_ptr         = (XMC_CCU4_SLICE_t*) CCU43_CC41,
  .ccu4_slice_number      = 1U,
  .time_interval_value_us = 1000000U,
  .timer_max_value_us     = 1533893500U,
  .timer_min_value_us     = 10U,
  .global_ccu4_handler    = (GLOBAL_CCU4_t*)&GLOBAL_CCU4_0,
  .ccu4_slice_config_ptr  = (XMC_CCU4_SLICE_COMPARE_CONFIG_t*)&TIMER_WATCHDOG_config,
  .shadow_mask            = (uint32_t)((uint32_t)XMC_CCU4_SHADOW_TRANSFER_SLICE_1 |
                                       (uint32_t)XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1),
  .ccu4_period_match_node = XMC_CCU4_SLICE_SR_ID_1,
  .timer_module           = TIMER_MODULE_CCU4,
  .period_value           = 43749U,
  .start_control          = false,
  .period_match_enable    = true,
  .initialized            = false
};

SYSTIMER_t SYSTIMER_0 =
{
  .init_status = false /* APP initialization status to ensure whether SYSTIMER_Init called or not */
};


/* SYSTIMER_OBJECT structure acts as the timer control block */

typedef struct SYSTIMER_OBJECT
{
  struct SYSTIMER_OBJECT *next; /**< pointer to next timer control block */
  struct SYSTIMER_OBJECT *prev; /**< Pointer to previous timer control block */
  SYSTIMER_CALLBACK_t callback; /**< Callback function pointer */
  SYSTIMER_MODE_t mode; /**< timer Type (single shot or periodic) */
  SYSTIMER_STATE_t state; /**< timer state */
  void *args; /**< Parameter to callback function */
  uint32_t id; /**< timer ID */
  uint32_t count; /**< timer count value */
  uint32_t reload; /**< timer Reload count value */
  bool delete_swtmr; /**< To delete the timer */
} SYSTIMER_OBJECT_t;

/** Table which save timer control block. */
SYSTIMER_OBJECT_t g_timer_tbl[SYSTIMER_CFG_MAX_TMR];

/* The header of the timer Control list. */
SYSTIMER_OBJECT_t *g_timer_list = NULL;

/* Timer ID tracker */
uint32_t g_timer_tracker = 0U;

/* SysTick counter */
volatile uint32_t g_systick_count = 0U;

/******************************************************************************
   3. LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

#ifdef TIMER_CCU4_USED
TIMER_STATUS_t TIMER_CCU4_lInit(TIMER_t* const handle_ptr);
void TIMER_CCU4_lShadowTransfer(TIMER_t* const handle_ptr);
#endif

#ifdef TIMER_CCU8_USED
TIMER_STATUS_t TIMER_CCU8_lInit(TIMER_t* const handle_ptr);
void TIMER_CCU8_lShadowTransfer(TIMER_t* const handle_ptr);
#endif

/*
 * This function is called to insert a timer into the timer list.
 */
static void SYSTIMER_lInsertTimerList(uint32_t tbl_index);

/*
 * This function is called to remove a timer from the timer list.
 */
static void SYSTIMER_lRemoveTimerList(uint32_t tbl_index);

/*
 * Handler function  called from SysTick event handler.
 */
static void SYSTIMER_lTimerHandler(void);

/*
 * SysTick handler which is the main interrupt service routine to service the
 * system timer's configured
 */
void SysTick_Handler(void);

/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/

/* API to initialize the CCU4 slice */
CCU4_SLICE_CONFIG_STATUS_t CCU4_SLICE_CONFIG_Init(const CCU4_SLICE_CONFIG_t *const handle_ptr)
{
  return handle_ptr->init();
}

/* Initializes the slice with the generated configuration */
GLOBAL_CCU4_STATUS_t GLOBAL_CCU4_Init(GLOBAL_CCU4_t* handle)
{
  XMC_ASSERT("GLOBAL_CCU4_Init:NULL handler", (NULL != handle));

  if (false == handle->is_initialized)
  {
    /* Enable CCU4 module */
    XMC_CCU4_Init(handle->module_ptr,handle->mcs_action);
    /* Start the prescaler */
    XMC_CCU4_StartPrescaler(handle->module_ptr);
    /* Restricts multiple initializations */
    handle->is_initialized = true;
  }

  return (GLOBAL_CCU4_STATUS_SUCCESS);
}

/* Initializes the slice with the generated configuration */
GLOBAL_CCU8_STATUS_t GLOBAL_CCU8_Init(GLOBAL_CCU8_t* handle)
{
  XMC_ASSERT("GLOBAL_CCU8_Init:NULL handler", (NULL != handle));

  if (false == handle->is_initialized)
  {
    /* Enable CCU8 module */
    XMC_CCU8_Init(handle->module_ptr,handle->mcs_action);
    /* Start the prescaler */
    XMC_CCU8_StartPrescaler(handle->module_ptr);
    /* Restricts multiple initializations */
    handle->is_initialized = true;
  }

  return (GLOBAL_CCU8_STATUS_SUCCESS);
}

/*
 * This function initializes a TIMER APP based on user configuration.
 *
 */
TIMER_STATUS_t TIMER_Init(TIMER_t* const handle_ptr)
{
  TIMER_STATUS_t status;

  XMC_ASSERT("TIMER_Init:handle_ptr NULL" , (handle_ptr != NULL));

  status = TIMER_STATUS_SUCCESS;
  /* Check for APP instance is initialized or not */
  if (false == handle_ptr->initialized)
  {
#ifdef TIMER_CCU4_USED
    if (TIMER_MODULE_CCU4 == handle_ptr->timer_module)
    {
      /* Configure CCU4 timer for the required time tick settings */
      status = TIMER_CCU4_lInit(handle_ptr);
    }
#endif

#ifdef TIMER_CCU8_USED
    if (TIMER_MODULE_CCU8 == handle_ptr->timer_module)
    {
      /* Configure CCU8 timer for the required time tick settings */
      status = TIMER_CCU8_lInit(handle_ptr);
    }
#endif
  }

  return (status);
}

/*
 * This function starts the timer to generate the events for the specified time_interval value
 */
TIMER_STATUS_t TIMER_Start(TIMER_t  *const handle_ptr)
{
  TIMER_STATUS_t status;

  XMC_ASSERT("TIMER_Start:handle_ptr NULL" , (handle_ptr != NULL));

  /* Check for APP instance is initialized or not */
  if (true == handle_ptr->initialized)
  {
#ifdef TIMER_CCU4_USED
    if (TIMER_MODULE_CCU4 == handle_ptr->timer_module)
    {
      /* Start the timer manually */
      XMC_CCU4_SLICE_StartTimer(handle_ptr->ccu4_slice_ptr);
    }
#endif

#ifdef TIMER_CCU8_USED
    if (TIMER_MODULE_CCU8 == handle_ptr->timer_module)
       {
      /* Start the timer manually */
      XMC_CCU8_SLICE_StartTimer(handle_ptr->ccu8_slice_ptr);
       }
#endif
    status = TIMER_STATUS_SUCCESS;
  }
  else
  {
    status = TIMER_STATUS_FAILURE;
  }

  return (status);
}

/*
 * This function stops and clears the timer
 */
TIMER_STATUS_t TIMER_Stop(TIMER_t  *const handle_ptr)
{
  TIMER_STATUS_t status;

  XMC_ASSERT("TIMER_Stop:handle_ptr NULL" , (handle_ptr != NULL));

  /* Check whether timer is initialized and in running state */
  if ((TIMER_GetTimerStatus(handle_ptr)) && (true == handle_ptr->initialized))
  {
#ifdef TIMER_CCU4_USED
    if (TIMER_MODULE_CCU4 == handle_ptr->timer_module)
    {
      /* Stops the timer */
      XMC_CCU4_SLICE_StopTimer(handle_ptr->ccu4_slice_ptr);
    }
#endif

#ifdef TIMER_CCU8_USED
    if (TIMER_MODULE_CCU8 == handle_ptr->timer_module)
       {
      /* Stops the timer */
      XMC_CCU8_SLICE_StopTimer(handle_ptr->ccu8_slice_ptr);
       }
#endif
    status = TIMER_STATUS_SUCCESS;
  }
  else
  {
    status = TIMER_STATUS_FAILURE;
  }

  return (status);
}

/*
 * This function returns the status of the timer
 */
bool TIMER_GetTimerStatus(TIMER_t  *const handle_ptr)
{
  bool status;

  XMC_ASSERT("TIMER_GetTimerStatus:handle_ptr NULL" , (handle_ptr != NULL));

  status = false;

#ifdef TIMER_CCU4_USED
  if (TIMER_MODULE_CCU4 == handle_ptr->timer_module)
  {
    /* Returns the current status of the timer */
    status = XMC_CCU4_SLICE_IsTimerRunning(handle_ptr->ccu4_slice_ptr);
  }
#endif

#ifdef TIMER_CCU8_USED
  if (TIMER_MODULE_CCU8 == handle_ptr->timer_module)
  {
    /* Returns the current status of the timer */
    status = XMC_CCU8_SLICE_IsTimerRunning(handle_ptr->ccu8_slice_ptr);
  }
#endif

  return (status);
}

/*
 * This function changes the PWM period which in turn changes the time tick interval value by checking that
 * the given time tick value is within supported range.
 */
TIMER_STATUS_t TIMER_SetTimeInterval(TIMER_t  *const handle_ptr, uint32_t time_interval)
{
  TIMER_STATUS_t status;
  uint32_t lfrequency;
  uint32_t lprescaler;

  XMC_ASSERT("TIMER_SetTimeInterval:handle_ptr NULL" , (handle_ptr != NULL));

  status = TIMER_STATUS_FAILURE;

  if (false == TIMER_GetTimerStatus(handle_ptr))
  {
    /* check for time_interval range */
    if ((time_interval >= handle_ptr->timer_min_value_us) && (time_interval <= handle_ptr->timer_max_value_us))
    {
      /* Initialize the prescaler */
      lprescaler = 0U;
      while (time_interval > (handle_ptr->timer_max_value_us >> (TIMER_PRESCALER_MAX - lprescaler)))
      {
        lprescaler++;
      }
#ifdef TIMER_CCU4_USED
      if (TIMER_MODULE_CCU4 == handle_ptr->timer_module)
      {
        lfrequency = handle_ptr->global_ccu4_handler->module_frequency;
        handle_ptr->ccu4_slice_config_ptr->prescaler_initval = lprescaler;
        /* Calculate the period register for the required time_interval value */
        handle_ptr->period_value = (uint16_t)((((uint64_t)time_interval * lfrequency) >> \
                                               handle_ptr->ccu4_slice_config_ptr->prescaler_initval) / \
                                              TIMER_RESOLUTION_SEC_TO_MICRO);
        /* Actual timer period values is Period_reg_val+1U */
        if (handle_ptr->period_value > TIMER_PERIOD_MIN)
        {
          (handle_ptr->period_value)--;
        }
        /* Update the prescaler */
        XMC_CCU4_SLICE_SetPrescaler(handle_ptr->ccu4_slice_ptr, handle_ptr->ccu4_slice_config_ptr->prescaler_initval);
        /* update period, compare and prescaler values */
        TIMER_CCU4_lShadowTransfer(handle_ptr);
        /* Update the status */
        status = TIMER_STATUS_SUCCESS;
      }
#endif

#ifdef TIMER_CCU8_USED
      if (TIMER_MODULE_CCU8 == handle_ptr->timer_module)
      {
        handle_ptr->ccu8_slice_config_ptr->prescaler_initval = lprescaler;
        lfrequency = handle_ptr->global_ccu8_handler->module_frequency;
        /* Calculate the period register for the required time_interval value */
        handle_ptr->period_value = (uint16_t)((((uint64_t)time_interval * lfrequency) >> \
                                               handle_ptr->ccu8_slice_config_ptr->prescaler_initval) / \
                                              TIMER_RESOLUTION_SEC_TO_MICRO);
        /* Actual timer period values is Period_reg_val+1U */
        if (handle_ptr->period_value > TIMER_PERIOD_MIN)
        {
          (handle_ptr->period_value)--;
        }
        /* Update the prescaler */
        XMC_CCU8_SLICE_SetPrescaler(handle_ptr->ccu8_slice_ptr, handle_ptr->ccu8_slice_config_ptr->prescaler_initval);
        /* update period, compare and prescaler values */
        TIMER_CCU8_lShadowTransfer(handle_ptr);
        /* Update the status */
        status = TIMER_STATUS_SUCCESS;
      }
#endif
    }
  }
  return (status);
}

/*
 * This function reads the timer event(period match interrupt) status of the given timer
 */
bool TIMER_GetInterruptStatus(TIMER_t * const handle_ptr)
{
  bool status;
  XMC_ASSERT("TIMER_GetInterruptStatus:handle_ptr NULL" , (handle_ptr != NULL));
  status = false;
#ifdef TIMER_CCU4_USED
  if (TIMER_MODULE_CCU4 == handle_ptr->timer_module)
  {
    /* Reads the interrupt status */
    status = XMC_CCU4_SLICE_GetEvent(handle_ptr->ccu4_slice_ptr, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
  }
#endif

#ifdef TIMER_CCU8_USED
  if (TIMER_MODULE_CCU8 == handle_ptr->timer_module)
  {
    /* Reads the interrupt status */
    status = XMC_CCU8_SLICE_GetEvent(handle_ptr->ccu8_slice_ptr, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
  }
#endif
  return (status);
}

/*
 * This function clears the period match interrupt status of the given timer.
 */
void TIMER_ClearEvent(TIMER_t *const handle_ptr)
{
  XMC_ASSERT("TIME_CCU_AcknowledgeInterrupt:handle_ptr NULL" , (handle_ptr != NULL));

#ifdef TIMER_CCU4_USED
  if (TIMER_MODULE_CCU4 == handle_ptr->timer_module)
  {
    /* clears the timer event(period match interrupt) */
    XMC_CCU4_SLICE_ClearEvent(handle_ptr->ccu4_slice_ptr, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
  }
#endif

#ifdef TIMER_CCU8_USED
  if (TIMER_MODULE_CCU8 == handle_ptr->timer_module)
  {
    /* clears the timer event(period match interrupt) */
    XMC_CCU8_SLICE_ClearEvent(handle_ptr->ccu8_slice_ptr, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
  }
#endif
}

/*
 * This function returns the current time value
 */
uint32_t TIMER_GetTime(TIMER_t *const handle_ptr)
{
  uint32_t ltimer_val;
  uint32_t lprescaler;
  uint32_t ltime_val;

  XMC_ASSERT("TIMER_GetTimerStatus:handle_ptr NULL" , (handle_ptr != NULL));
  ltime_val = 0U;

#ifdef TIMER_CCU4_USED
  if (TIMER_MODULE_CCU4 == handle_ptr->timer_module)
  {
    /* Added one to according to the edge aligned mode */
    ltimer_val = (uint32_t)XMC_CCU4_SLICE_GetTimerValue(handle_ptr->ccu4_slice_ptr) + 1U;
    lprescaler = handle_ptr->ccu4_slice_config_ptr->prescaler_initval;

    /* calculate the time value in micro seconds and scaled with 100 */
    ltime_val = (uint32_t)((uint64_t)((uint64_t)ltimer_val * (uint64_t)TIMER_CLK_CONST_SCALED) >> \
                           (TIMER_CLK_SCALE_FACTOR - lprescaler));
  }
#endif

#ifdef TIMER_CCU8_USED
  if (TIMER_MODULE_CCU8 == handle_ptr->timer_module)
  {
    /* Added one to according to the edge aligned mode */
    ltimer_val = (uint32_t)XMC_CCU8_SLICE_GetTimerValue(handle_ptr->ccu8_slice_ptr) + 1U;
    lprescaler = handle_ptr->ccu8_slice_config_ptr->prescaler_initval;

    /* calculate the time value in micro seconds and scaled with 100 */
    ltime_val = (uint32_t)((uint64_t)((uint64_t)ltimer_val * (uint64_t)TIMER_CLK_CONST_SCALED) >> \
                           (TIMER_CLK_SCALE_FACTOR - lprescaler));
  }
#endif

  return ltime_val;
}

/*
 * Clear the timer
 */
TIMER_STATUS_t TIMER_Clear(TIMER_t *const handle_ptr)
{
  TIMER_STATUS_t status;

  XMC_ASSERT("TIMER_Clear:handle_ptr NULL" , (handle_ptr != NULL));

  /* Check for APP instance is initialized or not */
  if (true == handle_ptr->initialized)
  {
#ifdef TIMER_CCU4_USED
    if (TIMER_MODULE_CCU4 == handle_ptr->timer_module)
    {
      /* Clear the timer register */
      XMC_CCU4_SLICE_ClearTimer(handle_ptr->ccu4_slice_ptr);
    }
#endif

#ifdef TIMER_CCU8_USED
    if (TIMER_MODULE_CCU8 == handle_ptr->timer_module)
    {
      /* Clear the timer register */
      XMC_CCU8_SLICE_ClearTimer(handle_ptr->ccu8_slice_ptr);
    }
#endif
    status = TIMER_STATUS_SUCCESS;
  }
  else
  {
    status = TIMER_STATUS_FAILURE;
  }

  return (status);
}


/*
 * Initialization function which initializes the SYSTIMER APP, configures SysTick timer and SysTick exception.
 */
SYSTIMER_STATUS_t SYSTIMER_Init(SYSTIMER_t *handle)
{
  SYSTIMER_STATUS_t status = SYSTIMER_STATUS_SUCCESS;

  XMC_ASSERT("SYSTIMER_Init: SYSTIMER APP handle pointer uninitialized", (handle != NULL));

  /* Check APP initialization status to ensure whether SYSTIMER_Init called or not, initialize SYSTIMER if
   * SYSTIMER_Init called first time.
   */
  if (false == handle->init_status)
  {
    /* Initialize the header of the list */
    g_timer_list = NULL;
    /* Initialize SysTick timer */
    status = (SYSTIMER_STATUS_t)SysTick_Config((uint32_t)(SYSTIMER_SYSTICK_CLOCK * SYSTIMER_TICK_PERIOD));

    if (SYSTIMER_STATUS_FAILURE == status)
    {
      XMC_DEBUG("SYSTIMER_Init: Timer reload value out of range");
    }
    else
    {
#if (UC_FAMILY == XMC4)
      /* setting of First SW Timer period is always and subpriority value for XMC4000 devices */
      NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(
      NVIC_GetPriorityGrouping(), SYSTIMER_PRIORITY, SYSTIMER_SUBPRIORITY));
#elif (UC_FAMILY == XMC1)
      /* setting of priority value for XMC1000 devices */
      NVIC_SetPriority(SysTick_IRQn, SYSTIMER_PRIORITY);
#endif
      g_timer_tracker = 0U;
      /* Update the Initialization status of the SYSTIMER APP instance */
      handle->init_status = true;
      status = SYSTIMER_STATUS_SUCCESS;
    }
  }

  return (status);
}

/*
 *  API for creating a new software Timer instance.
 */
uint32_t SYSTIMER_CreateTimer
(
  uint32_t period,
  SYSTIMER_MODE_t mode,
  SYSTIMER_CALLBACK_t callback,
  void  *args
)
{
  uint32_t id = 0U;
  uint32_t count = 0U;
  uint32_t period_ratio = 0U;

  XMC_ASSERT("SYSTIMER_CreateTimer: Timer creation failure due to invalid period value",
            ((period >= SYSTIMER_TICK_PERIOD_US) && (period > 0U) && (period <= 0xFFFFFFFFU)));
  XMC_ASSERT("SYSTIMER_CreateTimer: Timer creation failure due to invalid timer mode",
            ((SYSTIMER_MODE_ONE_SHOT == mode) || (SYSTIMER_MODE_PERIODIC == mode)));
  XMC_ASSERT("SYSTIMER_CreateTimer: Can not create software without user callback", (NULL != callback));

  if (period < SYSTIMER_TICK_PERIOD_US)
  {
    id = 0U;
  }
  else
  {
    for (count = 0U; count < SYSTIMER_CFG_MAX_TMR; count++)
    {
      /* Check for free timer ID */
      if (0U == (g_timer_tracker & (1U << count)))
      {
        /* If yes, assign ID to this timer */
        g_timer_tracker |= (1U << count);
        /* Initialize the timer as per input values */
        g_timer_tbl[count].id     = count;
        g_timer_tbl[count].mode   = mode;
        g_timer_tbl[count].state  = SYSTIMER_STATE_STOPPED;
        period_ratio = (uint32_t)(period / SYSTIMER_TICK_PERIOD_US);
        g_timer_tbl[count].count  = (period_ratio + HW_TIMER_ADDITIONAL_CNT);
        g_timer_tbl[count].reload  = period_ratio;
        g_timer_tbl[count].callback = callback;
        g_timer_tbl[count].args = args;
        g_timer_tbl[count].prev   = NULL;
        g_timer_tbl[count].next   = NULL;
        id = count + 1U;
        break;
      }
    }

  }

  return (id);
}

/*
 *  API to start the software timer.
 */
SYSTIMER_STATUS_t SYSTIMER_StartTimer(uint32_t id)
{
  SYSTIMER_STATUS_t status;
  status = SYSTIMER_STATUS_FAILURE;

  XMC_ASSERT("SYSTIMER_StartTimer: Failure in timer restart operation due to invalid timer ID",
            ((id <= SYSTIMER_CFG_MAX_TMR) && (id > 0U)));
  XMC_ASSERT("SYSTIMER_StartTimer: Error during start of software timer", (0U != (g_timer_tracker & (1U << (id - 1U)))));

  /* Check if timer is running */
  if (SYSTIMER_STATE_STOPPED == g_timer_tbl[id - 1U].state)
  {
    g_timer_tbl[id - 1U].count = (g_timer_tbl[id - 1U].reload + HW_TIMER_ADDITIONAL_CNT);
    /* set timer status as SYSTIMER_STATE_RUNNING */
    g_timer_tbl[id - 1U].state = SYSTIMER_STATE_RUNNING;
    /* Insert this timer into timer list */
    SYSTIMER_lInsertTimerList((id - 1U));
    status = SYSTIMER_STATUS_SUCCESS;
  }

  return (status);
}

/*
 *  API to stop the software timer.
 */
SYSTIMER_STATUS_t SYSTIMER_StopTimer(uint32_t id)
{
  SYSTIMER_STATUS_t status;
  status = SYSTIMER_STATUS_SUCCESS;

  XMC_ASSERT("SYSTIMER_StopTimer: Failure in timer restart operation due to invalid timer ID",
            ((id <= SYSTIMER_CFG_MAX_TMR) && (id > 0U)));
  XMC_ASSERT("SYSTIMER_StopTimer: Error during stop of software timer", (0U != (g_timer_tracker & (1U << (id - 1U)))));

  if (SYSTIMER_STATE_NOT_INITIALIZED == g_timer_tbl[id - 1U].state)
  {
    status = SYSTIMER_STATUS_FAILURE;
  }
  else
  {
    /* Check whether Timer is in Stop state */
    if (SYSTIMER_STATE_RUNNING == g_timer_tbl[id - 1U].state)
    {
        /* Set timer status as SYSTIMER_STATE_STOPPED */
        g_timer_tbl[id - 1U].state = SYSTIMER_STATE_STOPPED;

        /* remove Timer from node list */
        SYSTIMER_lRemoveTimerList(id - 1U);

    }
  }

  return (status);
}

/*
 *  API to reinitialize the time interval and to start the timer.
 */
SYSTIMER_STATUS_t SYSTIMER_RestartTimer(uint32_t id, uint32_t microsec)
{
  uint32_t period_ratio = 0U;
  SYSTIMER_STATUS_t status;
  status = SYSTIMER_STATUS_SUCCESS;

  XMC_ASSERT("SYSTIMER_RestartTimer: Failure in timer restart operation due to invalid timer ID",
            ((id <= SYSTIMER_CFG_MAX_TMR) && (id > 0U)));
  XMC_ASSERT("SYSTIMER_RestartTimer: Error during restart of software timer", (0U != (g_timer_tracker & (1U << (id - 1U)))));
  XMC_ASSERT("SYSTIMER_RestartTimer: Can not restart timer due to invalid period value",
            (microsec >= SYSTIMER_TICK_PERIOD_US) && (microsec > 0U));


  if (SYSTIMER_STATE_NOT_INITIALIZED == g_timer_tbl[id - 1U].state)
  {
      status = SYSTIMER_STATUS_FAILURE;
  }
  else
  {
    /* check whether timer is in run state */
    if( SYSTIMER_STATE_STOPPED != g_timer_tbl[id - 1U].state)
    {
         /* Stop the timer */
         status = SYSTIMER_StopTimer(id);
    }
    if (SYSTIMER_STATUS_SUCCESS == status)
    {
      period_ratio = (uint32_t)(microsec / SYSTIMER_TICK_PERIOD_US);
      g_timer_tbl[id - 1U].reload = period_ratio;
      /* Start the timer */
      status = SYSTIMER_StartTimer(id);
    }
  }

  return (status);
}

/*
 *  Function to delete the Timer instance.
 */
SYSTIMER_STATUS_t SYSTIMER_DeleteTimer(uint32_t id)
{
  SYSTIMER_STATUS_t status;
  status = SYSTIMER_STATUS_SUCCESS;

  XMC_ASSERT("SYSTIMER_DeleteTimer: Failure in timer restart operation due to invalid timer ID",
            ((id <= SYSTIMER_CFG_MAX_TMR) && (id > 0U)));
  XMC_ASSERT("SYSTIMER_DeleteTimer: Error during deletion of software timer", (0U != (g_timer_tracker & (1U << (id - 1U)))));

  /* Check whether Timer is in delete state */
  if (SYSTIMER_STATE_NOT_INITIALIZED == g_timer_tbl[id - 1U].state)
  {
      status = SYSTIMER_STATUS_FAILURE;
  }
  else
  {
    if (SYSTIMER_STATE_STOPPED == g_timer_tbl[id - 1U].state)
    {
      /* Set timer status as SYSTIMER_STATE_NOT_INITIALIZED */
      g_timer_tbl[id - 1U].state = SYSTIMER_STATE_NOT_INITIALIZED;
      /* Release resource which are hold by this timer */
      g_timer_tracker &= ~(1U << (id - 1U));
    }
    else
    {
      /* Yes, remove this timer from timer list during ISR execution */
      g_timer_tbl[id - 1U].delete_swtmr = true;
    }
  }

  return (status);
}

/*
 *  API to get the current SysTick time in microsecond.
 */
uint32_t SYSTIMER_GetTime(void)
{
  return (g_systick_count * SYSTIMER_TICK_PERIOD_US);
}

/*
 *  API to get the SysTick count.
 */
uint32_t SYSTIMER_GetTickCount(void)
{
  return (g_systick_count);
}

/*
 *  API to get the current state of software timer.
 */
SYSTIMER_STATE_t SYSTIMER_GetTimerState(uint32_t id)
{
  return (g_timer_tbl[id - 1U].state);
}

__attribute__((always_inline)) __STATIC_INLINE uint32_t critical_section_enter(void)
{
  uint32_t status;
  status = __get_PRIMASK();
  __disable_irq ();
  return status;
}

__attribute__((always_inline)) __STATIC_INLINE void critical_section_exit(uint32_t status)
{
  __set_PRIMASK(status);
}

void SYSTIMER_Start(void)
{
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void SYSTIMER_Stop(void)
{
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

uint32_t SYSTIMER_CreateTimerFromISR
(
  uint32_t period,
  SYSTIMER_MODE_t mode,
  SYSTIMER_CALLBACK_t callback,
  void  *args
)
{
  uint32_t id;

  uint32_t ics;
  ics = critical_section_enter();

  id = SYSTIMER_CreateTimer(period, mode, callback, args);

  critical_section_exit(ics);

  return (id);
}

SYSTIMER_STATUS_t SYSTIMER_StartTimerFromISR(uint32_t id)
{
  SYSTIMER_STATUS_t status;

  uint32_t ics;
  ics = critical_section_enter();

  status = SYSTIMER_StartTimer(id);

  critical_section_exit(ics);

  return (status);
}

SYSTIMER_STATUS_t SYSTIMER_StopTimerFromISR(uint32_t id)
{
  SYSTIMER_STATUS_t status;

  uint32_t ics;
  ics = critical_section_enter();

  status = SYSTIMER_StopTimer(id);

  critical_section_exit(ics);

  return (status);
}

SYSTIMER_STATUS_t SYSTIMER_RestartTimerFromISR(uint32_t id, uint32_t microsec)
{
  SYSTIMER_STATUS_t status;

  uint32_t ics;
  ics = critical_section_enter();

  status = SYSTIMER_RestartTimer(id, microsec);

  critical_section_exit(ics);

  return (status);
}

SYSTIMER_STATUS_t SYSTIMER_DeleteTimerFromISR(uint32_t id)
{
  SYSTIMER_STATUS_t status;

  uint32_t ics;
  ics = critical_section_enter();

  status = SYSTIMER_DeleteTimer(id);

  critical_section_exit(ics);

  return (status);
}

/******************************************************************************
   5. LOCAL FUNCTIONS
*******************************************************************************/
/* Channel initialization function */
CCU4_SLICE_CONFIG_STATUS_t CCU4_SLICE_CONFIG_ADC_TRIG_lInit(void)
{
GLOBAL_CCU4_Init(&GLOBAL_CCU4_0);
/* Configure CCU4x_CC4y slice as timer unit */
XMC_CCU4_SLICE_CompareInit(CCU43_CC43, &CCU4_SLICE_CONFIG_ADC_TRIG_compare_config);
/* Set initial value of timer */
XMC_CCU4_SLICE_SetTimerValue(CCU43_CC43, (uint16_t)0U);
/* Set timer compare register value */
XMC_CCU4_SLICE_SetTimerCompareMatch(CCU43_CC43, (uint16_t)32768U);
/* Set timer period register value */
XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU43_CC43, (uint16_t)65535U);
/* Register value update settings */
XMC_CCU4_SetMultiChannelShadowTransferMode(CCU43, (uint32_t)XMC_CCU4_MULTI_CHANNEL_SHADOW_TRANSFER_SW_SLICE3);
/* Transfer value from shadow registers to actual timer registers */
XMC_CCU4_EnableShadowTransfer(CCU43, (uint32_t)XMC_CCU4_SHADOW_TRANSFER_SLICE_3 |
                                   (uint32_t)XMC_CCU4_SHADOW_TRANSFER_DITHER_SLICE_3 |
                                   (uint32_t)XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_3);
/* Events and function settings */
XMC_CCU4_SLICE_ConfigureEvent(CCU43_CC43, XMC_CCU4_SLICE_EVENT_0, &CCU4_SLICE_CONFIG_ADC_TRIG_event0_config);
XMC_CCU4_SLICE_ConfigureEvent(CCU43_CC43, XMC_CCU4_SLICE_EVENT_1, &CCU4_SLICE_CONFIG_ADC_TRIG_event1_config);
XMC_CCU4_SLICE_ConfigureEvent(CCU43_CC43, XMC_CCU4_SLICE_EVENT_2, &CCU4_SLICE_CONFIG_ADC_TRIG_event2_config);
XMC_CCU4_SLICE_StartConfig(CCU43_CC43, XMC_CCU4_SLICE_EVENT_0, XMC_CCU4_SLICE_START_MODE_TIMER_START);
XMC_CCU4_SLICE_StopConfig(CCU43_CC43, XMC_CCU4_SLICE_EVENT_1, XMC_CCU4_SLICE_END_MODE_TIMER_STOP_CLEAR);
XMC_CCU4_SLICE_SetInterruptNode(CCU43_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU4_SLICE_SR_ID_3);
XMC_CCU4_SLICE_EnableMultipleEvents(CCU43_CC43, XMC_CCU4_SLICE_MULTI_IRQ_ID_PERIOD_MATCH);
/* clear IDLE mode for the slice*/
XMC_CCU4_EnableClock(CCU43, (uint8_t)3);
return CCU4_SLICE_CONFIG_STATUS_SUCCESS;
}


#ifdef TIMER_CCU4_USED
/*
 * This function configures timer ccu4 timer with required time tick value
 */
TIMER_STATUS_t TIMER_CCU4_lInit(TIMER_t* const handle_ptr)
{
  TIMER_STATUS_t status;
  /* Initialize the global registers */
  status = (TIMER_STATUS_t)GLOBAL_CCU4_Init(handle_ptr->global_ccu4_handler);

  /* Enable the clock for selected timer */
  XMC_CCU4_EnableClock(handle_ptr->global_ccu4_handler->module_ptr, handle_ptr->ccu4_slice_number);
  /* Configure the timer with required settings */
  XMC_CCU4_SLICE_CompareInit(handle_ptr->ccu4_slice_ptr, handle_ptr->ccu4_slice_config_ptr);
  /* programs the timer period and compare register according to time interval value and do the shadow transfer */
  TIMER_CCU4_lShadowTransfer(handle_ptr);

#ifdef  TIMER_INTERRUPT
  if (true == handle_ptr->period_match_enable)
  {
    /* Binds a period match event to an NVIC node  */
    XMC_CCU4_SLICE_SetInterruptNode(handle_ptr->ccu4_slice_ptr, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH,
                                    handle_ptr->ccu4_period_match_node);
    /* Enables a timer(period match) event  */
    XMC_CCU4_SLICE_EnableEvent(handle_ptr->ccu4_slice_ptr, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
  }
#endif
  /* Clears the timer register */
  XMC_CCU4_SLICE_ClearTimer(handle_ptr->ccu4_slice_ptr);

  /* update the initialization flag as true for particular instance*/
  handle_ptr->initialized = true;

  /* Check whether the start of the timer is enabled during initialization or not */
  if (handle_ptr->start_control == true)
  {
    /* Start the timer */
    XMC_CCU4_SLICE_StartTimer(handle_ptr->ccu4_slice_ptr);
  }

  return (status);
}

/*
 * This function configures timer period and compare values and triggers the shadow transfer operation
 */
void TIMER_CCU4_lShadowTransfer(TIMER_t* const handle_ptr)
{
  /* programs the timer period register according to time interval value */
  XMC_CCU4_SLICE_SetTimerPeriodMatch(handle_ptr->ccu4_slice_ptr, handle_ptr->period_value);
  /* programs the timer compare register for 50% duty cycle */
  XMC_CCU4_SLICE_SetTimerCompareMatch(handle_ptr->ccu4_slice_ptr, TIMER_CMP_100_DUTY);
  /* Transfers value from shadow timer registers to actual timer registers */
  XMC_CCU4_EnableShadowTransfer(handle_ptr->global_ccu4_handler->module_ptr, handle_ptr->shadow_mask);
}
#endif

#ifdef TIMER_CCU8_USED
/*
 * This function configures timer ccu8 timer with required time tick value
 */
TIMER_STATUS_t TIMER_CCU8_lInit(TIMER_t* const handle_ptr)
{
  TIMER_STATUS_t status;
  /* Initialize the global registers */
  status = (TIMER_STATUS_t)GLOBAL_CCU8_Init(handle_ptr->global_ccu8_handler);

  /* Enable the clock for selected timer */
  XMC_CCU8_EnableClock(handle_ptr->global_ccu8_handler->module_ptr, handle_ptr->ccu8_slice_number);
  /* Configure the timer with required settings */
  XMC_CCU8_SLICE_CompareInit(handle_ptr->ccu8_slice_ptr, handle_ptr->ccu8_slice_config_ptr);
  /* programs the timer period and compare register according to time interval value and do the shadow transfer */
  TIMER_CCU8_lShadowTransfer(handle_ptr);

#ifdef  TIMER_INTERRUPT
  if (true == handle_ptr->period_match_enable)
  {
    /* Binds a period match event to an NVIC node  */
    XMC_CCU8_SLICE_SetInterruptNode(handle_ptr->ccu8_slice_ptr, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH,
                                    handle_ptr->ccu8_period_match_node);
    /* Enables a timer(period match) event  */
    XMC_CCU8_SLICE_EnableEvent(handle_ptr->ccu8_slice_ptr, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
  }
#endif
  /* Clears the timer register */
  XMC_CCU8_SLICE_ClearTimer(handle_ptr->ccu8_slice_ptr);

  /* update the initialization flag as true for particular instance*/
  handle_ptr->initialized = true;

  /* Check whether the start of the timer is enabled during initialization or not */
  if (handle_ptr->start_control == true)
  {
    /* Start the timer */
    XMC_CCU8_SLICE_StartTimer(handle_ptr->ccu8_slice_ptr);
  }

  return (status);
}

/*
 * This function configures timer period and compare values and triggers the shadow transfer operation
 */
void TIMER_CCU8_lShadowTransfer(TIMER_t* const handle_ptr)
{
  /* programs the timer period register according to time interval value */
  XMC_CCU8_SLICE_SetTimerPeriodMatch(handle_ptr->ccu8_slice_ptr, handle_ptr->period_value);
  /* programs the timer compare register for 50% duty cycle in compare channel 1*/
  XMC_CCU8_SLICE_SetTimerCompareMatch(handle_ptr->ccu8_slice_ptr,
                                      XMC_CCU8_SLICE_COMPARE_CHANNEL_1,
                                      TIMER_CMP_100_DUTY);
  /* Transfers value from shadow timer registers to actual timer registers */
  XMC_CCU8_EnableShadowTransfer(handle_ptr->global_ccu8_handler->module_ptr, handle_ptr->shadow_mask);
}
#endif


/*
 * This function is called to insert a timer into the timer list.
 */
static void SYSTIMER_lInsertTimerList(uint32_t tbl_index)
{
  SYSTIMER_OBJECT_t *object_ptr;
  int32_t delta_ticks;
  int32_t timer_count;
  bool found_flag = false;
   /* Get timer time */
  timer_count = (int32_t)g_timer_tbl[tbl_index].count;
  /* Check if Timer list is NULL */
  if (NULL == g_timer_list)
  {
    /* Set this as first Timer */
    g_timer_list = &g_timer_tbl[tbl_index];
  }
  /* If not, find the correct place, and insert the specified timer */
  else
  {
    object_ptr = g_timer_list;
    /* Get timer tick */
    delta_ticks = timer_count;
    /* Find correct place for inserting the timer */
    while ((NULL != object_ptr) && (false == found_flag))
    {
      /* Get timer Count Difference */
      delta_ticks -= (int32_t)object_ptr->count;
      /* Check for delta ticks < 0 */
      if (delta_ticks <= 0)
      {
        /* Check If head item */
        if (NULL != object_ptr->prev)
        {
          /* If Insert to list */
          object_ptr->prev->next = &g_timer_tbl[tbl_index];
          g_timer_tbl[tbl_index].prev = object_ptr->prev;
          g_timer_tbl[tbl_index].next = object_ptr;
          object_ptr->prev = &g_timer_tbl[tbl_index];
        }
        else
        {
          /* Set Timer as first item */
          g_timer_tbl[tbl_index].next = g_timer_list;
          g_timer_list->prev = &g_timer_tbl[tbl_index];
          g_timer_list = &g_timer_tbl[tbl_index];
        }
        g_timer_tbl[tbl_index].count = g_timer_tbl[tbl_index].next->count + (uint32_t)delta_ticks;
        g_timer_tbl[tbl_index].next->count  -= g_timer_tbl[tbl_index].count;
        found_flag = true;
      }
      /* Check for last item in list */
      else
      {
        if ((delta_ticks > 0) && (NULL == object_ptr->next))
        {
          /* Yes, insert into */
          g_timer_tbl[tbl_index].prev = object_ptr;
          object_ptr->next = &g_timer_tbl[tbl_index];
          g_timer_tbl[tbl_index].count = (uint32_t)delta_ticks;
          found_flag = true;
        }
      }
      /* Get the next item in timer list */
      object_ptr = object_ptr->next;
    }
  }
}

/*
 * This function is called to remove a timer from the timer list.
 */
static void SYSTIMER_lRemoveTimerList(uint32_t tbl_index)
{
  SYSTIMER_OBJECT_t *object_ptr;
  object_ptr = &g_timer_tbl[tbl_index];
  /* Check whether only one timer available */
  if ((NULL == object_ptr->prev) && (NULL == object_ptr->next ))
  {
    /* set timer list as NULL */
    g_timer_list = NULL;
  }
  /* Check if the first item in timer list */
  else if (NULL == object_ptr->prev)
  {
    /* Remove timer from list, and reset timer list */
    g_timer_list  = object_ptr->next;
    g_timer_list->prev = NULL;
    g_timer_list->count += object_ptr->count;
    object_ptr->next    = NULL;
  }
  /* Check if the last item in timer list */
  else if (NULL == object_ptr->next)
  {
    /* Remove timer from list */
    object_ptr->prev->next = NULL;
    object_ptr->prev = NULL;
  }
  else
  {
    /* Remove timer from list */
    object_ptr->prev->next  =  object_ptr->next;
    object_ptr->next->prev  =  object_ptr->prev;
    object_ptr->next->count += object_ptr->count;
    object_ptr->next = NULL;
    object_ptr->prev = NULL;
  }
}

/*
 * Handler function called from SysTick event handler.
 */
static void SYSTIMER_lTimerHandler(void)
{
  SYSTIMER_OBJECT_t *object_ptr;
  /* Get first item of timer list */
  object_ptr = g_timer_list;
  while ((NULL != object_ptr) && (0U == object_ptr->count))
  {
    if (true == object_ptr->delete_swtmr)
    {
      /* Yes, remove this timer from timer list */
      SYSTIMER_lRemoveTimerList((uint32_t)object_ptr->id);
      /* Set timer status as SYSTIMER_STATE_NOT_INITIALIZED */
      object_ptr->state = SYSTIMER_STATE_NOT_INITIALIZED;
      /* Release resource which are hold by this timer */
      g_timer_tracker &= ~(1U << object_ptr->id);
    }
    /* Check whether timer is a one shot timer */
    else if (SYSTIMER_MODE_ONE_SHOT == object_ptr->mode)
    {
      if (SYSTIMER_STATE_RUNNING == object_ptr->state)
      {
        /* Yes, remove this timer from timer list */
        SYSTIMER_lRemoveTimerList((uint32_t)object_ptr->id);
        /* Set timer status as SYSTIMER_STATE_STOPPED */
        object_ptr->state = SYSTIMER_STATE_STOPPED;
        /* Call timer callback function */
        (object_ptr->callback)(object_ptr->args);
      }
    }
    /* Check whether timer is periodic timer */
    else if (SYSTIMER_MODE_PERIODIC == object_ptr->mode)
    {
      if (SYSTIMER_STATE_RUNNING == object_ptr->state)
      {
        /* Yes, remove this timer from timer list */
        SYSTIMER_lRemoveTimerList((uint32_t)object_ptr->id);
        /* Reset timer tick */
        object_ptr->count = object_ptr->reload;
        /* Insert timer into timer list */
        SYSTIMER_lInsertTimerList((uint32_t)object_ptr->id);
        /* Call timer callback function */
        (object_ptr->callback)(object_ptr->args);
      }
    }
    else
    {
      break;
    }
    /* Get first item of timer list */
    object_ptr = g_timer_list;
  }
}

/*
 *  SysTick Event Handler.
 */
void SysTick_Handler(void)
{
  SYSTIMER_OBJECT_t *object_ptr;
  object_ptr = g_timer_list;
  g_systick_count++;

  if (NULL != object_ptr)
  {
    if (object_ptr->count > 1UL)
    {
      object_ptr->count--;
    }
    else
    {
      object_ptr->count = 0U;
      SYSTIMER_lTimerHandler();
    }
  }
}

/* --- End of File --- */

