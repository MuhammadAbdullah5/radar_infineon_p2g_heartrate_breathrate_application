/**
 *
 *  drv_eru.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */
 
/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/drv_eru.h"

/******************************************************************************
   2. DATA
*******************************************************************************/
/**
 * @brief Contents entered via GUI
 */
const XMC_ERU_ETL_CONFIG_t EVENT_DETECTOR_PLL_TRIG1_ETL_Config =
{
  .input_a = (uint32_t)XMC_ERU_ETL_INPUT_A0, /* Event input selection for A(0-3) */
  .input_b = (uint32_t)XMC_ERU_ETL_INPUT_B0, /* Event input selection for B(0-3) */
  .enable_output_trigger = (uint32_t)0,
  .status_flag_mode = (XMC_ERU_ETL_STATUS_FLAG_MODE_t)XMC_ERU_ETL_STATUS_FLAG_MODE_HWCTRL, /* enable the status flag
                                                                                                   auto clear for opposite edge */
  .edge_detection = XMC_ERU_ETL_EDGE_DETECTION_RISING, /* Select the edge/s to convert as event */
  .output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL0, /* Select the source for event */
  .source = XMC_ERU_ETL_SOURCE_A
};


EVENT_DETECTOR_t EVENT_DETECTOR_PLL_TRIG1 =
{
  .eru       = XMC_ERU1, /* ERU module assigned */
  .channel   = 2U,    /* ERU channel assigned(0-3) */
  .config      = &EVENT_DETECTOR_PLL_TRIG1_ETL_Config, /* reference to hardware configuration */
  .init_status = false /* Initialized status */
};

/**
 * @brief Contents entered via GUI
 */
const XMC_ERU_OGU_CONFIG_t EVENT_GENERATOR_CCU4_OGU_Config =
{
  .peripheral_trigger         = 0U, /* OGU input peripheral trigger */
  .enable_pattern_detection   = true, /* Enables generation of pattern match event */
  .service_request            = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER, /* Interrupt gating signal */
  .pattern_detection_input    = XMC_ERU_OGU_PATTERN_DETECTION_INPUT2
};


EVENT_GENERATOR_t EVENT_GENERATOR_CCU4 =
{
  .eru         = XMC_ERU1, /* ERU module assigned */
  .channel     = 0U,    /* ERU channel assigned(0-3) */
  .config      = &EVENT_GENERATOR_CCU4_OGU_Config, /* reference to hardware configuration */
  .nmi_eru_msk = 0U, /**< Mask to enable the NMI feature */
  .init_status = false /* Initialized status */
};

const INTERRUPT_t INTERRUPT_FRAME =
{
#if(UC_SERIES == XMC14)
 .irqctrl = (XMC_SCU_IRQCTRL_t)0U,
#endif
  .node = (IRQn_Type)56,
  .priority = 61,
#if(UC_FAMILY == XMC4)
  .subpriority = 0,
#endif

  .enable_at_init = true

};

const INTERRUPT_t INTERRUPT_WATCHDOG =
{
#if(UC_SERIES == XMC14)
 .irqctrl = (XMC_SCU_IRQCTRL_t)0U,
#endif
  .node = (IRQn_Type)57,
  .priority = 61,
#if(UC_FAMILY == XMC4)
  .subpriority = 0,
#endif

  .enable_at_init = true

};
/******************************************************************************
   3. LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/
/*
 * This function Initializes a EVENT_DETECTOR APP instances (ERSx+ETLx) based on user configuration.
 */
EVENT_DETECTOR_STATUS_t EVENT_DETECTOR_Init(EVENT_DETECTOR_t *const handle)
{
  /* Checking for null handle */
  XMC_ASSERT("EVENT_DETECTOR_Init:handle NULL" , (handle != NULL));

  /* Checking for initialization state of the instance */
  if (false == handle->init_status)
  {
    /* Hardware initialization based on UI */
    XMC_ERU_ETL_Init(handle->eru, handle->channel, handle->config);
    /* Update the init status of the instance */
    handle->init_status = true;
  }

  return EVENT_DETECTOR_STATUS_SUCCESS;
}

/*
 * This function Initializes a EVENT_GENERATOR APP instances based on user configuration.
 */
EVENT_GENERATOR_STATUS_t EVENT_GENERATOR_Init(EVENT_GENERATOR_t *const handle)
{
  /* Checking for null handle */
  XMC_ASSERT("EVENT_GENERATOR_Init:handle NULL" , (handle != NULL));

  /* Checking for initialization state of the instance */
  if (false == handle->init_status)
  {
    /* Hardware initialization based on UI */
    XMC_ERU_OGU_Init(handle->eru, handle->channel, handle->config);
    #if (EVENT_GENERATOR_NMI_SUPPORTED == 1U)
    /* Promote the eru event as NMI, Applicable only for XMC4000 devices */
    XMC_SCU_INTERRUPT_EnableNmiRequest(handle->nmi_eru_msk);
    #endif
    /* Update the init status of the instance */
    handle->init_status = true;
  }

  return EVENT_GENERATOR_STATUS_SUCCESS;
}

/*
 * API to initialize the INTERRUPT APP
 */
INTERRUPT_STATUS_t INTERRUPT_Init(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("INTERRUPT_Init:HandlePtr NULL", (handler != NULL));

#if(UC_FAMILY == XMC4)

  NVIC_SetPriority(handler->node,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                                       handler->priority,
                                       handler->subpriority));
  if (handler->enable_at_init == true)
  {
    INTERRUPT_Enable(handler);
  }
#endif

#if(UC_FAMILY == XMC1)
  NVIC_SetPriority(handler->node, handler->priority);

#if (UC_SERIES == XMC14)
  XMC_SCU_SetInterruptControl((uint8_t)handler->node, (XMC_SCU_IRQCTRL_t)((handler->node << 8) | handler->irqctrl));
#endif

  /* Enable the interrupt if enable_at_init is enabled */
  if (handler->enable_at_init == true)
  {
    INTERRUPT_Enable(handler);
  }
#endif

  return (INTERRUPT_STATUS_SUCCESS);
}

/******************************************************************************
   5. LOCAL FUNCTIONS
*******************************************************************************/



/* --- End of File --- */

