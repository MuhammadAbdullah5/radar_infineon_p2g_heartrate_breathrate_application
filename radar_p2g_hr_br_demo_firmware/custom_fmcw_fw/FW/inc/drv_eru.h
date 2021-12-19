/**
 *
 *  drv_eru.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_ERU_H_
#define FW_INC_DRV_ERU_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
/* Allow to include the xmc_scu.h, if NMI is enabled */
#define EVENT_GENERATOR_NMI_SUPPORTED  (1U)

#include "xmc_eru.h"
#if (EVENT_GENERATOR_NMI_SUPPORTED == 1U)
#include "xmc_scu.h"
#endif

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/

#define triggerFrameISR IRQ_Hdlr_56

#define watchdog_isr IRQ_Hdlr_57

/******************************************************************************
   3. TYPES
*******************************************************************************/
/**
 *  @brief Initialization status.
 */
typedef enum EVENT_DETECTOR_STATUS
{
  EVENT_DETECTOR_STATUS_SUCCESS = 0U, /**< Status success */
  EVENT_DETECTOR_STATUS_FAILURE, /**< Status failure */
} EVENT_DETECTOR_STATUS_t;

/**
 *  @brief Initialization status.
 */
typedef enum EVENT_GENERATOR_STATUS
{
  EVENT_GENERATOR_STATUS_SUCCESS = 0U, /**< Status success */
  EVENT_GENERATOR_STATUS_FAILURE  /**< Status failure */
} EVENT_GENERATOR_STATUS_t;

typedef enum INTERRUPT_STATUS
{
  INTERRUPT_STATUS_SUCCESS = 0U,  /**< APP initialization success */
  INTERRUPT_STATUS_FAILURE = 1U   /**< APP initialization failure */
} INTERRUPT_STATUS_t;


/**
 * @brief This structure will hold the APP configuration parameters
 */
typedef struct EVENT_DETECTOR
{
  XMC_ERU_t* const eru; /**< ERU module assigned */
  const uint8_t channel; /**< ERU channel assigned(0-3) */
  const XMC_ERU_ETL_CONFIG_t *const config; /**< reference to hardware configuration */
  bool init_status; /**< This parameter indicates the initialized state of each instance */
} EVENT_DETECTOR_t;

/**
 * @brief This structure will hold the APP configuration parameters
 */
typedef struct EVENT_GENERATOR
{
  XMC_ERU_t* const eru; /**< ERU module assigned */
  const XMC_ERU_OGU_CONFIG_t *const config; /**< reference to hardware configuration */
#if (UC_FAMILY == XMC4)
  const uint32_t nmi_eru_msk; /**< Mask to enable the NMI feature */
#endif
  const uint8_t channel; /**< ERU channel assigned(0-3) */
  bool init_status; /**< This parameter indicates the initialized state of each instance */
} EVENT_GENERATOR_t;

/**
 * @brief This structure holds run-time configurations of INTERRUPT APP.
 */
typedef struct INTERRUPT
{
#if(UC_SERIES == XMC14)
  const XMC_SCU_IRQCTRL_t irqctrl;  /**< selects the interrupt source for a NVIC interrupt node*/
#endif
  const IRQn_Type node;       /**< Mapped NVIC Node */
  const uint8_t priority; 	  /**< Node Interrupt Priority */
#if(UC_FAMILY == XMC4)
  const uint8_t subpriority;  /**< Node Interrupt SubPriority only valid for XMC4x */
#endif
  const bool enable_at_init;  /**< Interrupt enable for Node */
} INTERRUPT_t;


extern EVENT_DETECTOR_t  EVENT_DETECTOR_PLL_TRIG1;
extern EVENT_GENERATOR_t EVENT_GENERATOR_CCU4;

extern const INTERRUPT_t INTERRUPT_FRAME;
extern const INTERRUPT_t INTERRUPT_WATCHDOG;

/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/

/**
 * @brief Initializes a EVENT_DETECTOR APP instance (ERSx+ETLx where x = [0..3])
 *        with generated configuration.
 * @param handle pointer to the EVENT_DETECTOR APP configuration.
 * @return EVENT_DETECTOR_STATUS_t
 *         EVENT_DETECTOR_STATUS_SUCCESS  : if initialization is successful\n
 *         EVENT_DETECTOR_STATUS_FAILURE  : if initialization is failed
 *
 * \par<b>Description:</b><br>
 * <ul>
 * <li>Enable the clock for the ERU module and invoke the LLD Init API with generated configuration handle.</li>
 * <li>This configures the input event selection with selected polarity settings</li>
 * <li>Trigger source by combining the input signals</li>
 * </ul>
 *
 * Example Usage:
 *
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  // ... Initializes Apps configurations ...
 *  DAVE_Init(); //EVENT_DETECTOR_Init(&EVENT_DETECTOR_0) will be called from DAVE_Init()
 *  while(1U)
 *  {
 *  }
 *  return (1U);
 * }
 *
 * @endcode<BR>
 */
EVENT_DETECTOR_STATUS_t EVENT_DETECTOR_Init(EVENT_DETECTOR_t *const handle);


/**
 * @brief  Set the ETLx(x= [0..3]) status flag.
 * @param  handle pointer to the EVENT_DETECTOR APP configuration.
 * @return None
 * <BR>
 *
 * \par<b>Description:</b><br>
 * Status flag is connected with OGU block, where the state is utilized for pattern match.
 *
 * Example Usage:
 *
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  // ... Initializes Apps configurations ...
 *  DAVE_Init(); //EVENT_DETECTOR_Init(&EVENT_DETECTOR_0) will be called from DAVE_Init()
 *
 *  //user logic
 *
 *  EVENT_DETECTOR_SetStatusFlag(&EVENT_DETECTOR_0);
 *  while(1U)
 *  {
 *  }
 *  return (1U);
 * }
 *
 * @endcode<BR>
 */
__STATIC_INLINE void EVENT_DETECTOR_SetStatusFlag(const EVENT_DETECTOR_t *const handle)
{
  /* Checking for null handle */
  XMC_ASSERT("EVENT_DETECTOR_SetStatusFlag:handle NULL" , (handle != NULL));
  XMC_ERU_ETL_SetStatusFlag(handle->eru, handle->channel);
}


/**
 * @brief  Clears the ETLx(x= [0..3]) status flag.
 * @param  handle pointer to the EVENT_DETECTOR APP configuration.
 * @return None
 * <BR>
 *
 * \par<b>Description:</b><br>
 * If auto clear of the status flag is not enabled, EVENT_DETECTOR_ClearStatusFlag() has to be invoked to clear the
 * flag. So that next event is considered as a new event.
 *
 * Example Usage:
 *
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  // ... Initializes Apps configurations ...
 *  DAVE_Init(); //EVENT_DETECTOR_Init(&EVENT_DETECTOR_0) will be called from DAVE_Init()
 *
 *  //user logic
 *
 *  EVENT_DETECTOR_ClearStatusFlag(&EVENT_DETECTOR_0);
 *  while(1U)
 *  {
 *  }
 *  return (1U);
 * }
 *
 * @endcode<BR>
 */
__STATIC_INLINE void EVENT_DETECTOR_ClearStatusFlag(const EVENT_DETECTOR_t *const handle)
{
  /* Checking for null handle */
  XMC_ASSERT("EVENT_DETECTOR_ClearStatusFlag:handle NULL" , (handle != NULL));
  XMC_ERU_ETL_ClearStatusFlag(handle->eru, handle->channel);
}


/**
 * @brief  Returns the ETLx(x= [0..3]) status flag state.
 * @param  handle pointer to the EVENT_DETECTOR APP configuration.
 * @return uint32_t returns flag bit status
 * <BR>
 *
 * \par<b>Description:</b><br>
 * EVENT_DETECTOR_GetStatusFlag() can be used to check the whether any selected edge event is detected or not.
 *
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  uint32_t event_status;
 *  // ... Initializes Apps configurations ...
 *  DAVE_Init(); //EVENT_DETECTOR_Init(&EVENT_DETECTOR_0) will be called from DAVE_Init()
 *
 *  //user logic
 *
 *  event_status = EVENT_DETECTOR_GetStatusFlag(&EVENT_DETECTOR_0);
 *  while(1U)
 *  {
 *  }
 *  return (1U);
 * }
 *
 * @endcode<BR>
 */
__STATIC_INLINE uint32_t EVENT_DETECTOR_GetStatusFlag(const EVENT_DETECTOR_t *const handle)
{
  /* Checking for null handle */
  XMC_ASSERT("EVENT_DETECTOR_GetStatusFlag:handle NULL" , (handle != NULL));
  return XMC_ERU_ETL_GetStatusFlag(handle->eru, handle->channel);
}

/**
 * @brief Initializes a EVENT_GENERATOR APP instance (ERSx+ETLx where x= [0..3])
 *        with generated configuration.
 * @param handle pointer to the EVENT_GENERATOR APP configuration.
 * @return EVENT_GENERATOR_STATUS_t\n
 *         EVENT_GENERATOR_STATUS_SUCCESS  : if initialization is successful\n
 *         EVENT_GENERATOR_STATUS_FAILURE  : if initialization is failed
 *
 * \par<b>Description:</b><br>
 * <ul>
 * <li>Enable the clock for the ERU module and invoke the LLD Init API with generated configuration handle.</li>
 * <li>Set the gating scheme for service request generation.</li>
 * </ul>
 *
 * Example Usage:
 *
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  // ... Initializes Apps configurations ...
 *  DAVE_Init(); //EVENT_GENERATOR_Init(&EVENT_GENERATOR_0) will be called from DAVE_Init()
 *  while(1U)
 *  {
 *  }
 *  return (1U);
 * }
 *
 * @endcode<BR>
 */
EVENT_GENERATOR_STATUS_t EVENT_GENERATOR_Init(EVENT_GENERATOR_t *const handle);


/**
 * @brief return the pattern detection result.
 * @param handle pointer to the EVENT_GENERATOR APP configuration.
 * @retval uint32_t pattern detection status
 *            <BR>
 *
 * \par<b>Description:</b><br>
 * EVENT_GENERATOR_GetPatternDetectionStatus() can be used to find out that, all the input signals configured for
 * the pattern match are set or not.
 *
 * Example Usage:
 *
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *  uint32_t pattern_status;
 *  // ... Initializes Apps configurations ...
 *  DAVE_Init(); //EVENT_GENERATOR_Init(&EVENT_GENERATOR_0) will be called from DAVE_Init()
 *
 *  pattern_status = EVENT_GENERATOR_GetPatternDetectionStatus(&EVENT_GENERATOR_0);
 *  while(1U)
 *  {
 *  }
 *  return (1U);
 * }
 *
 * @endcode
 */
__STATIC_INLINE uint32_t EVENT_GENERATOR_GetPatternDetectionStatus(const EVENT_GENERATOR_t *const handle)
{
  /* Checking for null handle */
  XMC_ASSERT("EVENT_GENERATOR_GetPatternDetectionStatus:handle NULL" , (handle != NULL));
  return XMC_ERU_OGU_GetPatternDetectionStatus(handle->eru, handle->channel);
}

/**
 * @brief Initializes INTERRUPT APP instance.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return @ref INTERRUPT_STATUS_t
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    DAVE_Init();  // INTERRUPT_Init(&INTERRUPT_0) is called within DAVE_Init()
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 *
 */
INTERRUPT_STATUS_t INTERRUPT_Init(const INTERRUPT_t *const handler);

/**
 * @brief Enables the IRQ.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return None
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    INTERRUPT_Enable(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE void INTERRUPT_Enable(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  NVIC_EnableIRQ(handler->node);
}

/**
 * @brief Disables the IRQ.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return None
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    INTERRUPT_Disable(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE void INTERRUPT_Disable(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  NVIC_DisableIRQ(handler->node);
}

/**
 * @brief Get the pending IRQ.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return uint32_t IRQ node
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    uint32_t Status;
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    Status = INTERRUPT_GetPending(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE uint32_t INTERRUPT_GetPending(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  return NVIC_GetPendingIRQ(handler->node);
}

/**
 * @brief Set the IRQ to pending state.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return None
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    INTERRUPT_SetPending(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE void INTERRUPT_SetPending(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  NVIC_SetPendingIRQ(handler->node);
}

/**
 * @brief Clears the pending status of the IRQ.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return None
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate two instances of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  uint32_t pend_IRQ;
 *  int main(void)
 *  {
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    INTERRUPT_Enable(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 *
 *  void MyISR_handler(void)
 *  {
 *    INTERRUPT_Enable(&INTERRUPT_1);
 *    INTERRUPT_SetPending(&INTERRUPT_1);
 *    pend_IRQ = INTERRUPT_GetPending(&INTERRUPT_1);
 *    if(pend_IRQ)
 *    {
 *      INTERRUPT_Disable(&INTERRUPT_0);
 *      INTERRUPT_ClearPending(&INTERRUPT_1);
 *    }
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE void INTERRUPT_ClearPending(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  NVIC_ClearPendingIRQ(handler->node);
}

#if(UC_FAMILY == XMC4)
/**
 * @brief Get current running active status of the IRQ. This API is applicable
 *        only for XMC4000 devices.
 * @param handle Constant pointer to constant structure of type @ref INTERRUPT_t
 * @return uint32_t current active running IRQ node
 * <BR><P ALIGN="LEFT"><B>Example:</B>
 * Pre-requisite: Instantiate one instance of INTERRUPT APP
 * @code
 *  #include <DAVE.h>
 *
 *  int main(void)
 *  {
 *    uint32_t Status;
 *    DAVE_Init(); // INTERRUPT_Init() is called within DAVE_Init()
 *    Status = INTERRUPT_GetActive(&INTERRUPT_0);
 *    while(1)
 *    {}
 *    return 0;
 *  }
 * @endcode<BR> </p>
 */
__STATIC_INLINE uint32_t INTERRUPT_GetActive(const INTERRUPT_t *const handler)
{
  XMC_ASSERT("Handler NULL", (handler != NULL));
  return NVIC_GetActive(handler->node);
}

#endif

/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_ERU_H_ */

/* --- End of File --- */
