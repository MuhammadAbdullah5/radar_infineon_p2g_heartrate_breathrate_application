 /**
 *
 *  drv_adc.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_ADC_H_
#define FW_INC_DRV_ADC_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
#include <xmc_vadc.h>
#include <types.h>

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/

#define GLOBAL_ADC_HANDLE ((GLOBAL_ADC_t *)(void *) &GLOBAL_ADC_0) /**< Instance handle of the GLOBAL_ADC APP*/

#define GLOBAL_ADC_AREF_VALUE XMC_VADC_GLOBAL_SHS_AREF_EXTERNAL_VDD_UPPER_RANGE

#define GLOBAL_ADC_MAJOR_VERSION (4) /**< Major version number of GLOBAL_ADC APP*/
#define GLOBAL_ADC_MINOR_VERSION (0) /**< Minor version number of GLOBAL_ADC APP*/
#define GLOBAL_ADC_PATCH_VERSION (18) /**< Patch version number of GLOBAL_ADC APP*/

#define GLOBAL_ADC_NUM_INSTANCES (1U)


typedef enum GLOBAL_ADC_STATUS
{
  GLOBAL_ADC_SUCCESS = 0, /**< APP is in INITIALIZED state after execution of the Init function*/
  GLOBAL_ADC_FAILURE, 		 /**< Initialization failed returns this as status */
  GLOBAL_ADC_UNINITIALIZED	 /**< This is the default state after power on reset.*/
} GLOBAL_ADC_STATUS_t;

/**
 * @}
 */
/**********************************************************************************************************************
* DATA STRUCTURES
**********************************************************************************************************************/

#if XMC_VADC_GROUP_AVAILABLE == 1U
typedef struct GLOBAL_ADC_GROUP
{
  XMC_VADC_GROUP_t *const group_handle; 						/**<This holds the VADC group Registers. */

  const XMC_VADC_GROUP_CONFIG_t* const group_config_handle; /**<This is the pointer to the Handle of the Group APP. */

  const bool post_calibration;                              /**< This enables the post calibration for a specific group*/

  GLOBAL_ADC_STATUS_t state; 									/**<This enumerates the state of the APP. */
} GLOBAL_ADC_GROUP_t;
#endif

typedef struct GLOBAL_ADC
{
#if XMC_VADC_GROUP_AVAILABLE == 1U
  GLOBAL_ADC_GROUP_t* const group_ptrs_array[XMC_VADC_MAXIMUM_NUM_GROUPS]; /**<This is an array of pointers to the ADC Groups*/
#endif
  const XMC_VADC_GLOBAL_CONFIG_t* const global_config_handle; 	   /**<This is the pointer to the Global LLD Handle.*/

  XMC_VADC_GLOBAL_t* const module_ptr; /**<This is the register structure pointer to the VADC kernel. */

#if(XMC_VADC_SHS_AVAILABLE == 1U)
  XMC_VADC_GLOBAL_SHS_t* const global_shs_ptr; /**< This is the sample and hold structure pointer*/
  XMC_VADC_GLOBAL_SHS_CONFIG_t* const global_shscfg; /**< This is the sample and hold structure pointer*/
#endif
  GLOBAL_ADC_STATUS_t init_state; 		 /**< This hold the State of the GLOBAL_ADC APP*/

  const bool enable_startup_calibration;       /**< Enable startup calibration for all the converters*/
} GLOBAL_ADC_t;


/* If a queue request source is used by any of the ADC_MEASUREMENT_ADV APP
 * Instances this macro is generated*/
#define ADC_MEASUREMENT_ADV_QUEUE_USED (1U)

/* If a scan request source is used by any of the ADC_MEASUREMENT_ADV APP
 * Instances this macro is generated*/
#define ADC_MEASUREMENT_ADV_SCAN_USED (1U)

/* If synchronous conversion is used by any of the ADC_MEASUREMENT_ADV APP
 * Instances this macro is generated*/
#define ADC_MEASUREMENT_ADV_SYNC_USED (1U)

/* If synchronous conversion is not used by all the ADC_MEASUREMENT_ADV APP
 * Instances this macro is generated. Needed for optimization of the code.*/
#define ADC_MEASUREMENT_ADV_SYNC_NOT_ALL_USED (1U)

/* If event configuration is used by any of the ADC_MEASUREMENT_ADV APP
 * Instances this macro is generated.*/
#define ADC_MEASUREMENT_ADV_MUX_USED (1U)

/* If event configuration is not used by all the ADC_MEASUREMENT_ADV APP
 * Instances this macro is generated. Needed for optimization of the code.*/
#define ADC_MEASUREMENT_ADV_MUX_NOT_ALL_USED (1U)

/* If FIFO is not used by any of the ADC_MEASUREMENT_ADV APP
 * Instances this macro is generated. The Result register configuration holds only one config structure.*/
#define ADC_MEASUREMENT_ADV_RESULT_REG (1U)

/* If Internal consumption of scan request source takes place in any of the ADC_MEASUREMENT_ADV APP
 * Instances this macro is generated.*/
#define ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED (1U)

/* If Internal consumption of queue request source takes place in any of the ADC_MEASUREMENT_ADV APP
 * Instances this macro is generated.*/
#define ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED (1U)

/* If Start after initialization is used by any of the ADC_MEASUREMENT_ADV APP
 * Instances this macro is generated.*/
#define ADC_MEASUREMENT_ADV_START_ADC (1U)


 /* Channel and result related macros for the instance ADC_MEASUREMENT_ADV_G1*/
#define ADC_MEASUREMENT_ADV_G1_GROUP_PTR  ((XMC_VADC_GROUP_t*)(void*) VADC_G0)

#define ADC_MEASUREMENT_ADV_G1_IF1_I  (ADC_MEASUREMENT_ADV_G1_IF1_I_handle)
#define ADC_MEASUREMENT_ADV_G1_IF1_I_RES  (VADC_G0->RES[15])

#ifndef VADC_QUEUE_GROUP_0_POSITION_0
#define VADC_QUEUE_GROUP_0_POSITION_0 (0U)
#else
#error "Redefined the same Queue position. Please change the queue position to a different value in the UI editor of the APP"
#endif

 /* Channel and result related macros for the instance ADC_MEASUREMENT_ADV_G2*/
#define ADC_MEASUREMENT_ADV_G2_GROUP_PTR  ((XMC_VADC_GROUP_t*)(void*) VADC_G1)

#define ADC_MEASUREMENT_ADV_G2_IF1_Q  (ADC_MEASUREMENT_ADV_G2_IF1_Q_handle)
#define ADC_MEASUREMENT_ADV_G2_IF1_Q_RES  (VADC_G1->RES[3])

#ifndef VADC_QUEUE_GROUP_1_POSITION_0
#define VADC_QUEUE_GROUP_1_POSITION_0 (0U)
#else
#error "Redefined the same Queue position. Please change the queue position to a different value in the UI editor of the APP"
#endif

 /* Channel and result related macros for the instance ADC_MEASUREMENT_ADV_G3*/
#define ADC_MEASUREMENT_ADV_G3_GROUP_PTR  ((XMC_VADC_GROUP_t*)(void*) VADC_G2)

#define ADC_MEASUREMENT_ADV_G3_IF2_I  (ADC_MEASUREMENT_ADV_G3_IF2_I_handle)
#define ADC_MEASUREMENT_ADV_G3_IF2_I_RES  (VADC_G2->RES[0])

#ifndef VADC_QUEUE_GROUP_2_POSITION_0
#define VADC_QUEUE_GROUP_2_POSITION_0 (0U)
#else
#error "Redefined the same Queue position. Please change the queue position to a different value in the UI editor of the APP"
#endif

 /* Channel and result related macros for the instance ADC_MEASUREMENT_ADV_G4*/
#define ADC_MEASUREMENT_ADV_G4_GROUP_PTR  ((XMC_VADC_GROUP_t*)(void*) VADC_G3)

#define ADC_MEASUREMENT_ADV_G4_IF2_Q  (ADC_MEASUREMENT_ADV_G4_IF2_Q_handle)
#define ADC_MEASUREMENT_ADV_G4_IF2_Q_RES  (VADC_G3->RES[4])

#ifndef VADC_QUEUE_GROUP_3_POSITION_0
#define VADC_QUEUE_GROUP_3_POSITION_0 (0U)
#else
#error "Redefined the same Queue position. Please change the queue position to a different value in the UI editor of the APP"
#endif

 /* Channel and result related macros for the instance ADC_MEASUREMENT_SCAN*/
#define ADC_MEASUREMENT_SCAN_GROUP_PTR  ((XMC_VADC_GROUP_t*)(void*) VADC_G3)

#define ADC_MEASUREMENT_SCAN_BGT24_ANA  (ADC_MEASUREMENT_SCAN_BGT24_ANA_handle)
#define ADC_MEASUREMENT_SCAN_BGT24_ANA_RES  (VADC_G3->RES[9])

#define ADC_MEASUREMENT_ADV_GLOBAL_HANDLE ((GLOBAL_ADC_t *)(void *) &GLOBAL_ADC_0)


#define ADC_MEASUREMENT_ADV_MAJOR_VERSION (4U) /**< Major version number of ADC_MEASUREMENT_ADV APP*/
#define ADC_MEASUREMENT_ADV_MINOR_VERSION (0U) /**< Minor version number of ADC_MEASUREMENT_ADV APP*/
#define ADC_MEASUREMENT_ADV_PATCH_VERSION (14U) /**< Patch version number of ADC_MEASUREMENT_ADV APP*/


typedef enum ADC_MEASUREMENT_ADV_STATUS
{
  ADC_MEASUREMENT_ADV_STATUS_SUCCESS = 0,  /**< The API call is successful*/
  ADC_MEASUREMENT_ADV_STATUS_FAILURE,      /**< The API call is failed*/
  ADC_MEASUREMENT_ADV_STATUS_UNINITIALIZED /**< APP has not been Initialized */
} ADC_MEASUREMENT_ADV_STATUS_t;


/**
 * @brief The selected Request source.
 */
typedef enum ADC_MEASUREMENT_ADV_REQUEST_SOURCE
{
  ADC_MEASUREMENT_ADV_REQUEST_SOURCE_SCAN = 0, /**< Uses the ADC_SCAN APP's, scan request source.*/
  ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_SCAN, /**< Uses the internally consumed Scan request source.*/
  ADC_MEASUREMENT_ADV_REQUEST_SOURCE_QUEUE,    /**< Uses the ADC_QUEUE APP's, queue request source.*/
  ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_QUEUE,    /**< Uses the internally consumed Queue request source.*/
} ADC_MEASUREMENT_ADV_REQUEST_SOURCE_t;

/**
 * @brief The gain applied on the input signal.
 */
typedef enum ADC_MEASUREMENT_ADV_GAIN
{
  ADC_MEASUREMENT_ADV_GAIN_1 = 0U,   /**< The gain value of 1:1 is selected*/
  ADC_MEASUREMENT_ADV_GAIN_3 = 1U,   /**< The gain value of 1:3 is selected*/
  ADC_MEASUREMENT_ADV_GAIN_6 = 2U,   /**< The gain value of 1:6 is selected*/
  ADC_MEASUREMENT_ADV_GAIN_12 = 3U   /**< The gain value of 1:12 is selected*/
} ADC_MEASUREMENT_ADV_GAIN_t;

/**
 * @brief Alignment options for the subtraction value
 */
typedef enum ADC_MEASUREMENT_ADV_SUBTRATION
{
  ADC_MEASUREMENT_ADV_SUBTRATION_12BIT_LEFT_ALIGN  = 0U,     /**< Always align result to left */
  ADC_MEASUREMENT_ADV_SUBTRATION_12BIT_RIGHT_ALIGN = 0U,     /**< Always align result to right */
  ADC_MEASUREMENT_ADV_SUBTRATION_10BIT_LEFT_ALIGN  = 2U,     /**< Always align result to left */
  ADC_MEASUREMENT_ADV_SUBTRATION_10BIT_RIGHT_ALIGN = 0U,     /**< Always align result to right */
  ADC_MEASUREMENT_ADV_SUBTRATION_8BIT_LEFT_ALIGN   = 4U,     /**< Always align result to left */
  ADC_MEASUREMENT_ADV_SUBTRATION_8BIT_RIGHT_ALIGN  = 0U      /**< Always align result to right */
} ADC_MEASUREMENT_ADV_SUBTRATION_t;

/**
 * @brief The result of the fast compare operation.
 */
typedef enum ADC_MEASUREMENT_ADV_FAST_COMPARE
{
  ADC_MEASUREMENT_ADV_FAST_COMPARE_LOW     = 0U, /**< The result of fast conversion is low.*/
  ADC_MEASUREMENT_ADV_FAST_COMPARE_HIGH    = 1U, /**< The result of fast conversion is high.*/
  ADC_MEASUREMENT_ADV_FAST_COMPARE_INVALID = 2U  /**< The result is invalid since no new results are available.*/
} ADC_MEASUREMENT_ADV_FAST_COMPARE_t;

/**
 * @brief The result of the fast compare operation.
 */
typedef enum ADC_MEASUREMENT_ADV_SYNC_SEQ
{
  ADC_MEASUREMENT_ADV_SYNC_SEQ_POWER_DOWN = 0U, /**< The result of fast conversion is low.*/
  ADC_MEASUREMENT_ADV_SYNC_SEQ_STSEL_CONFIG, /**< The result of fast conversion is low.*/
  ADC_MEASUREMENT_ADV_SYNC_SEQ_EVAL_CONFIG, /**< The result of fast conversion is low.*/
} ADC_MEASUREMENT_ADV_SYNC_SEQ_t;


#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__TASKING__)
  #pragma warning 586
#endif

/******************************************************************************
   3. TYPES
*******************************************************************************/
typedef void (*ADC_MEASUREMENT_ADV_EVENT_CONFIG_t)(void); /**< Function pointer to the mux configuration*/

/**
 * @brief NVIC Configuration structure for request source interrupt.
 */
typedef struct ADC_MEASUREMENT_ADV_NVIC_CONFIG
{
  uint32_t node_id;    /**< This indicates the NVIC Node number.*/

  uint32_t priority;   /**< This indicates the NVIC priority.*/
#if(UC_FAMILY == XMC4)
  uint32_t sub_priority; /**< This indicates the NVIC sub priority in XMC4x Devices.*/
#endif
  bool interrupt_enable; /**< This flag indicates if a Interrupt has been requested.*/
#ifdef ADC_MEASUREMENT_ADV_NON_DEFAULT_IRQ_SOURCE_SELECTED
  uint8_t irqctrl;       /**< This indicates the service request source selected for the consumed NVIC node.*/
#endif
} ADC_MEASUREMENT_ADV_NVIC_CONFIG_t;

#ifdef ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED
/**
 * @brief Configuration Data structure of scan request source.
 */
typedef struct ADC_MEASUREMENT_ADV_SCAN
{
  const ADC_MEASUREMENT_ADV_NVIC_CONFIG_t rs_intr_handle;    /**< Holds the ISR Handle*/

  const XMC_VADC_GROUP_CLASS_t iclass_config_handle;         /**< Holds the ICLASS Configurations*/

  const XMC_VADC_SCAN_CONFIG_t *const scan_config_handle;    /**< Holds the LLD SCAN Structure*/

  const XMC_VADC_GATEMODE_t gating_mode;         /**< Gating mode configuration needed for Scan request source*/

  const XMC_VADC_SR_t srv_req_node;              /**< Source event interrupt node pointer*/

  const uint32_t  insert_mask;                  /**< Insert Mask for the scan request source*/

  const uint8_t iclass_num;                      /**< Holds the ICLASS ID either ICLASS-0 or ICLASS-1*/

} ADC_MEASUREMENT_ADV_SCAN_t;
#endif

#ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
/**
 * @brief Configuration Data structure of queue request source.
 */
typedef struct ADC_MEASUREMENT_ADV_QUEUE
{
  const ADC_MEASUREMENT_ADV_NVIC_CONFIG_t rs_intr_handle;    /**< Holds the ISR Handle*/

  const XMC_VADC_GROUP_CLASS_t iclass_config_handle;         /**< Holds the ICLASS Configurations*/

  const XMC_VADC_QUEUE_CONFIG_t *const queue_config_handle;    /**< Holds the LLD QUEUE Structure*/

  const XMC_VADC_GATEMODE_t gating_mode;         /**< Gating mode configuration needed for Scan request source*/

  const XMC_VADC_SR_t srv_req_node;              /**< Source event interrupt node pointer*/

  const uint8_t iclass_num;                      /**< Holds the ICLASS ID either ICLASS-0 or ICLASS-1*/

} ADC_MEASUREMENT_ADV_QUEUE_t;
#endif


/**
 * Structure to configure the channels in the ADC_MEASUREMENT_ADV APP.
 */
typedef struct ADC_MEASUREMENT_ADV_CHANNEL
{
  XMC_VADC_CHANNEL_CONFIG_t *ch_handle; /**< This holds the VADC Channel LLD structures*/

  XMC_VADC_RESULT_CONFIG_t *res_handle[ADC_MEASUREMENT_ADV_RESULT_REG]; /**< This hold the VADC LLD Result
                                                                              configuration structures*/

#ifdef ADC_MEASUREMENT_ADV_ANALOG_IO_USED
  ANALOG_IO_t   *analog_io_config;    /**< This hold the address of the ANALOG_IO configuration structure*/
#endif

#ifdef ADC_MEASUREMENT_ADV_FIFO_USED
  uint8_t max_fifo_required;             /**< The required number of FIFO elements*/

  uint8_t result_fifo_tail_number;      /**< The tail result register number if FIFO is selected. */
#endif
  uint8_t group_index;          /**< This holds the group index*/

  uint8_t ch_num;             /**< This Holds the Channel Number*/

#if(UC_FAMILY == XMC1)
  ADC_MEASUREMENT_ADV_GAIN_t shs_gain_factor; /**< The required gain factor for the channel.*/
#endif

} ADC_MEASUREMENT_ADV_CHANNEL_t;

/**
 * Structure to configure ADC_MEASUREMENT_ADV APP.
 */
typedef struct ADC_MEASUREMENT_ADV
{
  const ADC_MEASUREMENT_ADV_CHANNEL_t **const channel_array;      /**< This holds an array of channels configured
                                                              by the current instance of the ADC_MEASUREMENT_ADV APP*/
#if defined(ADC_MEASUREMENT_ADV_ADC_SCAN_USED) || defined(ADC_MEASUREMENT_ADV_ADC_QUEUE_USED) || \
    defined(ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED)
  union
  {
#ifdef ADC_MEASUREMENT_ADV_ADC_SCAN_USED
  const ADC_SCAN_ENTRY_t **const scan_entries;   /**< Holds the pointer to the scan entries. */
#endif
#ifdef ADC_MEASUREMENT_ADV_ADC_QUEUE_USED
  const ADC_QUEUE_ENTRY_t **const queue_entries;  /**< Holds the pointer to the queue entries.*/
#endif
#ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
  const XMC_VADC_QUEUE_ENTRY_t **const local_queue_entries;  /**< Holds the pointer to the queue entries.*/
#endif
  };
#endif

  ADC_MEASUREMENT_ADV_EVENT_CONFIG_t event_config; /**< This hold the pointer to the function
                                                      that does mux configuration. Which entails channel node and
                                                      result node configuration*/
  union
  {
#ifdef ADC_MEASUREMENT_ADV_ADC_SCAN_USED
    ADC_SCAN_t *const scan_handle;     /**< Pointer to the ADC_SCAN APP handle*/
#endif
#ifdef ADC_MEASUREMENT_ADV_ADC_QUEUE_USED
    ADC_QUEUE_t *const queue_handle;   /**< Pointer to the ADC_QUEUE APP handle*/
#endif
#ifdef ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED
    ADC_MEASUREMENT_ADV_SCAN_t *const local_scan_handle;   /**< Pointer to the scan handle*/
#endif
#ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
    ADC_MEASUREMENT_ADV_QUEUE_t *const local_queue_handle;   /**< Pointer to the queue handle*/
#endif
  };

  ADC_MEASUREMENT_ADV_STATUS_t *init_state;   /**< This enumeration gives information about the status of the APP*/

  ADC_MEASUREMENT_ADV_REQUEST_SOURCE_t req_src; /**< The request source used by this instance of
                                                      the ADC_MEASUREMENT_ADV APP */

#ifdef ADC_MEASUREMENT_ADV_SYNC_USED
  union
  {
    struct
    {
      uint8_t sync_slave_g0 :1; /**< If set the group-0 will be configured as the slave group.*/
      uint8_t sync_slave_g1 :1; /**< If set the group-1 will be configured as the slave group*/
      uint8_t sync_slave_g2 :1; /**< If set the group-2 will be configured as the slave group*/
      uint8_t sync_slave_g3 :1; /**< If set the group-3 will be configured as the slave group*/
      uint8_t               :4;
    };
    uint8_t sync_slaves;
  };

#endif
  const uint8_t group_index; /**< The group index number for the APP*/

  const uint8_t total_number_of_entries; /**< Indicates the total number of entries configured in
                                              the current APP instance*/

  const uint8_t total_number_of_channels; /**< Indicates the total number of channels configured in
                                               the current APP instance*/

  const bool start_at_initialization; /**< This determines if the insertion of the queue or
                                            scan entries should happen after initialization of the APP */
#ifdef ADC_MEASUREMENT_ADV_SYNC_USED
  const bool configure_globiclass1;  /**< Copy the master channels conversion parameters to the global iclass 1.
                                          Hence the slave channels are using the same features as that of the master.*/
#endif
} ADC_MEASUREMENT_ADV_t;

/**
 * @}
 */

/*Anonymous structure/union guard end*/
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__TASKING__)
  #pragma warning restore
#endif

extern GLOBAL_ADC_t GLOBAL_ADC_0;

extern const ADC_MEASUREMENT_ADV_t ADC_MEASUREMENT_ADV_G1;

extern const ADC_MEASUREMENT_ADV_CHANNEL_t  ADC_MEASUREMENT_ADV_G1_IF1_I_handle;
extern const ADC_MEASUREMENT_ADV_t ADC_MEASUREMENT_ADV_G2;

extern const ADC_MEASUREMENT_ADV_CHANNEL_t  ADC_MEASUREMENT_ADV_G2_IF1_Q_handle;
extern const ADC_MEASUREMENT_ADV_t ADC_MEASUREMENT_ADV_G3;

extern const ADC_MEASUREMENT_ADV_CHANNEL_t  ADC_MEASUREMENT_ADV_G3_IF2_I_handle;
extern const ADC_MEASUREMENT_ADV_t ADC_MEASUREMENT_ADV_G4;

extern const ADC_MEASUREMENT_ADV_CHANNEL_t  ADC_MEASUREMENT_ADV_G4_IF2_Q_handle;
extern const ADC_MEASUREMENT_ADV_t ADC_MEASUREMENT_SCAN;

extern const ADC_MEASUREMENT_ADV_CHANNEL_t  ADC_MEASUREMENT_SCAN_BGT24_ANA_handle;

/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/

/**
 * @brief Initializes the ADC global as per user configured values.
 * @return void
 * <BR>
 *
  \par<b>Description:</b><br>
 * Initializes the VADC peripheral.
 * Invokes various VADC LLD drivers to initialize the VADC peripheral. This would invoke
 * The XMC_VADC_GLOBAL_Init(), XMC_VADC_GROUP_Init(). It also invokes XMC_VADC_GROUP_SetPowerMode() to power
 * on available groups.
 *
 * Example Usage:
 * @code
 *  #include <DAVE.h>
 *  int main (void)
 *  {
 *    DAVE_Init(); //GLOBAL_ADC_Init is called within DAVE_Init
 *    while(1);
 *    return 0;
 *  }
 * @endcode
 */
GLOBAL_ADC_STATUS_t GLOBAL_ADC_Init(GLOBAL_ADC_t *const handle_ptr);


void ADC_MEASUREMENT_ADV_StartADC(const ADC_MEASUREMENT_ADV_t *const handle_ptr);

/**
 * @brief Starts the conversion of the required channels
 * @param handle_ptr constant pointer to the APP handle structure<BR>
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Trigger a load event for the required channels thus starting the conversion of the ADC channels.
 * If scan request source is selected then this API would write to GxASMR.LDEV bit, causing the conversion to start.
 * If queue request source is selected then this API would write to GxQMR0.TREV bit. At the time of writing to TREV bit
 * for the queue. If there was queue entry waiting in the queue buffer for a hardware trigger, writing to TREV bit
 * triggers the conversion for that entry.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 2.
// Goto interrupts tab enable request source interrupt.
// Goto the ADC_SCAN/ADC_QUEUE APP and enable the request source interrupt.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

uint16_t result[2];
void adc_measurement_adv_callback(void)
{
  result[0] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  result[1] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_B);
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
    // Continuously re-trigger the scan conversion sequence
    ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);
  }
}
 * @endcode
 */
void ADC_MEASUREMENT_ADV_SoftwareTrigger(const ADC_MEASUREMENT_ADV_t *const handle_ptr);

/**
 * @brief Initializes the ADC_MEASUREMENT_ADV APP Instance
 * @param handle_ptr constant pointer to the APP handle structure<BR>
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Initialize the ADC and all the required configurations. A call to this API would initialize the queue request source
 * or the scan request source depending on the GUI selection. The initialization is taken up by calling ADC_QUEUE_Init()
 * or ADC_SCAN_Init(). Following this the synchronous conversion related initializations are taken up. In the sync
 * initialization the slave groups and the master group are powered down and either GxSYNCTR.STSEL (if Slave) or
 * GxSYNCTR.EVALRy (if master) are configured. After the sync related configurations are completed the master group
 * alone is powered on. Following this the result event or channel event related service request node configurations
 * are done (if required). Then the GxCHCTR configurations are completed. After the channel initialization the
 * result handling initializations are done. This entails configuring the GxRCR registers for result filtering,
 * accumulation, subtraction and FIFO. After all these initialization are completed the channels configured
 * in the GUI is inserted into the appropriate ADC_QUEUE or ADC_SCAN APP buffer. If the GUI check box
 * "Insert channels at initialization" is enable then these entries is pushed to the Hardware.
 *
 * Example Usage:
 *
 * @code
 * #include <DAVE.h>
 * int main(void)
 * {
 *    DAVE_Init(); //ADC_MEASUREMENT_ADV_Init is called within DAVE_Init
 *    return 0;
 * }
 * @endcode
 */
ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_Init(const ADC_MEASUREMENT_ADV_t *const handle_ptr);

/**
 * @brief Returns the conversion result
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @return uint16_t conversion result.<BR>
 *                                Range: [ 0x0 to 0xFFF] without any filters/ accumulation/ subtraction enabled.
 *
 * \par<b>Description: </b><br>
 * Return the converted result stored in the result register [GxRESy.RESULT].
 * In the APP each channel is configured to a particular group result register (excluding FIFO). The result register
 * is defined in the channel handle structure @ref ADC_MEASUREMENT_ADV_CHANNEL_t. Hence this API shall call be called
 * with a pointer to the channel handle of type  @ref ADC_MEASUREMENT_ADV_CHANNEL_t (Directly use the channel handle
 * related macros which are defined in adc_measure_adv_conf.h).
 *
 * \par<b>Note: </b><br>
 * This API is not Applicable for reading the result from the result FIFO registers
 * Use @ref ADC_MEASUREMENT_ADV_GetFifoResult in order to read the FIFO result.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 2.
// Select the request source APP from ADC_SCAN to ADC_QUEUE.
// Goto the sequence plan and select Channel_A at position-0 and Channel_B at position-1.
// Enable Wait for trigger for the Channel_A.
// Goto interrupts tab enable request source interrupt.
// Goto the ADC_QUEUE APP and enable the request source interrupt.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

uint16_t result[2];
void adc_measurement_adv_callback(void)
{
  // Use the channel handle parameter in this format "<APP Name>_<CHANNEL Name>"
  result[0] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  result[1] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_B);
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  // Start the queue conversion sequence
  ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
  }
}
 * @endcode
 */
__STATIC_INLINE uint16_t ADC_MEASUREMENT_ADV_GetResult(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr)
{
  uint16_t result;
  extern XMC_VADC_GROUP_t *const group_ptrs[XMC_VADC_MAXIMUM_NUM_GROUPS];
  XMC_ASSERT("ADC_MEASUREMENT_ADV_GetResult:Invalid handle_ptr", (handle_ptr != NULL))

  result = XMC_VADC_GROUP_GetResult(group_ptrs[handle_ptr->group_index],
                                    (uint32_t) handle_ptr->ch_handle->result_reg_number);
  return(result);
}

/**
 * @brief Returns the complete conversion result
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @return XMC_VADC_DETAILED_RESULT_t returns the complete result register
 *
 * \par<b>Description: </b><br>
 * Return the completely 32 bit result register (GxRESy).
 * In the APP each channel is configured to a particular group result register (excluding FIFO).
 * The result of conversion as well as other information is returned from this API. The detailed result contains
 * result of the most recent conversion, the channel number requested the conversion, valid flag, converted request
 * source, fast compare result, the result data reduction counter and the EMUX channel number (if GxRES[0] only).
 * In polling mechanism the converted result can be read out after checking the valid flag bit.
 * The result register is defined in the channel handle structure @ref ADC_MEASUREMENT_ADV_CHANNEL_t.
 * Hence this API shall call be called with a pointer to the channel handle of type  @ref ADC_MEASUREMENT_ADV_CHANNEL_t
 * (Directly use the channel handle related macros which are defined in adc_measure_adv_conf.h).
 *
 * \par<b>Note: </b><br>
 * This API is not Applicable for reading the result from the result FIFO registers
 * Use @ref ADC_MEASUREMENT_ADV_GetFifoDetailedResult in order to read the FIFO result.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 2.
// Select the request source APP from ADC_SCAN to ADC_QUEUE.
// Goto the sequence plan and select Channel_A at position-0 and Channel_B at position-1.
// Enable Wait for trigger for the Channel_A.
// Goto interrupts tab enable request source interrupt.
// Goto the ADC_QUEUE APP and enable the request source interrupt.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)
#define QUEUE_SRC (0U)

XMC_VADC_DETAILED_RESULT_t result[2];
uint32_t queue_flag = 0U;

void adc_measurement_adv_callback(void)
{
  // Use the channel handle parameter in this format "<APP Name>_<CHANNEL Name>"
  result[0] = ADC_MEASUREMENT_ADV_GetDetailedResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  result[1] = ADC_MEASUREMENT_ADV_GetDetailedResult(&ADC_MEASUREMENT_ADV_0_Channel_B);

  if((result[0].converted_request_source == QUEUE_SRC) && (QUEUE_SRC == result[1].converted_request_source))
  {
    queue_flag++;
  }

}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

    // Start the queue conversion sequence
  ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);

  while(queue_flag != 0U);
  // do something

  while(1U)
  {
  }
}
 * @endcode
 */
__STATIC_INLINE XMC_VADC_DETAILED_RESULT_t ADC_MEASUREMENT_ADV_GetDetailedResult(const
                                                                       ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr)
{
  XMC_VADC_DETAILED_RESULT_t result_register;
  extern XMC_VADC_GROUP_t *const group_ptrs[XMC_VADC_MAXIMUM_NUM_GROUPS];
  XMC_ASSERT("ADC_MEASUREMENT_ADV_GetDetailedResult:Invalid handle_ptr", (handle_ptr != NULL))

  result_register.res = XMC_VADC_GROUP_GetDetailedResult(group_ptrs[handle_ptr->group_index],
                                                         (uint32_t) handle_ptr->ch_handle->result_reg_number);
  return(result_register);
}
#ifdef ADC_MEASUREMENT_ADV_FIFO_USED
/**
 * @brief Returns the conversion result
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @return uint16_t  conversion result.<BR>
 *                                Range: [ 0x0 to 0xFFF] without any filters/ accumulation/ subtraction enabled.
 *
 * \par<b>Description: </b><br>
 * Returns the converted result stored in the result FIFO register [GxRESy.RESULT].
 * If result FIFO is configured then the results are available in the FIFO tail register. The result register can only
 * be read at the tail of the FIFO, This result register number is defined in the channel handle structure
 * @ref ADC_MEASUREMENT_ADV_CHANNEL_t. Hence this API shall call be called with a pointer to the channel handle
 * of type  @ref ADC_MEASUREMENT_ADV_CHANNEL_t (Directly use the channel handle related macros which are
 * defined in adc_measure_adv_conf.h).
 *
 * \par<b>Note: </b><br>
 * This API is not Applicable for reading the result from channels which done use FIFO.
 * Use @ref ADC_MEASUREMENT_ADV_GetResult in order to read from a single result register.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 2.
// Select the request source APP from ADC_SCAN to ADC_QUEUE.
// Goto the sequence plan and select Channel_A at position-0 and Channel_B at position-1.
// Enable Wait for trigger for the Channel_A.
// Goto postprocessing Tab and select 8 stages FIFO for both Channel_A and Channel_B
// Goto interrupts tab enable request source interrupt.
// Goto the ADC_QUEUE APP and enable the request source interrupt.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#define MAX_FIFO (16U)
uint16_t result[16];
uint16_t i;
void adc_measurement_adv_callback(void)
{
  // Use the channel handle parameter in this format "<APP Name>_<CHANNEL Name>"
  result[i++] = ADC_MEASUREMENT_ADV_GetFifoResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  result[i++] = ADC_MEASUREMENT_ADV_GetFifoResult(&ADC_MEASUREMENT_ADV_0_Channel_B);

  if(MAX_FIFO == i)
  {
    i = (uint32_t)0;
  }
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

    // Start the queue conversion sequence
  ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
  }
}

 * @endcode
 */
__STATIC_INLINE uint16_t ADC_MEASUREMENT_ADV_GetFifoResult(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr)
{
  uint16_t result;
  extern XMC_VADC_GROUP_t *const group_ptrs[XMC_VADC_MAXIMUM_NUM_GROUPS];
  XMC_ASSERT("ADC_MEASUREMENT_ADV_GetFifoResult:Invalid handle_ptr", (handle_ptr != NULL))

  result = XMC_VADC_GROUP_GetResult(group_ptrs[handle_ptr->group_index], handle_ptr->result_fifo_tail_number);
  return(result);
}
/**
 * @brief Returns the complete conversion result
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @return XMC_VADC_DETAILED_RESULT_t returns the complete result register
 *
 * \par<b>Description: </b><br>
 * Return the completely 32 bit FIFO result register (GxRESy).
 * If result FIFO is configured then the results are available in the FIFO tail register.
 * The result of conversion as well as other information is returned from this API. The detailed result contains
 * result of the most recent conversion, the channel number requested the conversion, valid flag, converted request
 * source, fast compare result, the result data reduction counter and the EMUX channel number (if GxRES[0] only).
 * In polling mechanism the converted result can be read out after checking the valid flag bit.
 * The result register is defined in the channel handle structure @ref ADC_MEASUREMENT_ADV_CHANNEL_t.
 * Hence this API shall call be called with a pointer to the channel handle of type  @ref ADC_MEASUREMENT_ADV_CHANNEL_t
 * (Directly use the channel handle related macros which are defined in adc_measure_adv_conf.h).
 *
 * \par<b>Note: </b><br>
 * This API is not Applicable for reading the result from channels which done use FIFO.
 * Use @ref ADC_MEASUREMENT_ADV_GetDetailedResult in order to read from a single result register.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 2.
// Select the request source APP from ADC_SCAN to ADC_QUEUE.
// Goto the sequence plan and select Channel_A at position-0 and Channel_B at position-1.
// Enable Wait for trigger for the Channel_A.
// Goto interrupts tab enable request source interrupt.
// Goto the ADC_QUEUE APP and enable the request source interrupt.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#define MAX_FIFO (16U)
XMC_VADC_DETAILED_RESULT_t result[16];
uint16_t i = 0U;
void adc_measurement_adv_callback(void)
{
  // Use the channel handle parameter in this format "<APP Name>_<CHANNEL Name>"
  result[i++] = ADC_MEASUREMENT_ADV_GetFifoDetailedResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  result[i++] = ADC_MEASUREMENT_ADV_GetFifoDetailedResult(&ADC_MEASUREMENT_ADV_0_Channel_B);

  if(MAX_FIFO == i)
  {
    i = (uint32_t)0;
  }
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  // Start the queue conversion sequence
  ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
  }
}
 * @endcode
 */
__STATIC_INLINE XMC_VADC_DETAILED_RESULT_t ADC_MEASUREMENT_ADV_GetFifoDetailedResult(const
                                                                       ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr)
{
  XMC_VADC_DETAILED_RESULT_t result_register;
  extern XMC_VADC_GROUP_t *const group_ptrs[XMC_VADC_MAXIMUM_NUM_GROUPS];
  XMC_ASSERT("ADC_MEASUREMENT_ADV_GetFifoDetailedResult:Invalid handle_ptr", (handle_ptr != NULL))

  result_register.res = XMC_VADC_GROUP_GetDetailedResult(group_ptrs[handle_ptr->group_index],
                                                         handle_ptr->result_fifo_tail_number);
  return(result_register);
}

#endif

/**
 * @brief Returns the result of fast conversion
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @return XMC_VADC_FAST_COMPARE_t fast conversion result.<BR>
 *            Returns ::ADC_MEASUREMENT_ADV_FAST_COMPARE_HIGH if the sampled signal is greater than the compare value.
 *            Returns ::ADC_MEASUREMENT_ADV_FAST_COMPARE_LOW if the sampled signal is lower than the compare value.
 *            Returns ::ADC_MEASUREMENT_ADV_FAST_COMPARE_INVALID if there is no valid result available.
 *
 * \par<b>Description: </b><br>
 * Returns the fast conversion result stored in the result register [GxRESy.FCR].
 * In the APP each channel is configured to a particular group result register (excluding FIFO). The result register
 * is defined in the channel handle structure @ref ADC_MEASUREMENT_ADV_CHANNEL_t. Hence this API shall call be called
 * with a pointer to the channel handle of type  @ref ADC_MEASUREMENT_ADV_CHANNEL_t (Directly use the channel handle
 * related macros which are defined in adc_measure_adv_conf.h).
 *
 * \par<b>Note: </b><br>
 * This API is only applicable to read fast compare result of the particular channel.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 1.
// Select the request source APP from ADC_SCAN to ADC_QUEUE.
// Goto the sequence plan and select Channel_A at position-0 and enable the refill.
// Enable Wait for trigger for the Channel_A.
// Goto interrupts tab enable result event for Channel_A.
// Instantiate the interrupt APP.
// In the UI of the interrupt APP change the interrupt handler to "channel_event_callback"
// goto HW signal connectivity and connect event_result_Channel_A to interrupt APP.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#define MAX_LOCAL_BUFFER (16U)
ADC_MEASUREMENT_ADV_FAST_COMPARE_t result[16];
uint16_t i = 0U;

void channel_event_callback(void)
{
  result[i++] = ADC_MEASUREMENT_ADV_GetFastCompareResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  if(MAX_LOCAL_BUFFER == i)
  {
    i = 0U;
  }
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Set the threshold value as Vdd/2
  ADC_MEASUREMENT_ADV_SetFastCompareValue(&ADC_MEASUREMENT_ADV_0_Channel_A,512U);

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
  // Continuously trigger the queue conversion sequence
  ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);
  }
}

 * @endcode
 */
__STATIC_INLINE ADC_MEASUREMENT_ADV_FAST_COMPARE_t ADC_MEASUREMENT_ADV_GetFastCompareResult(const
                                                                       ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr)
{
  ADC_MEASUREMENT_ADV_FAST_COMPARE_t fast_compare_result;
  extern XMC_VADC_GROUP_t *const group_ptrs[XMC_VADC_MAXIMUM_NUM_GROUPS];
  XMC_ASSERT("ADC_MEASUREMENT_ADV_GetFastCompareResult:Invalid handle_ptr", (handle_ptr != NULL))

  fast_compare_result = (ADC_MEASUREMENT_ADV_FAST_COMPARE_t)XMC_VADC_GROUP_GetFastCompareResult(
                               group_ptrs[handle_ptr->group_index],(uint32_t) handle_ptr->ch_handle->result_reg_number);

  return(fast_compare_result);
}

/**
 * @brief Sets the fast conversion value
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @param compare_value constant pointer to the channel handle structure.
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Returns the converted result stored in the result register [GxRESy.RESULT].
 * In the APP each channel is configured to a particular group result register. If FIFO is enabled for the particular
 * channel this reads the fast compare result from the FIFO tail register. The result register
 * is defined in the channel handle structure @ref ADC_MEASUREMENT_ADV_CHANNEL_t. Hence this API shall call be called
 * with a pointer to the channel handle of type  @ref ADC_MEASUREMENT_ADV_CHANNEL_t (Directly use the channel handle
 * related macros which are defined in adc_measure_adv_conf.h).
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 1.
// Select the request source APP from ADC_SCAN to ADC_QUEUE.
// Goto the sequence plan and select Channel_A at position-0 and enable the refill.
// Enable Wait for trigger for the Channel_A.
// Goto interrupts tab enable result event for Channel_A.
// Instantiate the interrupt APP.
// In the UI of the interrupt APP change the interrupt handler to "result_event_callback"
// goto HW signal connectivity and connect event_result_Channel_A to interrupt APP.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#define MAX_LOCAL_BUFFER (16U)
ADC_MEASUREMENT_ADV_FAST_COMPARE_t result[16];
uint16_t i = 0U;

void result_event_callback(void)
{
  result[i++] = ADC_MEASUREMENT_ADV_GetFastCompareResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  if(MAX_LOCAL_BUFFER == i)
  {
    i = 0U;
  }
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Set the threshold value as Vdd/2
  ADC_MEASUREMENT_ADV_SetFastCompareValue(&ADC_MEASUREMENT_ADV_0_Channel_A,512U);

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
  // Continuously trigger the queue conversion sequence
  ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);
  }
}

 * @endcode
 */
ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_SetFastCompareValue(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                                                     uint16_t compare_value);

/**
 * @brief Set the subtraction value if a result difference mode is required.
 * @param handle_ptr constant pointer to the APP handle structure<BR>
 * @param subtraction_alignment The result alignment in the result register.<BR>
 * @param subtraction_value constant value that is used for subtraction.
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Set the subtraction value in the result register 0 [GxRES[0].RESULT].
 * In the subtraction mode the result register 0 is used as the subtrahend. Any channel in the GUI which
 * has selected the subtraction mode will have its converted value subtracted from the value stored in the
 * result register-0. This can be used as an offset for the converted values.
 *
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 2.
// Select the request source APP from ADC_SCAN to ADC_QUEUE.
// Goto the sequence plan and select Channel_A at position-0 and Channel_B at position-1.
// Enable Wait for trigger for the Channel_A.
// Goto interrupts tab enable request source interrupt.
// Goto the ADC_QUEUE APP and enable the request source interrupt.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#define VOLTAGE_OFFSET (50U)
uint16_t result[2];
void adc_measurement_adv_callback(void)
{
  result[0] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  result[1] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_B);
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  ADC_MEASUREMENT_ADV_SetSubtractionValue(&ADC_MEASUREMENT_ADV_0,ADC_MEASUREMENT_ADV_SUBTRATION_12BIT_RIGHT_ALIGN,
                                           VOLTAGE_OFFSET);

  // Start the queue conversion sequence
  ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
  }
}
 * @endcode
 */
void ADC_MEASUREMENT_ADV_SetSubtractionValue(const ADC_MEASUREMENT_ADV_t *const handle_ptr,
                                              ADC_MEASUREMENT_ADV_SUBTRATION_t subtraction_alignment,
                                              uint16_t subtraction_value);

/**
 * @brief Configures the input class (Resolution and Sampling time).
 * @param iclass_selection Select the input class to be configured<BR>
 *         Pass XMC_VADC_CHANNEL_CONV_GROUP_CLASS0 to configure group input class 0.
 *         Pass XMC_VADC_CHANNEL_CONV_GROUP_CLASS1 to configure group input class 1.<BR>
 * @param config Constant pointer to the input class configuration.<BR>
 * @param group_num The group number whose input class needs to be configured.
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Configures the input class for standard conversion ( GxICLASS[0] OR GxICLASS[1]).
 * A call to this API would configure the Resolution and sampling time for standard conversion.
 * The group-specific input class registers define the sample time and data conversion
 * mode for each channel of the respective group. And each channel can use these by selecting the
 * input class in GxCHCTRy.ICLSEL.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 2.
// Select the request source APP from ADC_SCAN to ADC_QUEUE.
// Goto the sequence plan and select Channel_A at position-0 and Channel_B at position-1.
// Enable Wait for trigger for the Channel_A.
// Goto interrupts tab enable request source interrupt.
// Goto the ADC_QUEUE APP and enable the request source interrupt.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

uint16_t result[2];
void adc_measurement_adv_callback(void)
{
  // converted result will be of 8bit resolution.
  result[0] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  result[1] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_B);
}

int main(void)
{
  DAVE_STATUS_t status;

  XMC_VADC_GROUP_CLASS_t res_8bit = {
      .conversion_mode_standard = XMC_VADC_CONVMODE_8BIT,
      .sample_time_std_conv     = 10U
  };

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  ADC_MEASUREMENT_ADV_ConfigureChannelClass(&ADC_MEASUREMENT_ADV_0_Channel_A,&res_8bit);

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  // Start the queue conversion sequence
  ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
  }
}
 * @endcode
 */
void ADC_MEASUREMENT_ADV_ConfigureChannelClass(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                               const XMC_VADC_GROUP_CLASS_t *config);

/**
 * @brief Selects alternate reference voltage for the channel.
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @param reference_select Voltage reference for the channel.<BR>
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Select the reference voltage for conversion.
 * For XMC4000 series, an internal voltage reference (VARef) or an external voltage reference fed to Ch-0 can serve
 * as a voltage reference for conversions.
 * For XMC1000 series, an internal ground reference (Vss) or an external reference ground from CH-0 can serve as an
 * alternate reference. A call to this API would configure the register bit field GxCHCTR.REFSEL.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 2.
// Goto interrupts tab enable request source interrupt.
// Goto the ADC_SCAN APP and enable the request source interrupt.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

uint16_t result[2];
void adc_measurement_adv_callback(void)
{
  result[0] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  result[1] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_B);
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  // connect the alternate reference to the channel-0 pin of Channel_A's group.
  ADC_MEASUREMENT_ADV_SetAlternateReference(&ADC_MEASUREMENT_ADV_0_Channel_A, XMC_VADC_CHANNEL_REF_ALT_CH0);

  while(1U)
  {
    // Continuously re-trigger the scan conversion sequence
    ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);
  }
}
 * @endcode
 */
void ADC_MEASUREMENT_ADV_SetAlternateReference(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                                const XMC_VADC_CHANNEL_REF_t reference_select);

#if (XMC_VADC_SHS_AVAILABLE == 1U)
/**
 * @brief Sets the channel gain.
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @param gain_factor The gain factor value.<BR>
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Set the gain value for the particular channel.
 * This API would set the SHS gain factor for the channel. The input voltage will get a gain proportional to the
 * selected \b gain_factor then will be converted by the ADC.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 2.
// Goto interrupts tab enable request source interrupt.
// Goto the ADC_SCAN APP and enable the request source interrupt.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

uint16_t result[2];
void adc_measurement_adv_callback(void)
{
  result[0] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  result[1] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_B);
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  // select the gain value of 12
  ADC_MEASUREMENT_ADV_SetChannelGain(&ADC_MEASUREMENT_ADV_0_Channel_A, ADC_MEASUREMENT_ADV_GAIN_12);

  while(1U)
  {
    // Continuously re-trigger the scan conversion sequence
    ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);
  }
}
 * @endcode
 */
void ADC_MEASUREMENT_ADV_SetChannelGain(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                         const ADC_MEASUREMENT_ADV_GAIN_t gain_factor);
#endif


/**
 * @brief Select the boundary for the channel.
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @param boundary The lower boundary or upper boundary of the channel to be configured.<BR>
 * @param boundary_selection Boundary register selection for the particular channel.<BR>
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Select the boundary for the channel. This API will select either lower boundary or the upper boundary depending on
 * \b boundary and configures the \b boundary_selection value into the GxCHCTRy.BNDSELL or GxCHCTRy.BNDSELU accordingly.
 *
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 1.
// Select the request source APP from ADC_SCAN to ADC_QUEUE.
// Goto the sequence plan and select Channel_A at position-0 and enable the refill.
// Enable Wait for trigger for the Channel_A.
// Goto the Boundary Settings tab and select the channel name as Channel_A.
// Change the channel event from No to "If Result Inside Band".
// Change the "Generate boundary flag" to Yes/Non-Inverted.
// Instantiate the interrupt APP.
// In the UI of the interrupt APP change the interrupt handler to "channel_event_callback"
// goto HW signal connectivity and connect event_channel_Channel_A to interrupt APP.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#define MAX_LOCAL_BUFFER (16U)
uint16_t result[16];
uint16_t i = 0U;

void channel_event_callback(void)
{
  result[i++] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  if(MAX_LOCAL_BUFFER == i)
  {
    i = 0U;
  }
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Set the boundary selection for Channel_A lower Bound as Group Bound-1
  ADC_MEASUREMENT_ADV_SelectBoundary(&ADC_MEASUREMENT_ADV_0_Channel_A, XMC_VADC_BOUNDARY_SELECT_LOWER_BOUND,
                                      XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1);

  // Set the boundary selection for Channel_A upper Bound as Group Bound-0 and also the value as 2048U
  ADC_MEASUREMENT_ADV_SetBoundaryUpper(&ADC_MEASUREMENT_ADV_0_Channel_A, 2048U);

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
  // Continuously trigger the queue conversion sequence
  ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);
  }
}
 * @endcode
 */
void ADC_MEASUREMENT_ADV_SelectBoundary(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                         XMC_VADC_BOUNDARY_SELECT_t boundary,
                                         XMC_VADC_CHANNEL_BOUNDARY_t boundary_selection);

/**
 * @brief Sets the upper boundary value for the channel.
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @param boundary_value The boundary value that needs to be configured in the upper boundary register.<BR>
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Sets the upper boundary value for the channel. This API will set the upper boundary value depending on
 * boundary selected for the channel in the GUI of the APP. Thus also configured in the API is either the register
 * GLOBBOUND or GxBOUND.
 *
 * \par<b>Note: </b><br>
 * This API will configure the upper boundary for the channel according to the selection done in the GUI. Runtime
 * change of the Upper boundary selection will not be handled by this API.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 1.
// Select the request source APP as ADC_SCAN.
// Goto the Boundary Settings tab and select the channel name as Channel_A.
// Select the Upper boundary as Group boundary 1.
// Change the channel event from No to "If Result Inside Band".
// Change the "Generate boundary flag" to Yes/Non-Inverted.
// Instantiate the interrupt APP.
// In the UI of the interrupt APP change the interrupt handler to "channel_event_callback"
// goto HW signal connectivity and connect event_channel_Channel_A to interrupt APP.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#define MAX_LOCAL_BUFFER (16U)
uint16_t result[16];
uint16_t i = 0U;

void channel_event_callback(void)
{
  result[i++] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  if(MAX_LOCAL_BUFFER == i)
  {
    i = 0U;
  }
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Set the boundary selection for Channel_A lower Bound as Group Bound-1
  ADC_MEASUREMENT_ADV_SetBoundaryLower(&ADC_MEASUREMENT_ADV_0_Channel_A,1024);

  // Set the boundary selection for Channel_A upper Bound as Group Bound-0 and also the value as 2048U
  ADC_MEASUREMENT_ADV_SetBoundaryUpper(&ADC_MEASUREMENT_ADV_0_Channel_A,2048);

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
    // Continuously re-trigger the scan conversion sequence
    ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);
  }
}
 * @endcode
 */
void ADC_MEASUREMENT_ADV_SetBoundaryUpper(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                          uint32_t boundary_value);

/**
 * @brief Sets the lower boundary value for the channel.
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @param boundary_value The boundary value that needs to be configured in the lower boundary register.<BR>
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Sets the lower boundary value for the channel. This API will set the lower boundary value depending on
 * boundary selected for the channel in the GUI of the APP. Thus also configured in the API is either the register
 * GLOBBOUND or GxBOUND.
 *
 * \par<b>Note: </b><br>
 * This API will configure the lower boundary for the channel according to the selection done in the GUI. Runtime
 * change of the lower boundary selection will not be handled by this API.
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 1.
// Select the request source APP as ADC_SCAN.
// Goto the Boundary Settings tab and select the channel name as Channel_A.
// Select the Upper boundary as Group boundary 1.
// Change the channel event from No to "If Result Inside Band".
// Change the "Generate boundary flag" to Yes/Non-Inverted.
// Instantiate the interrupt APP.
// In the UI of the interrupt APP change the interrupt handler to "channel_event_callback"
// goto HW signal connectivity and connect event_channel_Channel_A to interrupt APP.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#define MAX_LOCAL_BUFFER (16U)
uint16_t result[16];
uint16_t i = 0U;

void channel_event_callback(void)
{
  result[i++] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  if(MAX_LOCAL_BUFFER == i)
  {
    i = 0U;
  }
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Set the boundary selection for Channel_A lower Bound as Group Bound-1
  ADC_MEASUREMENT_ADV_SetBoundaryLower(&ADC_MEASUREMENT_ADV_0_Channel_A,1024);

  // Set the boundary selection for Channel_A upper Bound as Group Bound-0 and also the value as 2048U
  ADC_MEASUREMENT_ADV_SetBoundaryUpper(&ADC_MEASUREMENT_ADV_0_Channel_A,2048);

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
    // Continuously re-trigger the scan conversion sequence
    ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);
  }
}
 * @endcode
 */
void ADC_MEASUREMENT_ADV_SetBoundaryLower(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                          uint32_t boundary_value);


/**
 * @brief Returns the configured alias value
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @return XMC_VADC_CHANNEL_ALIAS_t
 *         returns XMC_VADC_CHANNEL_ALIAS_DISABLED if the alias is not applicable to the channel or if alias is
 *                 not enabled<BR>
 *         else it returns the alias value.<BR>
 *
 * \par<b>Description: </b><br>
 * Return the alias value for the channel.
 * If the alias feature is enabled then the channels CH-0 or CH-1 can convert any other channel's input signal.
 * The API returns  XMC_VADC_CHANNEL_ALIAS_DISABLED if the Channel is neither CH-0 nor CH-1. Also the value
 * XMC_VADC_CHANNEL_ALIAS_DISABLED is returned when the CH-0 or CH-1 is not configured with alias. If either
 * CH-0 or CH-1 is configured with alias then the appropriate aliased channel number is returned.
 * A call to this API would access the register GxALIAS.
 *
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 2.
// Select the request source APP as ADC_SCAN.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#define MAX_LOCAL_BUFFER (16U)
uint16_t result[16];
uint16_t i = 0U;

void channel_event_callback(void)
{
  result[i++] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  if(MAX_LOCAL_BUFFER == i)
  {
    i = 0U;
  }
}

int main(void)
{
  DAVE_STATUS_t status;
  XMC_VADC_CHANNEL_ALIAS_t alias_ch;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  // Check if the given channel is aliased.
  alias_ch = ADC_MEASUREMENT_ADV_GetAliasValue(&ADC_MEASUREMENT_ADV_0_Channel_A);

  if(XMC_VADC_CHANNEL_ALIAS_DISABLED == alias_ch)
  {
     // do something
  }

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
    // Continuously re-trigger the scan conversion sequence
    ADC_MEASUREMENT_ADV_SoftwareTrigger(&ADC_MEASUREMENT_ADV_0);
  }
}
 * @endcode
 */
XMC_VADC_CHANNEL_ALIAS_t ADC_MEASUREMENT_ADV_GetAliasValue(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr);

#ifdef ADC_MEASUREMENT_ADV_SYNC_USED
/**
 * @brief Enables uniform conversion configurations across slaves
 * @param handle_ptr constant pointer to the channel handle structure.
 *                   (Use the channel handle related macros which are defined in adc_measure_adv_conf.h)<BR>
 * @return None <BR>
 *
 * \par<b>Description: </b><br>
 * Enables uniform conversion configurations across slaves.
 * The ADC_QUEUE configures the input class settings for the master group. When slaves need to convert the input signals
 * at the same configuration as the master group then a call to this API is needed. A call to this API will ensure that
 * the master and the slave channels are converting the input signals at the same resolution and sampling time.
 *
 * \par<b>Note: </b><br>
 * <ul>
 * <li>ADC_ADVANCE_MEASURE APP will configure the input class used by ADC_QUEUE or ADC_SCAN in the channel
 * configuration. Thus the input class is either GxICLASS[0] or GxICLASS[1]. The slaves channels are also configured
 * by the same input class number in the GxCHCTR.ICLSEL. A call to this API will copy the configurations from the
 * master groups input class(GxICLASS[z]) to the slaves groups input class (GyICLASS[z], where in \a x is
 * the master group, \a y is the slave group and \a z is the input class number which is common
 * across master and slave).</li>
 *
 * <li>The channel iclass can be changed to global iclass at any time by calling runtime APIs.
 * This API cannot support such a situation.</li>
 * </ul>
 *
 * Example Usage:
 *
 * @code
// Initialize the ADC_MEASUREMENT_ADV APP. Set the number of required channels to 1.
// Select the request source APP from ADC_SCAN to ADC_QUEUE.
// Goto the sequence plan and select Channel_A at position-0 and enable the refill.
// Enable Wait for trigger for the Channel_A.
// Goto the Sync. Conversion tab and select the synchronized groups as 1 slave.
// Goto the Sync Master check box for the Channel_A and enable it.
// Goto interrupts tab enable request source interrupt.
// Goto the ADC_QUEUE APP and enable the request source interrupt.
// Generate the code and build.
// Replace this in the main.c.
#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)

#define MAX_LOCAL_BUFFER (16U)
uint16_t result[16],sync_result[16];
uint16_t i = 0U;

void adc_measurement_adv_callback(void)
{
  result[i++] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A);
  sync_result[i++] = ADC_MEASUREMENT_ADV_GetResult(&ADC_MEASUREMENT_ADV_0_Channel_A_SLAVE_A);
  if(MAX_LOCAL_BUFFER == i)
  {
    i = 0U;
  }
}

int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           // Initialization of DAVE APPs

  if(status == DAVE_STATUS_FAILURE)
  {
    // Placeholder for error handler code. The while loop below can be replaced with an user error handler.
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  //Set the same conversion characteristics for the slave groups channels as well
  ADC_MEASUREMENT_ADV_SetIclass(&ADC_MEASUREMENT_ADV_0);

  // Start the selected request source
  ADC_MEASUREMENT_ADV_StartADC(&ADC_MEASUREMENT_ADV_0);

  while(1U)
  {
  }
}
 * @endcode
 */
void ADC_MEASUREMENT_ADV_SetIclass(const ADC_MEASUREMENT_ADV_t *const handle_ptr);
#endif


/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_ADC_H_ */

/* --- End of File --- */
