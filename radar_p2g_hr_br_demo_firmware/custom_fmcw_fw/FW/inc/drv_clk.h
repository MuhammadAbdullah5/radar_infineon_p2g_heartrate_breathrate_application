/**
 *
 *  drv_clk.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_CLK_H_
#define FW_INC_DRV_CLK_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
#include <xmc_scu.h>
#include <xmc_gpio.h>

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/
/* High precision external oscillator enabled */
#define CLOCK_XMC4_OSCHP_ENABLED
/* High precision external oscillator frequency */
#define CLOCK_XMC4_OSCHP_FREQUENCY (40000000U)
/* Capture/Compare unit clock enabled */
#define CLOCK_XMC4_CCUCLK_ENABLED
/* USB clock enabled */
#define CLOCK_XMC4_USBCLK_ENABLED
/* Watchdog clock enabled */
#define CLOCK_XMC4_WDTCLK_ENABLED
/* EBU clock enabled */
#define CLOCK_XMC4_EBUCLK_ENABLED


/******************************************************************************
   3. TYPES
*******************************************************************************/

typedef enum CLOCK_XMC4_STATUS
{
  CLOCK_XMC4_STATUS_SUCCESS = 0U,        /**<APP initialization is success */
  CLOCK_XMC4_STATUS_FAILURE = 1U         /**<APP initialization is failure */
} CLOCK_XMC4_STATUS_t;

typedef struct CLOCK_XMC4
{
  bool init_status;  /**<APP is initialized or not. */
} CLOCK_XMC4_t;


extern CLOCK_XMC4_t CLOCK_XMC4_0;


/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/
/**
 * @brief Initializes a CLOCK_XMC4 APP instance
 * @param handle address of CLOCK_XMC4 APP handler
 * @return
 *            CLOCK_XMC4_STATUS_SUCCESS             : if initialization is successful\n
 *            CLOCK_XMC4_STATUS_FAILURE             : if initialization is failed
 *
 * \par<b>Description:</b><br>
 * CLOCK_XMC4_Init API is called during initialization of DAVE APPS. This API Initializes NMI TRAP Configuration.
 *
 * \par<b>Example Usage:</b><br>
 *
 * @code
 * #include <DAVE.h>
 *
 * int main(void)
 * {
 *   DAVE_STATUS_t status;
 *
 *   status = DAVE_Init();  //  CLOCK_XMC4_Init API is called during initialization of DAVE APPS
 *   if(DAVE_STATUS_SUCCESS == status)
 *   {
 *    // user code
 *
 *     while(1)
 *     {
 *
 *     }
 *   }
 *   return (1);
 * }
 *
 * @endcode<BR>
 */
CLOCK_XMC4_STATUS_t CLOCK_XMC4_Init(CLOCK_XMC4_t *handle);

#ifdef CLOCK_XMC4_OSCHP_ENABLED
/**
 * @brief This is a non-weak function, which retrieves high precision external oscillator frequency.<br>
 * <b>Note: This function is used by xmc4_scu LLD for internal operations. Therefore the user do not required to call
 * this API explicitly.</b>
 *
 * @return uint32_t Range: 4 to 25 in External Crystal Mode,  4 to 40 in External External Direct Input Mode.
 *
 * \par<b>Description:</b><br>
 * This function to retrieves the external high precision oscillator frequency value, derived from either "External
 * Crystal Mode" or "External Direct Input Mode"
 * <BR>
 */
uint32_t OSCHP_GetFrequency(void);
#endif

/**
 * @brief API for ramping down the system PLL clock frequency
 * @param kdiv PLL output divider K2DIV. Range: 1 to 128. Represents (K2DIV+1).
 * @return none
 *
 * \par<b>Description: </b><br>
 * The function can be used for ramping down the system PLL clock frequency.
 *
 * Example Usage:
 *
 * @code
 * #include <DAVE.h>
 *
 * int main(void)
 * {
 *   DAVE_STATUS_t init_status;
 *   uint32_t kdiv = 10U;  // (K2DIV+1) value for scaling down the system PLL clock frequency
 *   // Initialize CLOCK_XMC4 APP:
 *   // SystemCoreClockSetup() is called from SystemInit().
 *   init_status = DAVE_Init();
 *  if(DAVE_STATUS_SUCCESS == init_status)
 *  {
 *    // More code here
 *
 *
 *    // User decided to reduce the system power consumption by scaling down the system PLL clock frequency
 *    CLOCK_XMC4_StepSystemPllFrequency(kdiv); // fPLL frequency is scaling down by K2DIV factor.
 *
 *
 *    // More code here
 *    while(1) {
 *
 *    }
 *  }
 *  return (1);
 * }
 * @endcode<BR>
 */
void CLOCK_XMC4_StepSystemPllFrequency(uint32_t kdiv);


/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_CLK_H_ */

/* --- End of File --- */
