/**
 *
 *  drv_clk.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/drv_clk.h"

/******************************************************************************
   2. DATA
*******************************************************************************/
CLOCK_XMC4_t CLOCK_XMC4_0 =
{
  .init_status = false
};

/******************************************************************************
   3. LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/
void SystemCoreClockSetup(void)
{
  /* Local data structure for initializing the clock functional block */
  const XMC_SCU_CLOCK_CONFIG_t CLOCK_XMC4_0_CONFIG =
  {
    /* N-Divider Value */
    .syspll_config.n_div = 21U,
    /* P-Divider Value */
    .syspll_config.p_div = 3U,
    /* K2-Divider Value */
    .syspll_config.k_div = 1U,
    /* PLL Operating Mode */
    .syspll_config.mode = XMC_SCU_CLOCK_SYSPLL_MODE_NORMAL,
    /* PLL Clock Source */
    .syspll_config.clksrc = XMC_SCU_CLOCK_SYSPLLCLKSRC_OSCHP,
    /* High Precision Oscillator Operating Mode */
    .enable_oschp = true,
    /* Ultra Low Power Oscillator Setting */
    .enable_osculp = false,
    /* Calibration Mode */
    .calibration_mode = XMC_SCU_CLOCK_FOFI_CALIBRATION_MODE_FACTORY,
    /* Standby Clock Source */
    .fstdby_clksrc = XMC_SCU_HIB_STDBYCLKSRC_OSI,
    /* System Clock Source */
    .fsys_clksrc = XMC_SCU_CLOCK_SYSCLKSRC_PLL,
    /* System Clock Divider Value */
    .fsys_clkdiv = 2U,
    /* CPU Clock Divider Value */
    .fcpu_clkdiv = 1U,
#ifdef CLOCK_XMC4_CCUCLK_ENABLED
    /* CCU Clock Divider Value */
    .fccu_clkdiv = 1U,
#endif
    /* Peripheral Clock Divider Value */
    .fperipheral_clkdiv = 1U
  };
  /* Initialize the SCU clock */
  XMC_SCU_CLOCK_Init(&CLOCK_XMC4_0_CONFIG);
  /* RTC source clock */
  XMC_SCU_HIB_SetRtcClockSource(XMC_SCU_HIB_RTCCLKSRC_OSI);

#ifdef CLOCK_XMC4_USBCLK_ENABLED
  /* USB/SDMMC source clock */
  XMC_SCU_CLOCK_SetUsbClockSource(XMC_SCU_CLOCK_USBCLKSRC_USBPLL);
  /* USB/SDMMC divider setting */
  XMC_SCU_CLOCK_SetUsbClockDivider(4U);
#endif
  /* Start USB PLL */
  XMC_SCU_CLOCK_StartUsbPll(5U, 48U);

#ifdef CLOCK_XMC4_WDTCLK_ENABLED
  /* WDT source clock */
  XMC_SCU_CLOCK_SetWdtClockSource(XMC_SCU_CLOCK_WDTCLKSRC_OFI);
  /* WDT divider setting */
  XMC_SCU_CLOCK_SetWdtClockDivider(1U);
#endif

#ifdef CLOCK_XMC4_EBUCLK_ENABLED
  /* EBU divider setting */
  XMC_SCU_CLOCK_SetEbuClockDivider(1U);
#endif

}

/*
 * API to initialize the CLOCK_XMC4 APP TRAP events
 */
CLOCK_XMC4_STATUS_t CLOCK_XMC4_Init(CLOCK_XMC4_t *handle)
{
  CLOCK_XMC4_STATUS_t status = CLOCK_XMC4_STATUS_SUCCESS;

  XMC_ASSERT("CLOCK_XMC4 APP handle function pointer uninitialized", (handle != NULL));

  handle->init_status = true;

  return (status);
}

#ifdef CLOCK_XMC4_OSCHP_ENABLED
/*  API to retrieve high precision external oscillator frequency */
uint32_t OSCHP_GetFrequency(void)
{
  return (CLOCK_XMC4_OSCHP_FREQUENCY);
}
#endif

/*  API for ramping down the system PLL clock frequency */
void CLOCK_XMC4_StepSystemPllFrequency(uint32_t kdiv)
{
  XMC_ASSERT("Incorrect kdiv value", ((kdiv >= 1) && (kdiv >= 128)));
  XMC_SCU_CLOCK_StepSystemPllFrequency(kdiv);
}

/******************************************************************************
   5. LOCAL FUNCTIONS
*******************************************************************************/



/* --- End of File --- */

