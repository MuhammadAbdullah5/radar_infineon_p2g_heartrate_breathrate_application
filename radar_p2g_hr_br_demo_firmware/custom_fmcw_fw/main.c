/**
    @file: main.c

    @brief: This application runs on demo Position2Go board with BGT24MTR12 and XMC4700 MCU.
            It consists on radar RangeDoppler demonstration application.
*/

/* ===========================================================================
** Copyright (C) 2018-2019 Infineon Technologies AG
** All rights reserved.
** ===========================================================================
**
** ===========================================================================
** This document contains proprietary information of Infineon Technologies AG.
** Passing on and copying of this document, and communication of its contents
** is not permitted without Infineon's prior written authorization.
** ===========================================================================
*/

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
 */

#include "FW/inc/application.h"


static DAVE_STATUS_t DAVE_Init(void);


/*
==============================================================================
   2. MAIN METHOD
==============================================================================
 */

int main(void)
{
  DAVE_STATUS_t status;

  /* Initialize DAVE APPs */
  status = DAVE_Init();

  if(status != DAVE_STATUS_SUCCESS)
  {
    /* Placeholder for error handler code.
     * The while loop below can be replaced with an user error handler. */
    XMC_DEBUG("DAVE APPs initialization failed\n");
    while(1U);
  }

  /* Register algorithm processing function:
   * Set the algorithm processing function pointer, it will
   * be used by the application for algorithm data processing */
  //app_register_algo_process(range_doppler_do);

  /* Initialize the application */
  app_init();

  /* Infinite loop */
  while(1U)
  {
    /* Main application process */
    app_process();
  }
}

/*
==============================================================================
   3. USER CALLBACK FUNCTIONS
==============================================================================
 */

void acq_completed_cb(void)
{
 /*
  * The following code shows an example of how to access raw data buffer.
  */

  /*
  acq_buf_obj *p_acq_buf 	= ds_get_active_acq_buf();
  uint8_t *raw_data 		= p_acq_buf->p_acq_buf;
  uint32_t raw_data_size 	= p_acq_buf->used_size_of_acq_buffer;

  -- Add your code here --

  */
}

void algo_completed_cb(void)
{
 /*
  * The following sample code could be seen as a small example,
  * how to get the information from the algorithm and use it accordingly.
  */

  /*
  static float s_max_level = 0;
  static uint32_t countFrames = 0;
  static uint32_t s_max_num_targets = 0;

  extern Radar_Handle_t h_radar_device;

  Target_Info_t target_info[MAX_NUM_OF_TARGETS];

  uint8_t num_targets;

  if(radar_get_target_info(h_radar_device, target_info, &num_targets) == RADAR_ERR_OK)
  {
    float max_level = 0;
    for(int i= 0; i< num_targets; i++)
    {
      if(target_info[i].level > max_level)
      {
        s_max_level = target_info[i].level;
      }
    }
    countFrames += 1;

    if(num_targets > s_max_num_targets)
    {
      s_max_num_targets = num_targets;
    }

  }
  */
}


static DAVE_STATUS_t DAVE_Init(void)
{
  DAVE_STATUS_t init_status;

  init_status = DAVE_STATUS_SUCCESS;
     /** @Initialization of APPs Init Functions */
     init_status = (DAVE_STATUS_t)CLOCK_XMC4_Init(&CLOCK_XMC4_0);

  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of ADC_MEASUREMENT_ADV APP instance ADC_MEASUREMENT_ADV_G1 */
	 init_status = (DAVE_STATUS_t)ADC_MEASUREMENT_ADV_Init(&ADC_MEASUREMENT_ADV_G1);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_BGT_POWER_ENABLE */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_BGT_POWER_ENABLE);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_PLL_CE */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_PLL_CE);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_PLL_MOD */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_PLL_MOD);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_SPI_M_CLK */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_SPI_M_CLK);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_SPI_M_CS_BGT24 */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_SPI_M_CS_BGT24);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_SPI_M_CS_PLL */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_SPI_M_CS_PLL);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_SPI_M_DATA */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_SPI_M_DATA);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_PLL_TRIG1 */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_PLL_TRIG1);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_PLL_TRIG2 */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_PLL_TRIG2);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of E_EEPROM_XMC4 APP instance E_EEPROM_XMC4 */
	 init_status = (DAVE_STATUS_t)E_EEPROM_XMC4_Init(&E_EEPROM_XMC4);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of INTERRUPT APP instance INTERRUPT_FRAME */
	 init_status = (DAVE_STATUS_t)INTERRUPT_Init(&INTERRUPT_FRAME);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of SYSTIMER APP instance SYSTIMER_0 */
	 init_status = (DAVE_STATUS_t)SYSTIMER_Init(&SYSTIMER_0);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of TIMER APP instance TIMER_DELAY */
	 init_status = (DAVE_STATUS_t)TIMER_Init(&TIMER_DELAY);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of TIMER APP instance TIMER_FRAME_TRIG */
	 init_status = (DAVE_STATUS_t)TIMER_Init(&TIMER_FRAME_TRIG);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of USBD_VCOM APP instance USBD_VCOM_0 */
	 init_status = (DAVE_STATUS_t)USBD_VCOM_Init(&USBD_VCOM_0);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_BGT_LDO_ENABLE */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_BGT_LDO_ENABLE);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_SPI_DATA_PGA */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_SPI_DATA_PGA);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_SPI_M_CS_PGA */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_SPI_M_CS_PGA);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_PGA_LDO_ENABLE */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_PGA_LDO_ENABLE);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of ADC_MEASUREMENT_ADV APP instance ADC_MEASUREMENT_ADV_G2 */
	 init_status = (DAVE_STATUS_t)ADC_MEASUREMENT_ADV_Init(&ADC_MEASUREMENT_ADV_G2);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of ADC_MEASUREMENT_ADV APP instance ADC_MEASUREMENT_ADV_G3 */
	 init_status = (DAVE_STATUS_t)ADC_MEASUREMENT_ADV_Init(&ADC_MEASUREMENT_ADV_G3);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of ADC_MEASUREMENT_ADV APP instance ADC_MEASUREMENT_ADV_G4 */
	 init_status = (DAVE_STATUS_t)ADC_MEASUREMENT_ADV_Init(&ADC_MEASUREMENT_ADV_G4);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DMA_CH APP instance DMA_CH_I1 */
	 init_status = (DAVE_STATUS_t)DMA_CH_Init(&DMA_CH_I1);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DMA_CH APP instance DMA_CH_Q1 */
	 init_status = (DAVE_STATUS_t)DMA_CH_Init(&DMA_CH_Q1);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DMA_CH APP instance DMA_CH_I2 */
	 init_status = (DAVE_STATUS_t)DMA_CH_Init(&DMA_CH_I2);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DMA_CH APP instance DMA_CH_Q2 */
	 init_status = (DAVE_STATUS_t)DMA_CH_Init(&DMA_CH_Q2);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of ADC_MEASUREMENT_ADV APP instance ADC_MEASUREMENT_SCAN */
	 init_status = (DAVE_STATUS_t)ADC_MEASUREMENT_ADV_Init(&ADC_MEASUREMENT_SCAN);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of EVENT_DETECTOR APP instance EVENT_DETECTOR_PLL_TRIG1 */
	 init_status = (DAVE_STATUS_t)EVENT_DETECTOR_Init(&EVENT_DETECTOR_PLL_TRIG1);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of EVENT_GENERATOR APP instance EVENT_GENERATOR_CCU4 */
	 init_status = (DAVE_STATUS_t)EVENT_GENERATOR_Init(&EVENT_GENERATOR_CCU4);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of CCU4_SLICE_CONFIG APP instance CCU4_SLICE_CONFIG_ADC_TRIG */
	 init_status = (DAVE_STATUS_t)CCU4_SLICE_CONFIG_Init(&CCU4_SLICE_CONFIG_ADC_TRIG);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_PLL_MUXIN */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_PLL_MUXIN);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of DIGITAL_IO APP instance DIGITAL_IO_LED1 */
	 init_status = (DAVE_STATUS_t)DIGITAL_IO_Init(&DIGITAL_IO_LED1);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of TIMER APP instance TIMER_WATCHDOG */
	 init_status = (DAVE_STATUS_t)TIMER_Init(&TIMER_WATCHDOG);
   }
  if (init_status == DAVE_STATUS_SUCCESS)
  {
	 /**  Initialization of INTERRUPT APP instance INTERRUPT_WATCHDOG */
	 init_status = (DAVE_STATUS_t)INTERRUPT_Init(&INTERRUPT_WATCHDOG);
   }
  return init_status;
}
