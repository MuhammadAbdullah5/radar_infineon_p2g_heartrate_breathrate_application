/**
 *
 *  drv_dma.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */
 
/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/drv_dma.h"

/******************************************************************************
   2. DATA
*******************************************************************************/

extern uint32_t* pDMA_src_I1; /* Source address symbol */

extern uint32_t* pDMA_dst_I1; /* Destination address symbol */

void DMA_CH_I1_reload(DMA_CH_t *obj)
{
#if PORTING_COMPLETE
  XMC_DMA_CH_SetBlockSize(obj->dma_global->dma, obj->ch_num, 256);
  XMC_DMA_CH_SetSourceAddress(obj->dma_global->dma, obj->ch_num, (uint32_t)pDMA_src_I1);
  XMC_DMA_CH_SetDestinationAddress(obj->dma_global->dma, obj->ch_num, (uint32_t)pDMA_dst_I1);
#endif
}

DMA_CH_CONFIG_t DMA_CH_I1_CONFIG =
{
  {
	{
      .enable_interrupt = false, /* Interrupts disabled */
      .dst_transfer_width = XMC_DMA_CH_TRANSFER_WIDTH_16, /* Destination transfer width */
      .src_transfer_width = XMC_DMA_CH_TRANSFER_WIDTH_16, /* Source transfer width */
      .dst_address_count_mode = XMC_DMA_CH_ADDRESS_COUNT_MODE_INCREMENT, /* Destination address count mode */
      .src_address_count_mode = XMC_DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE, /* Source address count mode */
      .dst_burst_length = XMC_DMA_CH_BURST_LENGTH_1, /* Destination burst length */
      .src_burst_length = XMC_DMA_CH_BURST_LENGTH_1, /* Source burst length */
      .enable_src_gather = false, /* Source gather enabled? */
      .enable_dst_scatter = false, /* Destination scatter enabled? */
      .transfer_flow = XMC_DMA_CH_TRANSFER_FLOW_P2M_DMA, /* Transfer flow */
    },
    .block_size = 256U, /* Block size */
    .transfer_type = XMC_DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK, /* Transfer type */
    .priority = XMC_DMA_CH_PRIORITY_7, /* Priority level */
    .src_handshaking = XMC_DMA_CH_SRC_HANDSHAKING_HARDWARE, /* Source handshaking */
    .src_peripheral_request = DMA_PERIPHERAL_REQUEST(1, 1), /* Source peripheral trigger */
    .dst_handshaking = XMC_DMA_CH_DST_HANDSHAKING_SOFTWARE  /* Destination handshaking */
  },
  NULL,
  0U
};

DMA_CH_t DMA_CH_I1 =
{
  .dma_global = &GLOBAL_DMA_0, /* Which DMA_GLOBAL instance? */
  .config = &DMA_CH_I1_CONFIG, /* Channel configuration */
  .ch_num = 2U, /* Channel number */
  .reload = DMA_CH_I1_reload
};

extern uint32_t* pDMA_src_Q1; /* Source address symbol */

extern uint32_t* pDMA_dst_Q1; /* Destination address symbol */

void DMA_CH_Q1_reload(DMA_CH_t *obj)
{
	#if PORTING_COMPLETE
  XMC_DMA_CH_SetBlockSize(obj->dma_global->dma, obj->ch_num, 256);
  XMC_DMA_CH_SetSourceAddress(obj->dma_global->dma, obj->ch_num, (uint32_t)pDMA_src_Q1);
  XMC_DMA_CH_SetDestinationAddress(obj->dma_global->dma, obj->ch_num, (uint32_t)pDMA_dst_Q1);
  #endif
}

DMA_CH_CONFIG_t DMA_CH_Q1_CONFIG =
{
  {
	{
      .enable_interrupt = false, /* Interrupts disabled */
      .dst_transfer_width = XMC_DMA_CH_TRANSFER_WIDTH_16, /* Destination transfer width */
      .src_transfer_width = XMC_DMA_CH_TRANSFER_WIDTH_16, /* Source transfer width */
      .dst_address_count_mode = XMC_DMA_CH_ADDRESS_COUNT_MODE_INCREMENT, /* Destination address count mode */
      .src_address_count_mode = XMC_DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE, /* Source address count mode */
      .dst_burst_length = XMC_DMA_CH_BURST_LENGTH_1, /* Destination burst length */
      .src_burst_length = XMC_DMA_CH_BURST_LENGTH_1, /* Source burst length */
      .enable_src_gather = false, /* Source gather enabled? */
      .enable_dst_scatter = false, /* Destination scatter enabled? */
      .transfer_flow = XMC_DMA_CH_TRANSFER_FLOW_P2M_DMA, /* Transfer flow */
    },
    .block_size = 256U, /* Block size */
    .transfer_type = XMC_DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK, /* Transfer type */
    .priority = XMC_DMA_CH_PRIORITY_7, /* Priority level */
    .src_handshaking = XMC_DMA_CH_SRC_HANDSHAKING_HARDWARE, /* Source handshaking */
    .src_peripheral_request = DMA_PERIPHERAL_REQUEST(0, 1), /* Source peripheral trigger */
    .dst_handshaking = XMC_DMA_CH_DST_HANDSHAKING_SOFTWARE  /* Destination handshaking */
  },
  NULL,
  0U
};

DMA_CH_t DMA_CH_Q1 =
{
  .dma_global = &GLOBAL_DMA_0, /* Which DMA_GLOBAL instance? */
  .config = &DMA_CH_Q1_CONFIG, /* Channel configuration */
  .ch_num = 3U, /* Channel number */
  .reload = DMA_CH_Q1_reload
};

extern uint32_t* pDMA_src_I2; /* Source address symbol */

extern uint32_t* pDMA_dst_I2; /* Destination address symbol */

void DMA_CH_I2_reload(DMA_CH_t *obj)
{
	#if PORTING_COMPLETE
  XMC_DMA_CH_SetBlockSize(obj->dma_global->dma, obj->ch_num, 256);
  XMC_DMA_CH_SetSourceAddress(obj->dma_global->dma, obj->ch_num, (uint32_t)pDMA_src_I2);
  XMC_DMA_CH_SetDestinationAddress(obj->dma_global->dma, obj->ch_num, (uint32_t)pDMA_dst_I2);
  #endif
}

DMA_CH_CONFIG_t DMA_CH_I2_CONFIG =
{
  {
	{
      .enable_interrupt = false, /* Interrupts disabled */
      .dst_transfer_width = XMC_DMA_CH_TRANSFER_WIDTH_16, /* Destination transfer width */
      .src_transfer_width = XMC_DMA_CH_TRANSFER_WIDTH_16, /* Source transfer width */
      .dst_address_count_mode = XMC_DMA_CH_ADDRESS_COUNT_MODE_INCREMENT, /* Destination address count mode */
      .src_address_count_mode = XMC_DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE, /* Source address count mode */
      .dst_burst_length = XMC_DMA_CH_BURST_LENGTH_1, /* Destination burst length */
      .src_burst_length = XMC_DMA_CH_BURST_LENGTH_1, /* Source burst length */
      .enable_src_gather = false, /* Source gather enabled? */
      .enable_dst_scatter = false, /* Destination scatter enabled? */
      .transfer_flow = XMC_DMA_CH_TRANSFER_FLOW_P2M_DMA, /* Transfer flow */
    },
    .block_size = 256U, /* Block size */
    .transfer_type = XMC_DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK, /* Transfer type */
    .priority = XMC_DMA_CH_PRIORITY_7, /* Priority level */
    .src_handshaking = XMC_DMA_CH_SRC_HANDSHAKING_HARDWARE, /* Source handshaking */
    .src_peripheral_request = DMA_PERIPHERAL_REQUEST(3, 1), /* Source peripheral trigger */
    .dst_handshaking = XMC_DMA_CH_DST_HANDSHAKING_SOFTWARE  /* Destination handshaking */
  },
  NULL,
  0U
};

DMA_CH_t DMA_CH_I2 =
{
  .dma_global = &GLOBAL_DMA_0, /* Which DMA_GLOBAL instance? */
  .config = &DMA_CH_I2_CONFIG, /* Channel configuration */
  .ch_num = 1U, /* Channel number */
  .reload = DMA_CH_I2_reload
};

extern uint32_t* pDMA_src_Q2; /* Source address symbol */

extern uint32_t* pDMA_dst_Q2; /* Destination address symbol */

void DMA_CH_Q2_reload(DMA_CH_t *obj)
{
	#if PORTING_COMPLETE
  XMC_DMA_CH_SetBlockSize(obj->dma_global->dma, obj->ch_num, 256);
  XMC_DMA_CH_SetSourceAddress(obj->dma_global->dma, obj->ch_num, (uint32_t)pDMA_src_Q2);
  XMC_DMA_CH_SetDestinationAddress(obj->dma_global->dma, obj->ch_num, (uint32_t)pDMA_dst_Q2);
  #endif
}

DMA_CH_CONFIG_t DMA_CH_Q2_CONFIG =
{
  {
	{
      .enable_interrupt = true, /* Interrupts enabled */
      .dst_transfer_width = XMC_DMA_CH_TRANSFER_WIDTH_16, /* Destination transfer width */
      .src_transfer_width = XMC_DMA_CH_TRANSFER_WIDTH_16, /* Source transfer width */
      .dst_address_count_mode = XMC_DMA_CH_ADDRESS_COUNT_MODE_INCREMENT, /* Destination address count mode */
      .src_address_count_mode = XMC_DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE, /* Source address count mode */
      .dst_burst_length = XMC_DMA_CH_BURST_LENGTH_1, /* Destination burst length */
      .src_burst_length = XMC_DMA_CH_BURST_LENGTH_1, /* Source burst length */
      .enable_src_gather = false, /* Source gather enabled? */
      .enable_dst_scatter = false, /* Destination scatter enabled? */
      .transfer_flow = XMC_DMA_CH_TRANSFER_FLOW_P2M_DMA, /* Transfer flow */
    },
    .block_size = 256U, /* Block size */
    .transfer_type = XMC_DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK, /* Transfer type */
    .priority = XMC_DMA_CH_PRIORITY_7, /* Priority level */
    .src_handshaking = XMC_DMA_CH_SRC_HANDSHAKING_HARDWARE, /* Source handshaking */
    .src_peripheral_request = DMA_PERIPHERAL_REQUEST(2, 2), /* Source peripheral trigger */
    .dst_handshaking = XMC_DMA_CH_DST_HANDSHAKING_SOFTWARE  /* Destination handshaking */
  },
  DMA_Event_Handler,
  XMC_DMA_CH_EVENT_BLOCK_TRANSFER_COMPLETE
};

DMA_CH_t DMA_CH_Q2 =
{
  .dma_global = &GLOBAL_DMA_0, /* Which DMA_GLOBAL instance? */
  .config = &DMA_CH_Q2_CONFIG, /* Channel configuration */
  .ch_num = 0U, /* Channel number */
  .reload = DMA_CH_Q2_reload
};

GLOBAL_DMA_INTERRUPT_CONFIG_t GLOBAL_DMA_0_CONFIG =
{
  .priority = 63U, /* Node interrupt priority */
  .sub_priority = 0U  /* Node interrupt sub-priority */
};

GLOBAL_DMA_t GLOBAL_DMA_0 =
{
  .dma = XMC_DMA1, /* Which DMA module? */
  .config = &GLOBAL_DMA_0_CONFIG, /* A reference to interrupt config */
  .initialized = (bool)0U, /* Is DMA initialized yet? */
  .irq_node = (IRQn_Type)110U /* Allotted DMA IRQ node */
};

void GPDMA0_0_IRQHandler(void)
{
  XMC_DMA_IRQHandler(XMC_DMA0);
}

#ifdef XMC_DMA1
void GPDMA1_0_IRQHandler(void)
{
  XMC_DMA_IRQHandler(XMC_DMA1);
}
#endif

/******************************************************************************
   3. LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

/******************************************************************************
   4. EXPORTED FUNCTIONS
*******************************************************************************/
/* GLOBAL_DMA initialization function */
GLOBAL_DMA_STATUS_t GLOBAL_DMA_Init(GLOBAL_DMA_t *const obj)
{
  XMC_ASSERT("DMA_GLOBAL_Init: NULL DMA_GLOBAL_t object", (obj != NULL));

  if (obj->initialized == false)
  {
    /* Enable DMA module */
    XMC_DMA_Init(obj->dma);

    /* Enable DMA event handling */
    NVIC_SetPriority(obj->irq_node, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                                                        obj->config->priority,
                                                        obj->config->sub_priority));
    NVIC_EnableIRQ(obj->irq_node);
    obj->initialized = true;
  }

  return GLOBAL_DMA_STATUS_SUCCESS;
}

/* DMA_CH initialization function */
DMA_CH_STATUS_t DMA_CH_Init(DMA_CH_t *const obj)
{
  DMA_CH_STATUS_t status;

  XMC_ASSERT("DMA_CH_Init: NULL DMA_CH_t object", (obj != NULL));

  /* This cannot possibly fail! */
  GLOBAL_DMA_Init(obj->dma_global);

  if (XMC_DMA_CH_Init(obj->dma_global->dma, obj->ch_num, &obj->config->ch_config) == XMC_DMA_CH_STATUS_OK)
  {
	if(obj->config->events)
	{
		XMC_DMA_CH_EnableEvent(obj->dma_global->dma, obj->ch_num, obj->config->events);
		XMC_DMA_CH_SetEventHandler(obj->dma_global->dma, obj->ch_num, obj->config->callback);
	}
    obj->reload(obj);
    status = DMA_CH_STATUS_SUCCESS;
  }
  else
  {
    status = DMA_CH_STATUS_FAILURE;
  }

  return status;
}

/******************************************************************************
   5. LOCAL FUNCTIONS
*******************************************************************************/



/* --- End of File --- */

