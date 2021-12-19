/**
 *
 *  drv_adc.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */
 
/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include "FW/inc/drv_adc.h"

/******************************************************************************
   2. DATA
*******************************************************************************/

/**
 * This structure contains the group 0 Configuration.
 */
const XMC_VADC_GROUP_CONFIG_t group_init_handle0 =
{
  .emux_config	= {
					.stce_usage                = (uint32_t) 0, 					 /*Use STCE when the setting changes*/
					.emux_mode                 = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
					.emux_coding               = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
					.starting_external_channel = (uint32_t) 0,                   /* Channel starts at 0 for EMUX*/
					.connected_channel         = (uint32_t) 0                    /* Channel connected to EMUX*/
				   },
  .class0 		= {
             	    .sample_time_std_conv            = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
             	    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
             	    .sampling_phase_emux_channel     = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
					.conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
		    	  },  /* !<ICLASS-0 */
  .class1   	= {
             	    .sample_time_std_conv = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
             	    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
             	    .sampling_phase_emux_channel     = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
					.conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
             	  }, /* !< ICLASS-1 */
  .boundary0    = (uint32_t) 0,  /* Lower boundary value for Normal comparison mode*/
  .boundary1	= (uint32_t) 0,  /* Upper boundary value for Normal comparison mode*/
  .arbitration_round_length = (uint32_t) 0,  /* 4 arbitration slots per round selected (tarb = 4*tadcd) */
  .arbiter_mode             = (uint32_t) XMC_VADC_GROUP_ARBMODE_ONDEMAND,	/*Determines when the arbiter should run.*/
};

/**
 * This structure contains the group 1 Configuration.
 */
const XMC_VADC_GROUP_CONFIG_t group_init_handle1 =
{
  .emux_config	= {
					.stce_usage                = (uint32_t) 0, 					 /*Use STCE when the setting changes*/
					.emux_mode                 = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
					.emux_coding               = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
					.starting_external_channel = (uint32_t) 0,                   /* Channel starts at 0 for EMUX*/
					.connected_channel         = (uint32_t) 0                    /* Channel connected to EMUX*/
				   },
  .class0 		= {
             	    .sample_time_std_conv            = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
             	    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
             	    .sampling_phase_emux_channel     = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
					.conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
		    	  },  /* !<ICLASS-0 */
  .class1   	= {
             	    .sample_time_std_conv = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
             	    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
             	    .sampling_phase_emux_channel     = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
					.conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
             	  }, /* !< ICLASS-1 */
  .boundary0    = (uint32_t) 0,  /* Lower boundary value for Normal comparison mode*/
  .boundary1	= (uint32_t) 0,  /* Upper boundary value for Normal comparison mode*/
  .arbitration_round_length = (uint32_t) 0,  /* 4 arbitration slots per round selected (tarb = 4*tadcd) */
  .arbiter_mode             = (uint32_t) XMC_VADC_GROUP_ARBMODE_ONDEMAND,	/*Determines when the arbiter should run.*/
};

/**
 * This structure contains the group 2 Configuration.
 */
const XMC_VADC_GROUP_CONFIG_t group_init_handle2 =
{
  .emux_config	= {
					.stce_usage                = (uint32_t) 0, 					 /*Use STCE when the setting changes*/
					.emux_mode                 = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
					.emux_coding               = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
					.starting_external_channel = (uint32_t) 0,                   /* Channel starts at 0 for EMUX*/
					.connected_channel         = (uint32_t) 0                    /* Channel connected to EMUX*/
				   },
  .class0 		= {
             	    .sample_time_std_conv            = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
             	    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
             	    .sampling_phase_emux_channel     = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
					.conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
		    	  },  /* !<ICLASS-0 */
  .class1   	= {
             	    .sample_time_std_conv = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
             	    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
             	    .sampling_phase_emux_channel     = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
					.conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
             	  }, /* !< ICLASS-1 */
  .boundary0    = (uint32_t) 0,  /* Lower boundary value for Normal comparison mode*/
  .boundary1	= (uint32_t) 0,  /* Upper boundary value for Normal comparison mode*/
  .arbitration_round_length = (uint32_t) 0,  /* 4 arbitration slots per round selected (tarb = 4*tadcd) */
  .arbiter_mode             = (uint32_t) XMC_VADC_GROUP_ARBMODE_ONDEMAND,	/*Determines when the arbiter should run.*/
};

/**
 * This structure contains the group 3 Configuration.
 */
const XMC_VADC_GROUP_CONFIG_t group_init_handle3 =
{
  .emux_config	= {
					.stce_usage                = (uint32_t) 0, 					 /*Use STCE when the setting changes*/
					.emux_mode                 = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
					.emux_coding               = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
					.starting_external_channel = (uint32_t) 0,                   /* Channel starts at 0 for EMUX*/
					.connected_channel         = (uint32_t) 0                    /* Channel connected to EMUX*/
				   },
  .class0 		= {
             	    .sample_time_std_conv            = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
             	    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
             	    .sampling_phase_emux_channel     = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
					.conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
		    	  },  /* !<ICLASS-0 */
  .class1   	= {
             	    .sample_time_std_conv = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
             	    .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT,     /* 12bit conversion Selected*/
             	    .sampling_phase_emux_channel     = (uint32_t) 0,                /*The Sample time is (2*tadci)*/
					.conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT      /* 12bit conversion Selected*/
             	  }, /* !< ICLASS-1 */
  .boundary0    = (uint32_t) 0,  /* Lower boundary value for Normal comparison mode*/
  .boundary1	= (uint32_t) 0,  /* Upper boundary value for Normal comparison mode*/
  .arbitration_round_length = (uint32_t) 0,  /* 4 arbitration slots per round selected (tarb = 4*tadcd) */
  .arbiter_mode             = (uint32_t) XMC_VADC_GROUP_ARBMODE_ONDEMAND,	/*Determines when the arbiter should run.*/
};

/**
 * This structure contains the Global related Configuration.
 */
const XMC_VADC_GLOBAL_CONFIG_t global_config =
{
  .boundary0    = (uint32_t) 0, /* Lower boundary value for Normal comparison mode*/
  .boundary1    = (uint32_t) 0, /* Upper boundary value for Normal comparison mode*/

  .clock_config = {

	                .analog_clock_divider  = (uint32_t) 4, /*Divider Factor for the Analog Internal Clock*/
	                .arbiter_clock_divider = (uint32_t) 0, /*Divider Factor for the Arbiter Clock*/
	                .msb_conversion_clock  = (uint32_t) 0, /*Double Clock for the MSB Conversion */
	              },

  .class0 = {
     	      .sample_time_std_conv            = (uint32_t) 0,  		  /*The Sample time is (2*tadci)*/
     	      .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT, /* 12bit conversion Selected*/

     	      .sampling_phase_emux_channel     = (uint32_t) 0,			  /*The Sample time is (2*tadci)*/
			  .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT, /* 12bit conversion Selected*/

		    },  /* !<ICLASS-0 */
  .class1 = {
     	      .sample_time_std_conv = (uint32_t) 0,			  /*The Sample time is (2*tadci)*/
     	      .conversion_mode_standard        = XMC_VADC_CONVMODE_12BIT, /* 12bit conversion Selected*/

     	      .sampling_phase_emux_channel     = (uint32_t) 0,			  /*The Sample time is (2*tadci)*/
			  .conversion_mode_emux            = XMC_VADC_CONVMODE_12BIT, /* 12bit conversion Selected*/

            }, /* !< ICLASS-1 */

  .data_reduction_control         = (uint32_t) 0, /* Data Reduction disabled*/
  .wait_for_read_mode             = (uint32_t) 0, /* GLOBRES Register will not be overwriten untill the previous value is read*/
  .event_gen_enable               = (uint32_t) 0, /* Result Event from GLOBRES is disabled*/
  .disable_sleep_mode_control     = (uint32_t) 0  /* Sleep mode is enabled*/
};


/**
 * This structure contains the group LLD Config Structure with Group ptr and Group Index of Group 0
 */
GLOBAL_ADC_GROUP_t group_0_config =
{
  .group_handle	       = (XMC_VADC_GROUP_t *) (void*) VADC_G0,  			 /*Holds the hardware group pointer*/
  .group_config_handle = (XMC_VADC_GROUP_CONFIG_t *) &group_init_handle0, /*Holds the group config structure */
  .post_calibration    = (bool) true, /* Enable or disable the post calibration*/
  .state               = GLOBAL_ADC_UNINITIALIZED       /*The status of the ADC group */
};

/**
 * This structure contains the group LLD Config Structure with Group ptr and Group Index of Group 1
 */
GLOBAL_ADC_GROUP_t group_1_config =
{
  .group_handle	       = (XMC_VADC_GROUP_t *) (void*) VADC_G1,  			 /*Holds the hardware group pointer*/
  .group_config_handle = (XMC_VADC_GROUP_CONFIG_t *) &group_init_handle1, /*Holds the group config structure */
  .post_calibration    = (bool) true, /* Enable or disable the post calibration*/
  .state               = GLOBAL_ADC_UNINITIALIZED       /*The status of the ADC group */
};

/**
 * This structure contains the group LLD Config Structure with Group ptr and Group Index of Group 2
 */
GLOBAL_ADC_GROUP_t group_2_config =
{
  .group_handle	       = (XMC_VADC_GROUP_t *) (void*) VADC_G2,  			 /*Holds the hardware group pointer*/
  .group_config_handle = (XMC_VADC_GROUP_CONFIG_t *) &group_init_handle2, /*Holds the group config structure */
  .post_calibration    = (bool) true, /* Enable or disable the post calibration*/
  .state               = GLOBAL_ADC_UNINITIALIZED       /*The status of the ADC group */
};

/**
 * This structure contains the group LLD Config Structure with Group ptr and Group Index of Group 3
 */
GLOBAL_ADC_GROUP_t group_3_config =
{
  .group_handle	       = (XMC_VADC_GROUP_t *) (void*) VADC_G3,  			 /*Holds the hardware group pointer*/
  .group_config_handle = (XMC_VADC_GROUP_CONFIG_t *) &group_init_handle3, /*Holds the group config structure */
  .post_calibration    = (bool) true, /* Enable or disable the post calibration*/
  .state               = GLOBAL_ADC_UNINITIALIZED       /*The status of the ADC group */
};

/**
 * This structure contains the all the Global Related Structures and also GLOBAL_ADC_GROUP_t Structures
 */
GLOBAL_ADC_t GLOBAL_ADC_0 =
{
  .group_ptrs_array		      = { (GLOBAL_ADC_GROUP_t*) &group_0_config,
						          (GLOBAL_ADC_GROUP_t*) &group_1_config,
						          (GLOBAL_ADC_GROUP_t*) &group_2_config,
						          (GLOBAL_ADC_GROUP_t*) &group_3_config
                                },
  .global_config_handle	      = (XMC_VADC_GLOBAL_CONFIG_t*) &global_config, /*Holds the global config structure */
  .module_ptr			      = (XMC_VADC_GLOBAL_t*)(void*) VADC,    /*Holds the hardware module pointer*/
  .init_state                 = GLOBAL_ADC_UNINITIALIZED, 			   /*The status of the GLOBAL_ADC APP */
  .enable_startup_calibration = (uint32_t) true /* Enable Start up calibration*/
};


/* This function would be called in the ADC_MEASUREMENT_Init() to initialize the SR lines of the
 * Result event/channel event.
 */
static void ADC_MEASUREMENT_ADV_G1_event_config(void)
{

	/* Result Event Node Mux Configuration for IF1_I (Group-0 channel-4 Result_Register-15)*/
	XMC_VADC_GROUP_SetResultInterruptNode(VADC_G0, 15U, XMC_VADC_SR_SHARED_SR1);

}



/********************************* IF1_I ADC Channel configuration structure ********************************/
static const XMC_VADC_CHANNEL_CONFIG_t  ADC_MEASUREMENT_ADV_G1_IF1_I_ch_config =
{
  .input_class                = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
  .lower_boundary_select      = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .upper_boundary_select      = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .event_gen_criteria         = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
  .sync_conversion            = (uint32_t) false,                            /* Sync feature disabled*/
  .alternate_reference        = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,  /* Internal reference selected */
  .result_reg_number          = (uint32_t) 15,                           /* GxRES[15] selected */
  .result_alignment           = (uint32_t) XMC_VADC_RESULT_ALIGN_RIGHT,  /* Result alignment - Right Aligned*/
  .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND, /* No Broken wire mode select*/
  .broken_wire_detect         = (uint32_t) false,                     /* No Broken wire detection*/
  .bfl                        = (uint32_t) 0,                            /* No Boundary flag */
  .channel_priority           = (uint32_t) true,                      /* Highest Priority: 1 selected*/
  .alias_channel              = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,   /* ALIAS is Disabled*/
};


/*IF1_I Result configuration structure*/
static const XMC_VADC_RESULT_CONFIG_t ADC_MEASUREMENT_ADV_G1_IF1_I_res_config =
{
  .data_reduction_control  = (uint32_t) 0,
  .post_processing_mode    = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
  .wait_for_read_mode      = (uint32_t) false,
  .part_of_fifo            = (uint32_t) false , /* No FIFO */
  .event_gen_enable        = (uint32_t) true   /* Result event Enabled*/
};

/* IF1_I ADC channel Handle */
const ADC_MEASUREMENT_ADV_CHANNEL_t ADC_MEASUREMENT_ADV_G1_IF1_I_handle =
{
  .ch_num        = (uint8_t) 4,
  .group_index   = (uint8_t) 0,
  .ch_handle     = (XMC_VADC_CHANNEL_CONFIG_t*) &ADC_MEASUREMENT_ADV_G1_IF1_I_ch_config,
  .res_handle[0] = (XMC_VADC_RESULT_CONFIG_t*) &ADC_MEASUREMENT_ADV_G1_IF1_I_res_config,
};


static const XMC_VADC_QUEUE_CONFIG_t ADC_MEASUREMENT_ADV_G1_queue_config = {
  .conv_start_mode   = (uint32_t) XMC_VADC_STARTMODE_WFS,	/* Conversion start mode WFS/CIR/CNR*/
  .req_src_priority  = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_3, /*The queue request source priority */
  .trigger_signal    = (uint32_t) XMC_VADC_REQ_TR_F,        /*If trigger needed the signal input*/
  .trigger_edge      = (uint32_t) XMC_VADC_TRIGGER_EDGE_RISING,   /*Trigger edge needed if trigger enabled*/
  .gate_signal       = (uint32_t) XMC_VADC_REQ_GT_A,            /*If gating needed the signal input*/
  .timer_mode        = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger	 = (uint32_t) true,         /*External trigger Enabled/Disabled*/
};

static const ADC_MEASUREMENT_ADV_QUEUE_t ADC_MEASUREMENT_ADV_G1_queue_handle =
{
  .rs_intr_handle  = {
					   .node_id          = (uint32_t) 0,
					   .priority         = (uint32_t) 0,
             .sub_priority     = 0,
					   .interrupt_enable = (bool) false,
					 },
  .iclass_config_handle = {
							.sample_time_std_conv        = (uint32_t) 0,
							.conversion_mode_standard    = (uint32_t) XMC_VADC_CONVMODE_12BIT,
							.sampling_phase_emux_channel = (uint32_t) 0,
							.conversion_mode_emux        = (uint32_t) XMC_VADC_CONVMODE_12BIT
						  },
  .queue_config_handle = &ADC_MEASUREMENT_ADV_G1_queue_config,
  .gating_mode  = XMC_VADC_GATEMODE_IGNORE,
  .srv_req_node = XMC_VADC_SR_GROUP_SR0, /*Service Request line for Request source Event */
  .iclass_num = 0
};

static const XMC_VADC_QUEUE_ENTRY_t ADC_MEASUREMENT_ADV_G1_IF1_I_queue_entry_0 =
{
  .channel_num        = (uint8_t) 4,
  .refill_needed      = (bool) true,
  .generate_interrupt = (bool) false,
  .external_trigger   = (bool) true,
};


static const XMC_VADC_QUEUE_ENTRY_t *ADC_MEASUREMENT_ADV_G1_queue_entries[] =
{
  (XMC_VADC_QUEUE_ENTRY_t*) (void *) &ADC_MEASUREMENT_ADV_G1_IF1_I_queue_entry_0,
};



static const ADC_MEASUREMENT_ADV_CHANNEL_t *ADC_MEASUREMENT_ADV_G1_channel_array[] =
{
  (ADC_MEASUREMENT_ADV_CHANNEL_t *) (void*) &ADC_MEASUREMENT_ADV_G1_IF1_I_handle,

};

static ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_G1_initialization_status = ADC_MEASUREMENT_ADV_STATUS_UNINITIALIZED;

const ADC_MEASUREMENT_ADV_t ADC_MEASUREMENT_ADV_G1 =
{
  .channel_array            = ADC_MEASUREMENT_ADV_G1_channel_array,
  .local_queue_entries       = ADC_MEASUREMENT_ADV_G1_queue_entries,
  .event_config               = (ADC_MEASUREMENT_ADV_G1_event_config),
  .local_queue_handle       = (ADC_MEASUREMENT_ADV_QUEUE_t *) &ADC_MEASUREMENT_ADV_G1_queue_handle,
  .init_state               = &ADC_MEASUREMENT_ADV_G1_initialization_status,
  .req_src                  = ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_QUEUE,
  .sync_slave_g1            = (bool) true,
  .sync_slave_g2            = (bool) true,
  .sync_slave_g3            = (bool) true,
  .sync_slave_g0            = (bool) false,
  .group_index              = (uint8_t) 0, /* VADC Group-0 is acting as Master Group */
  .total_number_of_entries  = (uint8_t) 1,
  .total_number_of_channels = (uint8_t) 1,
  .start_at_initialization  = (bool) true,
  .configure_globiclass1    = (bool) true
};

/**********************************************************************************************************************/

/* This function would be called in the ADC_MEASUREMENT_Init() to initialize the SR lines of the
 * Result event/channel event.
 */
static void ADC_MEASUREMENT_ADV_G2_event_config(void)
{

	/* Result Event Node Mux Configuration for IF1_Q (Group-1 channel-6 Result_Register-3)*/
	XMC_VADC_GROUP_SetResultInterruptNode(VADC_G1, 3U, XMC_VADC_SR_SHARED_SR0);

}



/********************************* IF1_Q ADC Channel configuration structure ********************************/
static const XMC_VADC_CHANNEL_CONFIG_t  ADC_MEASUREMENT_ADV_G2_IF1_Q_ch_config =
{
  .input_class                = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
  .lower_boundary_select      = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .upper_boundary_select      = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .event_gen_criteria         = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
  .sync_conversion            = (uint32_t) true,                            /* Sync feature disabled*/
  .alternate_reference        = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,  /* Internal reference selected */
  .result_reg_number          = (uint32_t) 3,                           /* GxRES[3] selected */
  .result_alignment           = (uint32_t) XMC_VADC_RESULT_ALIGN_RIGHT,  /* Result alignment - Right Aligned*/
  .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND, /* No Broken wire mode select*/
  .broken_wire_detect         = (uint32_t) false,                     /* No Broken wire detection*/
  .bfl                        = (uint32_t) 0,                            /* No Boundary flag */
  .channel_priority           = (uint32_t) true,                      /* Highest Priority: 1 selected*/
  .alias_channel              = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,   /* ALIAS is Disabled*/
};


/*IF1_Q Result configuration structure*/
static const XMC_VADC_RESULT_CONFIG_t ADC_MEASUREMENT_ADV_G2_IF1_Q_res_config =
{
  .data_reduction_control  = (uint32_t) 0,
  .post_processing_mode    = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
  .wait_for_read_mode      = (uint32_t) false,
  .part_of_fifo            = (uint32_t) false , /* No FIFO */
  .event_gen_enable        = (uint32_t) true   /* Result event Enabled*/
};

/* IF1_Q ADC channel Handle */
const ADC_MEASUREMENT_ADV_CHANNEL_t ADC_MEASUREMENT_ADV_G2_IF1_Q_handle =
{
  .ch_num        = (uint8_t) 6,
  .group_index   = (uint8_t) 1,
  .ch_handle     = (XMC_VADC_CHANNEL_CONFIG_t*) &ADC_MEASUREMENT_ADV_G2_IF1_Q_ch_config,
  .res_handle[0] = (XMC_VADC_RESULT_CONFIG_t*) &ADC_MEASUREMENT_ADV_G2_IF1_Q_res_config,
};


static const XMC_VADC_QUEUE_CONFIG_t ADC_MEASUREMENT_ADV_G2_queue_config = {
  .conv_start_mode   = (uint32_t) XMC_VADC_STARTMODE_WFS,	/* Conversion start mode WFS/CIR/CNR*/
  .req_src_priority  = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_3, /*The queue request source priority */
  .trigger_signal    = (uint32_t) XMC_VADC_REQ_TR_F,        /*If trigger needed the signal input*/
  .trigger_edge      = (uint32_t) XMC_VADC_TRIGGER_EDGE_RISING,   /*Trigger edge needed if trigger enabled*/
  .gate_signal       = (uint32_t) XMC_VADC_REQ_GT_A,            /*If gating needed the signal input*/
  .timer_mode        = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger	 = (uint32_t) true,         /*External trigger Enabled/Disabled*/
};

static const ADC_MEASUREMENT_ADV_QUEUE_t ADC_MEASUREMENT_ADV_G2_queue_handle =
{
  .rs_intr_handle  = {
					   .node_id          = (uint32_t) 0,
					   .priority         = (uint32_t) 0,
             .sub_priority     = 0,
					   .interrupt_enable = (bool) false,
					 },
  .iclass_config_handle = {
							.sample_time_std_conv        = (uint32_t) 0,
							.conversion_mode_standard    = (uint32_t) XMC_VADC_CONVMODE_12BIT,
							.sampling_phase_emux_channel = (uint32_t) 0,
							.conversion_mode_emux        = (uint32_t) XMC_VADC_CONVMODE_12BIT
						  },
  .queue_config_handle = &ADC_MEASUREMENT_ADV_G2_queue_config,
  .gating_mode  = XMC_VADC_GATEMODE_IGNORE,
  .srv_req_node = XMC_VADC_SR_GROUP_SR0, /*Service Request line for Request source Event */
  .iclass_num = 0
};

static const XMC_VADC_QUEUE_ENTRY_t ADC_MEASUREMENT_ADV_G2_IF1_Q_queue_entry_0 =
{
  .channel_num        = (uint8_t) 6,
  .refill_needed      = (bool) true,
  .generate_interrupt = (bool) false,
  .external_trigger   = (bool) true,
};


static const XMC_VADC_QUEUE_ENTRY_t *ADC_MEASUREMENT_ADV_G2_queue_entries[] =
{
  (XMC_VADC_QUEUE_ENTRY_t*) (void *) &ADC_MEASUREMENT_ADV_G2_IF1_Q_queue_entry_0,
};



static const ADC_MEASUREMENT_ADV_CHANNEL_t *ADC_MEASUREMENT_ADV_G2_channel_array[] =
{
  (ADC_MEASUREMENT_ADV_CHANNEL_t *) (void*) &ADC_MEASUREMENT_ADV_G2_IF1_Q_handle,

};

static ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_G2_initialization_status = ADC_MEASUREMENT_ADV_STATUS_UNINITIALIZED;

const ADC_MEASUREMENT_ADV_t ADC_MEASUREMENT_ADV_G2 =
{
  .channel_array            = ADC_MEASUREMENT_ADV_G2_channel_array,
  .local_queue_entries       = ADC_MEASUREMENT_ADV_G2_queue_entries,
  .event_config               = (ADC_MEASUREMENT_ADV_G2_event_config),
  .local_queue_handle       = (ADC_MEASUREMENT_ADV_QUEUE_t *) &ADC_MEASUREMENT_ADV_G2_queue_handle,
  .init_state               = &ADC_MEASUREMENT_ADV_G2_initialization_status,
  .req_src                  = ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_QUEUE,
  .sync_slave_g0            = (bool) false,
  .sync_slave_g1            = (bool) false,
  .sync_slave_g2            = (bool) false,
  .sync_slave_g3            = (bool) false,
  .group_index              = (uint8_t) 1,
  .total_number_of_entries  = (uint8_t) 1,
  .total_number_of_channels = (uint8_t) 1,
  .start_at_initialization  = (bool) true,
  .configure_globiclass1    = (bool) true
};

/**********************************************************************************************************************/

/* This function would be called in the ADC_MEASUREMENT_Init() to initialize the SR lines of the
 * Result event/channel event.
 */
static void ADC_MEASUREMENT_ADV_G3_event_config(void)
{

	/* Result Event Node Mux Configuration for IF2_I (Group-2 channel-3 Result_Register-0)*/
	XMC_VADC_GROUP_SetResultInterruptNode(VADC_G2, 0U, XMC_VADC_SR_SHARED_SR3);

}



/********************************* IF2_I ADC Channel configuration structure ********************************/
static const XMC_VADC_CHANNEL_CONFIG_t  ADC_MEASUREMENT_ADV_G3_IF2_I_ch_config =
{
  .input_class                = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
  .lower_boundary_select      = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .upper_boundary_select      = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .event_gen_criteria         = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
  .sync_conversion            = (uint32_t) true,                            /* Sync feature disabled*/
  .alternate_reference        = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,  /* Internal reference selected */
  .result_reg_number          = (uint32_t) 0,                           /* GxRES[0] selected */
  .result_alignment           = (uint32_t) XMC_VADC_RESULT_ALIGN_RIGHT,  /* Result alignment - Right Aligned*/
  .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND, /* No Broken wire mode select*/
  .broken_wire_detect         = (uint32_t) false,                     /* No Broken wire detection*/
  .bfl                        = (uint32_t) 0,                            /* No Boundary flag */
  .channel_priority           = (uint32_t) true,                      /* Highest Priority: 1 selected*/
  .alias_channel              = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,   /* ALIAS is Disabled*/
};


/*IF2_I Result configuration structure*/
static const XMC_VADC_RESULT_CONFIG_t ADC_MEASUREMENT_ADV_G3_IF2_I_res_config =
{
  .data_reduction_control  = (uint32_t) 0,
  .post_processing_mode    = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
  .wait_for_read_mode      = (uint32_t) false,
  .part_of_fifo            = (uint32_t) false , /* No FIFO */
  .event_gen_enable        = (uint32_t) true   /* Result event Enabled*/
};

/* IF2_I ADC channel Handle */
const ADC_MEASUREMENT_ADV_CHANNEL_t ADC_MEASUREMENT_ADV_G3_IF2_I_handle =
{
  .ch_num        = (uint8_t) 3,
  .group_index   = (uint8_t) 2,
  .ch_handle     = (XMC_VADC_CHANNEL_CONFIG_t*) &ADC_MEASUREMENT_ADV_G3_IF2_I_ch_config,
  .res_handle[0] = (XMC_VADC_RESULT_CONFIG_t*) &ADC_MEASUREMENT_ADV_G3_IF2_I_res_config,
};


static const XMC_VADC_QUEUE_CONFIG_t ADC_MEASUREMENT_ADV_G3_queue_config = {
  .conv_start_mode   = (uint32_t) XMC_VADC_STARTMODE_WFS,	/* Conversion start mode WFS/CIR/CNR*/
  .req_src_priority  = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_3, /*The queue request source priority */
  .trigger_signal    = (uint32_t) XMC_VADC_REQ_TR_F,        /*If trigger needed the signal input*/
  .trigger_edge      = (uint32_t) XMC_VADC_TRIGGER_EDGE_RISING,   /*Trigger edge needed if trigger enabled*/
  .gate_signal       = (uint32_t) XMC_VADC_REQ_GT_A,            /*If gating needed the signal input*/
  .timer_mode        = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger	 = (uint32_t) true,         /*External trigger Enabled/Disabled*/
};

static const ADC_MEASUREMENT_ADV_QUEUE_t ADC_MEASUREMENT_ADV_G3_queue_handle =
{
  .rs_intr_handle  = {
					   .node_id          = (uint32_t) 0,
					   .priority         = (uint32_t) 0,
             .sub_priority     = 0,
					   .interrupt_enable = (bool) false,
					 },
  .iclass_config_handle = {
							.sample_time_std_conv        = (uint32_t) 0,
							.conversion_mode_standard    = (uint32_t) XMC_VADC_CONVMODE_12BIT,
							.sampling_phase_emux_channel = (uint32_t) 0,
							.conversion_mode_emux        = (uint32_t) XMC_VADC_CONVMODE_12BIT
						  },
  .queue_config_handle = &ADC_MEASUREMENT_ADV_G3_queue_config,
  .gating_mode  = XMC_VADC_GATEMODE_IGNORE,
  .srv_req_node = XMC_VADC_SR_GROUP_SR0, /*Service Request line for Request source Event */
  .iclass_num = 0
};

static const XMC_VADC_QUEUE_ENTRY_t ADC_MEASUREMENT_ADV_G3_IF2_I_queue_entry_0 =
{
  .channel_num        = (uint8_t) 3,
  .refill_needed      = (bool) true,
  .generate_interrupt = (bool) false,
  .external_trigger   = (bool) true,
};


static const XMC_VADC_QUEUE_ENTRY_t *ADC_MEASUREMENT_ADV_G3_queue_entries[] =
{
  (XMC_VADC_QUEUE_ENTRY_t*) (void *) &ADC_MEASUREMENT_ADV_G3_IF2_I_queue_entry_0,
};



static const ADC_MEASUREMENT_ADV_CHANNEL_t *ADC_MEASUREMENT_ADV_G3_channel_array[] =
{
  (ADC_MEASUREMENT_ADV_CHANNEL_t *) (void*) &ADC_MEASUREMENT_ADV_G3_IF2_I_handle,

};

static ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_G3_initialization_status = ADC_MEASUREMENT_ADV_STATUS_UNINITIALIZED;

const ADC_MEASUREMENT_ADV_t ADC_MEASUREMENT_ADV_G3 =
{
  .channel_array            = ADC_MEASUREMENT_ADV_G3_channel_array,
  .local_queue_entries       = ADC_MEASUREMENT_ADV_G3_queue_entries,
  .event_config               = (ADC_MEASUREMENT_ADV_G3_event_config),
  .local_queue_handle       = (ADC_MEASUREMENT_ADV_QUEUE_t *) &ADC_MEASUREMENT_ADV_G3_queue_handle,
  .init_state               = &ADC_MEASUREMENT_ADV_G3_initialization_status,
  .req_src                  = ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_QUEUE,
  .sync_slave_g0            = (bool) false,
  .sync_slave_g1            = (bool) false,
  .sync_slave_g2            = (bool) false,
  .sync_slave_g3            = (bool) false,
  .group_index              = (uint8_t) 2,
  .total_number_of_entries  = (uint8_t) 1,
  .total_number_of_channels = (uint8_t) 1,
  .start_at_initialization  = (bool) true,
  .configure_globiclass1    = (bool) true
};

/**********************************************************************************************************************/

/* This function would be called in the ADC_MEASUREMENT_Init() to initialize the SR lines of the
 * Result event/channel event.
 */
static void ADC_MEASUREMENT_ADV_G4_event_config(void)
{

	/* Result Event Node Mux Configuration for IF2_Q (Group-3 channel-0 Result_Register-4)*/
	XMC_VADC_GROUP_SetResultInterruptNode(VADC_G3, 4U, XMC_VADC_SR_GROUP_SR2);

}



/********************************* IF2_Q ADC Channel configuration structure ********************************/
static const XMC_VADC_CHANNEL_CONFIG_t  ADC_MEASUREMENT_ADV_G4_IF2_Q_ch_config =
{
  .input_class                = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS1,
  .lower_boundary_select      = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .upper_boundary_select      = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .event_gen_criteria         = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
  .sync_conversion            = (uint32_t) true,                            /* Sync feature disabled*/
  .alternate_reference        = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,  /* Internal reference selected */
  .result_reg_number          = (uint32_t) 4,                           /* GxRES[4] selected */
  .result_alignment           = (uint32_t) XMC_VADC_RESULT_ALIGN_RIGHT,  /* Result alignment - Right Aligned*/
  .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND, /* No Broken wire mode select*/
  .broken_wire_detect         = (uint32_t) false,                     /* No Broken wire detection*/
  .bfl                        = (uint32_t) 0,                            /* No Boundary flag */
  .channel_priority           = (uint32_t) true,                      /* Highest Priority: 1 selected*/
  .alias_channel              = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,   /* ALIAS is Disabled*/
};


/*IF2_Q Result configuration structure*/
static const XMC_VADC_RESULT_CONFIG_t ADC_MEASUREMENT_ADV_G4_IF2_Q_res_config =
{
  .data_reduction_control  = (uint32_t) 0,
  .post_processing_mode    = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
  .wait_for_read_mode      = (uint32_t) false,
  .part_of_fifo            = (uint32_t) false , /* No FIFO */
  .event_gen_enable        = (uint32_t) true   /* Result event Enabled*/
};

/* IF2_Q ADC channel Handle */
const ADC_MEASUREMENT_ADV_CHANNEL_t ADC_MEASUREMENT_ADV_G4_IF2_Q_handle =
{
  .ch_num        = (uint8_t) 0,
  .group_index   = (uint8_t) 3,
  .ch_handle     = (XMC_VADC_CHANNEL_CONFIG_t*) &ADC_MEASUREMENT_ADV_G4_IF2_Q_ch_config,
  .res_handle[0] = (XMC_VADC_RESULT_CONFIG_t*) &ADC_MEASUREMENT_ADV_G4_IF2_Q_res_config,
};


static const XMC_VADC_QUEUE_CONFIG_t ADC_MEASUREMENT_ADV_G4_queue_config = {
  .conv_start_mode   = (uint32_t) XMC_VADC_STARTMODE_WFS,	/* Conversion start mode WFS/CIR/CNR*/
  .req_src_priority  = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_3, /*The queue request source priority */
  .trigger_signal    = (uint32_t) XMC_VADC_REQ_TR_F,        /*If trigger needed the signal input*/
  .trigger_edge      = (uint32_t) XMC_VADC_TRIGGER_EDGE_RISING,   /*Trigger edge needed if trigger enabled*/
  .gate_signal       = (uint32_t) XMC_VADC_REQ_GT_A,            /*If gating needed the signal input*/
  .timer_mode        = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger	 = (uint32_t) true,         /*External trigger Enabled/Disabled*/
};

static const ADC_MEASUREMENT_ADV_QUEUE_t ADC_MEASUREMENT_ADV_G4_queue_handle =
{
  .rs_intr_handle  = {
					   .node_id          = (uint32_t) 0,
					   .priority         = (uint32_t) 0,
             .sub_priority     = 0,
					   .interrupt_enable = (bool) false,
					 },
  .iclass_config_handle = {
							.sample_time_std_conv        = (uint32_t) 0,
							.conversion_mode_standard    = (uint32_t) XMC_VADC_CONVMODE_12BIT,
							.sampling_phase_emux_channel = (uint32_t) 0,
							.conversion_mode_emux        = (uint32_t) XMC_VADC_CONVMODE_12BIT
						  },
  .queue_config_handle = &ADC_MEASUREMENT_ADV_G4_queue_config,
  .gating_mode  = XMC_VADC_GATEMODE_IGNORE,
  .srv_req_node = XMC_VADC_SR_GROUP_SR0, /*Service Request line for Request source Event */
  .iclass_num = 1
};

static const XMC_VADC_QUEUE_ENTRY_t ADC_MEASUREMENT_ADV_G4_IF2_Q_queue_entry_0 =
{
  .channel_num        = (uint8_t) 0,
  .refill_needed      = (bool) true,
  .generate_interrupt = (bool) false,
  .external_trigger   = (bool) true,
};


static const XMC_VADC_QUEUE_ENTRY_t *ADC_MEASUREMENT_ADV_G4_queue_entries[] =
{
  (XMC_VADC_QUEUE_ENTRY_t*) (void *) &ADC_MEASUREMENT_ADV_G4_IF2_Q_queue_entry_0,
};



static const ADC_MEASUREMENT_ADV_CHANNEL_t *ADC_MEASUREMENT_ADV_G4_channel_array[] =
{
  (ADC_MEASUREMENT_ADV_CHANNEL_t *) (void*) &ADC_MEASUREMENT_ADV_G4_IF2_Q_handle,

};

static ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_G4_initialization_status = ADC_MEASUREMENT_ADV_STATUS_UNINITIALIZED;

const ADC_MEASUREMENT_ADV_t ADC_MEASUREMENT_ADV_G4 =
{
  .channel_array            = ADC_MEASUREMENT_ADV_G4_channel_array,
  .local_queue_entries       = ADC_MEASUREMENT_ADV_G4_queue_entries,
  .event_config               = (ADC_MEASUREMENT_ADV_G4_event_config),
  .local_queue_handle       = (ADC_MEASUREMENT_ADV_QUEUE_t *) &ADC_MEASUREMENT_ADV_G4_queue_handle,
  .init_state               = &ADC_MEASUREMENT_ADV_G4_initialization_status,
  .req_src                  = ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_QUEUE,
  .sync_slave_g0            = (bool) false,
  .sync_slave_g1            = (bool) false,
  .sync_slave_g2            = (bool) false,
  .sync_slave_g3            = (bool) false,
  .group_index              = (uint8_t) 3,
  .total_number_of_entries  = (uint8_t) 1,
  .total_number_of_channels = (uint8_t) 1,
  .start_at_initialization  = (bool) true,
  .configure_globiclass1    = (bool) true
};

/**********************************************************************************************************************/


/********************************* BGT24_ANA ADC Channel configuration structure ********************************/
static const XMC_VADC_CHANNEL_CONFIG_t  ADC_MEASUREMENT_SCAN_BGT24_ANA_ch_config =
{
  .input_class                = (uint32_t) XMC_VADC_CHANNEL_CONV_GROUP_CLASS0,
  .lower_boundary_select      = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .upper_boundary_select      = (uint32_t) XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0,
  .event_gen_criteria         = (uint32_t) XMC_VADC_CHANNEL_EVGEN_NEVER,
  .sync_conversion            = (uint32_t) false,                            /* Sync feature disabled*/
  .alternate_reference        = (uint32_t) XMC_VADC_CHANNEL_REF_INTREF,  /* Internal reference selected */
  .result_reg_number          = (uint32_t) 9,                           /* GxRES[9] selected */
  .result_alignment           = (uint32_t) XMC_VADC_RESULT_ALIGN_RIGHT,  /* Result alignment - Right Aligned*/
  .broken_wire_detect_channel = (uint32_t) XMC_VADC_CHANNEL_BWDCH_VAGND, /* No Broken wire mode select*/
  .broken_wire_detect         = (uint32_t) false,                     /* No Broken wire detection*/
  .bfl                        = (uint32_t) 0,                            /* No Boundary flag */
  .channel_priority           = (uint32_t) true,                      /* Highest Priority: 1 selected*/
  .alias_channel              = (int8_t) XMC_VADC_CHANNEL_ALIAS_DISABLED,   /* ALIAS is Disabled*/
};


/*BGT24_ANA Result configuration structure*/
static const XMC_VADC_RESULT_CONFIG_t ADC_MEASUREMENT_SCAN_BGT24_ANA_res_config =
{
  .data_reduction_control  = (uint32_t) 0,
  .post_processing_mode    = (uint32_t) XMC_VADC_DMM_REDUCTION_MODE,
  .wait_for_read_mode      = (uint32_t) false,
  .part_of_fifo            = (uint32_t) false , /* No FIFO */
  .event_gen_enable        = (uint32_t) false   /* Disable Result event */
};

/* BGT24_ANA ADC channel Handle */
const ADC_MEASUREMENT_ADV_CHANNEL_t ADC_MEASUREMENT_SCAN_BGT24_ANA_handle =
{
  .ch_num        = (uint8_t) 5,
  .group_index   = (uint8_t) 3,
  .ch_handle     = (XMC_VADC_CHANNEL_CONFIG_t*) &ADC_MEASUREMENT_SCAN_BGT24_ANA_ch_config,
  .res_handle[0] = (XMC_VADC_RESULT_CONFIG_t*) &ADC_MEASUREMENT_SCAN_BGT24_ANA_res_config,
};


static const XMC_VADC_SCAN_CONFIG_t ADC_MEASUREMENT_SCAN_scan_config = {
  .conv_start_mode   = (uint32_t) XMC_VADC_STARTMODE_WFS,     /* Conversion start mode WFS/CIR/CNR*/
  .req_src_priority  = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_0, /*The scan request source priority */
  .trigger_signal    = (uint32_t) XMC_VADC_REQ_TR_A,      /*If trigger needed the signal input*/
  .trigger_edge      = (uint32_t) XMC_VADC_TRIGGER_EDGE_NONE,   /*Trigger edge needed if trigger enabled*/
  .gate_signal       = (uint32_t) XMC_VADC_REQ_GT_A,            /*If gating needed the signal input*/
  .timer_mode        = (uint32_t) 0,                            /* Disabled equidistant sampling*/
  .external_trigger	 = (uint32_t) false,          /*External trigger Enabled/Disabled*/
  .req_src_interrupt = (uint32_t) false,               /*Request source event Enabled/disabled*/
  .enable_auto_scan  = (uint32_t) false,  /*Autoscan enabled/disabled */
  .load_mode          = (uint32_t) XMC_VADC_SCAN_LOAD_COMBINE,   /*Response from SCAN when a Load event occours.*/
};

static const ADC_MEASUREMENT_ADV_SCAN_t ADC_MEASUREMENT_SCAN_scan_handle =
{
  .rs_intr_handle  = {
					   .node_id          = 0,
					   .priority         = 0,
					   .sub_priority     = 0,
					   .interrupt_enable = (bool) false,
					 },
  .iclass_config_handle = {
							.sample_time_std_conv        = (uint32_t) 0,
							.conversion_mode_standard    = (uint32_t) XMC_VADC_CONVMODE_12BIT,
							.sampling_phase_emux_channel = (uint32_t) 0,
							.conversion_mode_emux        = (uint32_t) XMC_VADC_CONVMODE_12BIT
						  },
  .scan_config_handle = &ADC_MEASUREMENT_SCAN_scan_config,
  .gating_mode  = XMC_VADC_GATEMODE_IGNORE,
  .srv_req_node = XMC_VADC_SR_GROUP_SR0, /*Service Request line for Request source Event */
  .insert_mask  = 32,
  .iclass_num = 0
};


static const ADC_MEASUREMENT_ADV_CHANNEL_t *ADC_MEASUREMENT_SCAN_channel_array[] =
{
  (ADC_MEASUREMENT_ADV_CHANNEL_t *) (void*) &ADC_MEASUREMENT_SCAN_BGT24_ANA_handle,

};

static ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_SCAN_initialization_status = ADC_MEASUREMENT_ADV_STATUS_UNINITIALIZED;

const ADC_MEASUREMENT_ADV_t ADC_MEASUREMENT_SCAN =
{
  .channel_array            = ADC_MEASUREMENT_SCAN_channel_array,

  .event_config               = NULL,
  .local_scan_handle        = (ADC_MEASUREMENT_ADV_SCAN_t *) (void *) &ADC_MEASUREMENT_SCAN_scan_handle,
  .init_state               = &ADC_MEASUREMENT_SCAN_initialization_status,
  .req_src                  = ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_SCAN,
  .sync_slave_g0            = (bool) false,
  .sync_slave_g1            = (bool) false,
  .sync_slave_g2            = (bool) false,
  .sync_slave_g3            = (bool) false,
  .group_index              = (uint8_t) 3,
  .total_number_of_entries  = (uint8_t) 1,
  .total_number_of_channels = (uint8_t) 1,
  .start_at_initialization  = (bool) true,
  .configure_globiclass1    = (bool) false
};


/* Pointer to the VADC GLOBAL*/
#define ADC_MEASUREMENT_ADV_GLOBAL_PTR ((XMC_VADC_GLOBAL_t *) (void *) VADC)

/* Max value possible with 10 bit resolution is 1023*/
#define ADC_MEASUREMENT_ADV_10_BIT_MAX_VALUE ((uint32_t)1023)

#if (XMC_VADC_SHS_AVAILABLE == 1U)
  /* Pointer to the SHS unit */
  #define ADC_MEASUREMENT_ADV_SHS_PTR ((XMC_VADC_GLOBAL_SHS_t *) (void *) SHS0)
#endif

/** One result register is reserved for FIFO tail and for the FIFO head.
 * This needs to be subtracted from the total required FIFO stages. These head and tail result registers needs different
 * settings based on the UI. The macro is hence helpful to determine the number of intermediate stages for the FIFO
 * buffer. Each of the intermediate stages need to only set the GxRCR.FEN bit.
 */
#define ADC_MEASUREMENT_ADV_RESERVED_REGISTERS ((uint32_t)2)

/** The number of result registers needed for each channel depends on the FIFO selected.
 * Always the ADC_MEASUREMENT_ADV_CHANNEL_t::res_handle varies with the FIFO selection.
 * This Macro depicts the configuration for the Head of the FIFO i.e ADC_MEASUREMENT_ADV_CHANNEL_t::res_handle[0].
 * It always represents the head result register(with FIFO) or the result register (without FIFO) configuration.
 */
#define ADC_MEASUREMENT_ADV_HEAD_RESULT_REG_CONFIG ((uint32_t)0)

/** The number of result registers needed for each channel depends on the FIFO selected.
 * Always the ADC_MEASUREMENT_ADV_CHANNEL_t::res_handle varies with the FIFO selection.
 * This Macro depicts the configuration for the tail of the FIFO i.e ADC_MEASUREMENT_ADV_CHANNEL_t::res_handle[1].
 * It always represents the tail result register of the FIFO.
 */
#define ADC_MEASUREMENT_ADV_TAIL_RESULT_REG_CONFIG ((uint32_t)1)

/* Configure the slave input class as global input class 1*/
#define ADC_MEASUREMENT_ADV_GLOBICLASS1 ((uint32_t)1)


/* Since the SCU is different for various devices a macro is defined here to enable check of clock-ungating*/
#if UC_FAMILY == XMC1
  #define ADC_MEASUREMENT_ADV_CHECK_CLOCK_GATING ((uint32_t)1)
#endif

/* Array of Group pointers*/
XMC_VADC_GROUP_t *const group_ptrs[XMC_VADC_MAXIMUM_NUM_GROUPS] =
{
   (VADC_G_TypeDef*)(void*) VADC_G0,
   (VADC_G_TypeDef*)(void*) VADC_G1
#if (XMC_VADC_MAXIMUM_NUM_GROUPS > 2U)
  ,(VADC_G_TypeDef*)(void*) VADC_G2,
   (VADC_G_TypeDef*)(void*) VADC_G3
#endif
};

#ifdef ADC_MEASUREMENT_ADV_FIFO_USED
/**
* Intermediate FIFO stages needs only the GxRCR.FEN bit to be set.Hence this is generated as a static constant.
*/
static const XMC_VADC_RESULT_CONFIG_t ADC_MEASUREMENT_ADV_fifo_intermediate_stage =
{
   .data_reduction_control = 0,
   .post_processing_mode   = 0,
   .wait_for_read_mode     = 0,
   .part_of_fifo           = (bool)true,
   .event_gen_enable       = 0
};
#endif

/*Anonymous structure/union guard start*/
#if defined(__CC_ARM)
 #pragma push
 #pragma anon_unions
#elif defined(__TASKING__)
 #pragma warning 586
#endif

/* Private structure to determine the ALIAS*/
typedef struct ADC_MEASUREMENT_ADV_ALIAS
{
 union
 {
   struct
   {
     uint32_t alias0 : 5; /* ALIAS for Channel 0*/
     uint32_t        : 3;
     uint32_t alias1 : 5; /* ALIAS for channel 1*/
     uint32_t        : 19;
   };
   uint32_t alias;
 };
}ADC_MEASUREMENT_ADV_ALIAS_t;

/*Anonymous structure/union guard end*/
#if defined(__CC_ARM)
 #pragma pop
#elif defined(__TASKING__)
 #pragma warning restore
#endif

/******************************************************************************
   3. FUNCTIONS
*******************************************************************************/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * This function initializes all instances of the ADC Global APP and low level app.
 */
GLOBAL_ADC_STATUS_t GLOBAL_ADC_Init(GLOBAL_ADC_t *const handle_ptr)
{
  XMC_ASSERT("GLOBAL_ADC_Init:Invalid handle_ptr", (handle_ptr != NULL))
#if (XMC_VADC_GROUP_AVAILABLE == 1U)
  uint32_t group_index;
#endif

  if (GLOBAL_ADC_UNINITIALIZED == handle_ptr->init_state)
  {
    /* Initialize an instance of Global hardware */
    XMC_VADC_GLOBAL_Init(handle_ptr->module_ptr, handle_ptr->global_config_handle);

    /* Initialize all the Groups */
#if (XMC_VADC_GROUP_AVAILABLE == 1U)
    for(group_index = (uint32_t)0; group_index < XMC_VADC_MAXIMUM_NUM_GROUPS; group_index++)
    {
      /*Initialize Group*/
      XMC_VADC_GROUP_Init(handle_ptr->group_ptrs_array[group_index]->group_handle,
    		            handle_ptr->group_ptrs_array[group_index]->group_config_handle);

      /* Switch on the converter of the Group[group_index]*/
      XMC_VADC_GROUP_SetPowerMode(handle_ptr->group_ptrs_array[group_index]->group_handle,
                                  XMC_VADC_GROUP_POWERMODE_NORMAL);

      /* Disable the post calibration option for the respective group*/
      if ((bool)false == handle_ptr->group_ptrs_array[group_index]->post_calibration)
      {
        XMC_VADC_GLOBAL_DisablePostCalibration(handle_ptr->module_ptr,group_index);
      }

#if(XMC_VADC_SHS_AVAILABLE == 1U)
      XMC_VADC_GLOBAL_SHS_EnableAcceleratedMode(handle_ptr->global_shs_ptr, (XMC_VADC_GROUP_INDEX_t)group_index);
#endif

      handle_ptr->group_ptrs_array[group_index]->state = GLOBAL_ADC_SUCCESS;
    }
#if(XMC_VADC_SHS_AVAILABLE == 1U)
    XMC_VADC_GLOBAL_SHS_SetClockDivider(handle_ptr->global_shs_ptr, (uint8_t) GLOBAL_ADC_DIVS_VALUE);
#endif
#endif /* _XMC_VADC_GROUP_AVAILABLE_ */
#if(XMC_VADC_SHS_AVAILABLE == 1U)
    XMC_VADC_GLOBAL_SHS_SetAnalogReference(handle_ptr->global_shs_ptr, GLOBAL_ADC_AREF_VALUE);
#endif
    if((bool)true == handle_ptr->enable_startup_calibration)
    {
    	XMC_VADC_GLOBAL_StartupCalibration(handle_ptr->module_ptr);
    }
    handle_ptr->init_state = GLOBAL_ADC_SUCCESS;
  }
  return (handle_ptr->init_state);
}



#if defined(ADC_MEASUREMENT_ADV_ADC_QUEUE_USED) || defined (ADC_MEASUREMENT_ADV_ADC_SCAN_USED)
/* Local function to insert an entry into the H/W*/
__STATIC_INLINE void ADC_MEASUREMENT_ADV_lInsertEntry(const ADC_MEASUREMENT_ADV_t *const handle_ptr, uint8_t ch_num)
{
#if defined( ADC_MEASUREMENT_ADV_SCAN_USED) && defined(ADC_MEASUREMENT_ADV_ADC_SCAN_USED)
  #if defined (ADC_MEASUREMENT_ADV_QUEUE_USED) && defined(ADC_MEASUREMENT_ADV_ADC_QUEUE_USED)
    if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_SCAN == handle_ptr->req_src)
  #endif
    {
      ADC_SCAN_InsertScanEntry(handle_ptr->scan_handle, handle_ptr->scan_entries[ch_num]);
    }
#endif
#if defined(ADC_MEASUREMENT_ADV_QUEUE_USED) && defined(ADC_MEASUREMENT_ADV_ADC_QUEUE_USED)
  #if defined(ADC_MEASUREMENT_ADV_SCAN_USED) && defined(ADC_MEASUREMENT_ADV_ADC_SCAN_USED)
    else
  #endif
    {
      ADC_QUEUE_InsertQueueEntry(handle_ptr->queue_handle, handle_ptr->queue_entries[ch_num]);
    }
#endif
}
#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
XMC_VADC_CHANNEL_CONV_t  ADC_MEASUREMENT_ADV_lGetIclass(const ADC_MEASUREMENT_ADV_t *const handle_ptr)
{
  XMC_VADC_CHANNEL_CONV_t req_iclass;

#ifdef ADC_MEASUREMENT_ADV_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
    if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_SCAN >= handle_ptr->req_src)
  #endif
    {
#ifdef ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_ADC_SCAN_USED
      if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_SCAN != handle_ptr->req_src)
  #endif
      {
        req_iclass = (XMC_VADC_CHANNEL_CONV_t)handle_ptr->local_scan_handle->iclass_num;
      }
#endif

#ifdef ADC_MEASUREMENT_ADV_ADC_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED
      else
  #endif
      {
        /* Call the function to initialise Clock and ADC global functional units*/
        req_iclass = (XMC_VADC_CHANNEL_CONV_t)handle_ptr->scan_handle->iclass_num;
      }
#endif
    }
#endif

#ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_SCAN_USED
    else
  #endif
    {
#ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_ADC_QUEUE_USED
      if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_QUEUE != handle_ptr->req_src)
  #endif
      {
        req_iclass = (XMC_VADC_CHANNEL_CONV_t)handle_ptr->local_queue_handle->iclass_num;
      }
#endif

#ifdef ADC_MEASUREMENT_ADV_ADC_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
      else
  #endif
      {
        req_iclass = (XMC_VADC_CHANNEL_CONV_t)handle_ptr->queue_handle->iclass_num;
      }
#endif
    }
#endif
    return (req_iclass);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#ifdef ADC_MEASUREMENT_ADV_SYNC_USED

/* Helper function to configure the eval bits in the slave*/
void ADC_MEASUREMENT_ADV_lSyncEvalConfig(uint32_t master_group, uint32_t slave_selected,
                                         uint32_t sync_group)
{
  int8_t group_index;
  sync_group |= (1U << master_group);
  sync_group &= ~(1U << slave_selected);
  for( group_index = XMC_VADC_MAXIMUM_NUM_GROUPS - (int32_t)1; group_index >= (int32_t)0  ; group_index--)
  {
    if ( (bool)false != (bool)((sync_group >> group_index) & 0x1 ))
    {
      XMC_VADC_GROUP_SetSyncSlaveReadySignal(group_ptrs[slave_selected], slave_selected, group_index);
    }
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Helper function to execute the sync init sequence*/
void ADC_MEASUREMENT_ADV_lSyncSequencer(const ADC_MEASUREMENT_ADV_t *const handle_ptr,
                                        uint32_t sync_group,
                                        ADC_MEASUREMENT_ADV_SYNC_SEQ_t sequence)
{
  int8_t group_index;
  for( group_index = XMC_VADC_MAXIMUM_NUM_GROUPS - (int32_t)1; group_index >= (int32_t)0  ; group_index--)
  {
    if ( (bool)false != (bool)((sync_group >> group_index) & 0x1 ))
    {
      switch( sequence)
      {
        case ADC_MEASUREMENT_ADV_SYNC_SEQ_POWER_DOWN:
             XMC_VADC_GROUP_SetPowerMode(group_ptrs[group_index],XMC_VADC_GROUP_POWERMODE_OFF);
             break;
        case ADC_MEASUREMENT_ADV_SYNC_SEQ_STSEL_CONFIG:
             XMC_VADC_GROUP_SetSyncSlave(group_ptrs[group_index], handle_ptr->group_index, group_index);
             XMC_VADC_GROUP_CheckSlaveReadiness(group_ptrs[handle_ptr->group_index],group_index);
             break;
        case ADC_MEASUREMENT_ADV_SYNC_SEQ_EVAL_CONFIG:
             ADC_MEASUREMENT_ADV_lSyncEvalConfig(handle_ptr->group_index, group_index, sync_group);
        default:
             break;
      }
    }
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Initialization of the all the sync related functions */
__STATIC_INLINE void ADC_MEASUREMENT_ADV_lSyncInit(const ADC_MEASUREMENT_ADV_t *const handle_ptr)
{
  uint8_t sync_group;

  /* shift to get the 4 bit position needed to or it with the slave groups */
  sync_group = handle_ptr->sync_slaves | ( 1 << handle_ptr->group_index);
  ADC_MEASUREMENT_ADV_lSyncSequencer(handle_ptr, sync_group, ADC_MEASUREMENT_ADV_SYNC_SEQ_POWER_DOWN);

  sync_group = handle_ptr->sync_slaves;
  ADC_MEASUREMENT_ADV_lSyncSequencer(handle_ptr, sync_group, ADC_MEASUREMENT_ADV_SYNC_SEQ_STSEL_CONFIG);
  ADC_MEASUREMENT_ADV_lSyncSequencer(handle_ptr, sync_group, ADC_MEASUREMENT_ADV_SYNC_SEQ_EVAL_CONFIG);

  /* Configure the iclass settings needed for the sync slaves*/
  if( (bool) false != handle_ptr->configure_globiclass1)
  {
    ADC_MEASUREMENT_ADV_SetIclass(handle_ptr);
  }

  XMC_VADC_GROUP_SetSyncMaster(group_ptrs[handle_ptr->group_index]);

  XMC_VADC_GROUP_SetPowerMode(group_ptrs[handle_ptr->group_index],XMC_VADC_GROUP_POWERMODE_NORMAL);
}
#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#ifdef ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED
__STATIC_INLINE ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_lScanInit(ADC_MEASUREMENT_ADV_SCAN_t *const handle_ptr,
                                                           uint8_t group_index)
{
  ADC_MEASUREMENT_ADV_STATUS_t status;

  /*Initialization of APP 'ADCGroup'*/
  status = (ADC_MEASUREMENT_ADV_STATUS_t) GLOBAL_ADC_Init(ADC_MEASUREMENT_ADV_GLOBAL_HANDLE);

  XMC_VADC_GROUP_InputClassInit(group_ptrs[group_index], handle_ptr->iclass_config_handle,
                                XMC_VADC_GROUP_CONV_STD, (uint32_t)handle_ptr->iclass_num);


  /*Initialization of scan request source*/
  XMC_VADC_GROUP_ScanInit(group_ptrs[group_index], handle_ptr->scan_config_handle);

  /* Configure the gating mode for Scan*/
  XMC_VADC_GROUP_ScanSetGatingMode(group_ptrs[group_index], handle_ptr->gating_mode);

  /*Interrupt Configuration*/
  if ((bool)true == handle_ptr->rs_intr_handle.interrupt_enable)
  {
#if (UC_FAMILY == XMC1)
    NVIC_SetPriority((IRQn_Type)handle_ptr->rs_intr_handle.node_id, handle_ptr->rs_intr_handle.priority);
#else
    NVIC_SetPriority((IRQn_Type)handle_ptr->rs_intr_handle.node_id,
                      NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                      handle_ptr->rs_intr_handle.priority,handle_ptr->rs_intr_handle.sub_priority));
#endif
#ifdef ADC_MEASUREMENT_ADV_NON_DEFAULT_IRQ_SOURCE_SELECTED
    XMC_SCU_SetInterruptControl(handle_ptr->rs_intr_handle.node_id,
                                ((handle_ptr->rs_intr_handle.node_id << 8) | handle_ptr->rs_intr_handle.irqctrl));
#endif

    /* Connect RS Events to NVIC nodes */
    XMC_VADC_GROUP_ScanSetReqSrcEventInterruptNode(group_ptrs[group_index], handle_ptr->srv_req_node);
  }

  return (status);
}
#endif
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
__STATIC_INLINE ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_lQueueInit(ADC_MEASUREMENT_ADV_QUEUE_t *const handle_ptr,
                                                           uint8_t group_index)
{
  ADC_MEASUREMENT_ADV_STATUS_t status;

  /*Initialization of APP 'GLOBAL_ADC'*/
  status = (ADC_MEASUREMENT_ADV_STATUS_t) GLOBAL_ADC_Init(ADC_MEASUREMENT_ADV_GLOBAL_HANDLE);

  /*Class Configuration*/
  XMC_VADC_GROUP_InputClassInit(group_ptrs[group_index],handle_ptr->iclass_config_handle,
                                XMC_VADC_GROUP_CONV_STD,handle_ptr->iclass_num);

  /* Initialize the Queue hardware */
  XMC_VADC_GROUP_QueueInit(group_ptrs[group_index],handle_ptr->queue_config_handle);

  /* Configure the gating mode for queue*/
  XMC_VADC_GROUP_QueueSetGatingMode(group_ptrs[group_index], handle_ptr->gating_mode);

  /*Interrupt Configuration*/
  if ((bool)true == handle_ptr->rs_intr_handle.interrupt_enable)
  {
#if (UC_FAMILY == XMC1)
    NVIC_SetPriority((IRQn_Type)handle_ptr->rs_intr_handle.node_id, handle_ptr->rs_intr_handle.priority);
#else
    NVIC_SetPriority((IRQn_Type)handle_ptr->rs_intr_handle.node_id,
                      NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                      handle_ptr->rs_intr_handle.priority,handle_ptr->rs_intr_handle.sub_priority));
#endif
#ifdef ADC_MEASUREMENT_ADV_NON_DEFAULT_IRQ_SOURCE_SELECTED
    XMC_SCU_SetInterruptControl(handle_ptr->rs_intr_handle.node_id,
                                ((handle_ptr->rs_intr_handle.node_id << 8) | handle_ptr->rs_intr_handle.irqctrl));
#endif

    /* Connect RS Events to NVIC nodes */
    XMC_VADC_GROUP_QueueSetReqSrcEventInterruptNode(group_ptrs[group_index], (XMC_VADC_SR_t)handle_ptr->srv_req_node);
  }

  return (status);
}
#endif
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Local function to do the request source initialization.*/
__STATIC_INLINE ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_lRequestSrcInit(const ADC_MEASUREMENT_ADV_t
                                                                                 *const handle_ptr)
{
  ADC_MEASUREMENT_ADV_STATUS_t status;

#ifdef ADC_MEASUREMENT_ADV_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
    if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_SCAN >= handle_ptr->req_src)
  #endif
    {
#ifdef ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_ADC_SCAN_USED
      if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_SCAN != handle_ptr->req_src)
  #endif
      {
        status = ADC_MEASUREMENT_ADV_lScanInit(handle_ptr->local_scan_handle,handle_ptr->group_index);
      }
#endif

#ifdef ADC_MEASUREMENT_ADV_ADC_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED
      else
  #endif
      {
        /* Call the function to initialise Clock and ADC global functional units*/
        status = (ADC_MEASUREMENT_ADV_STATUS_t) ADC_SCAN_Init(handle_ptr->scan_handle);
      }
#endif
    }
#endif

#ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_SCAN_USED
    else
  #endif
    {
#ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_ADC_QUEUE_USED
      if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_QUEUE != handle_ptr->req_src)
  #endif
      {
        status = ADC_MEASUREMENT_ADV_lQueueInit(handle_ptr->local_queue_handle,handle_ptr->group_index);
      }
#endif

#ifdef ADC_MEASUREMENT_ADV_ADC_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
      else
  #endif
      {
        /* Call the function to initialise Clock and ADC global functional units*/
        status = (ADC_MEASUREMENT_ADV_STATUS_t) ADC_QUEUE_Init(handle_ptr->queue_handle);
      }
#endif
    }
#endif
    return (status);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
/* Local function to insert the queue entries into the hardware.*/
__STATIC_INLINE void ADC_MEASUREMENT_ADV_lQueueInsertEntries(const ADC_MEASUREMENT_ADV_t *const handle_ptr)
{
  uint32_t entry_index;

  XMC_VADC_GROUP_t *queue_group_ptr = group_ptrs[handle_ptr->group_index];
  const XMC_VADC_QUEUE_ENTRY_t **const entries_array = handle_ptr->local_queue_entries;

  for(entry_index = 0; entry_index < handle_ptr->total_number_of_entries; entry_index++)
  {
    XMC_VADC_GROUP_QueueInsertChannel(queue_group_ptr, *entries_array[entry_index]);
  }
}
#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
__STATIC_INLINE bool ADC_MEASUREMENT_ADV_lArbitrationStatus(const ADC_MEASUREMENT_ADV_t *const handle_ptr)
{
  bool clock_reset_check;
  bool arbitration_status;

#if !defined(CLOCK_GATING_SUPPORTED) || !defined(ADC_MEASUREMENT_ADV_CHECK_CLOCK_GATING)
  clock_reset_check = (bool)false;
#endif
  arbitration_status = (bool)false;

  /* To check if the arbiter is already enabled. Before checking this ensure that clock and reset states are correct */
#if defined(CLOCK_GATING_SUPPORTED) && defined(ADC_MEASUREMENT_ADV_CHECK_CLOCK_GATING)
  clock_reset_check = !XMC_SCU_CLOCK_IsPeripheralClockGated(XMC_SCU_PERIPHERAL_CLOCK_VADC);
#endif
#ifdef PERIPHERAL_RESET_SUPPORTED
  clock_reset_check |= !XMC_SCU_RESET_IsPeripheralResetAsserted(XMC_SCU_PERIPHERAL_RESET_VADC);
#endif
  if(clock_reset_check != (bool)false)
  {
#ifdef ADC_MEASUREMENT_ADV_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
    if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_SCAN >= handle_ptr->req_src)
  #endif
      {
        arbitration_status = XMC_VADC_GROUP_ScanIsArbitrationSlotEnabled(group_ptrs[handle_ptr->group_index]);
      }
#endif

#ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_SCAN_USED
      else
  #endif
      {
        arbitration_status = XMC_VADC_GROUP_QueueIsArbitrationSlotEnabled(group_ptrs[handle_ptr->group_index]);
      }
#endif
  }
  return (arbitration_status);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
__STATIC_INLINE void ADC_MEASUREMENT_ADV_lDisableArbitration(const ADC_MEASUREMENT_ADV_t *const handle_ptr,
                                                             bool arbitration_status)
{
  if(arbitration_status == (bool)false)
  {
#ifdef ADC_MEASUREMENT_ADV_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
      if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_SCAN >= handle_ptr->req_src)
  #endif
      {
        XMC_VADC_GROUP_ScanDisableArbitrationSlot(group_ptrs[handle_ptr->group_index]);
      }
#endif

#ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_SCAN_USED
      else
  #endif
      {
        XMC_VADC_GROUP_QueueDisableArbitrationSlot(group_ptrs[handle_ptr->group_index]);
      }
#endif
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Insert channels into the hardware*/
void ADC_MEASUREMENT_ADV_lInsertChannels(const ADC_MEASUREMENT_ADV_t *const handle_ptr)
{
  XMC_ASSERT("ADC_MEASUREMENT_ADV_InsertChannels:Invalid handle_ptr", (handle_ptr != NULL))

#ifdef ADC_MEASUREMENT_ADV_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
    if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_SCAN >= handle_ptr->req_src)
  #endif
    {
#ifdef ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_ADC_SCAN_USED
      if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_SCAN != handle_ptr->req_src)
  #endif
      {
        XMC_VADC_GROUP_ScanAddMultipleChannels(group_ptrs[handle_ptr->group_index], handle_ptr->local_scan_handle->insert_mask);
      }
#endif

#ifdef ADC_MEASUREMENT_ADV_ADC_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED
      else
  #endif
      {
        ADC_SCAN_AllEntriesInserted(handle_ptr->scan_handle);
      }
#endif
    }
#endif

#ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_SCAN_USED
    else
  #endif
    {
#ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_ADC_QUEUE_USED
      if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_QUEUE != handle_ptr->req_src)
  #endif
      {
        ADC_MEASUREMENT_ADV_lQueueInsertEntries(handle_ptr);
      }
#endif

#ifdef ADC_MEASUREMENT_ADV_ADC_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
      else
  #endif
      {
        ADC_QUEUE_AllEntriesInserted(handle_ptr->queue_handle);
      }
#endif
    }
#endif
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Select the boundary for a channel and configure its value as well.*/
void ADC_MEASUREMENT_ADV_lSetBoundary(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                     XMC_VADC_CHANNEL_BOUNDARY_t boundary_select,
                                     uint32_t boundary_value)
{
  XMC_ASSERT("ADC_MEASUREMENT_ADV_SetBoundary:Invalid handle_ptr", (handle_ptr != NULL))

  switch(boundary_select)
  {
    case XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0:
    case XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND1:
         XMC_VADC_GROUP_SetIndividualBoundary(group_ptrs[handle_ptr->group_index], boundary_select,
                                              (uint16_t)boundary_value);
         break;

    case XMC_VADC_CHANNEL_BOUNDARY_GLOBAL_BOUND0:
    case XMC_VADC_CHANNEL_BOUNDARY_GLOBAL_BOUND1:
         XMC_VADC_GLOBAL_SetIndividualBoundary(ADC_MEASUREMENT_ADV_GLOBAL_PTR,
                                               boundary_select,
                                               (uint16_t)boundary_value);
         break;
}
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#ifndef ADC_MEASUREMENT_ADV_SYNC_USED
/* Address the errata for the incorrect conversion.*/
void ADC_MEASUREMENT_ADV_lSyncADCClocks(void)
{
  int32_t group_index;

  for (group_index = (int32_t)XMC_VADC_MAXIMUM_NUM_GROUPS - (int32_t)1; group_index >= (int32_t)0  ; group_index--)
  {
    XMC_VADC_GROUP_SetPowerMode(group_ptrs[group_index],XMC_VADC_GROUP_POWERMODE_OFF);
  }

  for (group_index = (int32_t)XMC_VADC_MAXIMUM_NUM_GROUPS - (int32_t)1; group_index > (int32_t)0  ; group_index--)
  {
    XMC_VADC_GROUP_SetSyncSlave(group_ptrs[group_index], (uint32_t)0, (uint32_t)group_index);

    XMC_VADC_GROUP_CheckSlaveReadiness(group_ptrs[0U], (uint32_t)group_index);
  }

  XMC_VADC_GROUP_SetSyncMaster(group_ptrs[0U]);

  XMC_VADC_GROUP_SetPowerMode(group_ptrs[0U],XMC_VADC_GROUP_POWERMODE_NORMAL);
}
#endif
/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Enables the arbiter of the selected request source*/
void ADC_MEASUREMENT_ADV_StartADC(const ADC_MEASUREMENT_ADV_t *const handle_ptr)
{
#ifdef ADC_MEASUREMENT_ADV_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
    if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_SCAN >= handle_ptr->req_src)
  #endif
      {
        XMC_VADC_GROUP_ScanEnableArbitrationSlot(group_ptrs[handle_ptr->group_index]);
      }
#endif

#ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_SCAN_USED
      else
  #endif
      {
        XMC_VADC_GROUP_QueueEnableArbitrationSlot(group_ptrs[handle_ptr->group_index]);
      }
#endif
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Starts the ADC conversions by causing a software start of conversion*/
void ADC_MEASUREMENT_ADV_SoftwareTrigger(const ADC_MEASUREMENT_ADV_t *const handle_ptr)
{
  XMC_ASSERT("ADC_MEASUREMENT_ADV_StartConversion:Invalid handle_ptr", (handle_ptr != NULL))

#ifdef ADC_MEASUREMENT_ADV_SCAN_USED
  #ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
    if ( ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_SCAN >= handle_ptr->req_src)
  #endif
    {
      XMC_VADC_GROUP_ScanTriggerConversion(group_ptrs[handle_ptr->group_index]);
    }
#endif
#ifdef ADC_MEASUREMENT_ADV_QUEUE_USED
  #ifdef ADC_MEASUREMENT_ADV_SCAN_USED
    else
  #endif
    {
      XMC_VADC_GROUP_QueueTriggerConversion(group_ptrs[handle_ptr->group_index]);
    }
#endif
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Enables the NVIC(if needed) when scan/queue request source is consumed internally in the APP. */
void ADC_MEASUREMENT_ADC_lNvicEnable(const ADC_MEASUREMENT_ADV_t *const handle_ptr)
{
#ifdef ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED
    if (((bool)true == handle_ptr->local_scan_handle->rs_intr_handle.interrupt_enable) &&
        (ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_SCAN == handle_ptr->req_src))
    {
      NVIC_EnableIRQ((IRQn_Type)handle_ptr->local_scan_handle->rs_intr_handle.node_id);
    }
#endif
#ifdef ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED
    if (((bool)true == handle_ptr->local_queue_handle->rs_intr_handle.interrupt_enable) &&
        (ADC_MEASUREMENT_ADV_REQUEST_SOURCE_LOCAL_QUEUE == handle_ptr->req_src))
    {
      NVIC_EnableIRQ((IRQn_Type)handle_ptr->local_queue_handle->rs_intr_handle.node_id);
    }
#endif
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Configures the result results . */
__STATIC_INLINE void ADC_MEASUREMENT_ADC_lResultInit(const ADC_MEASUREMENT_ADV_CHANNEL_t *indexed)
{
#ifdef ADC_MEASUREMENT_ADV_FIFO_USED
  uint8_t fifo_num_of_intermediate_stages;
  uint8_t fifo_index;
  uint8_t fifo_head;
#endif
#ifdef ADC_MEASUREMENT_ADV_FIFO_USED
  /* If FIFO is selected for the particular channel then do the FIFO initializations*/
  if ( (uint32_t)0 != indexed->max_fifo_required)
  {
    /*Excluding the head and tail from the total number of FIFO elements needed*/
    fifo_num_of_intermediate_stages = indexed->max_fifo_required - ADC_MEASUREMENT_ADV_RESERVED_REGISTERS;

    fifo_head = (uint8_t)indexed->ch_handle->result_reg_number;
    for (fifo_index = 1; fifo_index <= fifo_num_of_intermediate_stages; fifo_index++)
    {
      XMC_VADC_GROUP_ResultInit(group_ptrs[indexed->group_index], (uint32_t)fifo_head - fifo_index,
                                &ADC_MEASUREMENT_ADV_fifo_intermediate_stage);
      }

     /* For the FIFO Tail configuration*/
      XMC_VADC_GROUP_ResultInit(group_ptrs[indexed->group_index], (uint32_t)indexed->result_fifo_tail_number,
                                indexed->res_handle[ADC_MEASUREMENT_ADV_TAIL_RESULT_REG_CONFIG]);
  }
#endif
      /* Initialize for configured result registers For FIFO Head configuration*/
      XMC_VADC_GROUP_ResultInit(group_ptrs[indexed->group_index], (uint32_t)indexed->ch_handle->result_reg_number,
                                indexed->res_handle[ADC_MEASUREMENT_ADV_HEAD_RESULT_REG_CONFIG]);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Initialization routine to call ADC LLD API's */
ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_Init(const ADC_MEASUREMENT_ADV_t *const handle_ptr)
{
  XMC_ASSERT("ADC_MEASUREMENT_ADV_Init:Invalid handle_ptr", (handle_ptr != NULL))

  const ADC_MEASUREMENT_ADV_CHANNEL_t *indexed;
  uint8_t ch_num;
  uint8_t total_number_of_channels;
  ADC_MEASUREMENT_ADV_STATUS_t status;
#ifdef ADC_MEASUREMENT_ADV_SHS_GAIN_NON_DEFAULT
  uint8_t channel_number;
#endif
  bool arbitration_status = (bool)false;

  if (ADC_MEASUREMENT_ADV_STATUS_UNINITIALIZED == *handle_ptr->init_state)
  {

    arbitration_status = ADC_MEASUREMENT_ADV_lArbitrationStatus(handle_ptr);

   /* Initialize the scan/queue request source.*/
   status = ADC_MEASUREMENT_ADV_lRequestSrcInit(handle_ptr);

   /* Disable the Arbitration if no other instance has enabled it*/
   ADC_MEASUREMENT_ADV_lDisableArbitration(handle_ptr,arbitration_status);

#ifdef ADC_MEASUREMENT_ADV_SYNC_USED
  #ifdef ADC_MEASUREMENT_ADV_SYNC_NOT_ALL_USED
    if ((uint32_t)0 != handle_ptr->sync_slaves)
  #endif
    {
      /*  Configure the Sync conversion operation */
      ADC_MEASUREMENT_ADV_lSyncInit(handle_ptr);
    }
#else
   ADC_MEASUREMENT_ADV_lSyncADCClocks();
#endif

    /* Initialize the SR lines for the Channel event and the Result event, if required*/
#ifdef ADC_MEASUREMENT_ADV_MUX_USED
  #ifdef ADC_MEASUREMENT_ADV_MUX_NOT_ALL_USED
    if (handle_ptr->event_config != NULL)
  #endif
    {
      (handle_ptr->event_config)();
    }
#endif

    total_number_of_channels = (uint8_t)handle_ptr->total_number_of_channels;
    for (ch_num = (uint8_t)0; ch_num < (uint8_t)total_number_of_channels; ch_num++)
    {
      indexed = handle_ptr->channel_array[ch_num];

      /* Initialize for configured channels*/
      XMC_VADC_GROUP_ChannelInit(group_ptrs[indexed->group_index],(uint32_t)indexed->ch_num, indexed->ch_handle);

#if (XMC_VADC_SHS_AVAILABLE == 1U)
  #ifdef ADC_MEASUREMENT_ADV_SHS_GAIN_NON_DEFAULT
      channel_number = indexed->ch_num;
    #ifdef ADC_MEASUREMENT_ADV_SHS_GAIN_ALIAS
      if (indexed->ch_handle->alias_channel != XMC_VADC_CHANNEL_ALIAS_DISABLED)
      {
         channel_number = indexed->ch_handle->alias_channel;
      }
    #endif
      XMC_VADC_GLOBAL_SHS_SetGainFactor(ADC_MEASUREMENT_ADV_SHS_PTR,
                                        (uint8_t)indexed->shs_gain_factor,
                                        (XMC_VADC_GROUP_INDEX_t)indexed->group_index,
                                        channel_number);
  #endif
#endif

      /* Result Init both with and without FIFO */
      ADC_MEASUREMENT_ADC_lResultInit(indexed);

#ifdef ADC_MEASUREMENT_ADV_ANALOG_IO_USED
      /* ANALOG_IO initialization for the channel*/
#ifndef ADC_MEASUREMENT_ADV_ANALOG_IO_ALL_CHANNELS_USED
      if(indexed->analog_io_config != NULL)
#endif
      {
        status |= (ADC_MEASUREMENT_ADV_STATUS_t) ANALOG_IO_Init(indexed->analog_io_config);
      }
#endif
    }

#if defined(ADC_MEASUREMENT_ADV_ADC_SCAN_USED) || defined(ADC_MEASUREMENT_ADV_ADC_QUEUE_USED)
    /* Load the queue/scan entries into ADC_QUEUE/ADC_SCAN.
     * This would load the scan/ queue entries into the software buffers in the ADC_SCAN/ADC_QUEUE APPs.
     * A call to this API would only configure the ADC_SCAN/ADC_QUEUE software buffers and will not be
     * programmed into the Hardware. The programming into the hardware is taken care by another API.
     */
#if defined(ADC_MEASUREMENT_ADV_LOCAL_SCAN_USED) || defined(ADC_MEASUREMENT_ADV_LOCAL_QUEUE_USED)
    if( (ADC_MEASUREMENT_ADV_REQUEST_SOURCE_SCAN == handle_ptr->req_src) ||
        (ADC_MEASUREMENT_ADV_REQUEST_SOURCE_QUEUE == handle_ptr->req_src))
#endif
    {
      for (ch_num = (uint8_t)0; ch_num < (uint8_t)handle_ptr->total_number_of_entries; ch_num++)
      {
        ADC_MEASUREMENT_ADV_lInsertEntry(handle_ptr,ch_num);
      }
    }
#endif

    /* Enables the NVIC node if NVIC node is consumed inside the APP*/
    ADC_MEASUREMENT_ADC_lNvicEnable(handle_ptr);

    /* Load the queue/scan entries into the hardware */
    ADC_MEASUREMENT_ADV_lInsertChannels(handle_ptr);

    /*Start the arbiter of the ADC request source after the initialization. */
#ifdef ADC_MEASUREMENT_ADV_START_ADC
  #ifdef ADC_MEASUREMENT_ADV_NOT_ALL_REQ_START
    if ((bool)false  != handle_ptr->start_at_initialization)
  #endif
    {
      ADC_MEASUREMENT_ADV_StartADC(handle_ptr);
    }
#endif

    *handle_ptr->init_state = status;
  }
  return (*handle_ptr->init_state);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Set the Fast compare value*/
ADC_MEASUREMENT_ADV_STATUS_t ADC_MEASUREMENT_ADV_SetFastCompareValue(const ADC_MEASUREMENT_ADV_CHANNEL_t
                                                                     *const handle_ptr, uint16_t compare_value)
{
  ADC_MEASUREMENT_ADV_STATUS_t status;
  XMC_ASSERT("ADC_MEASUREMENT_ADV_SetFastCompareValue:Invalid handle_ptr", (handle_ptr != NULL))

  status = ADC_MEASUREMENT_ADV_STATUS_FAILURE;

  if ( (uint32_t)compare_value <= ADC_MEASUREMENT_ADV_10_BIT_MAX_VALUE)
  {
    XMC_VADC_GROUP_SetResultFastCompareValue(group_ptrs[handle_ptr->group_index],
                                             (uint32_t)  handle_ptr->ch_handle->result_reg_number,
                                             (XMC_VADC_RESULT_SIZE_t)compare_value);
    status = ADC_MEASUREMENT_ADV_STATUS_SUCCESS;
  }
  return (status);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Set the Subtraction value */
void ADC_MEASUREMENT_ADV_SetSubtractionValue(const ADC_MEASUREMENT_ADV_t *const handle_ptr,
                                             ADC_MEASUREMENT_ADV_SUBTRATION_t subtraction_alignment,
                                             uint16_t subtraction_value)
{
  uint32_t groups;
  uint8_t i;
  XMC_ASSERT("ADC_MEASUREMENT_ADV_SetSubtractionValue:Invalid handle_ptr", (handle_ptr != NULL))

#ifdef ADC_MEASUREMENT_ADV_SYNC_USED
  groups = handle_ptr->sync_slaves;
#else
  groups = (uint32_t)0;
#endif
  groups |= (uint32_t)((uint32_t)1 << (uint32_t)handle_ptr->group_index);
  for ( i = (uint8_t)0; i < (uint8_t)XMC_VADC_MAXIMUM_NUM_GROUPS ; i++)
  {
    if ( (bool)false != (bool)((groups >> i) & (uint32_t)0x1 ))
    {
      XMC_VADC_GROUP_SetResultSubtractionValue(group_ptrs[i], (uint16_t)(subtraction_value
                                                                         << (uint32_t)subtraction_alignment));
    }
  }

}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Configure the resolution and sampling time in an iclass */
void ADC_MEASUREMENT_ADV_ConfigureChannelClass(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                               const XMC_VADC_GROUP_CLASS_t *config)
{
  XMC_ASSERT("ADC_MEASUREMENT_ADV_ConfigureChannelClass:Invalid class configuration", (config != NULL))

  XMC_VADC_GROUP_InputClassInit(group_ptrs[handle_ptr->group_index], *config, XMC_VADC_GROUP_CONV_STD,
                                (uint32_t)handle_ptr->ch_handle->input_class);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Sets the alternate reference for a particular channel*/
void ADC_MEASUREMENT_ADV_SetAlternateReference(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                               const XMC_VADC_CHANNEL_REF_t reference_select)
{
  XMC_ASSERT("ADC_MEASUREMENT_ADV_SetAlternateReference:Invalid handle_ptr", (handle_ptr != NULL))

  XMC_VADC_GROUP_ChannelSetInputReference(group_ptrs[handle_ptr->group_index], (uint32_t)handle_ptr->ch_num,
                                          reference_select);

}

#if (XMC_VADC_SHS_AVAILABLE == 1U)
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Sets the gain ratio for a particular channel*/
void ADC_MEASUREMENT_ADV_SetChannelGain(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                        const ADC_MEASUREMENT_ADV_GAIN_t gain_factor)
{
  uint8_t channel_number;
  XMC_ASSERT("ADC_MEASUREMENT_ADV_SetChannelGain:Invalid handle_ptr", (handle_ptr != NULL))

  channel_number = handle_ptr->ch_num;
#ifdef ADC_MEASUREMENT_ADV_SHS_GAIN_ALIAS
  if (handle_ptr->ch_handle->alias_channel != XMC_VADC_CHANNEL_ALIAS_DISABLED)
  {
    channel_number = (uint8_t) handle_ptr->ch_handle->alias_channel;
  }
#endif
  XMC_VADC_GLOBAL_SHS_SetGainFactor(ADC_MEASUREMENT_ADV_SHS_PTR,
                                    (uint8_t)gain_factor,
                                    (XMC_VADC_GROUP_INDEX_t)handle_ptr->group_index,
                                    (uint32_t)channel_number);
}
#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Select the boundary for the channel*/
void ADC_MEASUREMENT_ADV_SelectBoundary(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                        XMC_VADC_BOUNDARY_SELECT_t boundary,
                                        XMC_VADC_CHANNEL_BOUNDARY_t boundary_selection)
{
  XMC_ASSERT("ADC_MEASUREMENT_ADV_SelectBoundary:Invalid handle_ptr", (handle_ptr != NULL))

  XMC_VADC_GROUP_ChannelSetBoundarySelection(group_ptrs[handle_ptr->group_index], (uint32_t)handle_ptr->ch_num,
                                               boundary, boundary_selection);

}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* configure the upper boundary for a channel.*/
void ADC_MEASUREMENT_ADV_SetBoundaryUpper(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                          uint32_t boundary_value)
{
  uint32_t boundary_select;
  XMC_ASSERT("ADC_MEASUREMENT_ADV_SetBoundary:Invalid handle_ptr", (handle_ptr != NULL))

  boundary_select = handle_ptr->ch_handle->upper_boundary_select;

  ADC_MEASUREMENT_ADV_lSetBoundary(handle_ptr, (XMC_VADC_CHANNEL_BOUNDARY_t)boundary_select, boundary_value);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* configure the lower boundary for a channel.*/
void ADC_MEASUREMENT_ADV_SetBoundaryLower(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr,
                                          uint32_t boundary_value)
{
  uint32_t boundary_select;
  XMC_ASSERT("ADC_MEASUREMENT_ADV_SetBoundary:Invalid handle_ptr", (handle_ptr != NULL))

  boundary_select = handle_ptr->ch_handle->lower_boundary_select;

  ADC_MEASUREMENT_ADV_lSetBoundary(handle_ptr, (XMC_VADC_CHANNEL_BOUNDARY_t)boundary_select, boundary_value);
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Aliased channel number is returned if the channel has alias enabled */
XMC_VADC_CHANNEL_ALIAS_t ADC_MEASUREMENT_ADV_GetAliasValue(const ADC_MEASUREMENT_ADV_CHANNEL_t *const handle_ptr)
{
  XMC_VADC_CHANNEL_ALIAS_t return_value;
  ADC_MEASUREMENT_ADV_ALIAS_t alias_value;

  XMC_ASSERT("ADC_MEASUREMENT_ADV_GetAliasValue:Invalid handle_ptr", (handle_ptr != NULL))

  alias_value.alias = XMC_VADC_GROUP_GetAlias(group_ptrs[handle_ptr->group_index]);
  if ((uint8_t)0 == handle_ptr->ch_num )
  {
    return_value = (XMC_VADC_CHANNEL_ALIAS_t)alias_value.alias0;
    if ((uint32_t)0 == alias_value.alias0)
    {
      return_value = XMC_VADC_CHANNEL_ALIAS_DISABLED;
    }
  }
  else if ((uint8_t)1 == handle_ptr->ch_num )
  {
    return_value = (XMC_VADC_CHANNEL_ALIAS_t)alias_value.alias1;
    if ((uint32_t)1 == alias_value.alias1)
    {
      return_value = XMC_VADC_CHANNEL_ALIAS_DISABLED;
    }
  }
  else
  {
    return_value = XMC_VADC_CHANNEL_ALIAS_DISABLED;
  }

  return(return_value);

}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#ifdef ADC_MEASUREMENT_ADV_SYNC_USED
/* Enables uniform conversion configurations across slaves*/
void ADC_MEASUREMENT_ADV_SetIclass(const ADC_MEASUREMENT_ADV_t *const handle_ptr)
{
  XMC_VADC_CHANNEL_CONV_t req_iclass;
  XMC_VADC_GROUP_CLASS_t conv_class;
  XMC_VADC_GLOBAL_CLASS_t conv_class_global;

  XMC_ASSERT("ADC_MEASUREMENT_ADV_SetIclass:Invalid handle_ptr", (handle_ptr != NULL))

  req_iclass = ADC_MEASUREMENT_ADV_lGetIclass(handle_ptr);
  conv_class = XMC_VADC_GROUP_GetInputClass(group_ptrs[handle_ptr->group_index], req_iclass);
  conv_class_global.globiclass = conv_class.g_iclass0;
  XMC_VADC_GLOBAL_InputClassInit(ADC_MEASUREMENT_ADV_GLOBAL_PTR, conv_class_global,
                                 XMC_VADC_GROUP_CONV_STD, (uint32_t)ADC_MEASUREMENT_ADV_GLOBICLASS1);

}
#endif


/* --- End of File --- */

