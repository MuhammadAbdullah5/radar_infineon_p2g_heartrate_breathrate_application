/**
 *
 *  drv_bgt.h
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */

#ifndef FW_INC_DRV_BGT_H_
#define FW_INC_DRV_BGT_H_

/* Enable C linkage if header is included in C++ files */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
   1. INCLUDE FILES
*******************************************************************************/
#include <types.h>

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/
/** BGT24MTR12 default configuration. */
#define	BGT24MTR1x_BASE_CONF         (0x1048U)   /**< 0x1048 = 0001 0000  0100 1000 */

/** BGT24MTR1x default power configuration. */
#define BGT24MTR1x_POWER_CONF               (0x1047U)   /**< 0x1047 = 0001 0000  0100 0111 */

/** BGT24MTR1x SPI control word settings. */
#define BGT24MTR1X_ENA_LNA_MASK             (0x7FFFU)   /**< LNA Gain increase; 0 = increase LNA gain */
#define BGT24MTR1X_DIS_LNA_MASK             (0x8000U)   /**< LNA Gain reduction; 1 = reduce LNA gain */
#define BGT24MTR1X_ENA_PA_MASK              (0xEFFFU)   /**< ENABLE PA - 0 = turn on TX power */
#define BGT24MTR1X_DIS_PA_MASK              (0x1000U)   /**< DISABLE PA - 1 = turn off TX power */
#define BGT24MTR1X_DIS_DIV64K_MASK          (0x0040U)   /**< 1 = Disable 64K divider for Q2 */
#define BGT24MTR1X_DIS_DIV16_MASK           (0x0020U)   /**< 1 = Disable 16 divider for Q2 */
#define BGT24MTR1X_PC2_BUF_MASK             (0x0010U)   /**< PC2_BUF 1 = High LO buffer output power */
#define BGT24MTR1X_PC1_BUF_MASK             (0x0008U)   /**< PC1_BUF 1 = High TX buffer output power */
#define BGT24MTR1X_PC_PA_MASK               (0x0007U)   /**< PC0_PA, PC1_PA, PC2_PA,  1 = reduce TX power */

#define BGT24MTR1X_AMUX_0                 (0x0080U)   /**< BGT AUMUX_0 Mask */
#define BGT24MTR1X_AMUX_1                 (0x0100U)   /**< BGT AUMUX_1 Mask */
#define BGT24MTR1X_AMUX_2                 (0x2000U)   /**< BGT AUMUX_2 Mask */
#define BGT24MTR1X_AMUX_VOUT_TX           (0xDE7FU)   /**< BGT AUMUX_VOUT Mask */

/**
 * @brief BGT24MTR1x index of the ANA output Command from AMUX pin.
 */
#define BGT24MTR1X_ANA_CMD_TEMPERATURE      (0x0U)   /**< Command to read the temperature output from AMUX pin */
#define BGT24MTR1X_ANA_CMD_TX_POWER         (0x1U)   /**< Command to read the output TX power from AMUX pin */
#define BGT24MTR1X_ANA_CMD_TX_POWER_REF     (0x2U)   /**< Command to read the reference TX power from AMUX pin */

/**
 * @brief BGT24MTR1x TX PA settings.
 */
#define	BGT24MTR1X_PC_PA_0                  (0x0000U)   /**< Max Tx power, no reduction */
#define	BGT24MTR1X_PC_PA_1                  (0x0001U)   /**< Max Tx power, no reduction */
#define	BGT24MTR1X_PC_PA_2                  (0x0002U)   /**< Reduction by 0.8dBm */
#define	BGT24MTR1X_PC_PA_3                  (0x0003U)   /**< Reduction by 1.4dBm */
#define	BGT24MTR1X_PC_PA_4                  (0x0004U)   /**< Reduction by 2.5dBm */
#define	BGT24MTR1X_PC_PA_5                  (0x0005U)   /**< Reduction by 4dBm */
#define	BGT24MTR1X_PC_PA_6                  (0x0006U)   /**< Reduction by 6dBm */
#define	BGT24MTR1X_PC_PA_7                  (0x0007U)   /**< Reduction by 9dBm */

#define BGT24MTR1X_NUM_RX_ANTENNAS          (2U)   		   /**< RX antennas in BGT24MTR12 */
#define BGT24MTR1X_NUM_TEMP_SENSORS         (uint8_t)1U   /**< Number of temperature sensor in BGT24MTR12 */
#define BGT24MTR1X_MAX_TX_POWER_LEVEL       (7U)          /**< Maximum BGT TX output power SPI input value range is [0 - 7] */
#define BGT24MTR1X_MIN_RF_FREQUENCY_KHZ     (24025000U)   /**< Minimum RF frequency supported by BGT24MTR1x in kHz */
#define BGT24MTR1X_MAX_RF_FREQUENCY_KHZ     (24225000U)   /**< Maximum RF frequency supported by BGT24MTR1x in kHz */
#define BGT24MTR1X_NUM_TX_ANTENNAS          (1U)   		/**< TX antennas in BGT24MTR1x */

/******************************************************************************
   3. TYPES
*******************************************************************************/

/** brief Defines LNA gain Enable/disable state.
 */
typedef enum
{
  BGT24MTR1X_LNA_DISABLE       = 0U,   /**< LNA gain disabled */
  BGT24MTR1X_LNA_ENABLE        = 1U    /**< LNA gain enabled */
} Bgt24mtr1x_LNAgain_t;

/** brief Defines possible power levels of BGT Tx power amplifier. Use type Bgt24mtr1x_Power_t for this enum.
 */
typedef enum
{
  BGT24MTR1X_TX_MIN            = 0U,   /**< Reduction by 9dBm */
  BGT24MTR1X_TX_LEVEL_1        = 1U,   /**< Reduction by 6dBm */
  BGT24MTR1X_TX_LEVEL_2        = 2U,   /**< Reduction by 4dBm */
  BGT24MTR1X_TX_MID            = 3U,   /**< Reduction by 2.5dBm */
  BGT24MTR1X_TX_LEVEL_4        = 4U,   /**< Reduction by 1.4dBm */
  BGT24MTR1X_TX_LEVEL_5        = 5U,   /**< Reduction by 0.8dBm */
  BGT24MTR1X_TX_LEVEL_6        = 6U,   /**< Reduction by 0.4dBm */
  BGT24MTR1X_TX_MAX            = 7U    /**< No Reduction, Max Tx Power */
} Bgt24mtr1x_Power_t;


/******************************************************************************
   4. FUNCTION PROTOTYPES
*******************************************************************************/
void bgt_init(uint32_t lna_gain, uint32_t power_level); // This function initializes the BGT24MTR1x 16-bit SPI shadow register and send BGT settings.
void bgt_set_config(uint16_t config_val);               // This function transmits the 16-bit SPI settings to BGT24MTR1x registers \ref bgt24mtr1x_global_config.
void bgt_set_tx_power(uint8_t power_level);             // This function sets the power level [0- 7] of BGT24MTR1x TX Power amplifier.

void bgt_start_tx(void);                                // his function enables BGT24MTR1x TX Power amplifier to start transmission.
void bgt_stop_tx(void);                                 // This function disables BGT TX Power amplifier.

void bgt_power_up(void);                                // This function is used to power-up the BGT.
void bgt_power_down(void);                              // This function is used to power-down the BGT.

void bgt_lna_gain_enable(void);                         // This function enables the Receiver LNA gain.
void bgt_lna_gain_disable(void);                        // This function disables the Receiver LNA gain.

void bgt_ana_temp(void);                                // This function sets the SPI settings for BGT24MTR1x to read the temperature output from AMUX pin.
void bgt_ana_vout_tx(void);                             // This function sets the SPI settings for BGT24MTR1x to read the output TX power from AMUX pin.
void bgt_ana_vref_tx(void);                             // This function sets the SPI settings for BGT24MTR1x to read the reference TX power from AMUX pin.

void bgt_ldo_enable(void);                              // This function enables the LDO for BGT.
void bgt_ldo_disable(void);                             // This function disables the LDO for BGT.

void bgt_lowest_power_with_q2_disable(void);            // This function sets the lowest power with q2 disable for BGT.

uint8_t bgt_get_tx_power(void);                         // This function returns the power level [0 - 7] of BGT24MTR1x TX Power amplifier.
uint8_t bgt_lna_gain_is_enable(void);                   // This function returns the current status of the Receiver LNA gain, if enabled it returns true else false.

uint16_t bgt_get_config(void);                          // This function transmits the 16-bit SPI settings to BGT24MTR1x registers \ref bgt24mtr1x_global_config.
uint16_t bgt_get_ana_config(void);                      // This function returns the current settings for Analog output defined by \ref bgt24mtr1x_ana_command.


/* Disable C linkage for C++ files */
#ifdef __cplusplus
}      /* extern "C" */
#endif /* __cplusplus */


#endif /* FW_INC_DRV_BGT_H_ */

/* --- End of File --- */
