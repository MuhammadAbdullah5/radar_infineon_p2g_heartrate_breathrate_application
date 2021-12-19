/**
 *
 *  drv_host_comm.c
 *
 *     Created on: Jul 3, 2020
 *         Author: Abdullah
 *          Brief:
 *
 */
 
/******************************************************************************
   1. INCLUDES
*******************************************************************************/

#include <stdint.h>
#include <FW/inc/drv_host_comm.h>
#include <FW/inc/drv_bsp.h>
#include <FW/inc/radar_control.h>

/******************************************************************************
   2. DEFINITIONS
*******************************************************************************/

/**
 * \defgroup InternalConstants
 *
 * \brief Constants used to define the message format.
 *
 * The messages exchanged between host and device contain some of those
 * constants to define the limits of a message in the byte stream.
 *
 * @{
 */
#define CNST_STARTBYTE_DATA           0x5A    /**< A payload message begins
                                                   with this code byte. */
#define CNST_STARTBYTE_STATUS         0x5B    /**< A status message begins
                                                   with this code byte. */
#define CNST_END_OF_PAYLOAD           0xE0DB  /**< Every payload message ends
                                                   with this 16 bit value
                                                   (transmitted with low byte
                                                   first!). */

#define CNST_MSG_QUERY_ENDPOINT_INFO  0x00    /**< This command code sent to
                                                   endpoint 0 tells the device
                                                   to send information about
                                                   the protocol endpoints. */
#define CNST_MSG_ENDPOINT_INFO        0x00    /**< A message containing
                                                   endpoint information sent
                                                   from endpoint 0 starts with
                                                   this message type code. */
#define CNST_MSG_QUERY_FW_INFO        0x01    /**< This command code sent to
                                                   endpoint 0 tells the device
                                                   to send information about
                                                   its firmware. */
#define CNST_MSG_FW_INFO              0x01    /**< A message containing
                                                   firmware information sent
                                                   from endpoint 0 starts with
                                                   this message type code. */
#define CNST_MSG_FIRMWARE_RESET       0x02    /**< This command code sent to
                                                   endpoint 0 tells the device
                                                   to do a firmware reset. */
/** @} */

/**
 * \defgroup StatusCodes
 *
 * \brief General protocol status codes.
 *
 * Whenever a message with improper format is received one of the following
 * status code will be returned. The status codes will always be sent from
 * endpoint 0.
 *
 * @{
 */
#define PROTOCOL_ERROR_OK                0x0000  /**< No error occurred. */
#define PROTOCOL_ERROR_TIMEOUT           0x0001  /**< The message seemed to
                                                      end, but more data was
                                                      expected. */
#define PROTOCOL_ERROR_BAD_MESSAGE_START 0x0002  /**< The start byte of the
                                                      message was incorrect.
                                                      */
#define PROTOCOL_ERROR_BAD_ENDPOINT_ID   0x0003  /**< An endpoint with the
                                                      requested ID is not
                                                      present. */
#define PROTOCOL_ERROR_NO_PAYLOAD        0x0005  /**< A message with no
                                                      payload is not
                                                      supported. */
#define PROTOCOL_ERROR_OUT_OF_MEMORY     0x0006  /**< There is not enough
                                                      memory to store the
                                                      payload. */
#define PROTOCOL_ERROR_BAD_PAYLOAD_END   0x0007  /**< The message did not end
                                                      with the expected
                                                      sequence. */
#define PROTOCOL_ERROR_BAD_COMMAND       0xFFFF  /**< The command sent to
                                                      endpoint 0 is not
                                                      understood. */

#ifndef PROTOCOL_MAX_PAYLOAD_SIZE
	#define PROTOCOL_MAX_PAYLOAD_SIZE    (512U)    /**< The maximum buffer size
													  for the received message. */
#endif

/**
 * \internal
 * \defgroup EndpointTargetDetectionCodes
 *
 * \brief These are the command codes to identify the payload type.
 *
 * Each payload message of the supported endpoint starts with one of these
 * commend codes.
 *
 * @{
 */
#define  MSG_GET_DSP_SETTINGS		    0x00 	/**< A message to retrieve DSP settings */

#define  MSG_SET_DSP_SETTINGS           0x01 	/**< A message to set DSP settings */

#define  MSG_GET_TARGETS                0x02	/**< A message to get targets information */

#define  MSG_GET_RANGE_THRESHOLD		0x03    /**< A message to get range detection threshold */

/** @} */

/**
 * \defgroup MessageTypes
 *
 * \brief This end point knows these message types.
 *
 * The first payload byte of every message is one of these codes.
 *
 * @{
 */

#define MSG_GET_PGA_LEVEL 			0x00  	/**< A message to query current PGA gain level. */

#define MSG_SET_PGA_LEVEL			0x01  	/**< A message containing PGA gain level. */

/** @} */


/**
 * \defgroup MessageTypes
 *
 * \brief This end point knows these message types.
 *
 * The first payload byte of every message is one of these codes.
 *
 * @{
 */

#define MSG_DISABLE_DUTY_CYCLE 			0x00  	/**< A message to disable duty cycle. */

#define MSG_ENABLE_DUTY_CYCLE			0x01  	/**< A message to enable duty cycle. */

#define MSG_IS_ENABLE_DUTY_CYCLE		0x02  	/**< A message to check the status of duty cycle. */

#define MSG_DISABLE_BGT_LNA 			0x03  	/**< A message to disable BGT Rx LNA gain. */

#define MSG_ENABLE_BGT_LNA				0x04  	/**< A message to enable BGT Rx LNA gain. */

#define MSG_IS_ENABLE_BGT_LNA			0x05  	/**< A message to check the status of BGT Rx LNA gain. */

/** @} */

/**
 * \defgroup MessageTypes
 *
 * \brief This end point knows these message types.
 *
 * The first payload byte of every message is one of these codes.
 *
 * @{
 */
#define MSG_GET_CONFIGURATION     0x00  /**< A message to query the FMCW
                                             configuration. */
#define MSG_SET_CONFIGURATION     0x01  /**< A message containing the FMCW
                                             configuration. */
#define MSG_GET_BW_PER_SECOND     0x02  /**< A message to query the bandwidth
                                             per second. */
#define MSG_SET_BW_PER_SECOND     0x02  /**< A message containing the
                                             bandwidth per second. */
/** @} */

/**
 * \defgroup MessageTypes
 *
 * \brief This end point knows these message types.
 *
 * The first payload byte of every message is one of these codes.
 *
 * @{
 */
#define MSG_GET_CONFIGURATION     0x00 /**< A message to query the Doppler
                                            radar configuration. */
#define MSG_SET_CONFIGURATION     0x01 /**< A message containing the Doppler
                                            radar configuration. */
/** @} */

/**
 * \defgroup MessageTypes
 *
 * \brief This end point knows these message types.
 *
 * The first payload byte of every message is one of these codes.
 *
 * @{
 */
#define MSG_FRAME_DATA            0x00 /**< A message containing radar frame
                                            data. */
#define MSG_GET_FRAME_DATA        0x01 /**< A message to retrieve frame data.
                                            */
#define MSG_SET_AUTOMATIC_TRIGGER 0x02 /**< A message to configure the
                                            automatic frame trigger. */
#define MSG_ENABLE_TEST_MODE      0x03 /**< A message to configure antenna
                                            test mode. */
#define MSG_GET_DRIVER_VERSION    0x20 /**< A message to query device
                                            information. */
#define MSG_SET_DRIVER_VERSION    0x21 /**< A message containing device
                                            information. */
#define MSG_GET_DEVICE_INFO       0x22 /**< A message to query device
                                            information. */
#define MSG_SET_DEVICE_INFO       0x23 /**< A message containing device
                                            information. */
#define MSG_GET_TEMPRATURE        0x30 /**< A message to query a measured
                                            temperature value. */
#define MSG_SET_TEMPRATURE        0x31 /**< A message containing a measured
                                            temperature value. */
#define MSG_GET_TX_POWER          0x32 /**< A message to query a measured
                                            TX power value. */
#define MSG_SET_TX_POWER          0x33 /**< A message containing a measured
                                            TX power value. */
#define MSG_GET_CHRIP_DURATION    0x34 /**< A message to query the current
                                            chirp duration. */
#define MSG_SET_CHRIP_DURATION    0x35 /**< A message containing the current
                                            chirp duration. */
#define MSG_GET_MIN_INTERVAL      0x36 /**< A message to query the minimum
                                            frame interval. */
#define MSG_SET_MIN_INTERVAL      0x37 /**< A message containing the minimum
                                            frame interval. */
#define MSG_GET_FRAME_FORMAT      0x40 /**< A message to query the frame
                                            format. */
#define MSG_SET_FRAME_FORMAT      0x41 /**< A message containing frame format
                                            parameters. */
/** @} */

/**
 * \internal
 * \defgroup EndpointCalibrationCodes
 *
 * \brief These are the command codes to identify the payload type.
 *
 * Each payload message of the supported endpoint starts with one of these
 * commend codes.
 *
 * @{
 */

#define  MSG_SET_ADC_FLASH_CALIBRATION		0x01	/**< A message to save ADC calibration data from EEPROM */

#define  MSG_GET_ADC_FLASH_CALIBRATION     	0x02	/**< A message to read ADC calibration data from EEPROM */

#define  MSG_CLEAR_ADC_FLASH_CALIBRATION    0x03	/**< A message to delete ADC calibration data from EEPROM */

#define  MSG_SET_ADC_SRAM_CALIBRATION      	0x04	/**< A message to save ADC calibration data in SRAM */

#define  MSG_GET_ADC_SRAM_CALIBRATION      	0x05	/**< A message to read ADC calibration data in SRAM */

#define  MSG_CLEAR_ADC_SRAM_CALIBRATION     0x06	/**< A message to delete ADC calibration data from SRAM */

#define  MSG_SET_ALGO_FLASH_CALIBRATION		0x07  	/**< A message to store the Algo calibration in the Flash memory. */

#define  MSG_GET_ALGO_FLASH_CALIBRATION		0x08  	/**< A message to read current Algo calibration data stored in the Flash memory. */

#define  MSG_CLEAR_ALGO_FLASH_CALIBRATION	0x09  	/**< A message to clear the Algo calibration data from Flash memory. */

#define  MSG_SET_ALGO_SRAM_CALIBRATION		0x0A  	/**< A message to store the Algo calibration data in the SRAM memory. */

#define  MSG_GET_ALGO_SRAM_CALIBRATION		0x0B  	/**< A message to read current Algo calibration data stored in the SRAM memory. */

#define  MSG_CLEAR_ALGO_SRAM_CALIBRATION	0x0C  	/**< A message to clear the Algo calibration data from SRAM memory. */

/** @} */




/******************************************************************************
   3. LOCAL TYPES
*******************************************************************************/
/**
 * \brief This enumeration lists receive state of the protocol.
 *
 * The protocol uses a variable as the state of a finite state machine which
 * controls what to do with received data.
 */
typedef enum
{
    PROTOCOL_STATE_IDLE,               /**< No reception is going on. The
                                            protocol is waiting for a message
                                            header. */
    PROTOCOL_STATE_RECEIVING_HEADER,   /**< The begin of a message header was
                                            received but the header has not
                                            been received completely. */
    PROTOCOL_STATE_RECEIVING_PAYLOAD,  /**< A message header has been received
                                            completely. Now the reception of
                                            the message payload is ongoing. */
    PROTOCOL_STATE_RECEIVING_TAIL,     /**< The message payload has been
                                            received completely. Now the
                                            reception of the message tail is
                                            ongoing. */
    PROTOCOL_STATE_CONFUSED            /**< An error occurred during message
                                            reception. The incoming data can't
                                            be parsed properly any more. All
                                            following data is ignored until a
                                            timeout occurs to sync again. */
} Protocol_State_t;

/**
 * \brief This structure contains all state information of the communication
 *        protocol.
 *
 * This structure contains the current receive state and some pointer to store
 * received data temporarily. It also contains a pointer to the list of
 * endpoints and the information about timeout function and period.
 *
 * Theoretically multiple instances of the protocol could coexist, each of
 * them having an instance of this team. Currently only a single instance is
 * supported as the only instance of this type is \ref instance.
 */
typedef struct
{
    Endpoint_t*      endpoints;          /**< The list of endpoints. */
    uint8_t          num_endpoints;      /**< The number of endpoints. */
    Protocol_State_t state;              /**< The current receive state. */
    uint8_t*         receive_pointer;    /**< A pointer to the memory
                                              location, where the next
                                              received bytes are stored. */
    uint16_t         num_bytes_awaited;  /**< The number of bytes to be
                                              received before the current
                                              section of message data is
                                              completed. */
    uint32_t         time_of_last_byte;  /**< The system time when the last
                                              bytes were received. This is
                                              used to detect a timeout. */
    uint8_t          message_frame[6];   /**< A buffer to store the received
                                              message header and tail. */
    uint8_t*         payload;            /**< A pointer to the payload. (Data
                                              buffer is allocated during
                                              reception) */
    uint32_t         (*get_time)(void);  /**< A pointer to the function that
                                              returns the current system time.
                                              */
    uint32_t         timeout_interval;   /**< The timeout period with respect
                                              to the unit of the return value
                                              of the get_time function. */
    void       (*do_system_reset)(void); /**< A pointer to a function that
                                              does a system reset. */
	size_t			max_payload;		 /**< holds the maximum payload buffer size, after allocation. */
} Instance_t;

/******************************************************************************
   4. DATA
*******************************************************************************/
/**
 * \brief The state of the protocol is stored here.
 *
 * See \ref Instance_t for more information.
 */
static Instance_t instance;

/**
 * \brief This array contains the firmware version.
 *
 * This array must be defined somewhere in the firmware project. The array
 * contains the following numbers:
 * - firmware_version[0] - The major firmware version.
 * - firmware_version[1] - The minor firmware version.
 * - firmware_version[2] - The firmware version revision.
 */
extern Firmware_Information_t firmware_information;
/**
 * \brief Static buffer used to store the received messages,
 *        in case of the payload size is less than PROTOCOL_MAX_PAYLOAD_SIZE.
 */
static uint8_t data_payload[PROTOCOL_MAX_PAYLOAD_SIZE] = { 0 };

Target_Info_t  target_list[MAX_NUM_OF_TARGETS];		/**< Defined global structured array to avoid stack over-flow issues */

/******************************************************************************
   5. LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
/**
 * \brief This function sends a status code to the host.
 *
 * Whenever the protocol receives a message from the host, it returns a status
 * code when processing of that message is done. This function builds the
 * according status message and sends it.
 *
 * \param[in] endpoint     The number of the endpoint that returned the status
 *                         code.
 * \param[in] status_code  The 16 bit error code to be sent.
 */
static void send_status_message(uint8_t endpoint, uint16_t status_code);

/**
 * \brief This function resets the receive state of the protocol.
 *
 * This function is called after a message has been received or if the
 * protocols recovers from a receive error. The function resets the receive
 * state and prepares the protocol to receive another message from the host.
 */
static void reset_state(void);

/**
 * \brief This function retrieves data from the host.
 *
 * This functions is repeatedly called during the function protocol_run. It
 * reads data bytes from the host that have been received through the
 * communication interface and stores them according to the current receive
 * state.
 *
 * The function also performs a timeout check when a valid get_time function
 * is available.
 */
static void receive_data(void);

/**
 * \brief This function handles payload message that are sent to the protocol
 *        itself.
 *
 * When the protocol receives a payload message from the host that is
 * addressed to endpoint 0 that message is forwarded to this function, where
 * it is parsed.
 *
 * The only valid message that is known to the protocol is the message to
 * query endpoint information. This function checks for that message and calls
 * \ref send_endpoint_info on success.
 *
 * \param[in] message_data  A pointer to the buffer containing the payload.
 * \param[in] num_bytes     The number of bytes in the payload buffer.
 *
 * \return The function returns a status code indicating if the payload could
 *         be handled successfully.
 */
static uint16_t handle_message(uint8_t* message_data, uint16_t num_bytes);

/**
 * \brief This function sends endpoint information to the host.
 *
 * This function is called by \ref handle_message if a message was received to
 * query information about the endpoints of the firmware. Type and version
 * information of all types are packed into a message, which is sent to the
 * host.
 */
static void send_endpoint_info(void);

/**
 * \brief This function sends information about the firmware to the host.
 *
 * This function is called by \ref handle_message if a message was received to
 * query the firmware version. The firmware information is expected in
 * an external structure. See \ref Firmware_Information_t for more
 * information.
 * \code
 * Firmware_Information_t firmware_information;
 * \endcode
 */
static void send_firmware_info(void);

/**
 * \internal
 * \brief The function extracts a signed 8 bit integer from a payload buffer.
 *
 * The function reads one byte from the given payload buffer at the requested
 * position and interprets it as a signed 8 bit integer value, which will be
 * returned.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the requested data value
 *                     starts.
 *
 * \return The function returns the value read from the payload.
 */
static inline int8_t rd_payload_i8(const uint8_t* payload, uint16_t offset)
{
    return (int8_t)payload[offset];
}

/**
 * \internal
 * \brief The function extracts an unsigned 8 bit integer from a payload
 *        buffer.
 *
 * The function reads one byte from the given payload buffer at the requested
 * position and interprets it as an unsigned 8 bit integer value, which will
 * be returned.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the requested data value
 *                     starts.
 *
 * \return The function returns the value read from the payload.
 */
static inline uint8_t rd_payload_u8(const uint8_t* payload, uint16_t offset)
{
    return payload[offset];
}

/**
 * \internal
 * \brief The function extracts a signed 16 bit integer from a payload buffer.
 *
 * The function reads two bytes from the given payload buffer starting at the
 * requested position and interprets them as a signed 16 bit integer value,
 * which will be returned. If needed the byte order is swapped to convert from
 * the payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the requested data value
 *                     starts.
 *
 * \return The function returns the value read from the payload.
 */
static inline int16_t rd_payload_i16(const uint8_t* payload, uint16_t offset);

/**
 * \internal
 * \brief The function extracts an unsigned 16 bit integer from a payload
 *        buffer.
 *
 * The function reads two bytes from the given payload buffer starting at the
 * requested position and interprets them as an unsigned 16 bit integer value,
 * which will be returned. If needed the byte order is swapped to convert from
 * the payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the requested data value
 *                     starts.
 *
 * \return The function returns the value read from the payload.
 */
static inline uint16_t rd_payload_u16(const uint8_t* payload,
                                      uint16_t offset);

/**
 * \internal
 * \brief The function extracts a signed 32 bit integer from a payload buffer.
 *
 * The function reads four bytes from the given payload buffer starting at the
 * requested position and interprets them as a signed 32 bit integer value,
 * which will be returned. If needed the byte order is swapped to convert from
 * the payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the requested data value
 *                     starts.
 *
 * \return The function returns the value read from the payload.
 */
static inline int32_t rd_payload_i32(const uint8_t* payload, uint16_t offset);

/**
 * \internal
 * \brief The function extracts an unsigned 32 bit integer from a payload
 *        buffer.
 *
 * The function reads four bytes from the given payload buffer starting at the
 * requested position and interprets them as an unsigned 32 bit integer value,
 * which will be returned. If needed the byte order is swapped to convert from
 * the payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the requested data value
 *                     starts.
 *
 * \return The function returns the value read from the payload.
 */
static inline uint32_t rd_payload_u32(const uint8_t* payload,
                                      uint16_t offset);

/**
 * \internal
 * \brief The function extracts a signed 64 bit integer from a payload buffer.
 *
 * The function reads four bytes from the given payload buffer starting at the
 * requested position and interprets them as a signed 64 bit integer value,
 * which will be returned. If needed the byte order is swapped to convert from
 * the payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the requested data value
 *                     starts.
 *
 * \return The function returns the value read from the payload.
 */
static inline int64_t rd_payload_i64(const uint8_t* payload, uint16_t offset);

/**
 * \internal
 * \brief The function extracts an unsigned 64 bit integer from a payload
 *        buffer.
 *
 * The function reads four bytes from the given payload buffer starting at the
 * requested position and interprets them as an unsigned 64 bit integer value,
 * which will be returned. If needed the byte order is swapped to convert from
 * the payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the requested data value
 *                     starts.
 *
 * \return The function returns the value read from the payload.
 */
static inline uint64_t rd_payload_u64(const uint8_t* payload,
                                      uint16_t offset);

/**
 * \internal
 * \brief The function writes a signed 8 bit integer value to a payload
 *        buffer.
 *
 * The function writes the given value into the given payload buffer at the
 * requested position.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the value will be written
 *                     to.
 * \param[in] value    The value to be written into the payload buffer.
 */
static inline void wr_payload_i8(uint8_t* payload, uint16_t offset,
                                 int8_t value)
{
    payload[offset] = (uint8_t)value;
}

/**
 * \internal
 * \brief The function writes an unsigned 8 bit integer value to a payload
 *        buffer.
 *
 * The function writes the given value into the given payload buffer at the
 * requested position.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the value will be written
 *                     to.
 * \param[in] value    The value to be written into the payload buffer.
 */
static inline void wr_payload_u8(uint8_t* payload, uint16_t offset,
                                 uint8_t value)
{
    payload[offset] = value;
}

/**
 * \internal
 * \brief The function writes a signed 16 bit integer value to a payload
 *        buffer.
 *
 * The function writes the given value into the given payload buffer at the
 * requested position. If needed the byte order is swapped to convert from the
 * payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the value will be written
 *                     to.
 * \param[in] value    The value to be written into the payload buffer.
 */
static inline void wr_payload_i16(uint8_t* payload, uint16_t offset,
                                  int16_t value);

/**
 * \internal
 * \brief The function writes an unsigned 16 bit integer value to a payload
 *        buffer.
 *
 * The function writes the given value into the given payload buffer at the
 * requested position. If needed the byte order is swapped to convert from the
 * payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the value will be written
 *                     to.
 * \param[in] value    The value to be written into the payload buffer.
 */
static inline void wr_payload_u16(uint8_t* payload, uint16_t offset,
                                  uint16_t value);

/**
 * \internal
 * \brief The function writes a signed 32 bit integer value to a payload
 *        buffer.
 *
 * The function writes the given value into the given payload buffer at the
 * requested position. If needed the byte order is swapped to convert from the
 * payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the value will be written
 *                     to.
 * \param[in] value    The value to be written into the payload buffer.
 */
static inline void wr_payload_i32(uint8_t* payload, uint16_t offset,
                                  int32_t value);

/**
 * \internal
 * \brief The function writes an unsigned 32 bit integer value to a payload
 *        buffer.
 *
 * The function writes the given value into the given payload buffer at the
 * requested position. If needed the byte order is swapped to convert from the
 * payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the value will be written
 *                     to.
 * \param[in] value    The value to be written into the payload buffer.
 */
static inline void wr_payload_u32(uint8_t* payload, uint16_t offset,
                                  uint32_t value);

/**
 * \internal
 * \brief The function writes a signed 64 bit integer value to a payload
 *        buffer.
 *
 * The function writes the given value into the given payload buffer at the
 * requested position. If needed the byte order is swapped to convert from the
 * payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the value will be written
 *                     to.
 * \param[in] value    The value to be written into the payload buffer.
 */
static inline void wr_payload_i64(uint8_t* payload, uint16_t offset,
                                  int64_t value);

/**
 * \internal
 * \brief The function writes an unsigned 64 bit integer value to a payload
 *        buffer.
 *
 * The function writes the given value into the given payload buffer at the
 * requested position. If needed the byte order is swapped to convert from the
 * payload's little endian format to the CPU's endian format.
 *
 * \param[in] payload  A pointer to a block of payload data.
 * \param[in] offset   The byte position at which the value will be written
 *                     to.
 * \param[in] value    The value to be written into the payload buffer.
 */
static inline void wr_payload_u64(uint8_t* payload, uint16_t offset,
                                  uint64_t value);

static uint16_t send_dsp_settings (uint8_t endpoint, Radar_Handle_t radar_driver);

static uint16_t send_targets_info (uint8_t endpoint, Radar_Handle_t radar_driver);

static uint16_t send_range_threshold (uint8_t endpoint, Radar_Handle_t radar_driver);

static uint16_t send_pga_level(Radar_Handle_t device, uint8_t endpoint);

static uint16_t send_fmcw_configuration(uint8_t endpoint,
                                        Radar_Handle_t radar_driver);

static uint16_t send_bandwidth_per_second(uint8_t endpoint,
                                          Radar_Handle_t radar_driver);

static uint16_t send_doppler_configuration(uint8_t endpoint,
                                           Radar_Handle_t radar_driver);

static void send_driver_version(uint8_t endpoint);

static uint16_t send_frame_data(uint8_t endpoint, Radar_Handle_t radar_driver,
                                uint8_t wait);

static uint16_t send_device_info(uint8_t endpoint,
                                 Radar_Handle_t radar_driver);

static uint16_t send_temperature(uint8_t endpoint,
                                 Radar_Handle_t radar_driver,
                                 uint8_t temp_sensor);

static uint16_t send_tx_power(uint8_t endpoint, Radar_Handle_t radar_driver,
                              uint8_t tx_antenna);

static uint16_t send_chirp_duration(uint8_t endpoint,
                                    Radar_Handle_t radar_driver);

static uint16_t send_min_frame_interval(uint8_t endpoint,
                                        Radar_Handle_t radar_driver);

static uint16_t send_frame_format(uint8_t endpoint,
                                 Radar_Handle_t radar_driver);

static uint16_t send_adc_configuration(uint8_t endpoint,
                                       Radar_Handle_t radar_driver);

static uint16_t send_adc_calibration_data(uint8_t endpoint, Radar_Handle_t radar_driver, Calibration_Target_t target);

static uint16_t send_algo_calibration_data(uint8_t endpoint, Radar_Handle_t radar_driver, Calibration_Target_t target);



/******************************************************************************
   6. EXPORTED FUNCTIONS
*******************************************************************************/
void protocol_init(Endpoint_t* endpoints, uint8_t num_endpoints,
                   uint32_t (*get_time)(void), uint32_t timeout_interval,
                   void (*do_system_reset)(void))
{
    /* initialize the internal state structure */
    instance.get_time = get_time;
    instance.timeout_interval = timeout_interval;
    instance.do_system_reset = do_system_reset;

    instance.endpoints = endpoints;
    instance.num_endpoints = num_endpoints;

    instance.state = PROTOCOL_STATE_IDLE;
    instance.receive_pointer = NULL;
    instance.num_bytes_awaited = 0;
    instance.time_of_last_byte = (get_time != NULL) ? instance.get_time() : 0;
    instance.payload = data_payload;
	instance.max_payload = PROTOCOL_MAX_PAYLOAD_SIZE;

    /* initialize state */
    reset_state();

    /* initialize communication interface */
    com_init();
}

void protocol_run(void)
{
    if (instance.state != PROTOCOL_STATE_CONFUSED)
    {
        /* if the protocol is in a defined state, read the next bytes from the
         * input stream
         */
        receive_data();

        /*
         * now process data until the end of a message has been reached
         * (note the missing brakes, the fall through is intended to proceed
         * with the message, so don't wonder)
         */
        switch (instance.state)
        {
        case PROTOCOL_STATE_IDLE:
            /* if at least one byte has been received, the state is not idle
             *  any more
             */
            if (instance.num_bytes_awaited < 4)
                instance.state = PROTOCOL_STATE_RECEIVING_HEADER;

            /* no break, fall through and continue with
             * PROTOCOL_STATE_RECEIVING_HEADER
             */

        case PROTOCOL_STATE_RECEIVING_HEADER:
            if (instance.num_bytes_awaited == 0)
            {
                uint8_t  sync_byte;
                uint8_t  endpoint;
                uint16_t payload_size;

                /* when the header has been received completely, check the
                 * header information
                 */
                sync_byte    = rd_payload_u8 (instance.message_frame, 0);
                endpoint     = rd_payload_u8 (instance.message_frame, 1);
                payload_size = rd_payload_u16(instance.message_frame, 2);

                /* check sync bye, message must be a payload message */
                if (sync_byte != CNST_STARTBYTE_DATA)
                {
                    instance.state = PROTOCOL_STATE_CONFUSED;
                    send_status_message(0, PROTOCOL_ERROR_BAD_MESSAGE_START);
                    break;
                }

                /* check message header for endpoint number */
                if (endpoint <= instance.num_endpoints)
                {
                    /* check payload size (A message with no payload makes no
                     * sense)
                     */
                    if (payload_size == 0)
                    {
                        instance.state = PROTOCOL_STATE_CONFUSED;
                        send_status_message(0, PROTOCOL_ERROR_NO_PAYLOAD);
                        break;
                    }

                    /* prepare for receiving payload */
                    if (payload_size > instance.max_payload)
                    {
                      instance.state = PROTOCOL_STATE_CONFUSED;
                      send_status_message(0, PROTOCOL_ERROR_OUT_OF_MEMORY);
                      break;
                    }

                    instance.receive_pointer = instance.payload;
                    instance.num_bytes_awaited = payload_size;
                    instance.state = PROTOCOL_STATE_RECEIVING_PAYLOAD;
                }
                else
                {
                    instance.state = PROTOCOL_STATE_CONFUSED;
                    send_status_message(0, PROTOCOL_ERROR_BAD_ENDPOINT_ID);
                    break;
                }

                /* try to read payload */
                receive_data();

                /* no break, fall through and continue with
                 * PROTOCOL_STATE_RECEIVING_PAYLOAD
                 */
            }
            else
            {
                /* header is not complete, but no more data has been received,
                 * stop and try again later
                 */
                break;
            }

        case PROTOCOL_STATE_RECEIVING_PAYLOAD:
            if (instance.num_bytes_awaited == 0)
            {
                /* prepare for receiving message tail */
                instance.receive_pointer = instance.message_frame + 4;
                instance.num_bytes_awaited = 2;
                instance.state = PROTOCOL_STATE_RECEIVING_TAIL;

                /* try to read payload */
                receive_data();

                /* no break, fall through and continue with
                 * PROTOCOL_STATE_RECEIVING_TAIL
                 */
            }
            else
            {
                /* payload is not complete, but no more data has been
                 * received, stop and try again later
                 */
                break;
            }

        case PROTOCOL_STATE_RECEIVING_TAIL:
            if (instance.num_bytes_awaited == 0)
            {
                uint8_t  endpoint;
                uint16_t payload_size;
                uint16_t end_sequence;

                /* message is complete, now process it */
                endpoint     = rd_payload_u8 (instance.message_frame, 1);
                payload_size = rd_payload_u16(instance.message_frame, 2);
                end_sequence = rd_payload_u16(instance.message_frame, 4);

                /* check end of payload sequence */
                if (end_sequence != CNST_END_OF_PAYLOAD)
                {
                    instance.state = PROTOCOL_STATE_CONFUSED;
                    send_status_message(0, PROTOCOL_ERROR_BAD_PAYLOAD_END);
                    break;
                }

                if (endpoint != 0)
                {
                    uint16_t status_code;
                    Endpoint_t* current_ep;

                    /* now send the payload package to the addressed endpoint
                     * and send the status code back
                     */
                    current_ep = &(instance.endpoints[endpoint-1]);
                    status_code = current_ep->handle_message(endpoint,
                                                             instance.payload,
                                                             payload_size,
                                                             current_ep->
                                                               context);
                    send_status_message(endpoint, status_code);
                }
                else
                {
                    uint16_t status_code;
                    /* endpoint 0 is the protocol itself, so handle message
                     * by the protocol
                     */
                    status_code = handle_message(instance.payload,
                                                 payload_size);
                    send_status_message(0, status_code);
                }

                /* go back to idle state to receive the next message */
                reset_state();
                break;
            }
            else
            {
                /* tail is not complete, but no more data has been received,
                 * stop and try again later
                 */
                break;
            }

        default:
            /* This shouldn't occur. Put this default branch here to suppress
             * a warning.
             */
            break;
        };
    }

    /* if state is confused, keep on reading into a dummy buffer, until
     * receive buffer is empty
     */
    if (instance.state == PROTOCOL_STATE_CONFUSED)
    {
        uint8_t dummy_buffer[64];
        do
        {
            instance.receive_pointer = dummy_buffer;
            instance.num_bytes_awaited = sizeof(dummy_buffer);
            receive_data();
        } while (instance.num_bytes_awaited == 0);

        /*
         * The receive_data function may switch the state back to idle, if a
         * timeout occurs, this is the  only way to get out of confused state.
         */
    }
}

void protocol_send_header(uint8_t endpoint, uint16_t num_bytes)
{
    /* setup message header and send it */
    uint8_t header[4];
    wr_payload_u8 (header, 0, CNST_STARTBYTE_DATA);
    wr_payload_u8 (header, 1, endpoint);
    wr_payload_u16(header, 2, num_bytes);

    com_send_data(header, sizeof(header));
}

void protocol_send_payload(const uint8_t* message_data, uint16_t num_bytes)
{
    /* send the data through the communication stream interface */
    com_send_data(message_data, num_bytes);
}

void protocol_send_tail(void)
{
    /* setup message tail and send it */
    uint8_t tail[2];
    wr_payload_u16(tail, 0, CNST_END_OF_PAYLOAD);

    com_send_data(tail, sizeof(tail));
    com_flush();
}

void protocol_broadcast_change(void* context, uint32_t what)
{
    uint8_t i;

    /* broadcast change notification to all endpoints that have a
     * handle_change function
     */
    for (i = 0; i < instance.num_endpoints; ++i)
    {
        if ((instance.endpoints[i].handle_change) &&
            (instance.endpoints[i].context == context))
        {
            instance.endpoints[i].handle_change(i + 1, context, what);
        }
    }
}

uint16_t ep_target_detection_handle_message (uint8_t endpoint, uint8_t* message_data, uint16_t num_bytes, void* context)
{
	Radar_Handle_t radar_driver = (Radar_Handle_t) context;

	switch (message_data[0])
	{
	case MSG_GET_DSP_SETTINGS:

		if (num_bytes == 1)
		{
			return send_dsp_settings(endpoint, radar_driver);
		}

		break;

	case MSG_SET_DSP_SETTINGS:

		if (num_bytes >= 18)
		{
			DSP_Settings_t dsp_settings;

			/* extract parameters from message */
			dsp_settings.range_mvg_avg_length = rd_payload_u8(message_data, 1);
			dsp_settings.min_range_cm     = rd_payload_u16(message_data, 2);
			dsp_settings.max_range_cm     = rd_payload_u16(message_data, 4);
			dsp_settings.min_speed_kmh    = rd_payload_u16(message_data, 6);
			dsp_settings.max_speed_kmh    = rd_payload_u16(message_data, 8);
			dsp_settings.min_angle_degree = rd_payload_u16(message_data, 10);
			dsp_settings.max_angle_degree = rd_payload_u16(message_data, 12);
			dsp_settings.range_threshold  = rd_payload_u16(message_data, 14);
			dsp_settings.speed_threshold  = rd_payload_u16(message_data, 16);

			// skip uint16 entry for index 18
			dsp_settings.enable_tracking  = rd_payload_u8 (message_data, 20);
			dsp_settings.num_of_tracks    = rd_payload_u8 (message_data, 21);

			dsp_settings.median_filter_length = rd_payload_u8 (message_data, 22);
			dsp_settings.enable_mti_filter    = rd_payload_u8 (message_data, 23);

			dsp_settings.mti_filter_length = rd_payload_u16(message_data, 24);
			// skip uint8 entry for index 26

			/* apply DSP Settings */
			return radar_set_dsp_settings(radar_driver, &dsp_settings);
		}

		break;

	case MSG_GET_TARGETS:

		if (num_bytes == 1)
		{
			return send_targets_info (endpoint, radar_driver);
		}

		break;

	case MSG_GET_RANGE_THRESHOLD:

		if (num_bytes == 1)
		{
			return send_range_threshold (endpoint, radar_driver);
		}

		break;

	default:
		break;
	}

	return PROTOCOL_STATUS_INVALID_PAYLOAD;

}

//==============================================================================

void ep_target_detection_handle_change (uint8_t endpoint, void* context, uint32_t what)
{
	Radar_Handle_t radar_driver = (Radar_Handle_t)context;

	switch (what)
	{
	case EP_RADAR_CHNG_FRAME_FORMAT:
		send_targets_info(endpoint, radar_driver);
		break;

	default:
		break;
	}
}

uint16_t ep_radar_p2g_handle_message(uint8_t endpoint,
                                     uint8_t* message_data,
                                       uint16_t num_bytes,
                                       void* context)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    (void) radar_driver; /* Suppress compiler warnings of unused variable */

	uint16_t error_code;

    switch (message_data[0])
    {

    case MSG_GET_PGA_LEVEL:

        if (num_bytes == 1)
        {
			error_code = send_pga_level(radar_driver, endpoint);

            return error_code;
        }
        break;

    case MSG_SET_PGA_LEVEL:

        if (num_bytes == 3)
        {
            uint16_t  gain_level;

            /* extract parameters from message */
            gain_level = rd_payload_u16(message_data, 1);

            /* apply PGA Gain Level */
            error_code = radar_set_gain_level(radar_driver, gain_level);

            return error_code;
        }
        break;

    default:

		break;
    }

    return PROTOCOL_STATUS_INVALID_PAYLOAD;
}

void ep_radar_p2g_handle_change(uint8_t endpoint, void* context,
                                uint32_t what)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    (void) radar_driver; /* Suppress compiler warnings of unused variable */

    switch (what)
    {
    default:
        break;
    }
}

uint16_t ep_radar_industrial_handle_message(uint8_t endpoint, uint8_t* message_data,
											uint16_t num_bytes,	void* context)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

	uint16_t error_code = 0;

    switch (message_data[0])
    {

    case MSG_DISABLE_DUTY_CYCLE:

        if (num_bytes == 1)
        {
			error_code = radar_set_duty_cycle(radar_driver, 0);

            return error_code;
        }
        break;

	case MSG_ENABLE_DUTY_CYCLE:

        if (num_bytes == 1)
        {
			error_code = radar_set_duty_cycle(radar_driver, 1);

            return error_code;
        }
        break;

	case MSG_IS_ENABLE_DUTY_CYCLE:

		if (num_bytes == 1)
		{
			uint8_t message[2];

			uint8_t flag;

			radar_get_duty_cycle(radar_driver, &flag);

			/* compile message */
			wr_payload_u8(message, 0, MSG_IS_ENABLE_DUTY_CYCLE);
			wr_payload_u8(message, 1, flag);

			/* send message */
			protocol_send_header(endpoint, sizeof(message));
			protocol_send_payload(message, sizeof(message));
			protocol_send_tail();

			return RADAR_ERR_OK;
		}

		break;

    case MSG_DISABLE_BGT_LNA:

        if (num_bytes == 1)
        {
        	radar_disable_lna_gain(radar_driver);

            return RADAR_ERR_OK;
        }
        break;

	case MSG_ENABLE_BGT_LNA:

        if (num_bytes == 1)
        {
        	radar_enable_lna_gain(radar_driver);

            return RADAR_ERR_OK;
        }
        break;

	case MSG_IS_ENABLE_BGT_LNA:

		if (num_bytes == 1)
		{
			uint8_t message[2];
			uint8_t flag;

			flag = radar_get_lna_gain_enable_status(radar_driver);
			/* compile message */
			wr_payload_u8(message, 0, MSG_IS_ENABLE_BGT_LNA);
			wr_payload_u8(message, 1, flag);

			/* send message */
			protocol_send_header(endpoint, sizeof(message));
			protocol_send_payload(message, sizeof(message));
			protocol_send_tail();

			return RADAR_ERR_OK;
		}

		break;

    default:

		break;
    }

    return PROTOCOL_STATUS_INVALID_PAYLOAD;
}

void ep_radar_industrial_handle_change(uint8_t endpoint, void* context,
                                  	   uint32_t what)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    (void) radar_driver; /* Suppress compiler warnings of unused variable */

    switch (what)
    {
    default:
        break;
    }
}

uint16_t ep_radar_fmcw_handle_message(uint8_t endpoint, uint8_t* message_data,
                                      uint16_t num_bytes, void* context)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    switch (message_data[0])
    {
    case MSG_GET_CONFIGURATION:
        if (num_bytes == 1)
        {
            return send_fmcw_configuration(endpoint, radar_driver);
        }
        break;

    case MSG_SET_CONFIGURATION:
        if (num_bytes == 11)
        {
            unsigned error_code;
            Fmcw_Configuration_t config;

            /* extract parameters from message */
            config.lower_frequency_kHz = rd_payload_u32(message_data,  1);
            config.upper_frequency_kHz = rd_payload_u32(message_data,  5);
            config.direction           = rd_payload_u8 (message_data,  9);
            config.tx_power            = rd_payload_u8 (message_data, 10);

            /* apply FMCW configuration */
            error_code = radar_set_fmcw_configuration(radar_driver, &config);

            /* inform other endpoints of the change */
            protocol_broadcast_change(radar_driver,
                                      EP_RADAR_CHNG_FMCW_SETTINGS);

            return error_code;
        }
        break;

    case MSG_GET_BW_PER_SECOND:
        if (num_bytes == 1)
        {
            return send_bandwidth_per_second(endpoint, radar_driver);
        }
        break;

    default:
        break;
    }
    return PROTOCOL_STATUS_INVALID_PAYLOAD;
}

void ep_radar_fmcw_handle_change(uint8_t endpoint, void* context,
                                 uint32_t what)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    switch (what)
    {
    case EP_RADAR_CHNG_FRAME_FORMAT:
        send_bandwidth_per_second(endpoint, radar_driver);
        break;

    case EP_RADAR_CHNG_FMCW_SETTINGS:
        send_fmcw_configuration(endpoint, radar_driver);
        send_bandwidth_per_second(endpoint, radar_driver);
        break;

    case EP_RADAR_CHNG_ADC_SETTINGS:
        send_bandwidth_per_second(endpoint, radar_driver);
        break;

    default:
        break;
    }
}

uint16_t ep_radar_doppler_handle_message(uint8_t endpoint,
                                         uint8_t* message_data,
                                         uint16_t num_bytes,
                                         void* context)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    switch (message_data[0])
    {
    case MSG_GET_CONFIGURATION:
        if (num_bytes == 1)
        {
            return send_doppler_configuration(endpoint, radar_driver);
        }
        break;

    case MSG_SET_CONFIGURATION:
        if (num_bytes == 6)
        {
            unsigned error_code;
            Doppler_Configuration_t configuration;

            /* extract parameters from message */
            configuration.frequency_kHz = rd_payload_u32(message_data, 1);
            configuration.tx_power      = rd_payload_u8 (message_data, 5);

            /* apply FMCW configuration */
            error_code = radar_set_doppler_configuration(radar_driver,
                                                         &configuration);

            /* inform other endpoints of the change */
            protocol_broadcast_change(radar_driver,
                                      EP_RADAR_CHNG_DOPPLER_SETTINGS);

            return error_code;
        }
        break;

    default:
        break;
    }
    return PROTOCOL_STATUS_INVALID_PAYLOAD;
}

void ep_radar_doppler_handle_change(uint8_t endpoint, void* context,
                                    uint32_t what)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    if (what == EP_RADAR_CHNG_DOPPLER_SETTINGS)
        send_doppler_configuration(endpoint, radar_driver);
}

uint16_t ep_radar_base_handle_message(uint8_t endpoint, uint8_t* message_data,
                                      uint16_t num_bytes, void* context)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    switch (message_data[0])
    {
    case MSG_GET_FRAME_DATA:
        if (num_bytes == 2)
        {
            uint8_t wait;

            /* extract parameters from message */
            wait = rd_payload_u8(message_data, 1);

            return send_frame_data(endpoint, radar_driver, wait);
        }
        break;

    case MSG_SET_AUTOMATIC_TRIGGER:
        if (num_bytes == 5)
        {
            unsigned error_code;
            uint32_t frame_interval;

            /* extract parameters from message */
            frame_interval = rd_payload_u32(message_data, 1);

            /* apply new settings */
            error_code = radar_set_automatic_frame_trigger(radar_driver,
                                                           frame_interval);

            return error_code;
        }
        break;

    case MSG_ENABLE_TEST_MODE:
        if (num_bytes == 8)
        {
            unsigned error_code;
            uint8_t tx_mask;
            uint8_t rx_mask;
            uint32_t frequency;
            uint8_t tx_power;

            /* extract parameters from message */
            tx_mask   = rd_payload_u8 (message_data, 1);
            rx_mask   = rd_payload_u8 (message_data, 2);
            frequency = rd_payload_u32(message_data, 3);
            tx_power  = rd_payload_u8 (message_data, 7);

            /* apply new settings */
            error_code = radar_test_antennas(radar_driver, tx_mask, rx_mask,
                                             frequency, tx_power);

            return error_code;
        }
        break;

    case MSG_GET_DRIVER_VERSION:
        if (num_bytes == 1)
        {
            /* send driver version info back to host */
            send_driver_version(endpoint);

            return RADAR_ERR_OK;
        }
        break;

    case MSG_GET_DEVICE_INFO:
        if (num_bytes == 1)
        {
            return send_device_info(endpoint, radar_driver);
        }
        break;

    case MSG_GET_TEMPRATURE:
    if (num_bytes == 2)
        {
            uint8_t temp_sensor;

            /* extract parameters from message */
            temp_sensor = rd_payload_u8(message_data, 1);

            /* send temperature value back to host */
            return send_temperature(endpoint, radar_driver, temp_sensor);
        }
        break;

    case MSG_GET_TX_POWER:
    if (num_bytes == 2)
        {
            uint8_t tx_antenna;

            /* extract parameters from message */
            tx_antenna = rd_payload_u8(message_data, 1);

            return send_tx_power(endpoint, radar_driver, tx_antenna);
        }
        break;

    case MSG_GET_CHRIP_DURATION:
    if (num_bytes == 1)
        {
            return send_chirp_duration(endpoint, radar_driver);
        }
        break;

    case MSG_GET_MIN_INTERVAL:
    if (num_bytes == 1)
        {
            return send_min_frame_interval(endpoint, radar_driver);
        }
        break;

    case MSG_GET_FRAME_FORMAT:
        if (num_bytes == 1)
        {
            return send_frame_format(endpoint, radar_driver);
        }
        break;

    case MSG_SET_FRAME_FORMAT:
        if (num_bytes == 11)
        {
            unsigned error_code;
            Frame_Format_t format;

            /* extract parameters from message */
            format.num_samples_per_chirp = rd_payload_u32(message_data,  1);
            format.num_chirps_per_frame  = rd_payload_u32(message_data,  5);
            format.rx_mask               = rd_payload_u8 (message_data,  9);
            format.signal_part           = rd_payload_u8 (message_data, 10);

            /* apply new frame format */
            error_code = radar_set_frame_format(radar_driver, &format);

            /* inform other endpoints of the change */
            protocol_broadcast_change(radar_driver,
                                      EP_RADAR_CHNG_FRAME_FORMAT);

            return error_code;
        }
        break;

    default:
        break;
    }
    return PROTOCOL_STATUS_INVALID_PAYLOAD;
}

void ep_radar_base_handle_change(uint8_t endpoint, void* context,
                                 uint32_t what)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    switch (what)
    {
    case EP_RADAR_CHNG_FRAME_FORMAT:
        send_frame_format(endpoint, radar_driver);
        send_chirp_duration(endpoint, radar_driver);
        send_min_frame_interval(endpoint, radar_driver);
        break;

    case EP_RADAR_CHNG_FMCW_SETTINGS:
    case EP_RADAR_CHNG_DOPPLER_SETTINGS:
    case EP_RADAR_CHNG_TX_MODE:
        {
            uint8_t tx_antenna = 0;
            uint16_t error_code = RADAR_ERR_OK;

            while (error_code == RADAR_ERR_OK)
            {
                error_code = send_tx_power(endpoint, radar_driver,
                                           tx_antenna++);
            }
        }
        break;

    case EP_RADAR_CHNG_ADC_SETTINGS:
        send_chirp_duration(endpoint, radar_driver);
        send_min_frame_interval(endpoint, radar_driver);
        break;

    default:
        break;
    }
}

uint16_t ep_radar_adcxmc_handle_message(uint8_t endpoint,
                                        uint8_t* message_data,
                                        uint16_t num_bytes,
                                        void* context)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    switch (message_data[0])
    {
    case MSG_GET_CONFIGURATION:
        if (num_bytes == 1)
        {
            return send_adc_configuration(endpoint, radar_driver);
        }
        break;

    case MSG_SET_CONFIGURATION:
        if (num_bytes == 7)
        {
            unsigned error_code;
            Adc_Configuration_t config;

            /* extract parameters from message */
            config.samplerate_Hz        = rd_payload_u32(message_data, 1);
            config.resolution           = rd_payload_u8 (message_data, 5);
            config.use_post_calibration = rd_payload_u8 (message_data, 6);

            /* apply ADC configuration */
            error_code = radar_set_adc_configuration(radar_driver, &config);

            /* inform other endpoints of the change */
            protocol_broadcast_change(radar_driver,
                                      EP_RADAR_CHNG_ADC_SETTINGS);

            return error_code;
        }
        break;

    default:
        break;
    }
    return PROTOCOL_STATUS_INVALID_PAYLOAD;
}

void ep_radar_adcxmc_handle_change(uint8_t endpoint, void* context,
                                   uint32_t what)
{
    Radar_Handle_t radar_driver = (Radar_Handle_t)context;

    if (what == EP_RADAR_CHNG_ADC_SETTINGS)
        send_adc_configuration(endpoint, radar_driver);
}

uint16_t ep_calibration_handle_message(uint8_t endpoint, uint8_t* message_data, uint16_t num_bytes, void* context)
{
	Radar_Handle_t radar_driver = (Radar_Handle_t) context;

	uint16_t error_code;

	switch (message_data[0])
	{

	case MSG_GET_ADC_FLASH_CALIBRATION:

		if (num_bytes == 1)
		{
			error_code = send_adc_calibration_data(endpoint, radar_driver, CALIBRATION_TARGET_FLASH);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	case MSG_GET_ADC_SRAM_CALIBRATION:

		if (num_bytes == 1)
		{
			error_code = send_adc_calibration_data(endpoint, radar_driver, CALIBRATION_TARGET_SRAM);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	case MSG_SET_ADC_FLASH_CALIBRATION:

		if (num_bytes == 1)
		{
			error_code = radar_save_calibration(radar_driver, CALIBRATION_TARGET_FLASH, CALIBRATION_DATA_ADC, NULL);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	case MSG_SET_ADC_SRAM_CALIBRATION:

		if (num_bytes == 1)
		{
			error_code = radar_save_calibration(radar_driver, CALIBRATION_TARGET_SRAM,CALIBRATION_DATA_ADC, NULL);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	case MSG_CLEAR_ADC_FLASH_CALIBRATION:

		if (num_bytes == 1)
		{
			error_code = radar_clear_calibration(radar_driver, CALIBRATION_TARGET_FLASH, CALIBRATION_DATA_ADC);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	case MSG_CLEAR_ADC_SRAM_CALIBRATION:

		if (num_bytes == 1)
		{
			error_code = radar_clear_calibration(radar_driver, CALIBRATION_TARGET_SRAM, CALIBRATION_DATA_ADC);

			return error_code;
		}

		break;

		//===============================================================

	case MSG_GET_ALGO_FLASH_CALIBRATION:

		if (num_bytes == 1)
		{
			error_code = send_algo_calibration_data(endpoint, radar_driver, CALIBRATION_TARGET_FLASH);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	case MSG_GET_ALGO_SRAM_CALIBRATION:

		if (num_bytes == 1)
		{
			error_code = send_algo_calibration_data(endpoint, radar_driver, CALIBRATION_TARGET_SRAM);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	case MSG_SET_ALGO_FLASH_CALIBRATION:

		if (num_bytes == 5)
		{
			Algo_Calibrations_t algo_calibration_settings;

			/* extract parameters from message */

			algo_calibration_settings.distance_offset_cm = rd_payload_u16(message_data, 1);
			algo_calibration_settings.angle_offset_deg   = rd_payload_i16(message_data, 3);

			error_code = radar_save_calibration(radar_driver, CALIBRATION_TARGET_FLASH, CALIBRATION_DATA_ALGO, &algo_calibration_settings);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	case MSG_SET_ALGO_SRAM_CALIBRATION:

		if (num_bytes == 5)
		{
			Algo_Calibrations_t algo_calibration_settings;

			/* extract parameters from message */
			algo_calibration_settings.distance_offset_cm = rd_payload_u16(message_data, 1);
			algo_calibration_settings.angle_offset_deg   = rd_payload_i16(message_data, 3);

			error_code = radar_save_calibration(radar_driver, CALIBRATION_TARGET_SRAM, CALIBRATION_DATA_ALGO, &algo_calibration_settings);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	case MSG_CLEAR_ALGO_FLASH_CALIBRATION:

		if (num_bytes == 1)
		{
			error_code = radar_clear_calibration(radar_driver, CALIBRATION_TARGET_FLASH, CALIBRATION_DATA_ALGO);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	case MSG_CLEAR_ALGO_SRAM_CALIBRATION:

		if (num_bytes == 1)
		{
			error_code = radar_clear_calibration(radar_driver, CALIBRATION_TARGET_SRAM, CALIBRATION_DATA_ALGO);

			return error_code;
		}

		break;

		//---------------------------------------------------------------

	default:

		break;
	}

	return PROTOCOL_STATUS_INVALID_PAYLOAD;
}

//==============================================================================

void ep_calibration_handle_change (uint8_t endpoint, void* context, uint32_t what)
{
	Radar_Handle_t radar_driver = (Radar_Handle_t)context;

	(void) radar_driver; /* Suppress compiler warnings of unused variable */

	switch (what)
	{
	case MSG_SET_ALGO_FLASH_CALIBRATION:
	case MSG_SET_ALGO_SRAM_CALIBRATION:
		break;
	default:
		break;
	}
}

/******************************************************************************
   7. LOCAL FUNCTIONS
*******************************************************************************/
static void send_status_message(uint8_t endpoint, uint16_t status_code)
{
    /* setup status message */
    uint8_t status_message[4];
    wr_payload_u8 (status_message, 0, CNST_STARTBYTE_STATUS);
    wr_payload_u8 (status_message, 1, endpoint);
    wr_payload_u16(status_message, 2, status_code);

    /* now send message */
    com_send_data(status_message, sizeof(status_message));
    com_flush();
}

static void reset_state(void)
{
    /* set to idle state and store incoming data in message frame buffer */
    instance.state            = PROTOCOL_STATE_IDLE;
    instance.receive_pointer   = instance.message_frame;
    instance.num_bytes_awaited = 4;
}

static void receive_data(void)
{
    /* receive data from communication interface */
    uint16_t num_received_bytes = com_get_data(instance.receive_pointer,
                                               instance.num_bytes_awaited);
    instance.receive_pointer += num_received_bytes;
    instance.num_bytes_awaited -= num_received_bytes;

    /* check for timeout */
    if (instance.get_time != NULL)
    {
        /* get current time */
        uint32_t current_time = instance.get_time();

        /* if no data has been received, check the time since last received
         * bytes
         */
        if (num_received_bytes == 0)
        {
            if ((instance.state != PROTOCOL_STATE_IDLE) &&
                (current_time >= instance.time_of_last_byte +
                                 instance.timeout_interval))
            {
                /* a timeout has occurred, reset protocol state */
                if (instance.state != PROTOCOL_STATE_CONFUSED)
                    send_status_message(0, PROTOCOL_ERROR_TIMEOUT);
                reset_state();
            }
        }
        else
        {
            /* data has been received, so remember current time for next
             * timeout check
             */
            instance.time_of_last_byte = current_time;
        }
    }
}

static uint16_t handle_message(uint8_t* message_data, uint16_t num_bytes)
{
    /* check command byte of received message */
    switch (message_data[0])
    {
    case CNST_MSG_QUERY_ENDPOINT_INFO:
        if (num_bytes == 1)
        {
            send_endpoint_info();
            return PROTOCOL_ERROR_OK;
        }
        break;

    case CNST_MSG_QUERY_FW_INFO:
        if (num_bytes == 1)
        {
            send_firmware_info();
            return PROTOCOL_ERROR_OK;
        }
        break;

    case CNST_MSG_FIRMWARE_RESET:
        if (num_bytes == 1)
        {
            /* if user did not set system reset function, this feature is not
             * supported.
             */
            if (instance.do_system_reset == NULL)
                return PROTOCOL_STATUS_INVALID_PAYLOAD;

            /* send status message first, after the reset it won't be
             * possible.
             */
            send_status_message(0, PROTOCOL_ERROR_OK);

            /* now do the system reset */
            instance.do_system_reset();

            return PROTOCOL_ERROR_OK;
        }
        break;

    default:
        break;
    }

    return PROTOCOL_ERROR_BAD_COMMAND;
}

static void send_endpoint_info(void)
{
    uint8_t i;

    /* setup message start (type und number of endpoints */
    uint8_t message_header[2];
    wr_payload_u8(message_header, 0, CNST_MSG_ENDPOINT_INFO);
    wr_payload_u8(message_header, 1, instance.num_endpoints);

    /* calculate message size and send start of message */
    protocol_send_header(0, 6 * instance.num_endpoints +
                            sizeof(message_header));
    protocol_send_payload(message_header, sizeof(message_header));

    /* now send type and version for each endpoint */
    for (i = 0; i < instance.num_endpoints; ++i)
    {
        uint8_t ep_info[6];
        wr_payload_u32(ep_info, 0, instance.endpoints[i].endpoint_type);
        wr_payload_u16(ep_info, 4, instance.endpoints[i].endpoint_version);
        protocol_send_payload(ep_info, sizeof(ep_info));
    }

    /* finish the message */
    protocol_send_tail();
}

static void send_firmware_info(void)
{
    uint16_t string_length = 0;

    /* setup message start (type and number of endpoints */
    uint8_t message[7];
    wr_payload_u8 (message, 0, CNST_MSG_FW_INFO);
    wr_payload_u16(message, 1, firmware_information.version_major);
    wr_payload_u16(message, 3, firmware_information.version_minor);
    wr_payload_u16(message, 5, firmware_information.version_build);

    /* count length of firmware description string */
    while (firmware_information.description[string_length] != 0)
        ++string_length;
    ++string_length;

    /* send message */
    protocol_send_header(0, sizeof(message)  + string_length);
    protocol_send_payload(message, sizeof(message));
    protocol_send_payload((const uint8_t*)firmware_information.description,
                          string_length);
    protocol_send_tail();
}

/*
 * The following function implementations are safe on any platform, but slower
 * than the native little endian implementations.
 */

static inline int16_t rd_payload_i16(const uint8_t* payload, uint16_t offset)
{
    return (int16_t)((((uint16_t)payload[offset + 1]) <<  8) |
                      ((uint16_t)payload[offset    ]));
}

static inline uint16_t rd_payload_u16(const uint8_t* payload, uint16_t offset)
{
    return (((uint16_t)payload[offset + 1]) <<  8) |
            ((uint16_t)payload[offset    ]);
}

static inline int32_t rd_payload_i32(const uint8_t* payload, uint16_t offset)
{
    return (int32_t)((((uint32_t)payload[offset + 3]) << 24) |
                     (((uint32_t)payload[offset + 2]) << 16) |
                     (((uint32_t)payload[offset + 1]) <<  8) |
                      ((uint32_t)payload[offset    ]));
}

static inline uint32_t rd_payload_u32(const uint8_t* payload, uint16_t offset)
{
    return (((uint32_t)payload[offset + 3]) << 24) |
           (((uint32_t)payload[offset + 2]) << 16) |
           (((uint32_t)payload[offset + 1]) <<  8) |
            ((uint32_t)payload[offset    ]);
}

static inline int64_t rd_payload_i64(const uint8_t* payload, uint16_t offset)
{
    return (int64_t)((((uint64_t)payload[offset + 7]) << 56) |
                     (((uint64_t)payload[offset + 6]) << 48) |
                     (((uint64_t)payload[offset + 5]) << 40) |
                     (((uint64_t)payload[offset + 4]) << 32) |
                     (((uint64_t)payload[offset + 3]) << 24) |
                     (((uint64_t)payload[offset + 2]) << 16) |
                     (((uint64_t)payload[offset + 1]) <<  8) |
                      ((uint64_t)payload[offset    ]));
}

static inline uint64_t rd_payload_u64(const uint8_t* payload, uint16_t offset)
{
    return (((uint64_t)payload[offset + 7]) << 56) |
           (((uint64_t)payload[offset + 6]) << 48) |
           (((uint64_t)payload[offset + 5]) << 40) |
           (((uint64_t)payload[offset + 4]) << 32) |
           (((uint64_t)payload[offset + 3]) << 24) |
           (((uint64_t)payload[offset + 2]) << 16) |
           (((uint64_t)payload[offset + 1]) <<  8) |
            ((uint64_t)payload[offset    ]);
}

static inline void wr_payload_i16(uint8_t* payload, uint16_t offset,
                                  int16_t value)
{
    payload[offset + 1] = (((uint16_t)value) >>  8) & 0xFF;
    payload[offset    ] =  ((uint16_t)value)        & 0xFF;
}

static inline void wr_payload_u16(uint8_t* payload, uint16_t offset,
                                  uint16_t value)
{
    payload[offset + 1] = (value >>  8) & 0xFF;
    payload[offset    ] =  value        & 0xFF;
}

static inline void wr_payload_i32(uint8_t* payload, uint16_t offset,
                                  int32_t value)
{
    payload[offset + 3] = (((uint32_t)value) >> 24) & 0xFF;
    payload[offset + 2] = (((uint32_t)value) >> 16) & 0xFF;
    payload[offset + 1] = (((uint32_t)value) >>  8) & 0xFF;
    payload[offset    ] =  ((uint32_t)value)        & 0xFF;
}

static inline void wr_payload_u32(uint8_t* payload, uint16_t offset,
                                  uint32_t value)
{
    payload[offset + 3] = (value >> 24) & 0xFF;
    payload[offset + 2] = (value >> 16) & 0xFF;
    payload[offset + 1] = (value >>  8) & 0xFF;
    payload[offset    ] =  value        & 0xFF;
}

static inline void wr_payload_i64(uint8_t* payload, uint16_t offset,
                                  int64_t value)
{
    payload[offset + 7] = (((uint64_t)value) >> 56) & 0xFF;
    payload[offset + 6] = (((uint64_t)value) >> 48) & 0xFF;
    payload[offset + 5] = (((uint64_t)value) >> 40) & 0xFF;
    payload[offset + 4] = (((uint64_t)value) >> 32) & 0xFF;
    payload[offset + 3] = (((uint64_t)value) >> 24) & 0xFF;
    payload[offset + 2] = (((uint64_t)value) >> 16) & 0xFF;
    payload[offset + 1] = (((uint64_t)value) >>  8) & 0xFF;
    payload[offset    ] =  ((uint64_t)value)        & 0xFF;
}

static inline void wr_payload_u64(uint8_t* payload, uint16_t offset,
                                  uint64_t value)
{
    payload[offset + 7] = (value >> 56) & 0xFF;
    payload[offset + 6] = (value >> 48) & 0xFF;
    payload[offset + 5] = (value >> 40) & 0xFF;
    payload[offset + 4] = (value >> 32) & 0xFF;
    payload[offset + 3] = (value >> 24) & 0xFF;
    payload[offset + 2] = (value >> 16) & 0xFF;
    payload[offset + 1] = (value >>  8) & 0xFF;
    payload[offset    ] =  value        & 0xFF;
}

static uint16_t send_dsp_settings (uint8_t endpoint, Radar_Handle_t radar_driver)
{
	uint16_t error_code = 0;

	DSP_Settings_t  dsp_settings;

	error_code = radar_get_dsp_settings(radar_driver, &dsp_settings);

	if (error_code == RADAR_ERR_OK)
	{
		uint8_t message[27];

		/* compile message */
		wr_payload_u8 (message,  0, MSG_GET_DSP_SETTINGS);

		wr_payload_u8 (message,  1, dsp_settings.range_mvg_avg_length);
		wr_payload_u16(message,  2, dsp_settings.min_range_cm);
		wr_payload_u16(message,  4, dsp_settings.max_range_cm);
		wr_payload_u16(message,  6, dsp_settings.min_speed_kmh);
		wr_payload_u16(message,  8, dsp_settings.max_speed_kmh);
		wr_payload_u16(message, 10, dsp_settings.min_angle_degree);
		wr_payload_u16(message, 12, dsp_settings.max_angle_degree);
		wr_payload_u16(message, 14, dsp_settings.range_threshold);
		wr_payload_u16(message, 16, dsp_settings.speed_threshold);

		wr_payload_u16(message, 18, 0); // see MMWSW-603
		wr_payload_u8 (message, 20, dsp_settings.enable_tracking);
		wr_payload_u8 (message, 21, dsp_settings.num_of_tracks);

		wr_payload_u8 (message, 22, dsp_settings.median_filter_length);
		wr_payload_u8 (message, 23, dsp_settings.enable_mti_filter);

		wr_payload_u16(message, 24, dsp_settings.mti_filter_length);
		wr_payload_u8 (message, 26, 0); // see MMWSW-603

		/* send message */
		protocol_send_header(endpoint, sizeof(message));
		protocol_send_payload(message, sizeof(message));
		protocol_send_tail();
	}

	return error_code;
}

//==============================================================================

static uint16_t send_targets_info (uint8_t endpoint, Radar_Handle_t radar_driver)
{
	uint16_t i;

	uint16_t error_code = 0;

	uint8_t num_of_targets = 0;

	uint8_t message = MSG_GET_TARGETS;

	for (i = 0; i < MAX_NUM_OF_TARGETS; i++)		// initialize target list
	{
		target_list[i].target_id = 0;
		target_list[i].level = 0.f;
		target_list[i].radius = 0.f;
		target_list[i].azimuth = 0.f;
		target_list[i].elevation = 0.f;
		target_list[i].radial_speed = 0.f;
		target_list[i].azimuth_speed = 0.f;
		target_list[i].elevation_speed = 0.f;
	}

	error_code = radar_get_target_info(radar_driver, target_list, &num_of_targets);

	if (error_code == RADAR_ERR_OK)
	{
		/* send message */
		protocol_send_header(endpoint, num_of_targets * sizeof(Target_Info_t) + 1 + 1);

		protocol_send_payload (&message, 1);

		protocol_send_payload ((const uint8_t*) &num_of_targets, 1);

		for (i = 0; i < num_of_targets; i++)
		{
			protocol_send_payload ((uint8_t*) &target_list[i], sizeof(Target_Info_t));
		}

		protocol_send_tail();
	}

	return error_code;
}

//==============================================================================

static uint16_t send_range_threshold (uint8_t endpoint, Radar_Handle_t radar_driver)
{
	uint16_t error_code = 0;

	uint16_t threshold = 0;

	error_code = radar_get_range_detection_threshold(radar_driver, &threshold);

	if (error_code == RADAR_ERR_OK)
	{
		uint8_t message[3];

		/* compile message */
		wr_payload_u8 (message, 0, MSG_GET_RANGE_THRESHOLD);
		wr_payload_u16(message, 1, threshold);

		/* send message */
		protocol_send_header(endpoint, sizeof(message));
		protocol_send_payload(message, sizeof(message));
		protocol_send_tail();
	}

	return error_code;
}

static uint16_t send_pga_level(Radar_Handle_t device, uint8_t endpoint)
{
    uint16_t error_code;

	uint16_t gain_level;

    /* get current PGA Gain level from driver */
	error_code = radar_get_gain_level(device, &gain_level);

    if (error_code == RADAR_ERR_OK)
    {
        uint8_t message[3];

        /* compile message */
        wr_payload_u8 (message, 0, MSG_SET_PGA_LEVEL);
        wr_payload_u16(message, 1, gain_level);

        /* send message */
        protocol_send_header(endpoint, sizeof(message));
        protocol_send_payload(message, sizeof(message));
        protocol_send_tail();
    }

    return error_code;
}

static uint16_t send_fmcw_configuration(uint8_t endpoint,
                                        Radar_Handle_t radar_driver)
{
    uint16_t error_code;
    Fmcw_Configuration_t fmcw_configuration;

    /* read FMCW configuration from device */
    error_code = radar_get_fmcw_configuration(radar_driver,
                                              &fmcw_configuration);

    if (error_code == RADAR_ERR_OK)
    {
        uint8_t message[11];

        /* compile message */
        wr_payload_u8 (message,  0, MSG_SET_CONFIGURATION);
        wr_payload_u32(message,  1, fmcw_configuration.lower_frequency_kHz);
        wr_payload_u32(message,  5, fmcw_configuration.upper_frequency_kHz);
        wr_payload_u8 (message,  9, fmcw_configuration.direction);
        wr_payload_u8 (message, 10, fmcw_configuration.tx_power);

        /* send message */
        protocol_send_header(endpoint, sizeof(message));
        protocol_send_payload(message, sizeof(message));
        protocol_send_tail();
    }
    return error_code;
}

static uint16_t send_bandwidth_per_second(uint8_t endpoint,
                                          Radar_Handle_t radar_driver)
{
    uint16_t error_code;
    uint32_t bandwidth_per_second;

    /* read bandwidth per second from device */
    error_code = radar_get_bandwidth_per_second(radar_driver,
                                                &bandwidth_per_second);

    if (error_code == RADAR_ERR_OK)
    {
        uint8_t message[5];

        /* compile message */
        wr_payload_u8 (message, 0, MSG_SET_BW_PER_SECOND);
        wr_payload_u32(message, 1, bandwidth_per_second);

        /* send message */
        protocol_send_header(endpoint, sizeof(message));
        protocol_send_payload(message, sizeof(message));
        protocol_send_tail();
    }
    return error_code;
}


static uint16_t send_doppler_configuration(uint8_t endpoint,
                                           Radar_Handle_t radar_driver)
{
    uint16_t error_code;
    Doppler_Configuration_t doppler_configuration;

    /* read FMCW configuration from device */
    error_code = radar_get_doppler_configuration(radar_driver,
                                                 &doppler_configuration);

    if (error_code == RADAR_ERR_OK)
    {
        uint8_t message[6];

        /* compile message */
        wr_payload_u8 (message, 0, MSG_SET_CONFIGURATION);
        wr_payload_u32(message, 1, doppler_configuration.frequency_kHz);
        wr_payload_u8 (message, 5, doppler_configuration.tx_power);

        /* send message */
        protocol_send_header(endpoint, sizeof(message));
        protocol_send_payload(message, sizeof(message));
        protocol_send_tail();
    }
    return error_code;
}

static void send_driver_version(uint8_t endpoint)
{
    const Driver_Version_t* driver_version = radar_get_driver_version();

    uint8_t message[4];

    /* compile message */
    wr_payload_u8(message, 0, MSG_SET_DRIVER_VERSION);
    wr_payload_u8(message, 1, driver_version->major);
    wr_payload_u8(message, 2, driver_version->minor);
    wr_payload_u8(message, 3, driver_version->revision);

    /* send message */
    protocol_send_header(endpoint, sizeof(message));
    protocol_send_payload(message, sizeof(message));
    protocol_send_tail();
}

static uint16_t send_frame_data(uint8_t endpoint, Radar_Handle_t radar_driver,
                                uint8_t wait)
{
    uint16_t error_code;

    Frame_Info_t frame_info;

    /* get frame data */
    error_code = radar_get_frame(radar_driver, &frame_info, wait);
    if (error_code == RADAR_ERR_OK)
    {
    	/* send message containing frame data */
        uint32_t num_samples;
        uint32_t data_size;
        uint8_t message_header[18];

        /* setup message header */
        wr_payload_u8 (message_header,  0, MSG_FRAME_DATA);
        wr_payload_u32(message_header,  1, frame_info.frame_number);
        wr_payload_u32(message_header,  5, frame_info.num_chirps);
        wr_payload_u8 (message_header,  9, frame_info.num_rx_antennas);
        wr_payload_u32(message_header, 10, frame_info.num_samples_per_chirp);
        wr_payload_u8 (message_header, 14, frame_info.rx_mask        );
        wr_payload_u8 (message_header, 15, frame_info.data_format    );
        wr_payload_u8 (message_header, 16, frame_info.adc_resolution );
        wr_payload_u8 (message_header, 17, frame_info.interleaved_rx );

        /* calculate size of data */
        /* calculate total number of bits and divide by 8 to get number of
           bytes. Always round upwards */
        num_samples = frame_info.num_chirps *
                      frame_info.num_samples_per_chirp *
                      frame_info.num_rx_antennas *
                      (frame_info.data_format == RADAR_RX_DATA_REAL ? 1 : 2);

        data_size = num_samples * frame_info.adc_resolution;
        data_size = (data_size >> 3) + ((data_size & 0x07) ? 1 : 0);

        /* send message header */
        protocol_send_header(endpoint, data_size + sizeof(message_header));
        protocol_send_payload(message_header, sizeof(message_header));

        if (frame_info.adc_resolution == 12)
        {
        	/** Send Raw Time Domain Data captured during one frame **/
            /* for 12 bit resolution, do optimized in-place packing of data */
            const uint16_t* unpacked_data = frame_info.sample_data;
            uint8_t* packed_data = (uint8_t*)unpacked_data;

            while (num_samples >= 2)
            {
                /* pack two samples into one 32 bit word */
                uint32_t packed_word;
                packed_word  = *unpacked_data++;
                packed_word |= *unpacked_data++ << 12;

                /* write packed word into buffer */
                wr_payload_u32(packed_data, 0, packed_word);
                packed_data += 3;

                num_samples -= 2;
            }
            if (num_samples != 0)
                wr_payload_u16(packed_data, 0, *unpacked_data);

            /* now send the packed data */
            protocol_send_payload((uint8_t*)frame_info.sample_data,
                                            data_size);
        }
        else
        {
            uint16_t sample_bit_mask = (1 << frame_info.adc_resolution) - 1;
            uint16_t sample_bit_stream = 0;
            uint16_t write_bit_position = 0;

            for (unsigned i = 0; i < num_samples; ++i)
            {
                const uint16_t* unpacked_data = frame_info.sample_data;
                sample_bit_stream |= (unpacked_data[i] & sample_bit_mask) <<
                                      write_bit_position;
                write_bit_position += frame_info.adc_resolution;

                protocol_send_payload((uint8_t*)&sample_bit_stream,
                                      write_bit_position >> 3);
                sample_bit_stream >>= write_bit_position & 0xF8;
                write_bit_position &= 0x07;
            }
            if (write_bit_position != 0)
                protocol_send_payload((uint8_t*)&sample_bit_stream, 1);
        }

        protocol_send_tail();

        /* send another message, containing the temperature */
        {
            uint8_t message[6];

            /* compile message */
            wr_payload_u8 (message, 0, MSG_SET_TEMPRATURE);
            wr_payload_u8 (message, 1, 0);
            wr_payload_i32(message, 2, frame_info.temperature_001C);

            /* send message */
            protocol_send_header(endpoint, sizeof(message));
            protocol_send_payload(message, sizeof(message));
            protocol_send_tail();
        }
    }
    return error_code;
}

static uint16_t send_device_info(uint8_t endpoint,
                                 Radar_Handle_t radar_driver)
{
    uint16_t error_code;
    Device_Info_t device_info;

    /* read device info from device */
    error_code = radar_get_device_info(radar_driver, &device_info);

    if (error_code == RADAR_ERR_OK)
    {
        uint16_t string_length;
        uint8_t message[17];

        /* compile message */
        wr_payload_u8 (message,  0, MSG_SET_DEVICE_INFO);
        wr_payload_u32(message,  1, device_info.min_rf_frequency_kHz);
        wr_payload_u32(message,  5, device_info.max_rf_frequency_kHz);
        wr_payload_u8 (message,  9, device_info.num_tx_antennas);
        wr_payload_u8 (message, 10, device_info.num_rx_antennas);
        wr_payload_u8 (message, 11, device_info.max_tx_power);
        wr_payload_u8 (message, 12, device_info.num_temp_sensors);
        wr_payload_u8 (message, 13, device_info.major_version_hw);
        wr_payload_u8 (message, 14, device_info.minor_version_hw);
        wr_payload_u8 (message, 15, device_info.interleaved_rx);
        wr_payload_u8 (message, 16, device_info.data_format);

        /* count string length (add 1 for the terminating 0) */
        string_length = 0;
        while (device_info.description[string_length] != 0)
            ++string_length;
        ++string_length;

        /* send message */
        protocol_send_header(endpoint, sizeof(message) + string_length);
        protocol_send_payload(message, sizeof(message));
        protocol_send_payload((const uint8_t*)device_info.description,
                              string_length);
        protocol_send_tail();
    }
    return error_code;
}

static uint16_t send_temperature(uint8_t endpoint,
                                 Radar_Handle_t radar_driver,
                                 uint8_t temp_sensor)
{
    uint16_t error_code;
    int32_t temperature;

    /* read temperature value */
    error_code = radar_get_temperature(radar_driver, temp_sensor,
                                       &temperature);

    if (error_code == RADAR_ERR_OK)
    {
        uint8_t message[6];

        /* compile message */
        wr_payload_u8 (message, 0, MSG_SET_TEMPRATURE);
        wr_payload_u8 (message, 1, temp_sensor);
        wr_payload_i32(message, 2, temperature);

        /* send message */
        protocol_send_header(endpoint, sizeof(message));
        protocol_send_payload(message, sizeof(message));
        protocol_send_tail();
    }
    return error_code;
}

static uint16_t send_tx_power(uint8_t endpoint, Radar_Handle_t radar_driver,
                              uint8_t tx_antenna)
{
    uint16_t error_code;
    int32_t tx_power;

    /* read TX power value value */
    error_code =  radar_get_tx_power(radar_driver, tx_antenna, &tx_power);

    if (error_code == RADAR_ERR_OK)
    {
        uint8_t message[6];

        /* compile message */
        wr_payload_u8 (message, 0, MSG_SET_TX_POWER);
        wr_payload_u8 (message, 1, tx_antenna);
        wr_payload_i32(message, 2, tx_power);

        /* send message */
        protocol_send_header(endpoint, sizeof(message));
        protocol_send_payload(message, sizeof(message));
        protocol_send_tail();
    }
    return error_code;
}

static uint16_t send_chirp_duration(uint8_t endpoint,
                                    Radar_Handle_t radar_driver)
{
    uint16_t error_code;
    uint32_t chirp_duration;

    /* read chirp duration */
    error_code = radar_get_chirp_duration(radar_driver, &chirp_duration);

    if (error_code == RADAR_ERR_OK)
    {
        uint8_t message[5];

        /* compile message */
        wr_payload_u8 (message, 0, MSG_SET_CHRIP_DURATION);
        wr_payload_u32(message, 1, chirp_duration);

        /* send message */
        protocol_send_header(endpoint, sizeof(message));
        protocol_send_payload(message, sizeof(message));
        protocol_send_tail();
    }
    return error_code;
}

static uint16_t send_min_frame_interval(uint8_t endpoint,
                                        Radar_Handle_t radar_driver)
{
    uint16_t error_code;
    uint32_t min_frame_interval;

    /* read minimum frame interval */
    error_code = radar_get_min_frame_interval(radar_driver,
                                              &min_frame_interval);

    if (error_code == RADAR_ERR_OK)
    {
        uint8_t message[5];

        /* compile message */
        wr_payload_u8 (message, 0, MSG_SET_MIN_INTERVAL);
        wr_payload_u32(message, 1, min_frame_interval);

        /* send message */
        protocol_send_header(endpoint, sizeof(message));
        protocol_send_payload(message, sizeof(message));
        protocol_send_tail();
    }
    return error_code;
}

static uint16_t send_frame_format(uint8_t endpoint,
                                 Radar_Handle_t radar_driver)
{
    uint16_t error_code;
    Frame_Format_t frame_format;

    /* get current frame format from radar driver */
    error_code = radar_get_frame_format(radar_driver, &frame_format);

    if (error_code == RADAR_ERR_OK)
    {
        uint8_t message[11];

        /* compile message */
        wr_payload_u8 (message,  0, MSG_SET_FRAME_FORMAT);
        wr_payload_u32(message,  1, frame_format.num_samples_per_chirp);
        wr_payload_u32(message,  5, frame_format.num_chirps_per_frame);
        wr_payload_u8 (message,  9, frame_format.rx_mask);
        wr_payload_u8 (message, 10, frame_format.signal_part);

        /* send message */
        protocol_send_header(endpoint, sizeof(message));
        protocol_send_payload(message, sizeof(message));
        protocol_send_tail();
    }
    return error_code;
}


static uint16_t send_adc_configuration(uint8_t endpoint,
                                       Radar_Handle_t radar_driver)
{
    uint16_t error_code;
    Adc_Configuration_t adc_configuration;

    /* read ADC configuration from device */
    error_code = radar_get_adc_configuration(radar_driver,
                                             &adc_configuration);

    if (error_code == RADAR_ERR_OK)
    {
        uint8_t message[7];

        /* compile message */
        wr_payload_u8 (message, 0, MSG_SET_CONFIGURATION);
        wr_payload_u32(message, 1, adc_configuration.samplerate_Hz);
        wr_payload_u8 (message, 5, adc_configuration.resolution);
        wr_payload_u8 (message, 6, adc_configuration.use_post_calibration);

        /* send message */
        protocol_send_header(endpoint, sizeof(message));
        protocol_send_payload(message, sizeof(message));
        protocol_send_tail();
    }
    return error_code;
}

static uint16_t send_adc_calibration_data(uint8_t endpoint, Radar_Handle_t radar_driver, Calibration_Target_t target)
{
	uint16_t error_code = 0;

	uint16_t num_of_bytes;

	void* calibration_data_ptr = NULL;

	//----------------------------------------------------------------------

	error_code = radar_read_calibration(radar_driver, target, CALIBRATION_DATA_ADC, &calibration_data_ptr, &num_of_bytes);

	if (error_code == RADAR_ERR_OK)
	{
		uint8_t message_header[3];

		/* setup message header */
		if (target == CALIBRATION_TARGET_FLASH)
		{
			wr_payload_u8(message_header, 0, MSG_GET_ADC_FLASH_CALIBRATION);
		}
		else
		{
			wr_payload_u8(message_header, 0, MSG_GET_ADC_SRAM_CALIBRATION);
		}

		wr_payload_u16(message_header, 1, num_of_bytes);

		/* send message header */
		protocol_send_header(endpoint, sizeof(message_header) + num_of_bytes);

		protocol_send_payload(message_header, sizeof(message_header));

		protocol_send_payload((uint8_t*)calibration_data_ptr, num_of_bytes);

		protocol_send_tail();
	}

	return error_code;
}

//==============================================================================

static uint16_t send_algo_calibration_data(uint8_t endpoint, Radar_Handle_t radar_driver, Calibration_Target_t target)
{
	uint16_t error_code = 0;

	uint16_t num_of_bytes;

	void* calibration_data_ptr;

	//----------------------------------------------------------------------

	error_code = radar_read_calibration(radar_driver, target, CALIBRATION_DATA_ALGO, &calibration_data_ptr, &num_of_bytes);

	if (error_code == RADAR_ERR_OK)
	{
		uint8_t message_header[5];

		/* setup message header */

		if (target == CALIBRATION_TARGET_FLASH)
		{
			wr_payload_u8(message_header, 0, MSG_GET_ALGO_FLASH_CALIBRATION);
		}
		else
		{
			wr_payload_u8(message_header, 0, MSG_GET_ALGO_SRAM_CALIBRATION);
		}

		wr_payload_u16(message_header, 1, ((Algo_Calibrations_t*)calibration_data_ptr)->distance_offset_cm);
		wr_payload_i16(message_header, 3, ((Algo_Calibrations_t*)calibration_data_ptr)->angle_offset_deg);

		/* send message header */
		protocol_send_header(endpoint, sizeof(message_header));

		protocol_send_payload(message_header, sizeof(message_header));

		protocol_send_tail();
	}

	return error_code;
}



/* --- End of File --- */

