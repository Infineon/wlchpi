/***************************************************************************//**
* \file hpi_internal.h
* \version 1.0
*
* Provides general utility macros and definitions for the PDStack Middleware.
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
/**
* \addtogroup group_autoHPI WLC HPI Library
* \{
* */

#ifndef HPI_INTERNAL_H_
#define HPI_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>
#include "cy_pdstack_common.h"
#include "i2c.h"

#if CCG_HPI_PD_ENABLE
#include "cy_pdstack_dpm.h"
#endif /* CCG_HPI_PD_ENABLE */

/** \addtogroup group_autoHPI_macros
* \{
* This section describes the WLC HPI Macros.
* Detailed information about the macros is available in each macro description.
*/

/**
 * @brief Maximum support flash row size.
 */
#define CCG_MAX_FLASH_ROW_SIZE          (256u)

/**
 * @brief FW Metadata Table size in bytes.
 */
#if USE_CYACD2_METADATA_FORMAT
#define CCG_METADATA_TABLE_SIZE                 (0x80u)
#else
#define CCG_METADATA_TABLE_SIZE                 (0x40u)
#endif /* USE_CYACD2_METADATA_FORMAT */

/**
 * @brief HPI interface clock frequency can be upto 1 MHz.
 */
#define HPI_SCB_CLOCK_FREQ              (I2C_SCB_CLOCK_FREQ_1_MHZ)

/**
 * @brief Minimum HPI write size: Two address bytes + 1 data byte.
 */
#define HPI_MIN_WRITE_SIZE              (3u)

/**
 * @brief Offset of the signature field in various HPI commands.
 */
#define HPI_SIGNATURE_OFFSET            (0x0u)

/**
 * @brief Offset of interrupt status update command in a interrupt clear write.
 */
#define HPI_INTR_CMD_OFFSET             (0x0u)

/**
 * @brief Signature value used to request a JUMP_TO_BOOT operation.
 */
#define HPI_JUMP_TO_BOOT_CMD_SIG        ('J')

/**
 * @brief Signature value used to request a JUMP_TO_ALT_FW operation.
 */
#define HPI_JUMP_TO_ALT_FW_SIG          ('A')

/**
 * @brief Size of the version information associated with a firmware binary.
 */
#define HPI_FW_VERSION_SIZE             (8u)

/**
 * @brief Size in bytes of the data to be written to request device reset.
 */
#define HPI_RESET_SIZE                  (0x2u)

/**
 * @brief Signature to be provided to request an I2C interface or device reset.
 */
#define HPI_RESET_CMD_SIG               ('R')

/**
 * @brief Signature to be provided to request read of device's Die and Version
 * information.
 */
#define HPI_READ_DIE_INFO_SIG           ('I')

/**
 * @brief Offset to reset command byte.
 */
#define HPI_RESET_CMD_OFFSET            (0x1u)

/**
 * @brief Command code for a device reset request.
 */
#define HPI_RESET_DEVICE_RESET_CMD      (0x1u)

/**
 * @brief Command code for an I2C interface reset request.
 */
#define HPI_RESET_I2C_CMD               (0x0u)

/**
 * @brief Signature used to request entry to flash update mode.
 */
#define HPI_ENTER_FLASHING_CMD_SIG      ('P')

/**
 * @brief Size of command to be written for flash read/write.
 */
#define HPI_FLASH_READ_WRITE_SIZE       (0x4u)

/**
 * @brief Signature for valid flash read/write requests.
 */
#define HPI_FLASH_READ_WRITE_CMD_SIG    ('F')

/**
 * @brief Offset to flash command field in the read/write request.
 */
#define HPI_FLASH_READ_WRITE_CMD_OFFSET (0x1u)

/**
 * @brief Command code used to request chunk of 128(FLASH_ROW_SIZE) bytes of FWCT signature.
 */
#define HPI_FWCT_SIG_WRITE_CMD         (0x3u)

/**
 * @brief Command code used to request chunk of 128(FLASH_ROW_SIZE) bytes write of FWCT.
 */
#define HPI_FWCT_ROW_WRITE_CMD         (0x2u)

/**
 * @brief Command code used to request a flash row write.
 */
#define HPI_FLASH_ROW_WRITE_CMD         (0x1u)

/**
 * @brief Command code used to request a flash row read.
 */
#define HPI_FLASH_ROW_READ_CMD          (0x0u)

/**
 * @brief Offset to MSB of flash row number in read/write command.
 */
#define HPI_FLASH_READ_WRITE_ROW_MSB    (0x3u)

/**
 * @brief Offset to LSB of flash row number in read/write command.
 */
#define HPI_FLASH_READ_WRITE_ROW_LSB    (0x2u)

/**
 * @brief Offset of PORT_ENABLE command field in the corresponding HPI request.
 */
#define HPI_PORT_ENABLE_CMD_OFFSET      (0x0u)

/**
 * @brief Mask bit to retrieve PORT0 enable/disable field.
 */
#define HPI_PORT0_ENABLE_MASK           (0x1u)

/**
 * @brief Mask bit to retrieve PORT1 enable/disable field.
 */
#define HPI_PORT1_ENABLE_MASK           (0x2u)

/**
 * @brief Signature for read customer info command.
 */
#define HPI_READ_CUSTOMER_INFO_SIG      ('C')

/**
 * @brief Size of customer info in bytes.
 */
#define HPI_CUSTOMER_INFO_SIZE_BYTES    (0x20u)

/**
 * @brief Size of VDM_CTRL HPI request.
 */
#define HPI_VDM_CTRL_SIZE               (0x2u)

/**
 * @brief Size of SOP_TYPE field in VDM_CTRL HPI request.
 */
#define HPI_VDM_SOP_TYPE_OFFSET         (0x0u)

/**
 * @brief Size of LEN field in VDM_CTRL HPI request.
 */
#define HPI_VDM_LEN_OFFSET              (0x1u)

/**
 * @brief Size of HPI event mask register.
 */
#define HPI_EVENT_MASK_SIZE             (0x4u)

/**
 * @brief Mask to set/clear all HPI interrupt status bits.
 */
#define HPI_INTR_REG_CLEAR_ALL_RQT      (0x7u)

/**
 * @brief Size of data receive buffer to be used for the HPI interface. This size includes two bytes of
 * register address and 256 bytes corresponding to the maximum write size.
 */
#define HPI_BUFFER_SIZE                 (0x120u)

/**
 * @brief Size of response/event header.
 */
#define HPI_MESSAGE_HEADER_SIZE         (4u)

/**
 * @brief Size of data message extension header in bytes.
 */
#define HPI_DATAMSG_HEADER_SIZE         (4u)

/**
 * @brief Maximum number of events/responses that can be queued for each USB-PD port.
 */
#define HPI_QUEUE_MAX_ENTRIES           (8u)

/**
 * @brief Maximum data size associated with an HPI event or response.
 */
#define HPI_EVENT_DATA_SIZE             (0x20u)

/**
 * @brief Memory required to store each event data along with event code and length.
 */
#define HPI_EVENT_MAX_SIZE              (HPI_EVENT_DATA_SIZE + 2u)

/**
 * @brief Total memory requirement for the HPI event queue corresponding to each USB-PD port.
 */
#define HPI_QUEUE_MAX_LENGTH            (CY_PD_MAX_EXTD_PKT_SIZE + HPI_MESSAGE_HEADER_SIZE * 5)

/**
 * @brief Signature used to indicate valid source PDO list.
 */
#define HPI_SRCPDO_LIST_SIG     (0x53524350)    /* "SRCP" */

/**
 * @brief Signature used to indicate valid sink PDO list.
 */
#define HPI_SNKPDO_LIST_SIG     (0x534E4B50)    /* "SNKP" */

/**
 * @brief Header used on source PDO read data list.
 */
#define HPI_SRCPDO_LIST_HDR     (0x00000000)

/**
 * @brief Header used on sink PDO read data list.
 */
#define HPI_SNKPDO_LIST_HDR     (0x00000001)

/**
 * @brief Macro defines the HPI Auto Minimum command size in bytes.
 */
#define HPI_AUTO_COMMAND_MIN_SIZE               (1u)

/**
 * @brief Macro defines the HPI Auto command data starting offset.
 */
#define HPI_AUTO_CMD_DATA_OFFSET                (4u)

/**
 * @brief Macro defines the HPI Auto command data size per port.
 */
#define HPI_AUTO_CMD_DATA_PORT_OFFSET           (32u)

/**
 * Macro defines the command value to retrieve/reset TCC data.
 */
#define HPI_TEST_CODE_DATA                      (1u)

/**
 * Macro defines the command value to retrieve TCC statistics. If no argument is set,
 * this will return the percentage of functions executed.
 */
#define HPI_TEST_CODE_STAT                      (2u)

/**
 * Macro defines the TCC command minimum size in bytes.
 */
#define HPI_TEST_CODE_CMD_MIN_SIZE              (1u)

/**
 * Macro defines the command argument to retrieve TCC Raw data.
 */
#define HPI_TEST_CODE_GET_RAW_DATA              (0u)

/**
 * Macro defines the command argument to reset TCC Raw data.
 */
#define HPI_TEST_CODE_RESET_RAW_DATA            (1u)

/**
 * Macro defines the command argument to retrieve percentage of functions executed.
 */
#define HPI_TEST_CODE_GET_PERCENTAGE            (0u)

/**
 * Macro defines the command argument to retrieve TCC raw data.
 */
#define HPI_TEST_CODE_GET_RAW_DATA_SIZE         (1u)

/**
 * Macro defines the size of data bytes returned for TCC percentage command.
 */
#define HPI_TEST_CODE_PERCENTAGE_DATA_SIZE_BYTES (1u)

/**
 * Macro defines the size of data bytes returned for retrieving raw data size.
 */
#define HPI_TEST_CODE_RAW_DATA_SIZE_BYTES        (2u)

/**
 * Macro defines the size of raw data chunk returned (in bytes) for each
 * raw data extract request.
 */
#define HPI_TEST_CODE_RAW_DATA_CHUNK_SIZE        (64u)

/** \} group_autoHPI_macros */

/** \addtogroup group_autoHPI_enums
* \{
* This section describes the WLC HPI Enumerated Types.
* Detailed information about the enums is available in each enum description.
*/


/**
 * @typedef hpi_auto_command_opcode_t
 * @brief HPI auto command opcodes.
 *
 * This enumeration lists the various commands supported by the HPI auto module.
 */
typedef enum {
    HPI_ENABLE_HEART_BEAT_CMD        = (1u),  /**< Command value for enabling Heart Beat */
    HPI_DISABLE_HEART_BEAT_CMD       = (2u),  /**< Command value for disabling Heart Beat */
    HPI_ENABLE_TEMP_THROTTLING_CMD   = (3u),  /**< Command value for enabling temperature throttling */
    HPI_DISABLE_TEMP_THROTTLING_CMD  = (4u),  /**< Command value for disabling temperature throttling */
    HPI_ENABLE_VIN_THROTTLING_CMD    = (5u),  /**< Command value for enabling VIN throttling */
    HPI_DISABLE_VIN_THROTTLING_CMD   = (6u),  /**< Command value for disabling VIN throttling */
    HPI_SET_SYSTEM_OC_CMD            = (7u),  /**< Command value for setting system operating condition */
    HPI_ENABLE_VCONN_CMD             = (8u),  /**< Command value for enabling VCONN */
    HPI_DISABLE_VCONN_CMD            = (9u),  /**< Command value for disabling VCONN */
    HPI_GET_AUTO_FW_CONFIG_CMD       = (10u), /**< Command value for obtaining HPI Auto specific parameters */
    HPI_GET_PORT_STATUS              = (11u), /**< Command value for obtaining port status information */
    HPI_GET_PORT_PDP                 = (12u), /**< Command value for obtaining port power budget */
    HPI_GET_PD_CONTRACT_DETAILS      = (13u), /**< Command value for obtaining PD contract details */
    HPI_GET_SYSTEM_OC                = (14u), /**< Command value for obtaining system operating condition */
    HPI_GET_FAULT_STATUS             = (15u), /**< Command value for obtaining Fault status information */
    HPI_ENABLE_LS_CMD                = (16u), /**< Command value for run time enabling of Load Sharing Algorithm */
    HPI_DISABLE_LS_CMD               = (17u), /**< Command value for run time disabling of Load Sharing Algorithm */
    HPI_SET_PORT_PDP                 = (18u), /**< Command value for setting port power budget */
    HPI_GET_SENSOR_TEMPERATURE       = (19u), /**< Command value for obtaining sensor reported temperature values> */
    HPI_GET_VBUS_READING             = (20u), /**< Command value for extracting actual VBUS voltage and current> */
    HPI_GET_BATTERY_VOLTAGE          = (21u), /**< Command value for extracting actual input Battery Voltage> */
    HPI_GET_SET_OC_CONFIGURATION     = (22u)  /**< Command value for setting/extracting configurable OC> */

} hpi_auto_command_opcode_t;

/**
 * @typedef hpi_evmask_t
 * @brief HPI event mask definitions.
 *
 * This enumeration lists the various mask values that control the reporting of HPI events.
 */
typedef enum {
    HPI_EVMASK_DISABLED     = 0x00000000u,      /**< Disabled mask value. */
    HPI_EVMASK_RSRVD        = 0x00000001u,      /**< Reserved mask value. */
    HPI_EVMASK_OCP          = 0x00000002u,      /**< Mask to enable/disable Over Current event. */
    HPI_EVMASK_OVP          = 0x00000004u,      /**< Mask to enable/disable Over Voltage event. */
    HPI_EVMASK_CC_CONNECT   = 0x00000008u,      /**< Mask to enable/disable Type-C connect event. */
    HPI_EVMASK_CC_DISCON    = 0x00000010u,      /**< Mask to enable/disable Type-C disconnect event. */
    HPI_EVMASK_CONTRACT     = 0x00000020u,      /**< Mask to enable/disable PD contract complete event. */
    HPI_EVMASK_CTRL_MSG     = 0x00000040u,      /**< Mask to enable/disable PD control message events. */
    HPI_EVMASK_VDM_RCVD     = 0x00000080u,      /**< Mask to enable/disable VDM received event. */
    HPI_EVMASK_SRC_CAP      = 0x00000100u,      /**< Mask to enable/disable Source Capabilities received event. */
    HPI_EVMASK_SNK_CAP      = 0x00000200u,      /**< Mask to enable/disable Sink Capabilities received event. */
    HPI_EVMASK_ALT_MODE     = 0x00000400u,      /**< Mask to enable/disable alternate mode related events. */
    HPI_EVMASK_ERROR        = 0x00000800u,      /**< Mask to enable/disable error events. */
    HPI_EVMASK_EMCA         = 0x00001000u,      /**< Mask to enable/disable EMCA connection related events. */
    HPI_EVMASK_MISC         = 0x00002000u,      /**< Mask to enable/disable miscellaneous events like Rp change. */
    HPI_EVMASK_BB           = 0x00004000u,      /**< Mask to enable/disable Billboard events. */
    HPI_EVMASK_OTP          = 0x00010000u,      /**< Mask to enable/disable Over-Temperature event. */
    HPI_EVMASK_DATA_MSG     = 0x00020000u,      /**< Mask to enable/disable data message event. */
    HPI_EVMASK_SYS_ERROR    = 0x00040000u,      /**< Mask to enable/disable system error notifications. */
    HPI_EVMASK_PF_EVENTS    = 0x00080000u,      /**< Mask to enable/disable Intel platform specific events. */
    HPI_EVMASK_PWR_SNK_EVNT = 0x00100000u,      /**< Mask to enable/disable Intel platform specific events. */

} hpi_evmask_t;

/**
 * @typedef hpi_response_t
 * @brief List of HPI response and event codes.
 */
typedef enum
{
    HPI_RESPONSE_NO_RESPONSE            = 0x00, /**< No valid response. */
    HPI_RESPONSE_SUCCESS                = 0x02, /**< Success response. */
    HPI_RESPONSE_FLASH_DATA_AVAILABLE   = 0x03, /**< Flash read data is available in flash data memory. */
    HPI_RESPONSE_INVALID_COMMAND        = 0x05, /**< Invalid command received. */
    HPI_RESPONSE_CMD_FAILED             = 0x06, /**< Load sharing command failed. */
    HPI_RESPONSE_FLASH_UPDATE_FAILED    = 0x07, /**< Flash read/write operation failed. */
    HPI_RESPONSE_INVALID_FW             = 0x08, /**< Firmware binary checked is invalid. */
    HPI_RESPONSE_INVALID_ARGUMENT       = 0x09, /**< Invalid parameter specified for the command. */
    HPI_RESPONSE_NOT_SUPPORTED          = 0x0A, /**< Command received is not supported. */
    HPI_RESPONSE_PD_TRANSACTION_FAILED  = 0x0C, /**< USB-PD transaction failed. */
    HPI_RESPONSE_PD_COMMAND_FAILED      = 0x0D, /**< USB-PD command failed. */
    HPI_RESPONSE_UNDEFINED_ERROR        = 0x0F, /**< Undefined error occurred. */
    HPI_RESPONSE_PDO_DATA               = 0x10, /**< PDO read data is available in read data memory. */
    HPI_RESPONSE_CMD_ABORTED            = 0x11, /**< HPI initiated PD command has been aborted. */
    HPI_RESPONSE_PORT_BUSY              = 0x12, /**< HPI initiated PD command failed due to port busy. */
    HPI_RESPONSE_MIN_MAX_CUR            = 0x13, /**< HPI response providing Sink Min/Max current data. */
    HPI_RESPONSE_EXT_SRC_CAP            = 0x14, /**< HPI response providing ext. source cap data. */
    HPI_RESPONSE_UCSI_COMMAND_FAILED    = 0x15, /**< UCSI command failed. */
    HPI_RESPONSE_DISC_ID_VDM            = 0x18, /**< HPI response providing DISC_ID VDM response. */
    HPI_RESPONSE_DISC_SVID_VDM          = 0x19, /**< HPI response providing DISC_SVID VDM response. */
    HPI_RESPONSE_DISC_MODE_VDM          = 0x1A, /**< HPI response providing DISC_MODE VDM response. */
    HPI_RESPONSE_VCONN_DISABLED         = 0x1B, /**< PD command could not be initiated due to VConn being OFF. */
    HPI_RESPONSE_EXT_SNK_CAP            = 0x1C, /**< HPI response providing ext. sink cap data. */
    HPI_RESPONSE_I2C_REG                = 0x1D, /**< HPI response providing I2C device read data. */

    HPI_RESPONSE_INVALID_ID             = 0x40, /**< Invalid FWCT identity received for signed FW upgrade >**/
    HPI_RESPONSE_INVALID_GUID           = 0x41, /**< Invalid GUID received for signed FW upgrade >**/
    HPI_RESPONSE_INVALID_VER            = 0x42, /**< Invalid/older Primary FW version received for signed FW upgrade >**/
    HPI_RESPONSE_OUT_OF_SEQ_CMD         = 0x43, /**< Command sent is not expected in current state during signed FW upgrade >**/
    HPI_RESPONSE_INVALID_FWCT           = 0x44, /**< Unauthentic FWCT received for signed FW upgrade >**/
    HPI_RESPONSE_HASH_CMP_FAILED        = 0x45, /**< Hash comparison of fw image and hash in FWCT is not matched during signed FW upgrade >**/
    
    HPI_EVENT_RESET_COMPLETE            = 0x80, /**< Reset complete event notification. */
    HPI_EVENT_MSG_OVERFLOW              = 0x81, /**< Event queue overflow. */
    HPI_EVENT_OC_DETECT                 = 0x82, /**< Over-Current event detected. */
    HPI_EVENT_OV_DETECT                 = 0x83, /**< Over-Voltage event detected. */
    HPI_EVENT_CONNECT_DETECT            = 0x84, /**< Type-C connect detected. */
    HPI_EVENT_DISCONNECT_DETECT         = 0x85, /**< Type-C disconnect detected. */
    HPI_EVENT_NEGOTIATION_COMPLETE      = 0x86, /**< PD contract negotiation completed. */
    HPI_EVENT_SWAP_COMPLETE             = 0x87, /**< Swap (DR_SWAP, PR_SWAP or VCONN_SWAP) complete. */
    HPI_EVENT_PS_RDY_RECEIVED           = 0x8A, /**< PS_RDY message received. */
    HPI_EVENT_GOTO_MIN_RECEIVED         = 0x8B, /**< GOTO_MIN message received. */
    HPI_EVENT_ACCEPT_RECEIVED           = 0x8C, /**< ACCEPT message received. */
    HPI_EVENT_REJECT_RECEIVED           = 0x8D, /**< REJECT message received. */
    HPI_EVENT_WAIT_RECEIVED             = 0x8E, /**< WAIT message received. */
    HPI_EVENT_HARD_RESET_RECEIVED       = 0x8F, /**< HARD RESET received. */
    HPI_EVENT_VDM_RECEIVED              = 0x90, /**< VDM received. VDM contents are stored in data memory. */
    HPI_EVENT_SOURCE_CAP_RECEIVED       = 0x91, /**< Source Capabilities received. PDOs received are stored
                                                     in data memory. */
    HPI_EVENT_SINK_CAP_RECEIVED         = 0x92, /**< Sink Capabilities received. PDOs received are stored in
                                                     data memory. */
    HPI_EVENT_DATA_RESET_RECEIVED       = 0x93, /**< Data reset received. No event data. */
    HPI_EVENT_DATA_RESET_COMPLETE       = 0x94, /**< Data reset sequence completed. No event data. */
    HPI_EVENT_USB_ENTRY_COMPLETE        = 0x95, /**< Enter_USB handshake completed successfully. */
    HPI_EVENT_HARD_RESET_SENT           = 0x9A, /**< HARD RESET has been sent by CCG. */
    HPI_EVENT_SOFT_RESET_SENT           = 0x9B, /**< SOFT RESET has been sent by CCG. Event data will indicate
                                                     SOFT RESET packet type (SOP, SOP' or SOP''). */
    HPI_EVENT_CABLE_RESET_SENT          = 0x9C, /**< CABLE RESET has been sent by CCG. */
    HPI_EVENT_SOURCE_DISABLED           = 0x9D, /**< Source Disabled state has been entered. */
    HPI_EVENT_SENDER_TIMEOUT            = 0x9E, /**< Sender Response timeout occurred. */
    HPI_EVENT_VDM_NO_RESPONSE           = 0x9F, /**< No response to VDM sent by CCG. */
    HPI_EVENT_UNEXPECTED_VOLTAGE        = 0xA0, /**< Unexpected VBus voltage detected. */
    HPI_EVENT_ERROR_RECOVERY            = 0xA1, /**< Type-C error recovery started. */
    HPI_EVENT_BAT_STATUS_RECEIVED       = 0xA2, /**< Battery status message received. */
    HPI_EVENT_ALERT_RECEIVED            = 0xA3, /**< Alert message received. */
    HPI_EVENT_NOTSUPP_RECEIVED          = 0xA4, /**< Not-supported message received. */
    HPI_EVENT_EMCA_DETECT               = 0xA6, /**< EMCA has been detected. */
    HPI_EVENT_EMCA_NOT_DETECT           = 0xA7, /**< No EMCA present in current connection. */
    HPI_EVENT_RP_CHANGE_DETECT          = 0xAA, /**< Change in Rp termination detected. */
    HPI_EVENT_BB_UPDATE                 = 0xAB, /**< Billboard update event. */
    HPI_EVENT_PD_EXTD_MSG_SOP           = 0xAC, /**< PD extended data message (SOP) received. */
    HPI_EVENT_CCG2_RESERVED_AD          = 0xAD, /**< Reserved event. */
    HPI_EVENT_CCG2_RESERVED_AE          = 0xAE, /**< Reserved event. */
    HPI_EVENT_CCG2_RESERVED_AF          = 0xAF, /**< Reserved event. */

    HPI_EVENT_ALT_MODE                  = 0xB0, /**< Alternate mode related event. Event data contains the SVID
                                                     corresponding to the mode as well as the event type. */
    HPI_EVENT_APP_HW                    = 0xB1, /**< Alt. Mode control hardware (MUX, HPD etc.) related
                                                     event occurred. */

    HPI_EVENT_PD_EXTD_MSG_SOP_PRIME     = 0xB4, /**< PD extended data message (SOP') received. */
    HPI_EVENT_PD_EXTD_MSG_SOP_DPRIME    = 0xB5, /**< PD extended data message (SOP'') received. */
    HPI_EVENT_OT_DETECT                 = 0xB6, /**< Over-temperature condition detected. */
    HPI_EVENT_RESERVED_B7               = 0xB7, /**< Reserved event code. */
    HPI_EVENT_HARDWARE_ERROR            = 0xB8, /**< System hardware (MUX, regulator etc.) error. */
    HPI_EVENT_VCONN_OC_DETECT           = 0xB9, /**< Over-current condition detected on VConn supply. */
    HPI_EVENT_CC_VBUS_SHORT             = 0xBA, /**< Over-voltage condition (possible VBus short) on CC lines. */
    HPI_EVENT_SBU_VBUS_SHORT            = 0xBB, /**< Over-voltage condition (possible VBus short) on SBU lines. */
    HPI_EVENT_SC_DETECT                 = 0xBC, /**< Short-Circuit fault condition. */
    HPI_EVENT_RC_DETECT                 = 0xBD, /**< Reverse Current fault condition. */
    HPI_EVENT_STANDBY_CURRENT           = 0xBE, /**< Standby Current. */
    HPI_EVENT_UV_DETECT                 = 0xBF, /**< VBus under-voltage fault condition. */

    HPI_EVENT_SOC_TIMEOUT               = 0xC0, /**< SoC ACK timeout in Intel host designs. */
    HPI_EVENT_SRC_RP_REMOVED            = 0xC1, /**< Rp termination applied by the source has been removed. */
    HPI_EVENT_PR_SWAP_ACCEPTED          = 0xC2, /**< PR-SWAP has been accepted by CCGx or the port partner. */
    HPI_EVENT_SNK_POWER_REDUCED         = 0xC3, /**< PD contract being re-negotiated to reduce power available. */  
    HPI_EVENT_BC_12_EVENTS              = 0xC4, /**< BC12 related notification. */

    HPI_EVENT_CUST_NTF_BEFORE_HR        = 0xD0, /** Event to notify host before sending HARD_RESET */
    HPI_EVENT_CUST_UVDM_RCVD            = 0xD1, /** Customer-defined uVDM Received */
    HPI_EVENT_CUST_ALT_MODE_CHANGE      = 0xD2, /** This event is sent for notification of alternate mode specific conditions 
                                                    like mode entry and mode exit. */
    HPI_EVENT_CUST_DISCOVERY_COMPLETE   = 0xD3, /** Customer-defined Discovery Process Completed */
} hpi_response_t;

/**
 * @typedef hpi_port_intr_type_t
 * @brief USB-PD port specific interrupt status bits.
 *
 * This type lists the various USB-PD specific interrupt status bits that
 * are reported through the intr_status register. The interrupt status
 * register content is independent of the details provided through the
 * event queue.
 */
typedef enum
{
    HPI_PORT_INTR_CC_CONNECT            = ((uint32_t)1u << 0),    /**< Type-C connect interrupt. */
    HPI_PORT_INTR_CC_DISCONNECT         = ((uint32_t)1u << 1),    /**< Type-C disconnect interrupt. */
    HPI_PORT_INTR_CONTRACT_CMPLT        = ((uint32_t)1u << 2),    /**< Power contract negotiation complete. */
    HPI_PORT_INTR_PRSWAP_CMPLT          = ((uint32_t)1u << 3),    /**< PR-SWAP complete. */
    HPI_PORT_INTR_DRSWAP_CMPLT          = ((uint32_t)1u << 4),    /**< DR-SWAP complete. */
    HPI_PORT_INTR_VCONSWAP_CMPLT        = ((uint32_t)1u << 5),    /**< VCONN-SWAP complete. */
    HPI_PORT_INTR_HARDRESET_RCVD        = ((uint32_t)1u << 6),    /**< Hard Reset received. */
    HPI_PORT_INTR_HARDRESET_SENT        = ((uint32_t)1u << 7),    /**< Hard Reset sent by CCG. */
    HPI_PORT_INTR_SOFTRESET_SENT        = ((uint32_t)1u << 8),    /**< Soft Reset sent by CCG. */
    HPI_PORT_INTR_CABLERESET_SENT       = ((uint32_t)1u << 9),    /**< Cable Reset sent by CCG. */
    HPI_PORT_INTR_CC_ERROR_RCVRY        = ((uint32_t)1u << 10),   /**< Type-C error recovery initiated by CCG. */
    HPI_PORT_INTR_SRC_DISABLED          = ((uint32_t)1u << 11),   /**< Source disabled state entered by CCG. */
    HPI_PORT_INTR_EMCA_DETECTED         = ((uint32_t)1u << 12),   /**< EMCA controller detected (D_ID response received). */
    HPI_PORT_INTR_CABLEDISC_FAIL        = ((uint32_t)1u << 13),   /**< Cable discovery failed. */
    HPI_PORT_INTR_FASTROLESWAP_CMPLT    = ((uint32_t)1u << 14),   /**< Fast Role Swap complete. */

    HPI_PORT_INTR_ALTMODE_ENTRY         = ((uint32_t)1u << 16),   /**< Alternate mode entered. */
    HPI_PORT_INTR_ALTMODE_EXIT          = ((uint32_t)1u << 17),   /**< Alternate mode exited. */

    HPI_PORT_INTR_CC_OVP_EVT            = ((uint32_t)1u << 25),   /**< CC over-voltage (VBus short) event. */
    HPI_PORT_INTR_SBU_OVP_EVT           = ((uint32_t)1u << 26),   /**< SBU over-voltage (VBus short) event. */
    HPI_PORT_INTR_VCONN_OVERCUR_EVT     = ((uint32_t)1u << 27),   /**< VConn Over-Current event. */
    HPI_PORT_INTR_OVERTEMP_EVT          = ((uint32_t)1u << 28),   /**< Over-temperature event. */
    HPI_PORT_INTR_UNEXPECT_VOLTAGE      = ((uint32_t)1u << 29),   /**< Unexpected VBus voltage detected. */
    HPI_PORT_INTR_OVERVOLTAGE_EVT       = ((uint32_t)1u << 30),   /**< VBus Over-Voltage event. */
    HPI_PORT_INTR_OVERCURRENT_EVT       = -2147483648             /**< VBus Over-Current event. */
} hpi_port_intr_type_t;

/**
 * @typedef hpi_dev_reg_address_t
 * @brief List of HPI device information register addresses.
 *
 * This enumeration lists the addresses for the HPI device information registers supported by CCGx devices.
 * These registers are used to retrieve firmware mode and version information, and for doing firmware and
 * configuration table updates.
 */
typedef enum
{
    HPI_DEV_REG_DEVICE_MODE             = 0x00,         /**< Device mode register: Specifies FW mode, PD port count
                                                             and flash row size. */
    HPI_DEV_REG_BOOT_MODE_REASON        = 0x01,         /**< Boot mode reason register: Specifies validity of each
                                                             firmware image. Also reports reason for device staying
                                                             in boot mode. */
    HPI_DEV_REG_SI_ID                   = 0x02,         /**< Silicon ID: MS Byte. */
    HPI_DEV_REG_SI_ID_LSB               = 0x03,         /**< Silicon ID: LS Byte. */
    HPI_DEV_REG_BL_LAST_ROW             = 0x04,         /**< Boot-loader last row: LS Byte. This is actually the
                                                             last flash row before the start of the firmware. */
    HPI_DEV_REG_BL_LAST_ROW_MSB         = 0x05,         /**< Boot-loader last row: MS Byte. */
    HPI_DEV_REG_INTR_ADDR               = 0x06,         /**< Interrupt status register. */
    HPI_DEV_REG_JUMP_TO_BOOT            = 0x07,         /**< Jump to boot (or alt firmware) request register. */
    HPI_DEV_REG_RESET_ADDR              = 0x08,         /**< Reset request signature register. */
    HPI_DEV_REG_RESET_CMD               = 0x09,         /**< Reset command register: I2C or Device reset. */
    HPI_DEV_REG_ENTER_FLASH_MODE        = 0x0A,         /**< Enter flashing mode request register. */
    HPI_DEV_REG_VALIDATE_FW_ADDR        = 0x0B,         /**< Validate firmware request register. */
    HPI_DEV_REG_FLASH_READ_WRITE        = 0x0C,         /**< Flash read/write signature register. */
    HPI_DEV_REG_FLASH_READ_WRITE_CMD    = 0x0D,         /**< Flash read/write command register: 0=Read, 1=Write */
    HPI_DEV_REG_FLASH_ROW_LSB           = 0x0E,         /**< Flash row to read/write: LSB */
    HPI_DEV_REG_FLASH_ROW_MSB           = 0x0F,         /**< Flash row to read/write: MSB */
    HPI_DEV_REG_ALL_VERSION             = 0x10,         /**< Boot loader version: LSB of build number. */
    HPI_DEV_REG_ALL_VERSION_BYTE_1      = 0x11,         /**< Boot loader version: MSB of build number. */
    HPI_DEV_REG_ALL_VERSION_BYTE_2      = 0x12,         /**< Boot loader version: Patch level. */
    HPI_DEV_REG_ALL_VERSION_BYTE_3      = 0x13,         /**< Boot loader version: Major and Minor number. */
    HPI_DEV_REG_ALL_VERSION_BYTE_4      = 0x14,         /**< Boot loader version: LSB of Application Type. */
    HPI_DEV_REG_ALL_VERSION_BYTE_5      = 0x15,         /**< Boot loader version: MSB of Application Type. */
    HPI_DEV_REG_ALL_VERSION_BYTE_6      = 0x16,         /**< Boot loader version: Hardware circuit version. */
    HPI_DEV_REG_ALL_VERSION_BYTE_7      = 0x17,         /**< Boot loader version: App Major and Minor number. */
    HPI_DEV_REG_ALL_VERSION_BYTE_8      = 0x18,         /**< FW1 version: LSB of build number. */
    HPI_DEV_REG_ALL_VERSION_BYTE_9      = 0x19,         /**< FW1 version: MSB of build number. */
    HPI_DEV_REG_ALL_VERSION_BYTE_10     = 0x1A,         /**< FW1 version: Patch level. */
    HPI_DEV_REG_ALL_VERSION_BYTE_11     = 0x1B,         /**< FW1 version: Major and Minor number. */
    HPI_DEV_REG_ALL_VERSION_BYTE_12     = 0x1C,         /**< FW1 version: LSB of Application Type. */
    HPI_DEV_REG_ALL_VERSION_BYTE_13     = 0x1D,         /**< FW1 version: MSB of Application Type. */
    HPI_DEV_REG_ALL_VERSION_BYTE_14     = 0x1E,         /**< FW1 version: Hardware circuit version. */
    HPI_DEV_REG_ALL_VERSION_BYTE_15     = 0x1F,         /**< FW1 version: App Major and Minor number. */
    HPI_DEV_REG_FW_2_VERSION            = 0x20,         /**< FW2 version: LSB of build number. */
    HPI_DEV_REG_FW_2_VERSION_BYTE_1     = 0x21,         /**< FW2 version: MSB of build number. */
    HPI_DEV_REG_FW_2_VERSION_BYTE_2     = 0x22,         /**< FW2 version: Patch level. */
    HPI_DEV_REG_FW_2_VERSION_BYTE_3     = 0x23,         /**< FW2 version: Major and Minor number. */
    HPI_DEV_REG_FW_2_VERSION_BYTE_4     = 0x24,         /**< FW2 version: LSB of Application Type. */
    HPI_DEV_REG_FW_2_VERSION_BYTE_5     = 0x25,         /**< FW2 version: MSB of Application Type. */
    HPI_DEV_REG_FW_2_VERSION_BYTE_6     = 0x26,         /**< FW2 version: Hardware circuit version. */
    HPI_DEV_REG_FW_2_VERSION_BYTE_7     = 0x27,         /**< FW2 version: App Major and Minor number. */
    HPI_DEV_REG_FW_1_BIN_LOC_LSB        = 0x28,         /**< FW1 binary location: LSB */
    HPI_DEV_REG_FW_1_BIN_LOC_MSB        = 0x29,         /**< FW1 binary location: MSB */
    HPI_DEV_REG_FW_2_BIN_LOC_LSB        = 0x2A          /**< FW2 binary location: LSB */,
    HPI_DEV_REG_FW_2_BIN_LOC_MSB        = 0x2B          /**< FW2 binary location: MSB */,
    HPI_DEV_REG_PORT_ENABLE             = 0x2C,         /**< Port enable command/status register. */
    HPI_DEV_REG_SLEEP_CTRL              = 0x2D,         /**< Deep sleep control register. */
    HPI_DEV_REG_POWER_STAT              = 0x2E,         /**< Power status register used to update status message. */
    HPI_DEV_REG_SET_APP_PRIORITY        = 0x2F,         /**< Set APP Priority. */
    HPI_DEV_REG_READ_CUSTOMER_INFO      = 0x30,         /**< Read 32 Bytes of customer info. */
    HPI_DEV_REG_BATTERY_STAT            = 0x31,         /**< Register showing the current battery status. */
    HPI_DEV_REG_WD_RESET_COUNT          = 0x32,         /**< Watchdog reset count register. */
    HPI_DEV_REG_READ_DIE_INFO           = 0x33,         /**< Read 32 bytes of device's Die and Version Information. */
    HPI_DEV_REG_RESERVED_34H            = 0x34,         /**< Reserved address: 0x0034. */
    HPI_DEV_REG_RESERVED_35H            = 0x35,         /**< Reserved address: 0x0035. */
    HPI_DEV_REG_RESERVED_36H            = 0x36,         /**< Reserved address: 0x0036. */
    HPI_DEV_REG_RESERVED_37H            = 0x37,         /**< Reserved address: 0x0037. */

    HPI_DEV_REG_UCSI_STATUS             = 0x38,         /**< UCSI status register. */
    HPI_DEV_REG_UCSI_CONTROL            = 0x39,         /**< UCSI control register. */

    HPI_DEV_REG_CFGTAB_VERSION          = 0x3A,         /**< Config table version register. */
    HPI_DEV_REG_SYS_PWR_STATE           = 0x3B,         /**< System Power State register. */
    HPI_DEV_REG_HPI_VERSION             = 0x3C,         /**< HPI version register: LSB */
    HPI_DEV_REG_HPI_VERSION_B1          = 0x3D,         /**< HPI version register: LSB */
    HPI_DEV_REG_HPI_VERSION_B2          = 0x3E,         /**< HPI version register: LSB */
    HPI_DEV_REG_HPI_VERSION_B3          = 0x3F,         /**< HPI version register: LSB */

    HPI_DEV_REG_USERDEF_00              = 0x40,         /**< User defined register number 0. */
    HPI_DEV_REG_USERDEF_01              = 0x41,         /**< User defined register number 1. */
    HPI_DEV_REG_USERDEF_02              = 0x42,         /**< User defined register number 2. */
    HPI_DEV_REG_USERDEF_03              = 0x43,         /**< User defined register number 3. */
    HPI_DEV_REG_USERDEF_04              = 0x44,         /**< User defined register number 4. */
    HPI_DEV_REG_USERDEF_05              = 0x45,         /**< User defined register number 5. */
    HPI_DEV_REG_USERDEF_06              = 0x46,         /**< User defined register number 6. */
    HPI_DEV_REG_USERDEF_07              = 0x47,         /**< User defined register number 7. */
    HPI_DEV_REG_USERDEF_08              = 0x48,         /**< User defined register number 8. */
    HPI_DEV_REG_USERDEF_09              = 0x49,         /**< User defined register number 9. */
    HPI_DEV_REG_USERDEF_0A              = 0x4A,         /**< User defined register number 10. */
    HPI_DEV_REG_USERDEF_0B              = 0x4B,         /**< User defined register number 11. */
    HPI_DEV_REG_USERDEF_0C              = 0x4C,         /**< User defined register number 12. */
    HPI_DEV_REG_USERDEF_0D              = 0x4D,         /**< User defined register number 13. */
    HPI_DEV_REG_USERDEF_0E              = 0x4E,         /**< User defined register number 14. */
    HPI_DEV_REG_USERDEF_0F              = 0x4F,         /**< User defined register number 15. */

    HPI_DEV_BB_DISABLE_RESET            = 0x50,         /**< Disable Billboard reset request register. */
    HPI_DEV_BB_ALT_MODE_STATUS          = 0x52,         /**< Alt. Mode status for use by Billboard device. */
    HPI_DEV_BB_ALT_MODE_STAT_B1         = 0x53,         /**< Alt. Mode status for use by Billboard device. */
    HPI_DEV_BB_OPER_MODEL               = 0x54,         /**< Billboard operating mode configuration. */
    HPI_DEV_BB_ADDL_FAILURE_INFO        = 0x55,         /**< Used to provide bAdditionalFailureInfo value for Billboard device. */
    HPI_DEV_BB_CMD_REG                  = 0x56,         /**< Billboard command register. */
    HPI_DEV_BB_MISC_INFO                = 0x57,         /**< Billboard misc info such as Self-powered, etc. */
    HPI_DEV_REG_BB_BL_VER_B0            = 0x58,         /**< Billboard bootloader version: Byte 0 */
    HPI_DEV_REG_BB_BL_VER_B1            = 0x59,         /**< Billboard bootloader version: Byte 1 */
    HPI_DEV_REG_BB_BL_VER_B2            = 0x5A,         /**< Billboard bootloader version: Byte 2 */
    HPI_DEV_REG_BB_BL_VER_B3            = 0x5B,         /**< Billboard bootloader version: Byte 3 */
    HPI_DEV_REG_BB_BL_VER_B4            = 0x5C,         /**< Billboard bootloader version: Byte 4 */
    HPI_DEV_REG_BB_BL_VER_B5            = 0x5D,         /**< Billboard bootloader version: Byte 5 */
    HPI_DEV_REG_BB_BL_VER_B6            = 0x5E,         /**< Billboard bootloader version: Byte 6 */
    HPI_DEV_REG_BB_BL_VER_B7            = 0x5F,         /**< Billboard bootloader version: Byte 7 */
    HPI_DEV_REG_BB_FW_VER_B0            = 0x60,         /**< Billboard firmware version: Byte 0 */
    HPI_DEV_REG_BB_FW_VER_B1            = 0x61,         /**< Billboard firmware version: Byte 1 */
    HPI_DEV_REG_BB_FW_VER_B2            = 0x62,         /**< Billboard firmware version: Byte 2 */
    HPI_DEV_REG_BB_FW_VER_B3            = 0x63,         /**< Billboard firmware version: Byte 3 */
    HPI_DEV_REG_BB_FW_VER_B4            = 0x64,         /**< Billboard firmware version: Byte 4 */
    HPI_DEV_REG_BB_FW_VER_B5            = 0x65,         /**< Billboard firmware version: Byte 5 */
    HPI_DEV_REG_BB_FW_VER_B6            = 0x66,         /**< Billboard firmware version: Byte 6 */
    HPI_DEV_REG_BB_FW_VER_B7            = 0x67,         /**< Billboard firmware version: Byte 7 */
    HPI_DEV_REG_AUTO_CMD_ADDR_P0        = 0x68,         /**< HPI Port 0 Auto specific register: Byte 0 */
    HPI_DEV_REG_AUTO_CMD_PARAM_P0       = 0x69,         /**< HPI Port 0 Auto specific register: Byte 1 */
    HPI_DEV_REG_RESERVED_00             = 0x70,         /**< HPI Reserved Device Register 0 */
    HPI_DEV_REG_BLACK_BOX               = 0x71,         /**< Black box register. */
    HPI_TEST_CODE_ADDR                  = 0x72,         /**< TestCode coverage register */
    HPI_DEV_REG_AUTO_CMD_ADDR_P1        = 0x73,         /**< HPI Port 1 Auto specific register: Byte 0 */
    HPI_DEV_REG_AUTO_CMD_PARAM_P1       = 0x74,         /**< HPI Port 1 Auto specific register: Byte 1 */
    HPI_DEV_REG_RESPONSE_LIN            = 0x7C,         /**< Response type register for LIN */
    HPI_DEV_REG_RESPONSE_LEN_LIN        = 0x7D,         /**< Response length register for LIN. */
    HPI_DEV_REG_RESPONSE                = 0x7E,         /**< Response type register. */
    HPI_DEV_REG_RESPONSE_LEN            = 0x7F,         /**< Response length register. */

    HPI_DEV_SCRATCHPAD_REGISTER         = 0xA0,         /**< EC can read/write any value to this register */
    HPI_DEV_IECS_COMMAND                = 0xA4,         /**< Four CC cmds for IECS firmware update */
    
    HPI_DEV_REG_FLASH_MEM               = 0x0200        /**< Flash read/write memory region. This extends for 256
                                                             bytes from this address. */
} hpi_dev_reg_address_t;

/** \} group_autoHPI_enums */

/** \addtogroup group_autoHPI_macros
* \{
* This section describes the WLC HPI Macros.
* Detailed information about the macros is available in each macro description.
*/

/** Number of reserved device space registers. */
#define HPI_DEVICE_RESERVED_COUNT      ((uint8_t)HPI_DEV_REG_RESPONSE_LIN - (uint8_t)HPI_DEV_REG_AUTO_CMD_PARAM_P1 - 1u)

/** Number of user defined HPI registers supported. */
#define HPI_USERDEF_REG_COUNT           (16u)

/** FRS RX enable bit in FRS_ENABLE register. */
#define HPI_FRS_ENABLE_RX               (0x01u)

/** FRS TX enable bit in FRS_ENABLE register. */
#define HPI_FRS_ENABLE_TX               (0x02u)

/** Bit position for boot priority field in HPI version register. */
#define HPI_VERS_BOOT_PRIO_POS          (13)

/** Mask for boot priority field in HPI version register. */
#define HPI_VERS_BOOT_PRIO_MASK         (0xE000)

/** \} group_autoHPI_macros */

/** \addtogroup group_autoHPI_enums
* \{
* This section describes the WLC HPI Enumerated Types.
* Detailed information about the enums is available in each enum description.
*/


/**
 * @typedef hpi_port_reg_address_t
 * @brief List of USB-PD port specified register addresses.
 *
 * This enumeration lists the registers used to control one USB-PD port on the CCG device. The addresses listed are
 * offsets on top of the base register for the port. Separate sets of these registers will exist for each port.
 */
typedef enum
{
    HPI_PORT_REG_VDM_CTRL               = 0x00,         /**< VDM control register. */
    HPI_PORT_REG_VDM_CTRL_LEN           = 0x01,         /**< VDM length register. */
    HPI_PORT_REG_EFF_SRC_PDO_MASK       = 0x02,         /**< Effective source PDO mask register: Read-only. */
    HPI_PORT_REG_EFF_SINK_PDO_MASK      = 0x03,         /**< Effective sink PDO mask register: Read-only. */
    HPI_PORT_REG_SOURCE_PDO_ADDR        = 0x04,         /**< Source PDO mask selection register. */
    HPI_PORT_REG_SINK_PDO_ADDR          = 0x05,         /**< Sink PDO mask selection register. */
    HPI_PORT_REG_PD_CTRL                = 0x06,         /**< PD control register. */
    HPI_PORT_REG_BYTE_7_RESERVED        = 0x07,         /**< Reserved register. */
    HPI_PORT_REG_PD_STATUS              = 0x08,         /**< PD status register: LS Byte */
    HPI_PORT_REG_PD_STATUS_BYTE_1       = 0x09,         /**< PD status register: Byte 1 */
    HPI_PORT_REG_PD_STATUS_BYTE_2       = 0x0A,         /**< PD status register: Byte 2 */
    HPI_PORT_REG_PD_STATUS_BYTE_3       = 0x0B,         /**< PD status register: MS Byte */
    HPI_PORT_REG_TYPE_C_STATUS          = 0x0C,         /**< Type-C status register. */
    HPI_PORT_REG_BUS_VOLTAGE            = 0x0D,         /**< VBus voltage in 100 mV units. */
    HPI_PORT_REG_BYTE_14_RESERVED       = 0x0E,         /**< Reserved register. */
    HPI_PORT_REG_BYTE_15_RESERVED       = 0x0F,         /**< Reserved register. */
    HPI_PORT_REG_CUR_PDO                = 0x10,         /**< Current PDO register: LS Byte */
    HPI_PORT_REG_CUR_PDO_BYTE_1         = 0x11,         /**< Current PDO register: Byte 1 */
    HPI_PORT_REG_CUR_PDO_BYTE_2         = 0x12,         /**< Current PDO register: Byte 2 */
    HPI_PORT_REG_CUR_PDO_BYTE_3         = 0x13,         /**< Current PDO register: MS Byte */
    HPI_PORT_REG_CUR_RDO                = 0x14,         /**< Current RDO register: LS Byte */
    HPI_PORT_REG_CUR_RDO_BYTE_1         = 0x15,         /**< Current RDO register: Byte 1 */
    HPI_PORT_REG_CUR_RDO_BYTE_2         = 0x16,         /**< Current RDO register: Byte 2 */
    HPI_PORT_REG_CUR_RDO_BYTE_3         = 0x17,         /**< Current RDO register: MS Byte */
    HPI_PORT_REG_CABLE_VDO              = 0x18,         /**< Cable VDO register: LS Byte */
    HPI_PORT_REG_CABLE_VDO_BYTE_1       = 0x19,         /**< Cable VDO register: Byte 1 */
    HPI_PORT_REG_CABLE_VDO_BYTE_2       = 0x1A,         /**< Cable VDO register: Byte 2 */
    HPI_PORT_REG_CABLE_VDO_BYTE_3       = 0x1B,         /**< Cable VDO register: MS Byte */

    HPI_PORT_REG_ALT_MODE_CMD           = 0x1C,         /**< ALT MODE command register: LS Byte */
    HPI_PORT_REG_ALT_MODE_CMD_BYTE_1    = 0x1D,         /**< ALT MODE command register: Byte 1 */
    HPI_PORT_REG_ALT_MODE_CMD_BYTE_2    = 0x1E,         /**< ALT MODE command register: Byte 2 */
    HPI_PORT_REG_ALT_MODE_CMD_BYTE_3    = 0x1F,         /**< ALT MODE command register: MS Byte */
    HPI_PORT_REG_APP_HW_CMD             = 0x20,         /**< HW control command register: LS Byte */
    HPI_PORT_REG_APP_HW_CMD_BYTE_1      = 0x21,         /**< HW control command register: Byte 1 */
    HPI_PORT_REG_APP_HW_CMD_BYTE_2      = 0x22,         /**< HW control command register: Byte 2 */
    HPI_PORT_REG_APP_HW_CMD_BYTE_3      = 0x23,         /**< HW control command register: MS Byte */

    HPI_PORT_REG_EVENT_MASK             = 0x24,         /**< Event mask register: LS Byte */
    HPI_PORT_REG_EVENT_MASK_BYTE_1      = 0x25,         /**< Event mask register: Byte 1 */
    HPI_PORT_REG_EVENT_MASK_BYTE_2      = 0x26,         /**< Event mask register: Byte 2 */
    HPI_PORT_REG_EVENT_MASK_BYTE_3      = 0x27,         /**< Event mask register: MS Byte */
    HPI_PORT_REG_SWAP_RESPONSE          = 0x28,         /**< Swap response register. */
    HPI_PORT_REG_ACTIVE_EC_MODES        = 0x29,         /**< EC ALT MODES active register. */
    HPI_PORT_REG_VDM_EC_CTRL            = 0x2A,         /**< VDM control by EC enable register. */
    HPI_PORT_REG_ALT_MODE_STATUS        = 0x2B,         /**< Alternate mode status register. */
    HPI_PORT_REG_BC_1_2_CONTROL         = 0x2C,         /**< BC 1.2 control register. */

    HPI_PORT_REG_BYTE_45_RESERVED       = 0x2D,         /**< Reserved register. */
    HPI_PORT_REG_BYTE_46_RESERVED       = 0x2E,         /**< Reserved register. */
    HPI_PORT_REG_BYTE_47_RESERVED       = 0x2F,         /**< Reserved register. */
    HPI_PORT_REG_CMD_TIMEOUT            = 0x30,         /**< Timeout associated with VDM and PD commands (ms). */

    HPI_PORT_REG_FRS_ENABLE             = 0x31,         /**< Fast Role Swap enable register. */
    HPI_PORT_REG_CONSUMER_FET_CTRL      = 0x32,         /**< VBUS Consumer FET Control register. */
    HPI_PORT_REG_BC_1_2_STATUS          = 0x33,         /**< BC 1.2 Status register. */
    HPI_PORT_REG_INTERRUPT_STATUS       = 0x34,         /**< Interrupt status register: LSB. */
    HPI_PORT_REG_INT_STAT_BYTE_1        = 0x35,         /**< Interrupt status register: Byte 1. */
    HPI_PORT_REG_INT_STAT_BYTE_2        = 0x36,         /**< Interrupt status register: Byte 2. */
    HPI_PORT_REG_INT_STAT_BYTE_3        = 0x37,         /**< Interrupt status register: MSB. */

    HPI_PORT_REG_USERDEF_00             = 0x38,         /**< User defined register number 00. */
    HPI_PORT_REG_USERDEF_01             = 0x39,         /**< User defined register number 01. */
    HPI_PORT_REG_USERDEF_02             = 0x3A,         /**< User defined register number 02. */
    HPI_PORT_REG_USERDEF_03             = 0x3B,         /**< User defined register number 03. */
    HPI_PORT_REG_USERDEF_04             = 0x3C,         /**< User defined register number 04. */
    HPI_PORT_REG_USERDEF_05             = 0x3D,         /**< User defined register number 05. */
    HPI_PORT_REG_USERDEF_06             = 0x3E,         /**< User defined register number 06. */
    HPI_PORT_REG_USERDEF_07             = 0x3F,         /**< User defined register number 07. */
    HPI_PORT_REG_USERDEF_08             = 0x40,         /**< User defined register number 08. */
    HPI_PORT_REG_USERDEF_09             = 0x41,         /**< User defined register number 09. */
    HPI_PORT_REG_USERDEF_0A             = 0x42,         /**< User defined register number 0A. */
    HPI_PORT_REG_USERDEF_0B             = 0x43,         /**< User defined register number 0B. */
    HPI_PORT_REG_USERDEF_0C             = 0x44,         /**< User defined register number 0C. */
    HPI_PORT_REG_USERDEF_0D             = 0x45,         /**< User defined register number 0D. */
    HPI_PORT_REG_USERDEF_0E             = 0x46,         /**< User defined register number 0E. */
    HPI_PORT_REG_USERDEF_0F             = 0x47,         /**< User defined register number 0F. */
    HPI_PORT_REG_USERDEF_10             = 0x48,         /**< User defined register number 10. */
    HPI_PORT_REG_USERDEF_11             = 0x49,         /**< User defined register number 11. */
    HPI_PORT_REG_USERDEF_12             = 0x4A,         /**< User defined register number 12. */
    HPI_PORT_REG_USERDEF_13             = 0x4B,         /**< User defined register number 13. */
    HPI_PORT_REG_USERDEF_14             = 0x4C,         /**< User defined register number 14. */
    HPI_PORT_REG_USERDEF_15             = 0x4D,         /**< User defined register number 15. */
    HPI_PORT_REG_USERDEF_16             = 0x4E,         /**< User defined register number 16. */
    HPI_PORT_REG_USERDEF_17             = 0x4F,         /**< User defined register number 17. */

    HPI_PORT_REG_ACT_CBL_VDO_2          = 0x50,         /**< Active Cable VDO 2 byte 0 (LSB). */
    HPI_PORT_REG_ACT_CBL_VDO_2_B1       = 0x51,         /**< Active Cable VDO 2 byte 1. */
    HPI_PORT_REG_ACT_CBL_VDO_2_B2       = 0x52,         /**< Active Cable VDO 2 byte 2. */
    HPI_PORT_REG_ACT_CBL_VDO_2_B3       = 0x53,         /**< Active Cable VDO 2 byte 3 (MSB). */

    HPI_PORT_REG_SINK_RDO_REQUEST       = 0x54,         /**< Send RDO from EC. */
    HPI_PORT_REG_BUS_CURRENT            = 0x58,         /**< VBus current in 50 mA units. */

    HPI_PORT_REG_HOST_CAP_CTRL          = 0x5C,         /**< Host Capabilities Control register. */

    HPI_PORT_REG_ALT_MODE_MASK          = 0x60,         /**< Alt Mode Mask register */
    HPI_PORT_REG_ALT_MODE_SVID          = 0x62,         /**< Alt Mode SVID register Byte 0 */
    HPI_PORT_REG_ALT_MODE_SVID_BYTE_1   = 0x63,         /**< Alt Mode SVID register Byte 1 */
    
    HPI_PORT_REG_FAULT_PIN_MASK         = 0x80,         /**< Register to send RDO request*/
    HPI_PORT_REG_SNK_FET_DISABLE        = 0x84,         /**< Reserved register. */
    HPI_PORT_REG_SNK_FET_CONTROL        = 0x85,         /**< Reserved register. */
    HPI_PORT_REG_SAFE_PWR_CONTROL       = 0x86,         /**< Reserved register. */
    HPI_PORT_REG_OVP_CONFIG             = 0x88,         /**< Reserved register. */
    HPI_PORT_REG_OCP_CONFIG             = 0x8C,         /**< Reserved register. */
    HPI_PORT_REG_OTP_CONFIG             = 0x90,         /**< Reserved register. */
    HPI_PORT_REG_CHARGING_MODE_DISABLE  = 0x94,         /**< Reserved register. */
    HPI_PORT_REG_CHARGING_MODE_STATUS   = 0x95,         /**< Reserved register. */

    HPI_PORT_REG_ADVDM_ENTER_DC_STATUS  = 0xA0,         /**< ADVDM EnterDC status. */
    HPI_PORT_REG_DELAY_FOR_HARD_RESET   = 0xA4,         /**< Register to set HARD_RESET notification time. */
    HPI_PORT_REG_ADAPTER_SRC_PDO_COUNT  = 0xA6,         /**< Count of port partner SRC PDOs */
    
    HPI_PORT_READ_DATA_MEM_ADDR         = 0x400,        /**< Read data memory. */
    HPI_PORT_WRITE_DATA_MEM_ADDR        = 0x800         /**< Write data memory. */

} hpi_port_reg_address_t;

/**
 * @typedef hpi_pd_ctrl_cmd_t
 * @brief PD control commands accepted through HPI.
 */
typedef enum
{
    HPI_PD_CMD_SET_RP_DFLT = 0,                 /**< Set Rp to default USB profile. */
    HPI_PD_CMD_SET_RP_1A5 = 1,                  /**< Set Rp to 1.5 A profile. */
    HPI_PD_CMD_SET_RP_3A0 = 2,                  /**< Set Rp to 3.0 A profile. */
    HPI_PD_CMD_DRSWAP = 5,                      /**< Initiate DR-Swap. */
    HPI_PD_CMD_PRSWAP = 6,                      /**< Initiate PR-Swap. */
    HPI_PD_CMD_VCONN_ON = 7,                    /**< Enable VConn. */
    HPI_PD_CMD_VCONN_OFF = 8,                   /**< Disable VConn. */
    HPI_PD_CMD_VCONSWAP = 9,                    /**< Initiate VConn-Swap. */
    HPI_PD_CMD_GETSRCCAP = 10,                  /**< Get Src Cap. */
    HPI_PD_CMD_GETSNKCAP = 11,                  /**< Get Snk Cap. */
    HPI_PD_CMD_GOTOMIN = 12,                    /**< Send Go To Min */
    HPI_PD_CMD_HARDRESET = 13,                  /**< Send Hard Reset */
    HPI_PD_CMD_SOFTRESET = 14,                  /**< Send Soft Reset. */
    HPI_PD_CMD_CABLERESET = 15,                 /**< Send Cable Reset. */
    HPI_PD_CMD_EC_READY = 16,                   /**< EC Init Complete. */
    HPI_PD_CMD_PD_DEINIT = 17,                  /**< PD port de-init. */
    HPI_PD_CMD_SOPP_RESET = 18,                 /**< Send Soft Reset - SOP' */
    HPI_PD_CMD_SOPDP_RESET = 19,                /**< Send Soft Reset - SOP'' */
    HPI_PD_CMD_CHANGE_PORT_PARAMS = 20,         /**< Command to change port parameters. The following can be changed:
                                                     1. Port role
                                                     2. Default port role
                                                     3. DRP toggle enable
                                                     4. Try.Src enable. */
    HPI_PD_CMD_ABORT_PENDING = 21,              /**< Abort pending command. */
    HPI_PD_CMD_GET_EXT_SRCCAP = 22,             /**< Send a get extended source capabilities message. */
    HPI_PD_CMD_GET_STATUS = 23,                 /**< Send a get status message. */
    HPI_PD_CMD_SEND_NOTSUPP = 24,               /**< Send a NOT_SUPPORTED control message. */
    HPI_PD_CMD_SEND_DATA_RESET = 25,            /**< Initiate a Data_Reset sequence. */
    HPI_PD_CMD_GET_EXT_SNKCAP = 26,             /**< Send a Get_Sink_Cap_Extended message. */

    HPI_PD_CMD_OVP_DETECT = 28,                 /**< OVP detected by EC. */
    HPI_PD_CMD_OCP_DETECT = 29,                 /**< OCP detected by EC. */
    HPI_PD_CMD_OTP_DETECT = 30,                 /**< OTP detected by EC. */
    HPI_PD_CMD_PING = 31,                       /**< Ping command used to see if CCG is responsive. */

    HPI_PD_CMD_READ_SRC_PDO = 32,               /**< Read Source PDO list */
    HPI_PD_CMD_READ_SNK_PDO = 33,               /**< Read Sink PDO list */
    HPI_PD_CMD_READ_SNK_CURRENT = 34,           /**< Read sink min/max current list. */
    HPI_PD_CMD_WRITE_SNK_CURRENT = 35,          /**< Write sink min/max current list. */
    HPI_PD_CMD_READ_EXT_SRCCAP = 36,            /**< Read extended source capabilities. */
    HPI_PD_CMD_WRITE_EXT_SRCCAP = 37,           /**< Write extended source capabilities. */
    HPI_PD_CMD_READ_EXT_SNKCAP = 38,            /**< Read extended sink capabilities. */
    HPI_PD_CMD_WRITE_EXT_SNKCAP = 39,           /**< Write extended sink capabilities. */

    HPI_PD_CMD_GET_DISC_ID_RESP = 48,           /**< Get Disc ID response from port partner. */
    HPI_PD_CMD_GET_DISC_SVID_RESP = 49,         /**< Get Disc SVID response from port partner. */
    HPI_PD_CMD_GET_DISC_MODE_RESP = 50,         /**< Get Disc Mode response for a specific SVID value. */

    HPI_PD_CMD_GET_PP_SRC_PDO = 51,             /**< Get the Port Partner's cached Source PDO list. */

    HPI_PD_CMD_RESERVED_52 = 52,                /**< Reserved command. Do not use. */
    HPI_PD_CMD_RESERVED_53 = 53,                /**< Reserved command. Do not use. */
    HPI_PD_CMD_SWITCH_VSYS_TO_VBUS = 54,        /**< Turn off VSYS switch and Turn on VBUS regulator */
    HPI_PD_CMD_SWITCH_VBUS_TO_VSYS = 55,        /**<  Turn off VBUS regulator and Turn on VSYS switch  */
    HPI_PD_CMD_RESTART_VDM = 56,                /**< Restart VDM. */
    HPI_PD_CMD_GET_PPS_STATUS = 57,             /**< Send a get PPS Status message */
    HPI_PD_CMD_GET_COUNTRY_CODES = 58,          /**< Send a get country Codes message */
    HPI_PD_CMD_RW_I2C_DEV_REG = 60,             /**< Read Write I2C Device Register.*/

    HPI_PD_CMD_PWR_SHARE_DISABLE = 64,          /**< Disable power sharing feature on dual-port devices. */
    HPI_PD_CMD_PWR_SHARE_ENABLE = 65,           /**< Enable power sharing feature on dual-port devices. */

#if (AUVDM_SUPPORT!= 0)
    /* AUVDM_SUPPORTGet Capabilities */        
    HPI_PD_CTRL_GET_SERIAL_NUMBER = 80,         /**< Get Serial Number from AUVDM messages */        
    HPI_PD_CTRL_GET_VENDOR_NAME,                /**< Get Vendor Name from AUVDM messages */        
    HPI_PD_CTRL_GET_PRODUCT_NAME,               /**< Get Product Name from AUVDM messages */        
    HPI_PD_CTRL_GET_USER_STRING,                /**< Get User String from AUVDM messages */        
    HPI_PD_CTRL_GET_MODEL_STRING,               /**< Get Model String from AUVDM messages */        
    HPI_PD_CTRL_GET_MANUFACTURER,               /**< Get Manufacturer from AUVDM messages */        
    HPI_PD_CTRL_GET_FW_VERSION,                 /**< Get FW Version from AUVDM messages */        
    HPI_PD_CTRL_GET_HW_VERSION,                 /**< Get HW Version from AUVDM messages */        
#endif /* (AUVDM_SUPPORT!= 0) */

} hpi_pd_ctrl_cmd_t;

/** \} group_autoHPI_enums */

/** \addtogroup group_autoHPI_data_structures
* \{
* This section describes the WLC HPI Structures.
* Detailed information about the structures is available in each structure description.
*/


/**
 * @brief Union to hold parameters associated with a VDM_CTRL command.
 */
typedef union
{
    
    uint16_t    val;                                            /**< VDM control parameter value */
    /** Structure associated with a VDM_CTRL command. */
    struct 
    {
    
        uint16_t sop_type           : 2;                       /**< SOP: Used for communication with port partner. */ 
        uint16_t pd3_support        : 1;                       /**< PD3 Support */ 
        uint16_t extended           : 1;                       /**< Extended messages support */
        uint16_t timeout_disable    : 1;                        /**< Timeout Disable */
        uint16_t rsrvd_1            : 1;                        /**< Reserved Field */
        uint16_t len_msb            : 2;                        /**< Total Length is 10 bits. This filed contains 2 bit MSB */ 
        uint16_t len_lsb            : 8;                        /**< Total Length is 10 bits. This filed contains 8 bit LSB */
    } param;

} hpi_vdm_ctrl_param_t;

/** \} group_autoHPI_data_structures */

#endif /* HPI_INTERNAL_H_ */

 /** \} group_autoHPI */
 
/* [] END OF FILE */
