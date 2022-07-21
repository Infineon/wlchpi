/***************************************************************************//**
* \file hpi.h
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
* \defgroup group_autoHPI_macros Macros
* \defgroup group_autoHPI_functions Functions
* \defgroup group_autoHPI_data_structures Data Structures
* \defgroup group_autoHPI_enums Enumerated Types
* */


#ifndef HPI_H_
#define HPI_H_

#include <stdbool.h>
#include <stdint.h>
#include "cy_pdstack_common.h"
#include "i2c.h"
#include "cy_usbpd_config_table.h"
#include "config.h"
#if CCG_HPI_ENABLE
#include "battery_charging.h"

#if CCG_HPI_PD_ENABLE
#include "cy_pdstack_dpm.h"
#include "app.h"
#endif /* CCG_HPI_PD_ENABLE */

/**************************************************************************************************
 ******************************************** MACROS **********************************************
 *************************************************************************************************/
 
/** \addtogroup group_autoHPI_enums
* \{
* This section describes the WLC HPI Enumerated Types.
* Detailed information about the enums is available in each enum description.
*/
#if CCG_UCSI_ENABLE

/**
 * @typedef hpi_ucsi_status_reg_t
 * @brief HPI UCSI Status register values.
 *
 * Status values for the UCSI Status register defined in the HPI register
 * space.
 */
typedef enum
{
    HPI_UCSI_STATUS_STARTED = 0x00,  /**< Status value to indicate UCSI is started. */
    HPI_UCSI_STATUS_CMD_IN_PROGRESS, /**< Status value to indicate UCSI Command in progress. */
    HPI_UCSI_STATUS_EVENT_PENDING,   /**< Status value to indicate UCSI event pending. */
} hpi_ucsi_status_reg_t;

/**
 * @typedef hpi_ucsi_control_cmds_t
 * @brief HPI UCSI Control commands.
 *
 * Commands to control the UCSI interface.
 */
typedef enum
{
    HPI_UCSI_START_CMD = 0x01,      /**< UCSI Control Register Start value.
                                      This commands starts the UCSI Interface.
                                      */
    HPI_UCSI_STOP_CMD,              /**< UCSI Control Register Stop value. This
                                      command stops the UCSI interface. */
    HPI_UCSI_SILENCE_CMD,           /**< UCSI Control Register Silence value.
                                      This command silences the UCSI Port. */
    HPI_UCSI_SIG_CONNECT_EVT_CMD,   /**< UCSI Control Register Signal
                                           Connect Event value. EC sends this
                                           command to ask CCG to send connect
                                           event information to OS. */

} hpi_ucsi_control_cmds_t;

#endif /* CCG_UCSI_ENABLE */

/** \} group_autoHPI_enums */

/** \addtogroup group_autoHPI_macros
* \{
* This section describes the WLC HPI Macros.
* Detailed information about the macros is available in each macro description.
*/

#define HPI_SCB_INDEX_INVALID           (0x0Fu)
/**< Invalid HPI index. This configuration disconnects I2C interface from HPI. */

#define HPI_ADDR_I2C_CFG_LOW            (0x40u)
/**< I2C slave address to be used for HPI interface, when the I2C_CFG pin is sensed as LOW. */

#define HPI_ADDR_I2C_CFG_HIGH           (0x42u)
/**< I2C slave address to be used for HPI interface, when the I2C_CFG pin is sensed as HIGH. */

#define HPI_ADDR_I2C_CFG_FLOAT          (0x08u)
/**< I2C slave address to be used for HPI interface, when the I2C_CFG pin is sensed as FLOATING. */

/** Default value when no CFG_STRAP for I2C address */
#define HPI_I2C_DEFAULT_ADDR                        (0x57)

/* VBUS CONSUMER FET CONTROL REGISTER bit definitions.*/
#define VBUS_CFET_CTRL_EC_CTRL_EN_POS           (0u)
/**< VBUS consumer FET control register EC control enable bit position.*/
#define VBUS_CFET_CTRL_EC_CTRL_EN               (1u << VBUS_CFET_CTRL_EC_CTRL_EN_POS)
/**< VBUS consumer FET control register EC control enable.*/
#define VBUS_CFET_CTRL_EC_CFET_ON_POS           (1u)
/**< VBUS consumer FET control register EC CFET on bit position.*/
#define VBUS_CFET_CTRL_EC_CFET_ON               (1u << VBUS_CFET_CTRL_EC_CFET_ON_POS)
/**< VBUS consumer FET control register EC CFET on.*/

/**************/
/* BCR MACROS */
/**************/
#if(BCR)
#define HPI_BCR_DISABLE_CODE            ('D')
/**< Disable value argument in HPI command for BCR. */
#define HPI_BCR_ENABLE_CODE             ('E')
/**< Enable value argument in HPI command for BCR. */
#define HPI_BCR_CONFIG_CODE             ('C')
/**< Configure value argument in HPI command for BCR. */
#define HPI_BCR_RESET_CODE              ('R')
/**< Reset to default value argument in HPI command for BCR. */
#define HPI_BCR_CLEAR_LOWEST_BYTE_MASK  (0xFFFFFF00u)
/**< Mask to clear lowest byte in protection configuration. */
#endif /* BCR */

/** \} group_autoHPI_macros */

/**************************************************************************************************
 ****************************************** DATA TYPES ********************************************
 *************************************************************************************************/
 
/** \addtogroup group_autoHPI_enums
* \{
* This section describes the WLC HPI Enumerated Types.
* Detailed information about the enums is available in each enum description.
*/

/**
 * @typedef hpi_reg_section_t
 * @brief HPI register section definitions.
 *
 * HPI registers are grouped into sections corresponding to the functions that are supported.
 */
typedef enum
{
    HPI_REG_SECTION_DEV = 0,            /**< Device information registers. */
    HPI_REG_SECTION_PORT_0,             /**< USB-PD Port 0 related registers. */
    HPI_REG_SECTION_PORT_1,             /**< USB-PD Port 1 related registers. */
    HPI_REG_SECTION_LIN_DEV = 0x03,     /**< Device Information section for LIN. */
    HPI_REG_SECTION_DEV_AUTO_P1 = 0x05, /**< HPI Auto Port 1 related registers */
    HPI_REG_SECTION_DEV_AUTO_P0 = 0x06, /**< HPI Auto Port 0 related registers */
#if CCG_UCSI_ENABLE
    HPI_REG_SECTION_UCSI = 0x0F,        /**< UCSI related registers. */
#endif /*CCG_UCSI_ENABLE*/
    HPI_REG_SECTION_ALL                 /**< Special definition to select all register spaces. */
} hpi_reg_section_t;

/**
 * @typedef hpi_reg_part_t
 * @brief Types of HPI register/memory regions.
 */
typedef enum
{
    HPI_REG_PART_REG = 0,               /**< Register region. */
    HPI_REG_PART_DATA = 1,              /**< Data memory for device section. */
    HPI_REG_PART_FLASH = 2,             /**< Flash memory. */
    HPI_REG_PART_PDDATA_READ = 4,       /**< Read Data memory for port section. */
    HPI_REG_PART_PDDATA_READ_H = 5,     /**< Upper fraction of read data memory for port section. */
    HPI_REG_PART_PDDATA_WRITE = 8,      /**< Write Data memory for port section. */
    HPI_REG_PART_PDDATA_WRITE_H = 9     /**< Upper fraction of write data memory for port section. */
} hpi_reg_part_t;

/**
 * @typedef hpi_mode_t
 * @brief Mode definitions for HPI bootloader and firmware mode.
 */
typedef enum
{
    HPI_MODE_BOOTLOADER = 0u,
    HPI_MODE_FIRMWARE,
    HPI_MODE_MAX
} hpi_mode_t;

/** \} group_autoHPI_enums */


/** \addtogroup group_autoHPI_data_structures
* \{
* This section describes the WLC HPI Structures.
* Detailed information about the structures is available in each structure description.
*/

/**
 * @typedef hpi_write_cb_t
 * @brief Handler for HPI register writes.
 * @return Type of response to be sent to the EC. Only a single byte response
 * can be sent from here. Use hpi_reg_enqueue_event to send longer responses.
 */
typedef uint8_t (*hpi_write_cb_t)(
        uint16_t  reg_addr,             /**< Address of register that got written. */
        uint8_t   wr_size,              /**< Size of write operation. */
        uint8_t  *wr_data               /**< Buffer containing data written. */
        );


#if SYS_BLACK_BOX_ENABLE

/**
 * @typedef hpi_black_box_cb_t
 * @brief Handler for black box access hpi command.
 * @return Address of black box.
 */
typedef uint32_t (*hpi_black_box_cb_t)(void);
#endif /* SYS_BLACK_BOX_ENABLE */

#if (CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)
typedef uint8_t (*hpi_auto_soln_cmd_handler_ptr) (cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t cmd, uint8_t *value, uint8_t cmd_len);
typedef struct
{
    bool hpi_bb_operation_enable;
    bool hpi_lin_operation_enable;
    bool hpi_lin_multi_master_mode;
    bool hpi_dual_firmware_enable;
    hpi_auto_soln_cmd_handler_ptr hpi_auto_soln_cmd_handler;
}hpi_config_run_time_t;
#endif /*(CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)*/
    
#if CCG_HPI_AUTO_CMD_ENABLE
/**
 * @typedef oc_placeholder_t 
 * @brief Structure to hold the OC reported values from solution space.
 */
typedef struct
{
    uint8_t oc_vin;
    uint8_t oc_temp;
} hpi_oc_buffer_t;

/**
 * @typedef hpi_auto_soln_cb_t
 * @brief Structure to hold the Solution interface for HPI Auto operation.
 * Solution is expected to fill the structure with required pointers to function
 * to accomplish the command specific tasks. All the registered functions
 * should be non-blocking and take minimum execution time.
 */
typedef struct
{
    const bc_status_t* (*soln_bc_get_status_handler) (
        cy_stc_pdstack_context_t *ptrPdStackContext               /**< PD port Context. */
        );
    auto_cfg_settings_t* (*soln_get_auto_config_table) (
        cy_stc_pdstack_context_t *ptrPdStackContext               /**< PD port Context. */
        );    
    uint32_t (*soln_get_fault_status) (
        cy_stc_pdstack_context_t *ptrPdStackContext               /**< PD port Context. */
        );
    
    cy_en_pdstack_status_t (*soln_get_sensor_temperature) (
        cy_stc_pdstack_context_t *ptrPdStackContext,              /**< PD port Context. */
        uint8_t *buffer                                           /**< Output temperature place holder. */
        );

    uint16_t (*soln_get_vbus_voltage) (
        cy_stc_usbpd_context_t *context,                            /**< USBPD port Context. */
        cy_en_usbpd_adc_id_t adcId,                                 /**< ADC ID. */
        cy_en_usbpd_adc_input_t input                               /**< ADC Input. */
        );

    uint16_t (*soln_get_vbus_current)  (
        cy_stc_usbpd_context_t *context                             /**< USBPD port Context. */
        );

    uint16_t (*soln_get_battery_voltage)  (
        cy_stc_pdstack_context_t *ptrPdStackContext               /**< PD port Context. */
        );

    uint8_t (*soln_get_oc_details)  (
        cy_stc_pdstack_context_t *ptrPdStackContext,              /**< PD port Context. */
        hpi_oc_buffer_t *buffer    /**<OC placeholder. */
        );
        
    uint32_t (*soln_get_version_details)  (
        cy_stc_pdstack_context_t *ptrPdStackContext              /**< PD port Context. */
        );
} hpi_auto_soln_cb_t;

/**
* @typedef hpi_auto_config_t
* @brief Defines the response structure for HPI Auto configuration.
*/
typedef struct
{
    bool vconn_is_present;
    bool heart_beat_on;
    uint8_t throttling;
    uint8_t system_oc;
} hpi_auto_config_t;
#endif /* CCG_HPI_AUTO_CMD_ENABLE */

/** \} group_autoHPI_data_structures */

/** \addtogroup group_autoHPI_enums
* \{
* This section describes the WLC HPI Enumerated Types.
* Detailed information about the enums is available in each enum description.
*/

#if (!CCG_LOAD_SHARING_ENABLE)
/**
 * @typedef hpi_boot_prio_conf_t
 * @brief Enumeration showing possible boot priority configurations for the firmware application.
 */
typedef enum
{
    HPI_BOOT_PRIO_LAST_FLASHED = 0,     /**< Last flashed firmware is prioritized. */
    HPI_BOOT_PRIO_APP_PRIO_ROW,         /**< Priority defined used App Priority flash row. */
    HPI_BOOT_PRIO_FW1,                  /**< FW1 is always prioritized. */
    HPI_BOOT_PRIO_FW2                   /**< FW2 is always prioritized. */
} hpi_boot_prio_conf_t;
#endif /* (!CCG_LOAD_SHARING_ENABLE) */

#if CCG_UCSI_ENABLE
/**
 * @typedef i2c_owner_t
 * @brief List of possible owners for the I2C slave interface. The interface can be used for one of HPI or UCSI
 * functionality at a time.
 */
typedef enum {
    I2C_OWNER_UCSI = 0,                 /**< I2C interface used for UCSI commands. */
    I2C_OWNER_HPI                       /**< I2C interface used for HPI commands. */
} i2c_owner_t;
#endif /* CCG_UCSI_ENABLE */

/** \} group_autoHPI_enums */

/**************************************************************************************************
 ************************************ FUNCTION DEFINITIONS ****************************************
 *************************************************************************************************/
 
/**
* \addtogroup group_autoHPI_functions
* \{
* This section describes the .
* Detailed information about each API is available in each function description.
*/

/**
 * @brief Initialize the HPI interface.
 *
 * This function initializes the I2C interface and EC_INT GPIO used for HPI hardware interface, and
 * initializes all HPI registers to their default values.
 *
 * @param PdStackContexts PdStack Library Context pointer
 * @param scb_idx Index of SCB block to be used for HPI. Please note that this parameter is
 * not validated, and it is the caller's responsibility to pass the correct value.
 * @return None
 */
void hpi_init(cy_stc_pdstack_context_t* PdStackContexts, uint8_t scb_idx);

#if (!CCG_LOAD_SHARING_ENABLE)
/**
 * @brief De-initialize the HPI interface.
 *
 * This function can be used to de-initialize the HPI interface. This can be used in applications
 * where the HPI master (Billboard controller) in the system may be powered off under control of
 * the CCG device.
 *
 * @return None
 */
void hpi_deinit(void);
#endif /* (!CCG_LOAD_SHARING_ENABLE) */

/**
 * @brief HPI task handler.
 *
 * This function handles the commands from the EC through the HPI registers. HPI writes from the EC
 * are handled in interrupt context, and any associated work is queued to be handled by this function.
 * The hpi_task is expected to be called periodically from the main task loop of the firmware
 * application.
 *
 * @return None
 */
void hpi_task(void);

/**
 * @brief Enqueue an event to the EC through the HPI interface.
 *
 * This function is used by the PD stack and application layers to send event
 * notifications to the EC through the HPI registers.
 *
 * Please note that only responses without associated data can be sent through
 * the response register for the HPI_REG_SECTION_DEV section. If any additional
 * response data is required, please use the user defined registers or the response
 * register associated with HPI_REG_SECTION_PORT_0 or HPI_REG_SECTION_PORT_1.
 *
 * @param section Register section through which event is to be reported.
 * @param status The event code to be stored into the response register.
 * @param length Length of the data associated with the event.
 * @param data Pointer to buffer containing data associated with the event.
 *
 * @return true if the event queue has space for the event, false if there
 * is an overflow.
 */
bool hpi_reg_enqueue_event(hpi_reg_section_t section, uint8_t status, uint16_t length,
    uint8_t *data);

#if (CCG_HPI_PD_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)

/**
 * @brief Handler for PD events reported from the stack.
 *
 * Internal function used to receive PD events from the stack and to update the HPI registers.
 *
 * @param ptrPdStackContext PdStack Library Context pointer
 * @param evt Event that is being notified.
 * @param data Data associated with the event. This is an opaque pointer that needs to be de-referenced
 * based on event type.
 *
 * @return None
 */
void hpi_pd_event_handler(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_app_evt_t evt, const void *data);

#endif /* (CCG_HPI_PD_ENABLE || CCG_HPI_AUTO_CMD_ENABLE) */

/**
 * @brief Check whether EC init complete event has been received.
 *
 * This function is used by the application to check whether the EC has sent
 * the EC initialization complete event notification.
 *
 * @return true if EC init has been received, false otherwise.
 */
bool hpi_is_ec_ready (void);

/**
 * @brief Update firmware version information in HPI registers.
 *
 * This is an internal function used to update the firmware version information
 * in the HPI registers.
 *
 * @param bl_version Buffer containing Bootloader version information.
 * @param fw1_version Buffer containing firmware-1 version information.
 * @param fw2_version Buffer containing firmware-2 version information.
 *
 * @return void
 */
void hpi_update_versions (uint8_t *bl_version, uint8_t *fw1_version,
        uint8_t *fw2_version);

#if (!CCG_LOAD_SHARING_ENABLE)
/**
 * @brief Check whether any HPI accesses have happened since start-up.
 * @return True if any HPI read/write access has happened.
 */
bool hpi_is_accessed (void);

#endif /* (!CCG_LOAD_SHARING_ENABLE) */

#if HPI_AUTO_SROM_ENABLE
/** 
 * @brief Set HPI mode when HPI is placed in ROM.
 *
 * @param Mode to set.
 *
 * @return Void.
 */
void hpi_set_mode(hpi_mode_t mode);

/** 
 * @brief Get HPI mode when HPI is placed in ROM.
 *
 * @param None.
 *
 * @return Current mode.
 */
hpi_mode_t hpi_get_mode(void);
#endif /* HPI_AUTO_SROM_ENABLE */

/**
 * @brief Set device mode and reason register values.
 *
 * This is an internal function used to update the device mode and boot mode
 * reason HPI registers.
 *
 * @param dev_mode Value to be set into the device mode register.
 * @param mode_reason Value to be set into the boot mode reason register.
 *
 * @return void
 */
 void hpi_set_mode_regs (uint8_t dev_mode, uint8_t mode_reason);

/**
 * @brief Update the firmware location HPI registers.
 *
 * This is an internal function used to update the firmware binary location
 * HPI registers.
 *
 * @param fw1_location Flash row where FW1 is located.
 * @param fw2_location Flash row where FW2 is located.
 *
 * @return void
 */
void hpi_update_fw_locations (uint16_t fw1_location, uint16_t fw2_location);

#if CCG_HPI_PD_ENABLE
/**
 * @brief Check whether EC control of VDMs is enabled.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return true if EC control is enabled, false otherwise.
 */
bool hpi_is_vdm_ec_ctrl_enabled (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Check whether handling of extended messages by EC is enabled. If not enabled,
 * CCG firmware will automatically respond with NOT_SUPPORTED messages.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return true if EC handling of extended messages is enabled, false otherwise.
 */
bool hpi_is_extd_msg_ec_ctrl_enabled (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Get the active EC alternate modes value.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return The Active EC modes setting programmed by EC.
 */
uint8_t hpi_get_ec_active_modes (cy_stc_pdstack_context_t *ptrPdStackContext);
#endif /* CCG_HPI_PD_ENABLE */

#if (ADVDM_SUPPORT!= 0)
/**
 * @brief Set the ADVDM enterdc status.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 * @param enterdc ADVDM enterdc status.
 *
 * @return void
 */
void hpi_set_advdm_enterdc_status (cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t enterdc);

/**
 * @brief Get the ADVDM enterdc status.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return enterdc status.
 */
uint8_t hpi_get_advdm_enterdc_status (cy_stc_pdstack_context_t *ptrPdStackContext);
#endif

#if (AUVDM_SUPPORT!= 0)
/**
 * @brief Get the status of ap vdm pass through.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return true if ap_vdm_pass is enabled.
 */
bool hpi_is_ap_vdm_pass_through_enabled (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Set the status of ap vdm pass through.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 * @param ap_vdm_pass_through set value.
 *
 * @return void.
 */
void hpi_set_ap_vdm_pass_through (cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t ap_vdm_pass_through);

/**
 * @brief Clear the ap mode status.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return void.
 */
void hpi_clear_ap_mode_status (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Set the ap mode status.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return void.
 */
void hpi_set_ap_mode_status (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Get the ap mode status.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return ap mode status.
 */
uint8_t hpi_get_ap_mode_status (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Set alt_mode_status to Exit mode.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return void.
 */
void hpi_ap_mode_status_exit_mode (cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Set source PDO count.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 * @param count Number of source PDO's to be Set.
 *
 * @return void.
 */
void hpi_set_src_pdo_count (cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t count);

/**
 * @brief Get delay_for_hard_reset.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return value of delay_for_hard_reset.
 */
uint8_t get_delay_before_hard_reset(cy_stc_pdstack_context_t *ptrPdStackContext);

#endif /* (AUVDM_SUPPORT!= 0) */

/**
 * @brief Check if the CCG device can be put into deep-sleep.
 *
 * @return true if deep sleep is possible, false otherwise.
 */
bool hpi_sleep_allowed (void);

/**
 * @brief Prepare the HPI interface for device deep sleep.
 *
 * This function checks whether the I2C interface is idle so that the CCG device can enter
 * deep sleep mode. It also enables an I2C address match as a wake-up trigger from deep sleep.
 * hpi_sleep_allowed should have been called prior to calling this function.
 *
 * @return true if the HPI interface is ready for sleep, false otherwise.
 */
bool hpi_sleep (void);

#if ((!CCG_LOAD_SHARING_ENABLE) || CCG_HPI_AUTO_CMD_ENABLE)
/**
 * @brief Get the Port Enable register value.
 *
 * @return The Port Enable HPI register value.
 */
uint8_t hpi_get_port_enable (void);
#endif /* (!CCG_LOAD_SHARING_ENABLE) */

/**
 * @brief Configure HPI to operate in No-boot support mode.
 * @param enable Whether to enable no-boot mode.
 * @return None
 */
void hpi_set_no_boot_mode (bool enable);

/**
 * @brief Set the I2C slave address to be used for the HPI interface.
 * @param slave_addr Slave address to be used.
 * @return None
 */
void hpi_set_fixed_slave_address (uint8_t slave_addr);

/**
 * @brief Configure HPI to use EC_INT pin for interrupt mode.
 * @param enable Whether to enable or disable interrupt mode.
 * @return None
 */
void hpi_set_ec_interrupt (bool enable);

/**
 * @brief Send a FW ready notification through HPI to the EC. This event is sent
 * to the EC to indicate that the device is out of reset and has loaded firmware.
 *
 * @return None
 */
void hpi_send_fw_ready_event (void);

#if ((!CCG_LOAD_SHARING_ENABLE) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
/**
 * @brief Set the CCG device flash parameters. These values are used for the
 * device status reporting and firmware update implementation.
 *
 * @param flash_size Total device flash size in bytes.
 * @param row_size Size of each flash row in bytes.
 * @param row_cnt Number of flash rows on the device.
 * @param bl_last_row Last flash row assigned to boot-loader.
 * @return None
 */
void hpi_set_flash_params (uint32_t flash_size, uint16_t row_size, uint16_t row_cnt, uint16_t bl_last_row);
#endif /* (!CCG_LOAD_SHARING_ENABLE) || defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */

#if (CCG_BOOT == 0)

/**
 * @brief This function initializes the user-defined HPI registers.
 *
 * This function is used to initialize the contents of the user-defined
 * registers that are part of the HPI register space.
 *
 * @param reg_addr The base address of registers to be updated. Should be in
 * the user defined address region.
 * @param size Number of registers to be updated. The upper limit should not
 * exceed the user defined address region.
 * @param data Buffer containing data to be copied into HPI registers.
 *
 * @return CCG_STAT_SUCCESS if operation is successful, CCG_STAT_BAD_PARAM
 * otherwise.
 */
cy_en_pdstack_status_t hpi_init_userdef_regs (uint16_t reg_addr, uint8_t size,
        uint8_t *data);

/**
 * @brief Enable handling of user-defined register writes in the Application.
 *
 * This function is used to enable handling of EC writes to the user-defined HPI
 * register region in the application code.
 *
 * @param wr_handler Pointer to function that handles the received HPI writes.
 */
void hpi_set_userdef_write_handler (hpi_write_cb_t wr_handler);

#endif /* (CCG_BOOT == 0) */


#if SYS_BLACK_BOX_ENABLE
/**
 * @brief Enable handling of black box through HPI.
 *
 * @param hpi_black_box_handler Pointer to function that handles black box access.
 */
void hpi_set_black_box_handler(hpi_black_box_cb_t hpi_black_box_handler);
#endif /* SYS_BLACK_BOX_ENABLE */


#if CCG_HPI_AUTO_CMD_ENABLE
/**
 * @brief Enables registration of solution space functions.
 *
 * This function is used to register solution space functions to accomplish HPI
 * Auto command related functionalities.
 *
 * @param cb Pointer to function that handles the received HPI writes.
 */
void hpi_auto_set_soln_cb_handler(hpi_auto_soln_cb_t cb);

/**
 * @brief Enable Auto command specific response data to be send.
 *
 * This function is used to update Auto specific response data to device
 * Flash memory section. This function should get triggered in response to
 * Auto commands extracting and returning any information.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 * @param data Placeholder for Auto data.
 * @param size Data size in bytes.
 */
void hpi_auto_copy_data_to_flash(cy_stc_pdstack_context_t *ptrPdStackContext, void *data, uint16_t size);

#endif /* CCG_HPI_AUTO_CMD_ENABLE */

#if HPI_WATCHDOG_RESET_ENABLE

/**
 * @brief Store the reset counter value into the appropriate HPI register.
 *
 * @param count The reset count to be stored.
 * @return None
 */
void hpi_set_reset_count(uint8_t count);
#endif /* HPI_WATCHDOG_RESET_ENABLE */


#if (!CCG_LOAD_SHARING_ENABLE)
#if HPI_WATCHDOG_RESET_ENABLE

/**
 * @brief Debug API to update registers at address 0x35 - 0x34
 * @param value 16-bit value to be updated into HPI registers 0x35 and 0x34.
 * @return None
 */
void hpi_set_reserved_reg_35(uint16_t value);

/**
 * @brief Debug API to update registers at address 0x37 - 0x36
 * @param value 16-bit value to be updated into HPI registers 0x37 and 0x36.
 * @return None
 */
void hpi_set_reserved_reg_37(uint16_t value);

#endif /* HPI_WATCHDOG_RESET_ENABLE */
/**
 * @brief Update the event mask value for the specified PD port.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 * @param mask Event mask value to be set.
 *
 * @return None
 */
void hpi_set_port_event_mask(cy_stc_pdstack_context_t *ptrPdStackContext, uint32_t mask);

#if CCG_HPI_PD_ENABLE
/**
 * @brief Notify EC about a system hardware access error.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 * @param err_type Type of error detected.
 *
 * @return None
 */
void hpi_send_hw_error_event(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t err_type);

/**
 * @brief Enable/disable PDO update through HPI.
 *
 * @param disable Whether PDO update is to be disabled.
 * @return None
 */
void hpi_update_pdo_change(bool disable);
#endif /* CCG_HPI_PD_ENABLE */
/**
 * @brief Update the firmware boot priority configuration reported through HPI.
 * @param conf Firmware boot priority supported by the firmware.
 * @return None
 */
void hpi_set_boot_priority_conf(uint8_t conf);

/**
 * @brief Get the content of the HPI system power state register.
 * @return The 8-bit unsigned content of the HPI syspwr_state register.
 */
uint8_t hpi_get_sys_pwr_state(void);

#else

#if HPI_AUTO_SROM_ENABLE
/**
 * @brief Update the firmware boot priority configuration reported through HPI.
 * @param conf Firmware boot priority supported by the firmware.
 * @return None
 */
void hpi_set_boot_priority_conf(uint8_t conf);
#endif /* HPI_AUTO_SROM_ENABLE */
#endif /* (!CCG_LOAD_SHARING_ENABLE) */

#if (CCG_HPI_BB_ENABLE != 0)

/**
 * @brief Set BB related register data.
 *
 * @param bb_reg_addr BB related register address.
 * @param data Pointer to data which writes to BB related register.
 * @return None.
 */
void hpi_bb_reg_update (uint8_t bb_reg_addr, void *data);

/**
 * @brief Get BB related register data.
 * @param bb_reg_addr BB related register address.
 * @return Data of selected BB related register.
 */
uint32_t hpi_bb_get_reg(uint8_t bb_reg_addr);

/**
 * @brief Get the billboard firmware version.
 * @return Pointer to firmware version buffer.
 */
uint8_t *hpi_bb_get_version(void);

#endif /* (CCG_HPI_BB_ENABLE != 0) */

#if !(CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)

/**
 * @brief Get the HPI soft reset delay value
 * @return Delay value
 */
uint16_t get_hpi_soft_reset_delay(void);

/**
 * @brief Update the HPI soft reset delay value
 * @param value Delay value in ms
 * @return None
 */
void update_hpi_soft_reset_delay(uint16_t value);

/**
 * @brief Get the Timer ID related to HPI soft reset delay
 * @return Delay Timer ID
 */
uint8_t get_hpi_sof_reset_timer_id(void);

/**
 * @brief Update the Timer ID related to HPI soft reset delay
 * @param value Timer ID
 * @return None
 */
void update_hpi_sof_reset_timer_id(uint8_t value);


#endif /* !(CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE) */

#if CCG_UCSI_ENABLE

/**
 * @brief Update the Interrupt Status register and assert the EC_INT pin.
 * @param evt_code Event code to be set.
 * @return None
 */
void hpi_set_event (uint8_t evt_code);

/**
 * @brief Clear the Interrupt Status register and de-assert the EC_INT pin.
 * @param evt_code Event code to be cleared.
 * @return None
 */
void hpi_clear_event (uint8_t evt_code);

/**
 * @brief Clear status & control register.
 * @return None
 */
void ucsi_reg_reset(void);

/**
 * @brief Clear appropriate bit_idx to 0 in the status register.
 * @return None
 */
void ucsi_clear_status_bit(uint8_t bit_idx);

/**
 * @brief Set appropriate bit_idx to 1 in the status register.
 * @return None
 */
void ucsi_set_status_bit(uint8_t bit_idx);

/**
 * @brief Get the UCSI Control register value.
 * @return The UCSI Control HPI register value.
 */
uint8_t hpi_get_ucsi_control(void);

/**
 * @brief Get the UCSI Status bit.
 * @return The UCSI Status bit value.
 */
uint8_t ucsi_get_status_bit(uint8_t bit_idx);

#endif /* CCG_UCSI_ENABLE */

/**
 * @brief This function sets the hpi version info.
 * @param hpi_vers hpi version information.
 * @return void.
 */
void hpi_set_hpi_version(uint32_t hpi_vers);

#if CCG_HPI_BC_12_ENABLE
/**
 * @brief Check whether EC control of BC 1.2 Source is enabled.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return true if EC control is enabled, false otherwise.
 */
bool hpi_is_bc_12_src_enabled(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Check whether EC control of BC 1.2 Sink is enabled.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return true if EC control is enabled, false otherwise.
 */
bool hpi_is_bc_12_snk_enabled(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief Enqueue an BC 1.2 event to EC through the HPI interface.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return true if the event queue has space for the event, false if there
*  is an overflow.
 */
bool hpi_queue_bc_evt(cy_stc_pdstack_context_t *ptrPdStackContext);
#endif /* CCG_HPI_BC_12_ENABLE */
#if (BATTERY_CHARGING_ENABLE && CY_PD_SINK_ONLY)
/**
 * @brief This function updates legacy charging status in hpi register.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 * @param status legacy charging status.
 *
 * @return void.
 */    
void hpi_set_bc_snk_status (cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t status);
#endif

#if (CCG_BOOT || HPI_AUTO_SROM_ENABLE) 
#include "hpi_internal.h"
typedef void (*fp)(uint8_t cmd_opcode, uint8_t *cmd_prm, uint8_t *flash_mem_addr, cy_en_pdstack_status_t *stat, hpi_response_t *code, bool *is_handled);
#if HPI_AUTO_SROM_ENABLE
typedef bool (*fpoverload)(uint8_t  cmd_opcode, uint8_t *cmd_param, uint8_t  cmd_length, hpi_response_t *code, uint8_t *hpi_auto_cmd, uint8_t *hpi_auto_port);
void hpi_register_command_overload(fpoverload ptr);
#endif /* HPI_AUTO_SROM_ENABLE */
void hpi_register_i2c_fsm(fp ptr);
#endif /* (CCG_BOOT || HPI_AUTO_SROM_ENABLE) */
#if (CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)
/**
 * @brief Configure LIN and BillBoard Features for HPI.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 * @param hpi_config run time HPI configurations.
 *
 * @return None.
 */
void hpi_config_run_time_params(cy_stc_pdstack_context_t *ptrPdStackContext, hpi_config_run_time_t hpi_config);
#endif /*(CCG_LOAD_SHARING_ENABLE || CCG_HPI_AUTO_CMD_ENABLE)*/

/* QAC suppression 3332: The macro is defined in the source file where this module is included. */
#if SROM_CODE_HPISS_HPI /* PRQA S 3332 */
/**
 * @brief To initialize global variables for HPI.
 *
 * @param None.
 *
 * @return None.
 */
void hpi_init_globals(void);

/**
 * @brief I2C command callback for the HPI implementation.
 *
 * @param cmd Input Command.
 * @param i2c_state Status of I2C communication.
 * @param count No of bytes to write/read.
 *
 * @return status of operation.
 */
bool hpi_i2c_cmd_callback(i2c_cb_cmd_t cmd, i2c_scb_state_t i2c_state, uint16_t count);
#if CCG_HPI_OVER_LIN_ENABLE
/**
 * @brief LIN command callback for the HPI implementation.
 *
 * @param cmd Input Command.
 * @param i2c_state Status of LIN communication.
 * @param count No of bytes to write/read.
 *
 * @return status of operation.
 */
bool hpi_lin_cmd_callback(i2c_cb_cmd_t cmd, i2c_scb_state_t i2c_state, uint16_t count);
#endif /* CCG_HPI_OVER_LIN_ENABLE */
#endif /* SROM_CODE_HPISS_HPI */

#if CCG_HPI_VBUS_C_CTRL_ENABLE
/**
 * @brief To Update the VBUS Consumer FET control.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 * @param cfet_ctrl Consumer FET control.
 *
 * @return None.
 */
void hpi_update_vbus_cfet_status (cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t cfet_ctrl);
#endif /* CCG_HPI_VBUS_C_CTRL_ENABLE */

#if CCG_HPI_PD_ENABLE
/**
 * @brief To Set Host Capabilities Control register.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 * @param host_cap_ctrl Host Capabilities.
 *
 * @return None.
 */
void hpi_set_host_cap_ctrl_reg(cy_stc_pdstack_context_t *ptrPdStackContext, uint8_t host_cap_ctrl);

/**
 * @brief To Get Host Capabilities Control register.
 *
 * @param ptrPdStackContext PdStack Library Context pointer.
 *
 * @return Host Capabilities.
 */
uint8_t hpi_get_host_cap_ctrl_reg(cy_stc_pdstack_context_t *ptrPdStackContext);

/**
 * @brief To Set AMD Custom register.
 *
 * @param value Value of Register.
 *
 * @return None.
 */
void hpi_set_amd_custom_reg(uint8_t value);

/**
 * @brief To Get AMD Custom register.
 *
 * @return AMD Custom register.
 */
uint8_t hpi_get_amd_custom_reg(void);
#endif /* CCG_HPI_PD_ENABLE */

/** \} group_autoHPI_functions */

#endif /* CCG_HPI_ENABLE */

#endif /* HPI_H_ */

/** \} group_autoHPI */

/* [] END OF FILE */
