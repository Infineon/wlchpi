/***************************************************************************//**
* \file hpi.h
* \version 2.0
*
* Provides general utility macros and definitions for the QiStack Middleware.
*
********************************************************************************
* \copyright
* Copyright 2022-2023, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/**
* \addtogroup group_wlcHPI WLC HPI Library
* \{
* \defgroup group_wlcHPI_macros Macros
* \defgroup group_wlcHPI_functions Functions
* \defgroup group_wlcHPI_data_structures Data Structures
* \defgroup group_wlcHPI_enums Enumerated Types
* */

#ifndef HPI_H_
#define HPI_H_

#include <stdbool.h>
#include <stdint.h>
#include "cy_pdstack_common.h"
#include "cy_qistack_common.h"
#include "i2c.h"
#include "cy_usbpd_config_table.h"
#include "config.h"
#if CCG_HPI_ENABLE
#include "battery_charging.h"

/**************************************************************************************************
 ******************************************** MACROS **********************************************
 *************************************************************************************************/
 
/** \addtogroup group_wlcHPI_enums
* \{
* This section describes the WLC HPI Enumerated Types.
* Detailed information about the enums is available in each enum description.
*/
/**
 * @typedef hpi_debug_monitor_cmds_t
 * @brief HPI debug and monitor commands.
 *  Commands to read and update WLC HPI registers.
 */
typedef enum {
    /**< Monitor commands */
    HPI_RD_CHIP = 0x00,            /**< Read Chip info */
    HPI_RD_VIN_TX_AUTH,            /**< Read Vin, Tx Capability and Auth information */
    HPI_RD_STATE,                  /**< Read state info */
    HPI_RD_VIN_EXT,                /**< Read VIN PD contract voltage and current details */
    HPI_RD_NEG_PWR,                /**< Read negotiated power profile */
    HPI_RD_OPER,                   /**< Read operating details */
    HPI_RD_PACKET,                 /**< Read packet debug info */

    /**< Debug commands */
    HPI_RD_ASK_DEMOD_GAIN = 0x10,  /**< Read ASK Demodulator Gain parameters */
    HPI_WR_ASK_DEMOD_GAIN,         /**< Update ASK Demodulator Gain parameters */
    HPI_RD_Q_FO_CFG,               /**< Read Q-Factor FO details */
    HPI_WR_Q_FO_CFG,               /**< Update Q-Factor FO details */
    HPI_RD_MAX_POWER_CAP,          /**< Read Max Power Cap parameters */
    HPI_WR_MAX_POWER_CAP,          /**< Update Max Power Cap parameters */
    HPI_RD_PLOSS_FO_CFG,           /**< Read PLOSS FO configuration */
    HPI_WR_PLOSS_FO_CFG            /**< Update PLOSS FO configuration */
} hpi_debug_monitor_cmds_t;


/** \} group_wlcHPI_enums */

/** \addtogroup group_wlcHPI_macros
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

#define HPI_I2C_DEFAULT_ADDR                        (0x57)
/**< Default value when no CFG_STRAP for I2C address */

#if (CCG_HPI_WLC_CMD_ENABLE != 0)
#define HPI_I2C_WLC_INTR_BIT                        (0x4)
/**< WLC Command INTR bit position */
#endif /* CCG_HPI_WLC_CMD_ENABLE */

/* VBUS CONSUMER FET CONTROL REGISTER bit definitions.*/
#define VBUS_CFET_CTRL_EC_CTRL_EN_POS           (0u)
/**< VBUS consumer FET control register EC control enable bit position.*/
#define VBUS_CFET_CTRL_EC_CTRL_EN               (1u << VBUS_CFET_CTRL_EC_CTRL_EN_POS)
/**< VBUS consumer FET control register EC control enable.*/
#define VBUS_CFET_CTRL_EC_CFET_ON_POS           (1u)
/**< VBUS consumer FET control register EC CFET on bit position.*/
#define VBUS_CFET_CTRL_EC_CFET_ON               (1u << VBUS_CFET_CTRL_EC_CFET_ON_POS)
/**< VBUS consumer FET control register EC CFET on.*/

/** \} group_wlcHPI_macros */

/**************************************************************************************************
 ****************************************** DATA TYPES ********************************************
 *************************************************************************************************/
 
/** \addtogroup group_wlcHPI_enums
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
#if (CCG_HPI_WLC_CMD_ENABLE != 0)
    HPI_REG_SECTION_WLC = 0x08,         /**< HPI WLC related registers */
#endif /* CCG_HPI_WLC_CMD_ENABLE */ 
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
#if (CCG_HPI_WLC_CMD_ENABLE != 0)
    HPI_REG_PART_DM_WR = 0,             /**< Write Data memory for WLC command register section. */
    HPI_REG_PART_DM_RD = 4,             /**< Read Data memory for WLC command register section. */
#endif /* CCG_HPI_WLC_CMD_ENABLE */
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

/** \} group_wlcHPI_enums */


/** \addtogroup group_wlcHPI_data_structures
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

#if SOL_CMD_HANDLER
#if (CCG_HPI_WLC_CMD_ENABLE != 0)
typedef uint8_t (*hpi_wlc_soln_cmd_handler_ptr) (cy_stc_pdstack_context_t *ptrPdStackContext, uint16_t cmd, uint8_t *value, uint8_t cmd_len);
typedef struct
{
    bool hpi_bb_operation_enable;
    bool hpi_lin_operation_enable;
    bool hpi_lin_multi_master_mode;
    bool hpi_dual_firmware_enable;
    hpi_wlc_soln_cmd_handler_ptr hpi_wlc_soln_cmd_handler;
}hpi_config_run_time_t;
#endif /*(CCG_HPI_WLC_CMD_ENABLE)*/
#endif /* Leaving for reference */
#if (CCG_HPI_WLC_CMD_ENABLE != 0)
/**
 * @typedef hpi_wlc_soln_cb_t
 * @brief Structure to hold the Solution interface for HPI Auto operation.
 * Solution is expected to fill the structure with required pointers to function
 * to accomplish the command specific tasks. All the registered functions
 * should be non-blocking and take minimum execution time.
 */
typedef struct
{
    uint16_t (*soln_get_vbus_voltage) (
        cy_stc_usbpd_context_t *context,                            /**< USBPD port Context. */
        cy_en_usbpd_adc_id_t adcId,                                 /**< ADC ID. */
        cy_en_usbpd_adc_input_t input                               /**< ADC Input. */
        );

    uint16_t (*soln_get_vbus_current)  (
        cy_stc_usbpd_context_t *context                             /**< USBPD port Context. */
        );
} hpi_wlc_soln_cb_t;
#endif /* CCG_HPI_WLC_CMD_ENABLE */

/** \} group_wlcHPI_data_structures */
/** \addtogroup group_wlcHPI_enums
* \{
* This section describes the WLC HPI Enumerated Types.
* Detailed information about the enums is available in each enum description.
*/

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

/** \} group_wlcHPI_enums */

/**************************************************************************************************
 ************************************ FUNCTION DEFINITIONS ****************************************
 *************************************************************************************************/
 
/**
* \addtogroup group_wlcHPI_functions
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
 * @param QiStackContexts QiStack Library Context pointer
 * @param scb_idx Index of SCB block to be used for HPI. Please note that this parameter is
 * not validated, and it is the caller's responsibility to pass the correct value.
 * @return None
 */
void hpi_init(cy_stc_pdstack_context_t* PdStackContexts, cy_stc_qi_context_t *QiStackContexts, uint8_t scb_idx);

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

/**
 * @brief Check whether any HPI accesses have happened since start-up.
 * @return True if any HPI read/write access has happened.
 */
bool hpi_is_accessed (void);


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

#if (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S))
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
#endif /* (defined(CY_DEVICE_CCG7D) || defined(CY_DEVICE_CCG7S) */

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

#if (CCG_HPI_WLC_CMD_ENABLE != 0)
/**
 * @brief Enables registration of solution space functions.
 *
 * This function is used to register solution space functions to accomplish HPI
 * WLC command related functionalities.
 *
 * @param cb Pointer to function that handles the received HPI writes.
 */
void hpi_wlc_set_soln_cb_handler(hpi_wlc_soln_cb_t cb);
/**
 * @brief Enable WLC command specific response data to be send.
 *
 * This function is used to update WLC specific response data to device
 * Flash memory section. This function should get triggered in response to
 * WLC commands extracting and returning any information.
 *
 * @param data Placeholder for WLC data.
 * @param size Data size in bytes.
 */
void hpi_wlc_copy_data_to_flash(void *data, uint16_t size);

#endif /* CCG_HPI_WLC_CMD_ENABLE */

#if HPI_WATCHDOG_RESET_ENABLE

/**
 * @brief Store the reset counter value into the appropriate HPI register.
 *
 * @param count The reset count to be stored.
 * @return None
 */
void hpi_set_reset_count(uint8_t count);
#endif /* HPI_WATCHDOG_RESET_ENABLE */

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

/**
 * @brief Update the firmware boot priority configuration reported through HPI.
 * @param conf Firmware boot priority supported by the firmware.
 * @return None
 */
void hpi_set_boot_priority_conf(uint8_t conf);

#if ((!CCG_LOAD_SHARING_ENABLE) || CCG_HPI_AUTO_CMD_ENABLE)
/**
 * @brief Get the Port Enable register value.
 *
 * @return The Port Enable HPI register value.
 */
uint8_t hpi_get_port_enable (void);
#endif /* (!CCG_LOAD_SHARING_ENABLE) */

/**
 * @brief Get the content of the HPI system power state register.
 * @return The 8-bit unsigned content of the HPI syspwr_state register.
 */
uint8_t hpi_get_sys_pwr_state(void);

#if ((!CCG_BOOT) && !(CCG_HPI_WLC_CMD_ENABLE))

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

#endif /* ((!CCG_BOOT) && !(CCG_HPI_WLC_CMD_ENABLE)) */


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

#if (CCG_BOOT)
#include "hpi_internal.h"
typedef void (*fp)(uint8_t cmd_opcode, uint8_t *cmd_prm, uint8_t *flash_mem_addr, cy_en_pdstack_status_t *stat, hpi_response_t *code, bool *is_handled);
void hpi_register_i2c_fsm(fp ptr);
#endif /* (CCG_BOOT) */

/** \} group_wlcHPI_functions */

#endif /* CCG_HPI_ENABLE */
#endif /* HPI_H_ */

/** \} group_wlcHPI */

/* [] END OF FILE */
