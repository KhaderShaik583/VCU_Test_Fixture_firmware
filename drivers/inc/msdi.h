/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file msdi.h
 *
 * MSDI driver supporting boards based on following NXP parts: CD1020, CD1030,
 * MC33978 and MC34978.
 *
 * This module is common for all supported models.
 *
 * Note that from the perspective of this driver, MC33978 and MC34978 devices
 * are similar. Therefore, comments targeting MC33978 device are intended for 
 * both MC33978 and MC34978 devices.
 */

#ifndef MSDI_H_
#define MSDI_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "msdi_cd10x0.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Maximal MSDI device SPI frequence. */
#define MSDI_SPI_FREQ_MAX        8000000U

/*******************************************************************************
 * Enumerations
 ******************************************************************************/

/*!
 * @addtogroup enum_group
 * @{
 */

/*!
 * @brief Error codes.
 */
typedef enum
{
    MSDI_STATUS_SUCCESS          = 0U,    /*!< No error. */
    MSDI_STATUS_SPI_INIT         = 1U,    /*!< SPI initialization failure. */
    MSDI_STATUS_SPI_BUSY         = 2U,    /*!< SPI instance is busy. */
    MSDI_STATUS_SPI_TIMEOUT      = 3U,    /*!< Communication timeout. */
    MSDI_STATUS_COMM_ERROR       = 4U,    /*!< Error in communication. */
    MSDI_STATUS_PARAM_RANGE      = 5U     /*!< Parameter out of range. */
} msdi_status_t;

/*!
 * @brief Register map.
 */
typedef enum
{
    MSDI_SPI_CHECK               = 0x00U, /*!< SPI check. */
    MSDI_DEV_CFG                 = 0x01U, /*!< Device configuration register. */
    MSDI_TRI_STATE_SP            = 0x02U, /*!< Tri-state SP register. */
    MSDI_TRI_STATE_SG            = 0x03U, /*!< Tri-state SG register. */
    MSDI_WET_CUR_SP_0            = 0x04U, /*!< Wetting current level SP 0 register. */
    MSDI_WET_CUR_SG_0            = 0x05U, /*!< Wetting current level SG 0 register. */
    MSDI_WET_CUR_SG_1            = 0x06U, /*!< Wetting current level SG 1 register. */
    MSDI_WET_CUR_SG_2            = 0x07U, /*!< Wetting current level SG 2 register, CD1030 only. */
    MSDI_WET_CUR_SP_1            = 0x08U, /*!< Wetting current level SP 1 register, CD1030 only. */
    MSDI_CONT_WET_SP             = 0x0BU, /*!< Continuous wetting current SP register. */
    MSDI_CONT_WET_SG             = 0x0CU, /*!< Continuous wetting current SG register. */
    MSDI_INT_EN_SP               = 0x0DU, /*!< Interrupt enable SP register. */
    MSDI_INT_EN_SG               = 0x0EU, /*!< Interrupt enable SG register. */
    MSDI_LPM_CFG                 = 0x0FU, /*!< Low-power mode configuration. */
    MSDI_WAKE_EN_SP              = 0x10U, /*!< Wake-up enable register SP. */
    MSDI_WAKE_EN_SG              = 0x11U, /*!< Wake-up enable register SG. */
    MSDI_COMP_ONLY_SP            = 0x12U, /*!< Comparator only SP. */
    MSDI_COMP_ONLY_SG            = 0x13U, /*!< Comparator only SG. */
    MSDI_LPM_THR_SP              = 0x14U, /*!< LPM voltage threshold SP configuration. */
    MSDI_LPM_THR_SG              = 0x15U, /*!< LPM voltage threshold SG configuration. */
    MSDI_POLL_CUR_SP             = 0x16U, /*!< Polling current SP configuration. */
    MSDI_POLL_CUR_SG             = 0x17U, /*!< Polling current SG configuration. */
    MSDI_SLOW_POLL_SP            = 0x18U, /*!< Slow polling SP, CD1030 and MC33978 only. */
    MSDI_SLOW_POLL_SG            = 0x19U, /*!< Slow polling SG, CD1030 and MC33978 only. */
    MSDI_WAKE_DEB_SP             = 0x1AU, /*!< Wake-up debounce SP, CD1030 and MC33978 only. */
    MSDI_WAKE_DEB_SG             = 0x1BU, /*!< Wake-up debounce SG, CD1030 and MC33978 only. */
    MSDI_ENTER_LPM               = 0x1CU, /*!< Enter low-power mode. */
    MSDI_AMUX_CTRL               = 0x1DU, /*!< AMUX control register. */
    MSDI_READ_SW_STAT_SP         = 0x1EU, /*!< Read switch status register SP, CD1030 only. */
    MSDI_READ_SW_STAT            = 0x1FU, /*!< Read switch status (CD1020, MC33978); Read switch status register SG (CD1030). */
    MSDI_FLT_STAT                = 0x21U, /*!< Fault status register. */
    MSDI_INT_REQ                 = 0x23U, /*!< Interrupt request. */
    MSDI_RESET                   = 0x24U  /*!< Reset register. */
} msdi_register_t;

/*!
 * @brief Polling time for SP channels configured as SB.
 */
typedef enum
{
    MSDI_SBPOLLTIME_1000         = 0x00U, /*!< Active polling timer: 1 ms. */
    MSDI_SBPOLLTIME_55           = 0x01U  /*!< active polling timer: 55 us. */
} msdi_sbpolltime_t;

/*!
 * @brief VBATP Overvoltage protection state.
 */
typedef enum
{
    MSDI_VBAT_OV_EN              = 0x00U, /*!< VBATP OV protection enabled. */
    MSDI_VBAT_OV_DIS             = 0x01U  /*!< VBATP OV protection disabled. */
} msdi_vbat_ov_t;

/*!
 * @brief Enable/Disable WAKE_B to wake-up the device on falling edge when V_DDQ
 * is not present.
 */
typedef enum
{
    MSDI_WAKE_B_IGNORED          = 0x00U, /*!< WAKE_B is pulled up to V_DDQ (internally and/or externally). WAKE_B is ignored while in LPM if VDDQ is low. */
    MSDI_WAKE_B_NO_CHECK         = 0x01U  /*!< WAKE_B is externally pulled up to VBATP or VDDQ and wakes upon a falling edge of the WAKE_B pin regardless of the VDDQ status. */
} msdi_wake_vddq_check_t;

/*!
 * @brief Interrupt pin behavior.
 */
typedef enum
{
    MSDI_INT_LOW                 = 0x00U, /*!< INT_B pin stays low when interrupt occurs. */
    MSDI_INT_PULSE               = 0x01U  /*!< INT_B pin pulse low and return high. */
} msdi_int_behavior_t;

/*!
 * @brief AMUX output control method.
 */
typedef enum
{
    MSDI_AMUX_CTRL_SPI_DEF       = 0x00U, /*!< SPI (default). */
    MSDI_AMUX_CTRL_SPI           = 0x01U, /*!< SPI. */
    MSDI_AMUX_CTRL_HW_2B         = 0x02U, /*!< HW 2-bit. */
    MSDI_AMUX_CTRL_HW_3B         = 0x03U  /*!< HW 3-bit. */
} msdi_amux_ctrl_t;

/*!
 * @brief Wetting current level.
 */
typedef enum
{
    MSDI_WET_CUR_2               = 0x00U, /*!< 2 mA. */
    MSDI_WET_CUR_6               = 0x01U, /*!< 6 mA, CD1030 and MC33978 only. */
    MSDI_WET_CUR_8               = 0x02U, /*!< 8 mA. */
    MSDI_WET_CUR_10              = 0x03U, /*!< 10 mA, CD1030 and MC33978 only. */
    MSDI_WET_CUR_12              = 0x04U, /*!< 12 mA. */
    MSDI_WET_CUR_14              = 0x05U, /*!< 14 mA, CD1030 and MC33978 only. */
    MSDI_WET_CUR_16              = 0x06U, /*!< 16 mA. */
    MSDI_WET_CUR_20              = 0x07U  /*!< 20 mA, CD1030 and MC33978 only. */
} msdi_wet_cur_t;

/*!
 * @brief Polling rate for switch detection.
 */
typedef enum
{
    MSDI_POLL_RATE_3             = 0x00U, /*!< 3 ms. */
    MSDI_POLL_RATE_6             = 0x01U, /*!< 6 ms. */
    MSDI_POLL_RATE_12            = 0x02U, /*!< 12 ms. */
    MSDI_POLL_RATE_24            = 0x03U, /*!< 24 ms. */
    MSDI_POLL_RATE_48            = 0x04U, /*!< 48 ms. */
    MSDI_POLL_RATE_68            = 0x05U, /*!< 68 ms. */
    MSDI_POLL_RATE_76            = 0x06U, /*!< 76 ms. */
    MSDI_POLL_RATE_128           = 0x07U, /*!< 128 ms. */
    MSDI_POLL_RATE_32            = 0x08U, /*!< 32 ms. */
    MSDI_POLL_RATE_36            = 0x09U, /*!< 36 ms. */
    MSDI_POLL_RATE_40            = 0x0AU, /*!< 40 ms. */
    MSDI_POLL_RATE_44            = 0x0BU, /*!< 44 ms. */
    MSDI_POLL_RATE_52            = 0x0CU, /*!< 52 ms. */
    MSDI_POLL_RATE_56            = 0x0DU, /*!< 56 ms. */
    MSDI_POLL_RATE_60            = 0x0EU, /*!< 60 ms. */
    MSDI_POLL_RATE_64            = 0x0FU  /*!< 64 ms (default). */
} msdi_poll_rate_t;

/*!
 * @brief Interrupt timer value, CD1030 and MC33978 only.
 */
typedef enum
{
    MSDI_INT_TMR_VAL_OFF         = 0x00U, /*!< OFF (default). */
    MSDI_INT_TMR_VAL_6           = 0x01U, /*!< 6 ms. */
    MSDI_INT_TMR_VAL_12          = 0x02U, /*!< 12 ms. */
    MSDI_INT_TMR_VAL_24          = 0x03U, /*!< 24 ms. */
    MSDI_INT_TMR_VAL_48          = 0x04U, /*!< 48 ms. */
    MSDI_INT_TMR_VAL_96          = 0x05U, /*!< 96 ms. */
    MSDI_INT_TMR_VAL_192         = 0x06U, /*!< 192 ms. */
    MSDI_INT_TMR_VAL_394         = 0x07U, /*!< 394 ms. */
    MSDI_INT_TMR_VAL_4           = 0x08U, /*!< 4 ms. */
    MSDI_INT_TMR_VAL_8           = 0x09U, /*!< 8 ms. */
    MSDI_INT_TMR_VAL_16          = 0x0AU, /*!< 16 ms. */
    MSDI_INT_TMR_VAL_32          = 0x0BU, /*!< 32 ms. */
    MSDI_INT_TMR_VAL_64          = 0x0CU, /*!< 64 ms. */
    MSDI_INT_TMR_VAL_128         = 0x0DU, /*!< 128 ms. */
    MSDI_INT_TMR_VAL_256         = 0x0EU, /*!< 256 ms. */
    MSDI_INT_TMR_VAL_512         = 0x0FU  /*!< 512 ms. */
} msdi_int_tmr_val_t;

/*!
 * @brief Interrupt timer value, CD1030 only.
 */
typedef enum
{
    MSDI_AMUX_ASETT0_HIZ         = 0x00U, /*!< Hi Z (default). */
    MSDI_AMUX_ASETT0_IWET        = 0x01U  /*!< I_WET. */
} msdi_amux_current_t;

/*!
 * @brief  MSDI device type.
 */
typedef enum
{
    MSDI_DEVICE_CD1020           = 0U,    /*!< CD1020. */
    MSDI_DEVICE_CD1030           = 1U,    /*!< CD1030. */
    MSDI_DEVICE_MC33978          = 2U     /*!< MC33978, MC34978. */
} msdi_dev_type_t;

/*! @} */

/*******************************************************************************
 * Struct types definition
 ******************************************************************************/

/*!
 * @addtogroup struct_group
 * @{
 */

/*!
 * @brief Device configuration.
 */
typedef struct
{
    msdi_sbpolltime_t sbpolltime;         /*!< Polling time for SP channels configured as SB. */
    msdi_vbat_ov_t vbatOvDis;             /*!< VBATP OV protection state. */
    msdi_wake_vddq_check_t wakeVddqCheck; /*!< WAKE_B to wake-up the device when VDDQ is not present. */
    msdi_int_behavior_t intBehavior;      /*!< Interrupt pin behavior. */
    msdi_amux_ctrl_t amuxCtrl;            /*!< MUX output control method. If CD1020 is used this field will be ignored. */
    uint16_t spx;                         /*!< SP pins configuration. Use MSDI_CFG_SPX_BAT(n) macro to Switch to Battery and MSDI_CFG_SPX_GND(n) to Switch to ground for each SP pin. */
} msdi_dev_cfg_t;

/*!
 * @brief Low Power Mode configuration.
 */
typedef struct
{
    msdi_poll_rate_t pollRate;            /*!< Polling rate for switch detection. */
    msdi_int_tmr_val_t intTmrVal;         /*!< Interrupt timer value. If CD1020 is used this field will be ignored. */
} msdi_lpm_cfg_t;

/*!
 * @brief AMUX configuration.
 */
typedef struct
{
    msdi_amux_current_t current;          /*!< AMUX current select. */
    uint8_t channel;                      /*!< AMUX channel select. Use MSDI_AMUX_ASEL_CD1020_*, MSDI_AMUX_ASEL_CD1030_* or MSDI_AMUX_ASEL_MC33978_* macros from cd10x0.h. */
} msdi_amux_cfg_t;

/*!
 * @brief Faults provided by Fault status register.
 */
typedef struct
{
    bool spiError;                        /*!< Any SPI error (Wrong address, incorrect modulo). */
    bool hashFault;                       /*!< SPI register and hash mismatch. */
    bool uv;                              /*!< V_BATP voltage was in UV range. */
    bool ov;                              /*!< V_BATP was higher than OV threshold. */
    bool tempFlag;                        /*!< Temperature warning to note elevated IC temperature. */
    bool ot;                              /*!< T_LIM event occurred on the IC. */
    bool intBWake;                        /*!< Part awaken via INT_B falling edge. */
    bool wakeBWake;                       /*!< Part awaken via WAKE_B falling edge. */
    bool spiWake;                         /*!< Part awaken via a SPI message. */
    bool por;                             /*!< POR event occurred. */
} msdi_faults_t;

/*!
 * @brief Initialization values of configurable registers.
 * For more information about specified registers, see MSDI device datasheet or
 * related MSDI_Set* functions of this driver.
 */
typedef struct
{
    msdi_dev_cfg_t devConfig;             /*!< Device configuration. */
    uint32_t triStateSp;                  /*!< Tri-state register SP. */
    uint32_t triStateSg;                  /*!< Tri-state register SG. */
    msdi_wet_cur_t wetCurLevelSp[MSDI_SP_CNT_MAX]; /*!< Wetting current levels SP. */
    msdi_wet_cur_t wetCurLevelSg[MSDI_SG_CNT_MAX]; /*!< Wetting current levels SG. */
    uint32_t contWetCurEnSp;              /*!< Continuous wetting current SP. */
    uint32_t contWetCurEnSg;              /*!< Continuous wetting current SG. */
    uint32_t intEnSp;                     /*!< Interrupt enable SP. */
    uint32_t intEnSg;                     /*!< Interrupt enable SG. */
    msdi_lpm_cfg_t lpmConfig;             /*!< Low Power Mode configuration. */
    uint32_t wakeUpEnSp;                  /*!< Wake-up enable register SP. */
    uint32_t wakeUpEnSg;                  /*!< Wake-up enable register SG. */
    uint32_t lpmCompOnlySp;               /*!< Comparator only SP. */
    uint32_t lpmCompOnlySg;               /*!< Comparator only SG. */
    uint32_t lpmVoltThresholdSp;          /*!< LPM voltage threshold SP configuration. */
    uint32_t lpmVoltThresholdSg;          /*!< LPM voltage threshold SG configuration. */
    uint32_t lpmPollCurConfigSp;          /*!< Polling current SP configuration. */
    uint32_t lpmPollCurConfigSg;          /*!< Polling current SG configuration. */
    uint32_t slowPollingSp;               /*!< Slow polling SP, CD1030 and MC33978 only. */
    uint32_t slowPollingSg;               /*!< Slow polling SG, CD1030 and MC33978 only. */
    uint32_t wakeUpDebounceSp;            /*!< Wake-up debounce SP, CD1030 and MC33978 only. */
    uint32_t wakeUpDebounceSg;            /*!< Wake-up debounce SG, CD1030 and MC33978 only. */
} msdi_init_config_t;

/*!
 * @brief Driver configuration.
 *
 * This structure contains all information needed for proper functionality of
 * the driver - used MSDI device type and driver instance (passed to external
 * functions). Moreover, it contains FAULT STATUS bit and INTflg bit from the
 * last SPI transfer, which contained these two flags.
 */
typedef struct
{
    uint8_t drvInstance;                  /*!< MSDI driver instance. Passed to the external
                                               functions defined by the user. */
    msdi_dev_type_t deviceType;           /*!< Type of MSDI device. */

    bool faultStatus;                     /*!< Fault status parsed from the last SPI transfer. */
    bool intFlg;                          /*!< INTflg flag aggregated from last MSDI_GetIntFlag
                                               function call. */
} msdi_drv_config_t;

/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @addtogroup function_group
 * @{
 */

/*!
 * @brief This function fills msdi_init_config_t structure with default MSDI
 * POR device register configuration.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param initConfig Pointer to device configuration that will be filled by the
 *                   device POR configuration.
 */
void MSDI_GetDefaultInitConfig(const msdi_drv_config_t* const drvConfig,
        msdi_init_config_t* const initConfig);

/*!
 * @brief This function checks SPI communication with the MSDI device and
 * initializes all SPI configurable registers (except of AMUX current channel).
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param initConfig Pointer to structure which contains configuration of all
 *                   registers. Use NULL if no register should be updated.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_Init(msdi_drv_config_t* const drvConfig,
        const msdi_init_config_t* const initConfig);

/*!
 * @brief This function checks the SPI communication using Check SPI
 * command.
 *
 * @param drvConfig Pointer to driver instance configuration.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_CheckSpi(msdi_drv_config_t* const drvConfig);

/*!
 * @brief This function sets the Device Configuration register.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cfg Device configuration to be written into the device.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetDeviceConfig(msdi_drv_config_t* const drvConfig,
        const msdi_dev_cfg_t* const cfg);

/*!
 * @brief This function reads the Device Configuration register.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cfg Pointer where read device configuration will be stored.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetDeviceConfig(msdi_drv_config_t* const drvConfig,
        msdi_dev_cfg_t* const cfg);

/*!
 * @brief This function sets the Tri-state registers. By setting a specified
 * register bits to logic 1, corresponding inputs are high-impedance regardless
 * of the Wetting current setting. In order to create values of SP and SG
 * registers, MSDI_TRI_STATE_EN and MSDI_TRI_STATE_DIS macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Tri-state SP register value.
 * @param sg Tri-state SG register value.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetTriStateEnable(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg);

/*!
 * @brief This function reads the Tri-state registers. In order to parse both SP
 * and SG registers, MSDI_TRI_STATE_MASK, MSDI_TRI_STATE_EN and
 * MSDI_TRI_STATE_DIS macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where Tri-state SP register value will be stored.
 *           Use NULL if this register should not be read.
 * @param sg Pointer to memory where Tri-state SG register value will be stored.
 *           Use NULL if this register should not be read.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetTriStateEnable(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg);

/*!
 * @brief This function sets the Wetting Current Level registers.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Wetting current levels of SP pins. Use NULL if Wetting Current
 *           Level SP registers should not be written.
 * @param sg Wetting current levels of SG pins. Use NULL if Wetting Current
 *           Level SG registers should not be written.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetWetCurLevel(msdi_drv_config_t* const drvConfig,
        const msdi_wet_cur_t* const sp, const msdi_wet_cur_t* const sg);

/*!
 * @brief This function reads the Wetting Current Level registers.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where Wetting current levels of SP pins will be
 *           stored. Use NULL if Wetting Current Level SP registers should not
 *           be read.
 * @param sg Pointer to memory where Wetting current levels of SG pins will be
 *           stored. Use NULL if Wetting Current Level SG registers should not
 *           be read.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetWetCurLevel(msdi_drv_config_t* const drvConfig,
        msdi_wet_cur_t* const sp, msdi_wet_cur_t* const sg);

/*!
 * @brief This function sets the Continuous wetting current registers.
 * Programming specified bits to logic 1 enables the continuous wetting current
 * for corresponding pins and result in a full time wetting current level. In
 * order to create values of SP and SG registers, MSDI_WET_CUR_EN and
 * MSDI_WET_CUR_DIS macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Continuous wetting current SP register value.
 * @param sg Continuous wetting current SG register value.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetContWetCurEnable(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg);

/*!
 * @brief This function reads the Continuous wetting current registers. In order
 * to parse both SP and SG registers, MSDI_WET_CUR_MASK, MSDI_WET_CUR_EN
 * and MSDI_WET_CUR_DIS macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where Continuous wetting current SP register
 *           value will be stored. Use NULL if this register should not be read.
 * @param sg Pointer to memory where Continuous wetting current SG register
 *           value will be stored. Use NULL if this register should not be read.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetContWetCurEnable(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg);

/*!
 * @brief This function sets the Interrupt enable registers. Programming
 * specified bits to logic 0 disables corresponding inputs from generating an
 * interrupt. In order to create values of SP and SG registers, MSDI_INT_EN and
 * MSDI_INT_DIS macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Interrupt enable SP register value.
 * @param sg Interrupt enable SG register value.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetIntEnable(msdi_drv_config_t* const drvConfig, uint32_t sp,
        uint32_t sg);

/*!
 * @brief This function reads the Interrupt enable registers. In order to parse
 * both SP and SG registers, MSDI_INT_EN_MASK, MSDI_INT_EN and MSDI_INT_DIS
 * macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where Interrupt enable SP register value will be
 *           stored. Use NULL if this register should not be read.
 * @param sg Pointer to memory where Interrupt enable SG register value will be
 *           stored. Use NULL if this register should not be read.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetIntEnable(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg);

 /*!
  * @brief This function sets the Low-power mode configuration register.
  *
  * @param drvConfig Pointer to driver instance configuration.
  * @param lpmConfig Low-power mode configuration.
  *
  * @return status_t Error code.
  */
msdi_status_t MSDI_SetLPMConfig(msdi_drv_config_t* const drvConfig,
        const msdi_lpm_cfg_t* const lpmConfig);

/*!
 * @brief This function reads the Low-power mode configuration register.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param lpmConfig Pointer to memory where read Low-power mode configuration
 *                  will be stored.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetLPMConfig(msdi_drv_config_t* const drvConfig,
        msdi_lpm_cfg_t* const lpmConfig);

/*!
 * @brief This function sets the Wake-up enable registers. Programming
 * specified bits to logic 0 disables corresponding inputs from waking the IC.
 * In order to create values of SP and SG registers, MSDI_WAKE_EN and
 * MSDI_WAKE_DIS macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Wake-up enable SP register value.
 * @param sg Wake-up enable SG register value.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetWakeUpEnable(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg);

/*!
 * @brief This function reads the Wake-up enable registers. In order to parse
 * both SP and SG registers, MSDI_WAKE_EN_MASK, MSDI_WAKE_EN and
 * MSDI_WAKE_DIS macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where Wake-up enable SP register value will be
 *           stored. Use NULL if this register should not be read.
 * @param sg Pointer to memory where Wake-up enable SG register value will be
 *           stored. Use NULL if this register should not be read.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetWakeUpEnable(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg);

/*!
 * @brief This function sets the Comparator-only registers. Programming
 * specified bits to logic 1 allows corresponding input comparators to be active
 * during LPM with no polling current. In this case, the inputs can receive
 * a digital signal on the order of the LPM clock cycle and wake-up on a change
 * of state. In order to create values of SP and SG registers,
 * MSDI_COMP_ONLY_WO_POLLING and MSDI_COMP_ONLY_W_POLLING macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Comparator-only enable SP register value.
 * @param sg Comparator-only enable SG register value.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetComparatorOnly(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg);

/*!
 * @brief This function reads the Comparator-only registers. In order to parse
 * both SP and SG registers, MSDI_COMP_ONLY_MASK, MSDI_COMP_ONLY_WO_POLLING
 * and MSDI_COMP_ONLY_W_POLLING macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where Comparator-only SP register value will be
 *           stored. Use NULL if this register should not be read.
 * @param sg Pointer to memory where Comparator-only SG register value will be
 *           stored. Use NULL if this register should not be read.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetComparatorOnly(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg);

/*!
 * @brief This function sets the LPM voltage threshold (to wake-up from LPM)
 * configuration registers.
 *
 * For SG pins, a logic 0 means that corresponding input uses the LPM delta
 * voltage threshold to determine the state of the switch. A logic 1 means the
 * corresponding input uses the Normal threshold (VICTHR) to determine the state
 * of the switch.
 * When configured as an SB, it only uses the 4.0 V threshold regardless the
 * status of the LPM voltage threshold bit.
 *
 * In order to create values of SP and SG registers, MSDI_LPM_THR_NORMAL and
 * MSDI_LPM_THR_DELTA macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Comparator-only enable SP register value.
 * @param sg Comparator-only enable SG register value.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetLPMVoltThreshold(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg);

/*!
 * @brief This function reads the LPM voltage threshold configuration registers.
 * In order to parse both SP and SG registers, MSDI_LPM_THR_MASK,
 * MSDI_LPM_THR_NORMAL and MSDI_LPM_THR_DELTA macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where value of LPM voltage threshold SP
 *           configuration register will be stored. Use NULL if this register
 *           should not be read.
 * @param sg Pointer to memory where value of LPM voltage threshold SG
 *           configuration register will be stored. Use NULL if this register
 *           should not be read.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetLPMVoltThreshold(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg);

/*!
 * @brief This function sets the Polling current configuration registers.
 *
 * The normal polling current for LPM is 2.2 mA for SB channels and 1.0 mA for
 * SG channels. A logic 0 selects the normal polling current for each individual
 * channel. The IWET current value (defined in the Wetting current level
 * registers) is selected by writing a logic 1 for specified channels.
 *
 * In order to create values of SP and SG registers, MSDI_POLL_CUR_IWET and
 * MSDI_POLL_CUR_NORMAL macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Polling current SP configuration register value.
 * @param sg Polling current SG configuration register value.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetLPMPollCurrent(msdi_drv_config_t* const drvConfig,
        uint32_t sp, uint32_t sg);

/*!
 * @brief This function reads the Polling current configuration registers.
 * In order to parse both SP and SG registers, MSDI_POLL_CUR_MASK,
 * MSDI_POLL_CUR_NORMAL and MSDI_POLL_CUR_IWET macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where Polling current SP configuration register
 *           value will be stored. Use NULL if this register should not be read.
 * @param sg Pointer to memory where Polling current SG configuration register
 *           value will be stored. Use NULL if this register should not be read.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetLPMPollCurrent(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg);

/*!
 * @brief This function sets the Slow polling registers. Programming specified
 * bits to logic 1 enables 4x slower polling rate for corresponding pins.
 * In order to create values of SP and SG registers, MSDI_SLOW_POLL_NORMAL and
 * MSDI_SLOW_POLL_4X_SLOWER macros can be used.
 *
 * This function cannot be called in case CD1020 is used as it has no Slow
 * polling registers.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Slow polling SP register value.
 * @param sg Slow polling SG register value.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetSlowPolling(msdi_drv_config_t* const drvConfig,
        const uint32_t sp, const uint32_t sg);

/*!
 * @brief This function reads the Slow polling registers. In order to parse
 * both SP and SG registers, MSDI_SLOW_POLL_MASK, MSDI_SLOW_POLL_4X_SLOWER
 * and MSDI_SLOW_POLL_NORMAL macros can be used.
 *
 * This function cannot be called in case CD1020 is used as it has no Slow
 * polling registers.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where Slow polling SP register value will be
 *           stored. Use NULL if this register should not be read.
 * @param sg Pointer to memory where Slow polling SG register value will be
 *           stored. Use NULL if this register should not be read.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetSlowPolling(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg);

/*!
 * @brief This function sets the Wake-up debounce registers.
 *
 * The IC is able to extend the time the active polling takes place to ensure
 * a true change of state has occurred in LPM, and reduce the chance noise has
 * impacted the measurement. If this bit is 0, the IC uses a voltage difference
 * technique to determine if a switch has changed state. If this bit is set 1,
 * the IC debounces the measurement by continuing to source the LPM polling
 * current for an additional 1.2 ms and take the measurement based on the final
 * voltage level. This helps to ensure the switch is detected correctly in noisy
 * systems.
 *
 * In order to create values of SP and SG registers, MSDI_WAKE_DEB_NORMAL and
 * MSDI_WAKE_DEB_EXTENDED macros can be used.
 *
 * This function cannot be called in case CD1020 is used as it has no Wake-up
 * debounce registers.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Slow polling SP register value.
 * @param sg Slow polling SG register value.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetWakeUpDebounce(msdi_drv_config_t* const drvConfig,
        const uint32_t sp, const uint32_t sg);

/*!
 * @brief This function reads the Wake-up debounce registers. In order to parse
 * both SP and SG registers, MSDI_WAKE_DEB_MASK, MSDI_WAKE_DEB_NORMAL and
 * MSDI_WAKE_DEB_EXTENDED macros can be used.
 *
 * This function cannot be called in case CD1020 is used as it has no Wake-up
 * debounce registers.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where Wake-up debounce SP register value will be
 *           stored. Use NULL if this register should not be read.
 * @param sg Pointer to memory where Wake-up debounce SG register value will be
 *           stored. Use NULL if this register should not be read.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetWakeUpDebounce(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg);

/*!
 * @brief This function switches MSDI device into Low-power mode.
 *
 * @param drvConfig Pointer to driver instance configuration.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_EnterLPM(msdi_drv_config_t* const drvConfig);

/*!
 * @brief This function interrupts MSDI Low-power mode by CSB pin and waits
 * t_CSB_WAKEUP time. If device is already in the normal mode, no register is
 * affected.
 *
 * @param drvConfig Pointer to driver instance configuration.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_ExitLPM(msdi_drv_config_t* const drvConfig);

/*!
 * @brief This function sets the AMUX control register.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param amuxCfg AMUX configuration to be written.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_SetAMUXChannel(msdi_drv_config_t* const drvConfig,
        const msdi_amux_cfg_t* const amuxCfg);

/*!
 * @brief This function reads the AMUX control register.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param amuxCfg Pointer where read AMUX configuration will be stored.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetAMUXChannel(msdi_drv_config_t* const drvConfig,
        msdi_amux_cfg_t* const amuxCfg);

/*!
 * @brief This function reads status of the switches. Logic 1 at selected bits
 * determines closed switches. Logic 0 says that related switches are opened.
 * In order to parse results, READ_SW_STAT_MASK, READ_SW_STAT_CLOSED(n) and
 * READ_SW_STAT_OPEN macros can be used.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param sp Pointer to memory where state of SP switches will be stored.
 * @param sg Pointer to memory where state of SP switches will be stored.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_ReadSwitchStatus(msdi_drv_config_t* const drvConfig,
        uint32_t* const sp, uint32_t* const sg);

/*!
 * @brief This function reads the Fault status register.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param faultStatus Fault status flags.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_GetFaultStatus(msdi_drv_config_t* const drvConfig,
        msdi_faults_t* const faultStatus);

/*!
 * @brief This function requests MSDI device for an interrupt pulse.
 *
 * @param drvConfig Pointer to driver instance configuration.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_IntPulseRequest(msdi_drv_config_t* const drvConfig);

/*!
 * @brief This function causes all of the SPI registers to reset.
 *
 * Note that after calling this function, MCU should wait until the MSDI device
 * returns into Normal mode (at least 50 us).
 *
 * @param drvConfig Pointer to driver instance configuration.
 *
 * @return status_t Error code.
 */
msdi_status_t MSDI_Reset(msdi_drv_config_t* const drvConfig);

/*!
 * @brief This function returns Fault status flag from the last SPI transfer,
 * which contained this flag.
 *
 * @param drvConfig Pointer to driver instance configuration.
 *
 * @return status_t Fault status flag.
 */
bool MSDI_GetFaultFlag(const msdi_drv_config_t* const drvConfig);

/*!
 * @brief This function returns true if INTflg flag was set in any SPI transfer
 * from last calling of this function.
 *
 * @param drvConfig Pointer to driver instance configuration.
 *
 * @return status_t INTflg flag.
 */
bool MSDI_GetIntFlag(msdi_drv_config_t* const drvConfig);

/*******************************************************************************
 * Platform specific functions
 ******************************************************************************/

/*!
 * @brief User implementation of assert.
 *
 * @param x - True if everything is OK.
 */
/*extern inline void MSDI_Assert(bool x);*/

/*!
 * @brief This function performs one 32b LPSPI transfer to MSDI device. This
 * function needs to be implemented for specified MCU by the user.
 *
 * @param drvInstance Instance of MSDI driver.
 * @param sendData Data to be sent to MSDI device.
 * @param rcvData Pointer to memory where data received on MISO is stored. Use
 *                NULL if no data should be stored.
 */
extern msdi_status_t MSDI_TransferSpi(uint8_t drvInstance,
		const uint32_t sendData, uint32_t* const rcvData);

/*!
 * @brief Waits for specified amount of milliseconds. This function needs to be
 * implemented for specified MCU by the user.
 *
 * @param delay - Number of milliseconds to wait.
 */
extern void MSDI_WaitMs(uint16_t delay);


/*! @} */

#endif /* MSDI_H_ */
