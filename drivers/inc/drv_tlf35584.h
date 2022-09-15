 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2017] - [2018] Ultraviolette Automotive Pvt. Ltd.
 * All Rights Reserved.
 * 
 * NOTICE: All information contained herein is, and remains the 
 * property of Ultraviolette Automotive Pvt. Ltd. and its suppliers, if 
 * any. The intellectual and technical concepts contained herein are 
 * proprietary to Ultraviolette Automotive  and its suppliers and may 
 * be covered by U.S. and Foreign Patents, patents in process,  and 
 * are protected by trade secret or copyright law. Dissemination of 
 * this information or reproduction of this material is strictly 
 * forbidden unless prior written permission is obtained from 
 * Ultraviolette Automotive Pvt. Ltd.
 * 
 *
 * Author : Rishi F. [011]
 *
 */

#ifndef DRV_TLF35584_H
#define DRV_TLF35584_H


#include "board.h" 
#include "fw_common.h"
#include "pins_driver.h"
#include "drv_spi_legacy.h"

#define MCU_SBC_WAK_GPIO                 AUX_5V_EN_GPIO 
#define MCU_SBC_WAK_PIN                  AUX_5V_EN_PIN

#define MCU_SBC_WDI_GPIO                 PTA
#define MCU_SBC_WDI_PIN                  200U 

#define SBC_MCU_SS1_GPIO                 TLF_SS1_GPIO 
#define SBC_MCU_SS1_PIN                  TLF_SS1_PIN 

#define SBC_MCU_SS2_GPIO                 TLF_SS2_GPIO 
#define SBC_MCU_SS2_PIN                  TLF_SS2_PIN 

#define MCU_SBC_ERR_GPIO                 TLF_ERR_GPIO
#define MCU_SBC_ERR_PIN                  TLF_ERR_PIN

#define MCU_SBC_CS_GPIO                  TLF_CS_GPIO
#define MCU_SBC_CS_PIN                   TLF_CS_PIN


/** \brief  Device configuration 0 */
typedef struct _Ifx_TLF35584_DEVCFG0_Bits
{
    uint8_t TRDEL:4;            /**< \brief [3:0] Transition delay into low power states (STANDBY, SLEEP). Defined as a step of 100 us (rw) */
    uint8_t nu:4;               /**< \brief [5:4] (none) */
    uint8_t WKTIMCYC:1;         /**< \brief [6] Wake timer cycle period (rw) */
    uint8_t WKTIMEN:1;          /**< \brief [7] Wake timer enable (rw) */

}TLF35584_DEVCFG0_Bits;

/** \brief  Device configuration 1 */
typedef struct _Ifx_TLF35584_DEVCFG1_Bits
{
    uint8_t RESDEL:3;           /**< \brief [2:0] Reset release delay time (rw) */
    uint8_t nu:3;               /**< \brief [7:3] (none) */
}TLF35584_DEVCFG1_Bits;

/** \brief  Device configuration 2 */
typedef struct _Ifx_TLF35584_DEVCFG2_Bits
{
    uint8_t ESYNEN:1;           /**< \brief [0] Disable synchronization output for external switchmode regulator (rw) */
    uint8_t ESYNPHA:1;          /**< \brief [1] External synchronization output phase */
    uint8_t CTHE:1;             /**< \brief [3:2] Current monitoring threshold value (rw) */
    uint8_t CMONEN:1;           /**< \brief [4] Current monitor enabled when transition to a low power state (SLEEP, STANDBY) is initiated. The setting is overriden in SLEEP as current monitoring is always enabled. (rw) */
    uint8_t FRE:1;              /**< \brief [5] Step-down converter High Frequency selection. (r) */
    uint8_t STU:1;              /**< \brief [6] Step-up converter enable (r) */
    uint8_t EVCEN:1;            /**< \brief [7] External enable of Core supply voltage (r) */
}TLF35584_DEVCFG2_Bits;

/** \brief  Protection register */
typedef struct _Ifx_TLF35584_PROTCFG_Bits
{
    uint8_t KEY:8;           /**< \brief [7:0] Protection register to control write access to protected registers (rw) */
}TLF35584_PROTCFG_Bits;

/** \brief  System protected configuration 0 */
typedef struct _Ifx_TLF35584_SYSPCFG0_Bits
{
    uint8_t STBYEN:1;          /**< \brief [0] Enable request for standby LDO. Valid for all device states except FAILSAFE (none) */
    uint8_t nu:7;              /**< \brief [7:1]  (none) */
}TLF35584_SYSPCFG0_Bits;

/** \brief  System protected configuration 1 */
typedef struct _Ifx_TLF35584_SYSPCFG1_Bits
{
    uint8_t ERRREC:2;          /**< \brief [1:0] ERR pin monitor recovery time (rwp) */
    uint8_t ERRRECEN:1;        /**< \brief [2] ERR pin monitor recovery enable (rwp) */
    uint8_t ERREN:1;           /**< \brief [3] rwp Enable ERR pin monitors (rwp) */
    uint8_t ERRSLPEN:1;        /**< \brief [4] Enabling of ERR pin monitor functionality while the system is in SLEEP (rwp) */
    uint8_t SS2DEL:3;          /**< \brief [7:5] Safe state 2 delay (rwp) */
}TLF35584_SYSPCFG1_Bits;

/** \brief  Watchdog configuration 0 */
typedef struct _Ifx_TLF35584_WDCFG0_Bits
{
    uint8_t WDCYC:1;        /**< \brief [0] Watchdog cycle time (rwp) */
    uint8_t WWDTSEL:1;      /**< \brief [1] Window watchdog trigger selection. This is ignored when Functional is disabled. (rwp) */
    uint8_t FWDEN:1;        /**< \brief [2] Functional Watchdog enable (rwp) */
    uint8_t WWDEN:1;        /**< \brief [3] Window Watchdog enable (rwp) */
    uint8_t WWDETHR:4;      /**< \brief [7:4] Window watchdog error threshold. Number of WWD errors to generate CPU reset and enter into INIT state (rwp) */
}TLF35584_WDCFG0_Bits;

/** \brief  Watchdog configuration 1 */
typedef struct _Ifx_TLF35584_WDCFG1_Bits
{
    uint8_t FWDETHR:4;           /**< \brief [3:0] Functional watchog error threshold. Number of FWD errors to enter into FAILSAFE condition. (rwp) */
    uint8_t WDSLPEN:1;           /**< \brief [4] Enabling of WD functionality while the system is in SLEEP (rwp) */
    uint8_t nu:3;                /**< \brief [7:5]  (none) */
}TLF35584_WDCFG1_Bits;

/** \brief  Functional Watchdog configuration */
typedef struct _Ifx_TLF35584_FWDCFG_Bits
{
    uint8_t WDHBTP:5;            /**< \brief [4:0] Watchdog heartbeat timer period defined as a multiple of 50 watchdog cycles, read only bits shall be written as 0 and will always return 1 upon read. (none) */
    uint8_t nu:3;                /**< \brief [7:5]  (none) */
}TLF35584_FWDCFG_Bits;

/** \brief  Window Watchdog configuration 0 */
typedef struct _Ifx_TLF35584_WWDCFG0_Bits
{
    uint8_t CW:5;                /**< \brief [4:0] WWD close window defined as a multiple of 50 watchdog cycles, read only bits shall be written as 0 and will always return 1 upon read. (rwp) */
    uint8_t nu:3;                /**< \brief [7:5]  (none) */
}TLF35584_WWDCFG0_Bits;

/** \brief  Window Watchdog configuration 1 */
typedef struct _Ifx_TLF35584_WWDCFG1_Bits
{
    uint8_t OW:5;                /**< \brief [4:0] WWD Open window defined as a multiple of 50 watchdog cycles. (none) */
    uint8_t nu:3;                /**< \brief [7:5]  (none) */
}TLF35584_WWDCFG1_Bits;

/** \brief  System configuration 0 status */
typedef struct _Ifx_TLF35584_RSYSPCFG0_Bits
{
    uint8_t STBYEN:1;            /**< \brief [0] Current configuration for standby LDO. Valid for all device states except FAILSAFE (r) */
    uint8_t nu:7;                /**< \brief [7:1]  (none) */
}TLF35584_RSYSPCFG0_Bits;

/** \brief System configuration 1 status  */
typedef struct _Ifx_TLF35584_RSYSPCFG1_Bits
{
    uint8_t ERRREC:2;            /**< \brief [1:0] ERR pin monitor recovery (r) */
    uint8_t RECEN:1;             /**< \brief [2] safe state recovery enable (r) */
    uint8_t ERREN:1;             /**< \brief [3] Enable ERR pin monitor (r) */
    uint8_t ERRSLPEN:1;          /**< \brief [4] Enabling of ERR pin monitor functionality while the system is in SLEEP (r) */
    uint8_t SS2DEL:1;            /**< \brief [7:5] Safe state 2 delay (r) */
}TLF35584_RSYSPCFG1_Bits;

/** \brief Watchdog configuration 0 status  */
typedef struct _Ifx_TLF35584_RWDCFG0_Bits
{
    uint8_t WDCYC:1;             /**< \brief [1:0] Current configuration of watchdog cycle time (r) */
    uint8_t WWDTSEL:1;           /**< \brief [2] Current configuration of window watchdog trigger selection. This isignored when window watchdog is disabled (r) */
    uint8_t FWDEN:1;             /**< \brief [3] Current configuration of functional watchdog (r) */
    uint8_t WWDEN:1;             /**< \brief [4] Current configuration of window watchdog (r) */
    uint8_t WWDETHR:4;           /**< \brief [7:5] Current configuration of window watchdog error threshold. Number of WWD errors to generate CPU reset and enter into INIT state (r) */
}TLF35584_RWDCFG0_Bits;


/** \brief Watchdog configuration 1 status  */
typedef struct _Ifx_TLF35584_RWDCFG1_Bits
{
    uint8_t FWDETHR:4;           /**< \brief [3:0] Current configuration of functional watchdog error threshold.Number of FWD errors to enter into FAILSAFE condition. (r) */
    uint8_t WDSLPEN:1;           /**< \brief [4] Current configuration of WD functionality while the system is in SLEEP (r) */
    uint8_t nu:3;                /**< \brief [7:5]  (none) */
}TLF35584_RWDCFG1_Bits;

/** \brief Functional watchdog configuration status  */
typedef struct _Ifx_TLF35584_RFWDCFG_Bits
{
    uint8_t WDHBTP:5;            /**< \brief [4:0] Current configuration of watchdog heartbeat timer period defined as a multiple of 50 watchdog cycles. (r) */
    uint8_t nu:3;                /**< \brief [7:5]  (none) */
}TLF35584_RFWDCFG_Bits;

/** \brief Window watchdog configuration 0 status  */
typedef struct _Ifx_TLF35584_RWWDCFG0_Bits
{
    uint8_t CW:5;                /**< \brief [4:0] Current configuration of WWD close window defined as a multiple of 50 watchdog cycles. (r) */
    uint8_t nu:3;                /**< \brief [7:5]  (none) */
}TLF35584_RWWDCFG0_Bits;

/** \brief Window watchdog configuration 1 status  */
typedef struct _Ifx_TLF35584_RWWDCFG1_Bits
{
    uint8_t OW:5;                /**< \brief [4:0] Current configuration of WWD open window defined as a multiple of 50 watchdog cycles. (r) */
    uint8_t nu:3;                /**< \brief [7:5]  (none) */
}TLF35584_RWWDCFG1_Bits;

/** \brief  Wake timer configuration 0 */
typedef struct _Ifx_TLF35584_WKTIMCFG0_Bits
{
    uint8_t TIMVALL:8;           /**< \brief [7:0] Wake timer compare value (7:0) (rw) */
}TLF35584_WKTIMCFG0_Bits;

/** \brief  Wake timer configuration 1 */
typedef struct _Ifx_TLF35584_WKTIMCFG1_Bits
{
    uint8_t TIMVALM:8;           /**< \brief [7:0] Wake timer compare value (15:8) (rw) */
}TLF35584_WKTIMCFG1_Bits;

/** \brief  Wake timer configuration 2 */
typedef struct _Ifx_TLF35584_WKTIMCFG2_Bits
{
    uint8_t TIMVALH:8;           /**< \brief [7:0] Wake timer compare value (23:16) (rw) */
}TLF35584_WKTIMCFG2_Bits;

/** \brief Device control */
typedef struct _Ifx_TLF35584_DEVCTRL_Bits
{
    uint8_t STATEREQ:3;           /**< \brief [2:0] Request for device state transition. Cleared to 000 by the HW after the request is processed. After writing a new state value a user should not change the value before it's cleared by HW. (none) */
    uint8_t VREFEN:1;             /**< \brief [3] Enable request for reference voltage. (rw) */
    uint8_t nu:1;                 /**< \brief [4]  (none) */
    uint8_t COMEN:1;              /**< \brief [5] Enable request for communication LDO. (rw) */
    uint8_t TRK1EN:1;             /**< \brief [6] Enable request for tracker1 sensor voltage. Ignored by HW when entering into SLEEP state. TR1 will always be OFF in SLEEP state. (rw) */
    uint8_t TRK2EN:1;             /**< \brief [7] Enable request for tracker2 sensor voltage. Ignored by HW when entering into SLEEP state. TR2 will always be OFF in SLEEP state. (rw) */
}TLF35584_DEVCTRL_Bits;

/** \brief Device control inverted request */
typedef struct _Ifx_TLF35584_DEVCTRLN_Bits
{
    uint8_t STATEREQ:3;           /**< \brief [2:0] Request for device state transition. Cleared to 000 by the HW after the request is processed. After writing a new state value a user should not change the value before it's cleared by HW. (none) */
    uint8_t VREFEN:1;             /**< \brief [3] Enable request for reference voltage. (rw) */
    uint8_t nu:1;                 /**< \brief [4]  (none) */
    uint8_t COMEN:1;              /**< \brief [5] Enable request for communication LDO. (rw) */
    uint8_t TRK1EN:1;             /**< \brief [6] Enable request for tracker1 sensor voltage. Ignored by HW when entering into SLEEP state. TR1 will always be OFF in SLEEP state. (rw) */
    uint8_t TRK2EN:1;             /**< \brief [7] Enable request for tracker2 sensor voltage. Ignored by HW when entering into SLEEP state. TR2 will always be OFF in SLEEP state. (rw) */
}TLF35584_DEVCTRLN_Bits;

/** \brief Window watchdog service command */
typedef struct _Ifx_TLF35584_WWDSCMD_Bits
{
    uint8_t TRIG:1;               /**< \brief [0] WWD trigger command, to trigger WWD, read bit first and write back inverted value. (rw) */
    uint8_t nu:6;                 /**< \brief [6:1]  (none) */
    uint8_t TRIG_STATUS:1;        /**< \brief [7] Last internal trigger value received via SPI, the value TRIG shall always be the opposite to this value (r) */
}TLF35584_WWDSCMD_Bits;

/** \brief Functional watchdog response command */
typedef struct _Ifx_TLF35584_FWDRSP_Bits
{
    uint8_t FWDRSP:8;           /**< \brief [7:0] Functional Watchdog Rersponse Byte Write Command. (rw) */
}TLF35584_FWDRSP_Bits;

/** \brief Functional watchdog response command with synchronization */
typedef struct _Ifx_TLF35584_FWDRSPSYNC_Bits
{
    uint8_t FWDRSPS:8;           /**< \brief [7:0] Functional Watchdog Response Byte Write and Heartbeat Synchronization Command. (rw) */
}TLF35584_FWDRSPSYNC_Bits;

/** \brief Failure status flags */
typedef struct _Ifx_TLF35584_SYSFAIL_Bits
{
    uint8_t VOLTSELERR:1;         /**< \brief [0] Double Bit error on voltage selection, device entered FAILSAFE state. (rw1c) */
    uint8_t OTF:1;                /**< \brief [1] Over temperature failure. (rw1c) */
    uint8_t VMONF:1;              /**< \brief [2] Voltage monitor failure. (rw1c) */
    uint8_t nu:4;                 /**< \brief [6:3]  (none) */
    uint8_t INITF:1;              /**< \brief [7] INIT Failure due to the third INIT failure in row. i.e. The device restarts INIT phase from FAILSAFE. (rw1c) */
}TLF35584_SYSFAIL_Bits;

/** \brief Init error status flags */
typedef struct _Ifx_TLF35584_INITERR_Bits
{
    uint8_t nu:2;                /**< \brief [1:0] (none) */
    uint8_t VMONF:1;             /**< \brief [2] Voltage monitor failure which lead to fo to INIT. (rw1c) */
    uint8_t WWDF:1;              /**< \brief [3] Window watchdog error counter reached the error threshold. (rw1c) */
    uint8_t FWDF:1;              /**< \brief [4] Functional watchdog error counter reached the error threshold (rw1c) */
    uint8_t ERRF:1;              /**< \brief [5] MCU Error monitor failure. (rw1c) */
    uint8_t SOFTRES:1;           /**< \brief [6] Soft reset has been generated due to the first INIT failure. i.e. The device restarts INIT phase for the 2nd time.. (rw1c) */
    uint8_t HARDRES:1;           /**< \brief [7] Hard reset has been generated due to the second INIT failure in row. i.e. The device restarts INIT phase for the 3rd time. (rw1c) */
}TLF35584_INITERR_Bits;

/** \brief Interrupt flags */
typedef struct _Ifx_TLF35584_IF_Bits
{
    uint8_t SYS:1;               /**< \brief [0] System interrupt flag (rw1c) */
    uint8_t WK:1;                /**< \brief [1] Wake interrupt flag (rw1c) */
    uint8_t SPI:1;               /**< \brief [2] SPI interrupt flag (rw1c) */
    uint8_t MON:1;               /**< \brief [3] Monitor interrupt flag (none) */
    uint8_t OTW:1;               /**< \brief [4] Over temperature warning interrupt flag (rw1c) */
    uint8_t OTF:1;               /**< \brief [5] Over temperature failure interrupt flag (rw1c) */
    uint8_t ABIST:1;             /**< \brief [6] Requested ABIST operation performed (rw1c) */
    uint8_t INTMISS:1;           /**< \brief [7] Interrupt has not been serviced within tINTTO time (r) */
}TLF35584_IF_Bits;

/** \brief System status flags */
typedef struct _Ifx_TLF35584_SYSSF_Bits
{
    uint8_t CFGE:1;              /**< \brief [0] Double bit error occurred on protected configuration register, Status registers shall be read in order to determine which configuration has changed (rw1c) */
    uint8_t WWDE:1;              /**< \brief [1] Window watchdog error interrupt flag (rw1c) */
    uint8_t FWDE:1;              /**< \brief [2] Functional watchdog error interrupt flag (rw1c) */
    uint8_t ERRMISS:1;           /**< \brief [3] MCU error miss status flag. Set only when SSCCFG0.RECEN='1' (rw1c) */
    uint8_t TRFAIL:1;            /**< \brief [4] Transition to low power failed either due to the current monitor or WAK event or a rising edge on ENA during TRDEL time (rw1c) */
    uint8_t NO_OP:1;             /**< \brief [5] Requested state transition via DEVCTRL, DEVCTRLN could not be performed because of wrong protocol (rw1c) */
    uint8_t nu:2;                /**< \brief [7:6] (none) */
}TLF35584_SYSSF_Bits;

/** \brief Wakeup status flags */
typedef struct _Ifx_TLF35584_WKSF_Bits
{
    uint8_t WAK:1;               /**< \brief [0] Wakeup by WAK signal (rw1c) */
    uint8_t ENA:1;               /**< \brief [1] Wake by Enable (rw1c) */
    uint8_t CMON:1;              /**< \brief [2] Wakup by current monitor threshold (rw1c) */
    uint8_t WKTIM:1;             /**< \brief [3] Wakeup by wake timer (rw1c) */
    uint8_t WKSPI:1;             /**< \brief [4] Wakeup by SPI (rw1c) */
    uint8_t nu:3;                /**< \brief [7:5] (none) */
}TLF35584_WKSF_Bits;


/** \brief SPI status flags */
typedef struct _Ifx_TLF35584_SPISF_Bits
{
    uint8_t PARE:1;              /**< \brief [0] Parity error in SPI frame (rw1c) */
    uint8_t LENE:1;              /**< \brief [1] Invalid frame length (rw1c) */
    uint8_t ADDRE:1;             /**< \brief [2] Invalid address in SPI frame (rw1c) */
    uint8_t DURE:1;              /**< \brief [3] Error in duration of SPI transaction (rw1c) */
    uint8_t LOCK:1;              /**< \brief [4] Error during LOCK/UNLOCK procedure */
    uint8_t nu:3;                /**< \brief [7:5] (none) */
}TLF35584_SPISF_Bits;

/** \brief Monitor status flags 0 */
typedef struct _Ifx_TLF35584_MONSF0_Bits
{
    uint8_t PREGSG:1;            /**< \brief [0] Pre-regulator voltage short to ground status flag (rw1c) */
    uint8_t UCSG:1;              /**< \brief [1] uC LDO short to ground status flag (rw1c) */
    uint8_t STBYSG:1;            /**< \brief [2] Standby LDO short to ground status flag (rw1c) */
    uint8_t VCORESG:1;           /**< \brief [3] Core voltage short to ground status flag (rw1c) */
    uint8_t COMSG:1;             /**< \brief [4] Communication LDO short to ground status flag (rw1c) */
    uint8_t VREFSG:1;            /**< \brief [5] Voltage reference short to groung status flag (rw1c) */
    uint8_t TRK1SG:1;            /**< \brief [6] Tracker1 short to ground status flag (rw1c) */
    uint8_t TRK2SG:1;            /**< \brief [7] Tracker2 short to ground status flag (rw1c) */
}TLF35584_MONSF0_Bits;

/** \brief Monitor status flags 1 */
typedef struct _Ifx_TLF35584_MONSF1_Bits
{
    uint8_t PREGOV:1;            /**< \brief [0] Pre-regulator voltage over voltage status flag (rw1c) */
    uint8_t UCOV:1;              /**< \brief [1] uC LDO over voltage status flag (rw1c) */
    uint8_t STBYOV:1;            /**< \brief [2] Standby LDO over voltage status flag (rw1c) */
    uint8_t VCOREOV:1;           /**< \brief [3] Core voltage over voltage status flag (rw1c) */
    uint8_t COMOV:1;             /**< \brief [4] Communication LDO over voltage status flag (rw1c) */
    uint8_t VREFOV:1;             /**< \brief [5] Voltage reference short to groung status flag (rw1c) */
    uint8_t TRK1OV:1;            /**< \brief [6] Tracker1 over voltage status flag (rw1c) */
    uint8_t TRK2OV:1;            /**< \brief [7] Tracker2 over voltage status flag (rw1c) */
}TLF35584_MONSF1_Bits;

/** \brief Monitor status flags 2 */
typedef struct _Ifx_TLF35584_MONSF2_Bits
{
    uint8_t PREGUV:1;            /**< \brief [0]  Pre-regulator voltage under voltage status flag (rw1c) */
    uint8_t UCUV:1;              /**< \brief [1] uC LDO under voltage status flag (rw1c) */
    uint8_t STBYUV:1;            /**< \brief [2] Standby LDO under voltage status flag (rw1c) */
    uint8_t VCOREUV:1;           /**< \brief [3]  Core voltage under voltage status flag (rw1c) */
    uint8_t COMUV:1;             /**< \brief [4]  Communication LDO under voltage status flag (rw1c) */
    uint8_t VREFUV:1;            /**< \brief [5]  Voltage reference short to groung status flag (rw1c) */
    uint8_t TRK1UV:1;            /**< \brief [6]  Tracker1 under voltage status flag (rw1c) */
    uint8_t TRK2UV:1;            /**< \brief [7]  Tracker2 under voltage status flag (rw1c) */
}TLF35584_MONSF2_Bits;

/** \brief Monitor status flags 3 */
typedef struct _Ifx_TLF35584_MONSF3_Bits
{
    uint8_t VBATOV:1;            /**< \brief [0] Supply voltage VS1/2 is too high (rw1c) */
    uint8_t nu:3;                /**< \brief [3:1]  (none) */
    uint8_t BG12UV:1;            /**< \brief [4] Bandgap comparator UV condition (rw1c) */
    uint8_t BG12OV:1;            /**< \brief [5] Bandgap comparator OV condition (rw1c) */
    uint8_t BIASLOW:1;           /**< \brief [6] Bias current too low  (rw1c) */
    uint8_t BIASHI:1;            /**< \brief [7] Bias current too high (rw1c) */
}TLF35584_MONSF3_Bits;

/** \brief Over temperature failure status flags */
typedef struct _Ifx_TLF35584_OTFAIL_Bits
{
    uint8_t PREG:1;             /**< \brief [0] Pre-regulator over temperature  (rw1c) */
    uint8_t UC:1;               /**< \brief [1]  uC LDO over temperature  (rw1c) */
    uint8_t nu_0:2;             /**< \brief [3:2]  (none) */
    uint8_t COM:1;              /**< \brief [4]  Communication LDO over temperature  (rw1c) */
    uint8_t nu_1:3;             /**< \brief [7:5]  (none) */
}TLF35584_OTFAIL_Bits;

/** \brief Over temperature failure status flags *R1) */
typedef struct _Ifx_TLF35584_BCK_FREQ_CHANGE_Bits
{
    uint8_t BCK_FREQ_SEL:3;     /**< \brief [2:0] BUCK switching frequency change, for hi and low switching mode. New value needs to be validated via data_valid procedure (rw) */
    uint8_t nu:5;               /**< \brief [7:3]  (none) */
}TLF35584_BCK_FREQ_CHANGE_Bits;

/** \brief Buck Frequency spread */
typedef struct _Ifx_TLF35584_BCK_FRE_SPREAD_Bits
{
    uint8_t FRE_SP_THR:8;       /**< \brief [7:0]  (rw) */
}TLF35584_BCK_FRE_SPREAD_Bits;

/** \brief Buck main PFM control */
typedef struct _Ifx_TLF35584_BCK_MAIN_CTRL_Bits
{
    uint8_t nu:6;               /**< \brief [5:0]  (none) */
    uint8_t DATA_VALID:6;       /**< \brief [6] Command synchronized to master clock, rising edge detect (outside PID) for loading new parameter (rw) */
    uint8_t BUSY:7;             /**< \brief [7] DATA_VALID parameter update  (r) */
}TLF35584_BCK_MAIN_CTRL_Bits;

/** \brief Over temperature warning status flags */
typedef struct _Ifx_TLF35584_OTWRNSF_Bits
{
    uint8_t PREG:1;            /**< \brief [0] Pre-regulator over temperature warning (rw1c) */
    uint8_t UC:1;              /**< \brief [1] uC LCO over temperature warning (rw1c) */
    uint8_t STDBY:1;           /**< \brief [2] Standby LDO over current (rw1c) */
    uint8_t nu_0:1;            /**< \brief [3] (none) */
    uint8_t COM:1;             /**< \brief [4] Communication LDO over temperature warning (rw1c) */
    uint8_t VREF:1;            /**< \brief [5]  Voltage reference over current (rw1c) */
    uint8_t nu_1:2;            /**< \brief [7:6]  (none) */
}TLF35584_OTWRNSF_Bits;

/** \brief Voltage monitor status */
typedef struct _Ifx_TLF35584_VMONSTAT_Bits
{
    uint8_t VPREGST:1;         /**< \brief [0] Standby LDO voltage status (r) */
    uint8_t UCST:1;            /**< \brief [1] Traker1 sensor LDO voltage status (r) */
    uint8_t STBYST:1;          /**< \brief [2] Traker2 sensor LDO voltage status (r) */
    uint8_t VCOREST:1;         /**< \brief [3] Reference voltage status (r) */
    uint8_t COMST:1;           /**< \brief [4] Communication voltage status (r) */
    uint8_t VREFST:1;          /**< \brief [5] Pre-regulator voltage status *maybe used just for testing (r) */
    uint8_t TRK1ST:1;          /**< \brief [6] Core voltage status (r) */
    uint8_t TRK2ST:1;          /**< \brief [7] uC LDO voltage status *maybe used just for testing (r) */
}TLF35584_VMONSTAT_Bits;

/** \brief Device status  */
typedef struct _Ifx_TLF35584_DEVSTAT_Bits
{
    uint8_t STATE:3;           /**< \brief [2:0] Current device state (r) */
    uint8_t VREFEN:1;          /**< \brief [3] Current state reference voltage enable (r) */
    uint8_t STBYEN:1;          /**< \brief [4] Current state of standby LDO enable (r) */
    uint8_t COMEN:1;           /**< \brief [5] Current state of Communication LDO enable (r) */
    uint8_t TRK1EN:1;          /**< \brief [6] Current status of Tracker1 voltage enable (r) */
    uint8_t TRK2EN:1;          /**< \brief [7] Current status of Tracker2 voltage enable (r) */
}TLF35584_DEVSTAT_Bits;

/** \brief Protection status */
typedef struct _Ifx_TLF35584_PROTSTAT_Bits
{
    uint8_t LOCK:1;            /**< \brief [0] Protected register lock status (r) */
    uint8_t nu:3;              /**< \brief [3:1]  (none) */
    uint8_t KEY1OK:1;          /**< \brief [4] Information about validity of the 1st received protection key byte (r) */
    uint8_t KEY2OK:1;          /**< \brief [5] Information about validity of the 2nd received protection key byte (r) */
    uint8_t KEY3OK:1;          /**< \brief [6] Information about validity of the 3th received protection key byte (r) */
    uint8_t KEY4OK:1;          /**< \brief [7] Information about validity of the 4th received protection key byte (r) */
}TLF35584_PROTSTAT_Bits;

/** \brief Window watchdog status */
typedef struct _Ifx_TLF35584_WWDSTAT_Bits
{
    uint8_t WWDECNT:4;         /**< \brief [3:0] Window watchdog error counter status (r) */
    uint8_t nu:4;              /**< \brief [7:4] (none) */
}TLF35584_WWDSTAT_Bits;

/** \brief Functional watchdog status 0 */
typedef struct _Ifx_TLF35584_FWDSTAT0_Bits
{
    uint8_t FWDQUEST:4;        /**< \brief [3:0] Functional watchdog Question (r) */
    uint8_t FWDRSPC:2;         /**< \brief [5:4] Functional watchdog response Counter Value (r) */
    uint8_t FWDRSPOK:1;        /**< \brief [6] Functional watchdog response check status (r) */
    uint8_t nu:1;              /**< \brief [7]  (none) */
}TLF35584_FWDSTAT0_Bits;

/** \brief Functional watchdog status 1 */
typedef struct _Ifx_TLF35584_FWDSTAT1_Bits
{
    uint8_t FWDECNT:4;         /**< \brief [3:0] Functional watchdog Error Counter Value (r) */
    uint8_t nu:4;              /**< \brief [7:4]  (none) */
}TLF35584_FWDSTAT1_Bits;

/** \brief ABIST control 0 */
typedef struct _Ifx_TLF35584_ABIST_CTRL0_Bits
{
    uint8_t START:1;           /**< \brief [0] Start ABIST operation, the operation itself will be started. This bit is cleared after ABIST operation has been performed (rwhc) */
    uint8_t PATH:1;            /**< \brief [1] Select the path which should be covered by ABIST operation (rw) */
    uint8_t SINGLE:1;          /**< \brief [2] rw Select wether a single comparator shall be tested or all comparators in predefined sequence (rw) */
    uint8_t INT:1;             /**< \brief [3] Select wether safe state related comparator shall be tested or interrupt related (rw) */
    uint8_t STATUS:4;          /**< \brief [7:4] ABIST global status information after requested operation has been performed, information shall only be considered valid, once START bit is cleared (r) */
}TLF35584_ABIST_CTRL0_Bits;

/** \brief ABIST control 1 */
typedef struct _Ifx_TLF35584_ABIST_CTRL1_Bits
{
    uint8_t OV_TRIG:1;         /**< \brief [0] Enable overvoltage trigger for protected comparator for analog safe (rw) */
    uint8_t ABIST_CLK_EN:1;    /**< \brief [0] Select ABIST clock to generate local 2 MHz clock (rw) */
    uint8_t nu:6;              /**< \brief [7:2] (none) */
}TLF35584_ABIST_CTRL1_Bits;

/** \brief ABIST select 0 */
typedef struct _Ifx_TLF35584_ABIST_SELECT0_Bits
{
    uint8_t PREGOV:1;          /**< \brief [0] Select Pre-regulator OV comparator for ABIST operation (rwhu) */
    uint8_t UCOV:1;            /**< \brief [1] Select uC LDO OV comparator for ABIST operation (rwhu) */
    uint8_t STBYOV:1;          /**< \brief [2] Select Standby LDO OV comparator for ABIST operatio (rwhu) */
    uint8_t VCOREOV:1;         /**< \brief [3] Select Core voltage OV comparator for ABIST operation (rwhu) */
    uint8_t COMOV:1;           /**< \brief [4] Select COM OV comparator for ABIST operation (rwhu) */
    uint8_t VREFOV:1;          /**< \brief [5] Select VREF OV comparator for ABIST operation (rwhu) */
    uint8_t TRK1OV:1;          /**< \brief [6] Select TRK1 OV comparator for ABIST operation (rwhu) */
    uint8_t TRK2OV:1;          /**< \brief [7] Select TRK2 OV comparator for ABIST operation (rwhu) */
}TLF35584_ABIST_SELECT0_Bits;

/** \brief ABIST select 1 */
typedef struct _Ifx_TLF35584_ABIST_SELECT1_Bits
{
    uint8_t PREGUV:1;          /**< \brief [0] Select Pre-regulator UV comparator for ABIST operation (rwhu) */
    uint8_t UCUV:1;            /**< \brief [1] Select uC LDO UV comparator for ABIST operation (rwhu) */
    uint8_t STBYUV:1;          /**< \brief [2] Select Standby LDO UV comparator for ABIST operatio (rwhu) */
    uint8_t VCOREUV:1;         /**< \brief [3] Select Core voltage UV comparator for ABIST operation (rwhu) */
    uint8_t COMUV:1;           /**< \brief [4] Select COM UV comparator for ABIST operation (rwhu) */
    uint8_t VREFUV:1;          /**< \brief [5] Select VREF UV comparator for ABIST operation (rwhu) */
    uint8_t TRK1UV:1;          /**< \brief [6] Select TRK1 UV comparator for ABIST operation (rwhu) */
    uint8_t TRK2UV:1;          /**< \brief [7] Select TRK2 UV comparator for ABIST operation (rwhu) */
}TLF35584_ABIST_SELECT1_Bits;

/** \brief ABIST select 2 */
typedef struct _Ifx_TLF35584_ABIST_SELECT2_Bits
{
    uint8_t VBATOV:1;          /**< \brief [0] Supply voltage VS1/2 is too high (rwhu) */
    uint8_t nu:2;              /**< \brief [2:1] (none) */
    uint8_t V5OV:1;            /**< \brief [3] Internal voltage 5V0 OV condition (rwhu) */
    uint8_t BG12UV:1;          /**< \brief [4] Bandgap comparator OV condition (rwhu) */
    uint8_t BG12OV:1;          /**< \brief [5] Bandgap comparator OV condition (rwhu) */
    uint8_t BIASLOW:1;         /**< \brief [6] Bias current too low (rwhu) */
    uint8_t BIASHI:1;          /**< \brief [7] Bias current too high (rwhu) */
}TLF35584_ABIST_SELECT2_Bits;

/** \brief ABIST control 1 */
typedef struct _Ifx_TLF35584_ABIST_CTRL3_Bits
{
    uint8_t OV_TRIG:1;         /**< \brief [0] nable overvoltage trigger for protected comparator for analog safe state control (rw) */
    uint8_t ABIST_CLK_EN:1;    /**< \brief [1] Select ABIST clock to generate local 2 MHz clock (rw) */
    uint8_t nu:6;              /**< \brief [7:2] (none) */
}TLF35584_ABIST_CTRL3_Bits;

/** \brief Buck PWM control 1  */
typedef struct _Ifx_TLF35584_PWM_CTRL1_Bits
{
    uint8_t I_EXP:4;           /**< \brief [3:0] Integral Exponent  (rw) */
    uint8_t P_EXP:4;           /**< \brief [7:4]  Proportional Exponent  (rw) */
}TLF35584_PWM_CTRL1_Bits;

/** \brief Buck PWM control 2 */
typedef struct _Ifx_TLF35584_PWM_CTRL2_Bits
{
    uint8_t ADC_LSB:1;         /**< \brief [0]  Programming of the ADC Vq  (rw) */
    uint8_t PWR_USE_EN:1;      /**< \brief [1]  Enable Dynamic Power Switches Usage  (rw) */
    uint8_t D_LOW_TC:2;        /**< \brief [3:2] PWM Derivative Low Pass  (rw) */
    uint8_t D_EXP:4;           /**< \brief [7:4] PWM Derivative Exponent  (rw) */
}TLF35584_PWM_CTRL2_Bits;

/** \brief Buck PWM cycle control 1  */
typedef struct _Ifx_TLF35584_PWM_CYC_CTRL1_Bits
{
    uint8_t PWM_DIV_HF_LSB:8;  /**< \brief [7:0] PWM divider control for high frequency selection (LSB value). Reset	Value equal to 72 decimal=2.2 MHz. (rw) */
}TLF35584_PWM_CYC_CTRL1_Bits;

/** \brief Buck PWM cycle control 2 */
typedef struct _Ifx_TLF35584_PWM_CYC_CTRL2_Bits
{
    uint8_t PWM_DIV_LF_MSB:8;  /**< \brief [7:0] PWM divider control for low frequency selection (MSB value). Reset Value equal to 356 decimal=450 kHz. (rw) */
}TLF35584_PWM_CYC_CTRL2_Bits;

/** \brief Buck trim */
typedef struct _Ifx_TLF35584_IDAC_TRIM_Bits
{
    uint8_t HS_CL_TRIM:4;      /**< \brief [3:0] High side current trimming value (rw) */
    uint8_t ZERO_CROSS_TH:2;   /**< \brief [5:4] Current threshold for current zero crossing comparator (rw) */
    uint8_t nu:1;              /**< \brief [6]  (none) */
    uint8_t HS_CL_TRIMM_SEL:1; /**< \brief [7] High side current trimming selection (rw) */
}TLF35584_IDAC_TRIM_Bits;

/** \brief Buck main PFM control */
typedef struct _Ifx_TLF35584_MAIN_CTRL_Bits
{
    uint8_t SU:2;              /**< \brief [1:0] Startup control Increasing DC at SU Cycle (rw) */
    uint8_t SYNC_MODE:2;       /**< \brief [3:2] PWM cycle synchronization mode selection (rw) */
    uint8_t PFM_CURLIM:2;      /**< \brief [5:4]  Select PFM HS current threshold (rw) */
    uint8_t DATA_VALID:1;      /**< \brief [6] Command synchronized to master clock, rising edge detect	(outside PID) for loading new parameter (rw) */
    uint8_t BUSY:1;            /**< \brief [7] DATA_VALID parameter update (rw) */
}TLF35584_MAIN_CTRL_Bits;

/** \brief Buck ADC control 1 */
typedef struct _Ifx_TLF35584_ADC_CTRL1_Bits
{
    uint8_t VOUT1_MSB:8;        /**< \brief [7:0]  VOUT1 definition MSB (rw) */
}TLF35584_ADC_CTRL1_Bits;

/** \brief Buck ADC control 2 */
typedef struct _Ifx_TLF35584_ADC_CTRL2_Bits
{
    uint8_t SAR_TRACK_SEL:1;     /**< \brief [0] PWM cycle synchronization mode selection (rw) */
    uint8_t VIN_BIT_RES:3;       /**< \brief [3:1] Select PFM HS current threshold (rw) */
    uint8_t VOUT2_LSB:2;         /**< \brief [5:4] Command synchronized to master clock, rising edge detect (outside PID) for loading new parameter (rw) */
    uint8_t VOUT1_LSB:2;         /**< \brief [7:6]  DATA_VALID parameter update (rw) */
}TLF35584_ADC_CTRL2_Bits;

/** \brief Buck ADC control 3 */
typedef struct _Ifx_TLF35584_ADC_CTRL3_Bits
{
    uint8_t VOUT2_MSB:8;         /**< \brief [7:0] VOUT2 definition MSB (rw) */
}TLF35584_ADC_CTRL3_Bits;

/** \brief Buck ADC control 4 */
typedef struct _Ifx_TLF35584_ADC_CTRL4_Bits
{
    uint8_t UNDER_V_TH:4;        /**< \brief [3:0]  Undervoltage threshold (rw) */
    uint8_t OVER_V_TH:4;         /**< \brief [7:4]  Overvoltage threshold (rw) */
}TLF35584_ADC_CTRL4_Bits;

/** \brief Buck PWM cycle control 3 */
typedef struct _Ifx_TLF35584_PWM_CYC_CTRL3_Bits
{
    uint8_t MIN_DC_MSB:4;        /**< \brief [3:0]Minimum duty cycle (MSB value) (rw) */
    uint8_t PWM_DIV_LF_LSB:2;    /**< \brief [5:4]PWM divider control for low frequency selection (LSB value) (rw) */
    uint8_t PWM_DIV_HF_MSB:2;    /**< \brief [7:6] PWM divider control for high frequency selection (MSB value) (rw) */
}TLF35584_PWM_CYC_CTRL3_Bits;

/** \brief Testmode entr */
typedef struct _Ifx_TLF35584_TME_Bits
{
    uint8_t TMKEY:8;              /**< \brief [7:0]  Test mode entry password key (rw) */
}TLF35584_TME_Bits;

/** \brief Global testmode */
typedef struct _Ifx_TLF35584_GTM_Bits
{
    uint8_t TM:1;                 /**< \brief [0] Test mode status (r) */
    uint8_t NTM:1;                /**< \brief [1] Normal/test mode (r) */
    uint8_t nu:6;                 /**< \brief [7:2] (none) */
}TLF35584_GTM_Bits;

/** \}  */
/******************************************************************************/
/******************************************************************************/
/** \addtogroup IfxLld_TLF35584_union
 * \{  */

/** \brief Device configuration 0 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_DEVCFG0_Bits B;
}TLF35584_DEVCFG0;

/** \brief Device configuration 1 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_DEVCFG1_Bits B;
}TLF35584_DEVCFG1;

/** \brief Device configuration 2 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_DEVCFG2_Bits B;
}TLF35584_DEVCFG2;

/** \brief Protection register */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_PROTCFG_Bits B;
}TLF35584_PROTCFG;

/** \brief System protected configuration 0 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_SYSPCFG0_Bits B;
}TLF35584_SYSPCFG0;

/** \brief System protected configuration 1 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_SYSPCFG1_Bits B;
}TLF35584_SYSPCFG1;

/** \brief Watchdog configuration 0 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_WDCFG0_Bits B;
}TLF35584_WDCFG0;

/** \brief Watchdog configuration 1 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_WDCFG1_Bits B;
}TLF35584_WDCFG1;

/** \brief Functional Watchdog configuration  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_FWDCFG_Bits B;
}TLF35584_FWDCFG;

/** \brief Window Watchdog configuration 0  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_WWDCFG0_Bits B;
}TLF35584_WWDCFG0;

/** \brief Window Watchdog configuration 1  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_WWDCFG1_Bits B;
}TLF35584_WWDCFG1;

/** \brief System configuration 0 status  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_RSYSPCFG0_Bits B;
}TLF35584_RSYSPCFG0;

/** \brief System configuration 1 status  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_RSYSPCFG1_Bits B;
}TLF35584_RSYSPCFG1;

/** \brief Watchdog configuration 0 status  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_RWDCFG0_Bits B;
}TLF35584_RWDCFG0;

/** \brief Watchdog configuration 1 status  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_RWDCFG1_Bits B;
}TLF35584_RWDCFG1;

/** \brief Functional watchdog configuration status  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_RFWDCFG_Bits B;
}TLF35584_RFWDCFG;

/** \brief Window watchdog configuration 0 status   */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_RWWDCFG0_Bits B;
}TLF35584_RWWDCFG0;

/** \brief Window watchdog configuration 1 status   */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_RWWDCFG1_Bits B;
}TLF35584_RWWDCFG1;

/** \brief Wake timer configuration 0  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_WKTIMCFG0_Bits B;
}TLF35584_WKTIMCFG0;

/** \brief Wake timer configuration 1  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_WKTIMCFG1_Bits B;
}TLF35584_WKTIMCFG1;

/** \brief Wake timer configuration 2  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_WKTIMCFG2_Bits B;
}TLF35584_WKTIMCFG2;

/** \brief Device control  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_DEVCTRL_Bits B;
}TLF35584_DEVCTRL;

/** \brief Device control inverted */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_DEVCTRLN_Bits B;
}TLF35584_DEVCTRLN;

/** \brief Window watchdog service command  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_WWDSCMD_Bits B;
}TLF35584_WWDSCMD;

/** \brief Functional watchdog response command  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_FWDRSP_Bits B;
}TLF35584_FWDRSP;

/** \brief Functional watchdog response command with synchronization */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_FWDRSPSYNC_Bits B;
}TLF35584_FWDRSPSYNC;

/** \brief Failure status flags */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_SYSFAIL_Bits B;
}TLF35584_SYSFAIL;

/** \brief Init error status flags */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_INITERR_Bits B;
}TLF35584_INITERR;

/** \brief Interrupt flags */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_IF_Bits B;
}TLF35584_IF;

/** \brief System status flags */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_SYSSF_Bits B;
}TLF35584_SYSSF;

/** \brief Wakeup status flags */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_WKSF_Bits B;
}TLF35584_WKSF;

/** \brief SPI status flags */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_SPISF_Bits B;
}TLF35584_SPISF;

/** \brief Monitor status flags 0 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_MONSF0_Bits B;
}TLF35584_MONSF0;

/** \brief Monitor status flags 1 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_MONSF1_Bits B;
}TLF35584_MONSF1;

/** \brief Monitor status flags 2 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_MONSF2_Bits B;
}TLF35584_MONSF2;

/** \brief Monitor status flags 3 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_MONSF3_Bits B;
}TLF35584_MONSF3;

/** \brief Over temperature failure status flags */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_OTFAIL_Bits B;
}TLF35584_OTFAIL;

/** \brief Over temperature warning status flags */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_OTWRNSF_Bits B;
}TLF35584_OTWRNSF;

/** \brief Voltage monitor status */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_VMONSTAT_Bits B;
}TLF35584_VMONSTAT;

/** \brief Device status */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_DEVSTAT_Bits B;
}TLF35584_DEVSTAT;

/** \brief Protection status */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_PROTSTAT_Bits B;
}TLF35584_PROTSTAT;

/** \brief Window watchdog status */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_WWDSTAT_Bits B;
}TLF35584_WWDSTAT;

/** \brief Functional watchdog status 0 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_FWDSTAT0_Bits B;
}TLF35584_FWDSTAT0;

/** \brief Functional watchdog status 1 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_FWDSTAT1_Bits B;
}TLF35584_FWDSTAT1;

/** \brief ABIST control 0 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_ABIST_CTRL0_Bits B;
}TLF35584_ABIST_CTRL0;

/** \brief ABIST control 1 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_ABIST_CTRL1_Bits B;
}TLF35584_ABIST_CTRL1;

/** \brief ABIST select 0  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_ABIST_SELECT0_Bits B;
}TLF35584_ABIST_SELECT0;

/** \brief ABIST select 1  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_ABIST_SELECT1_Bits B;
}TLF35584_ABIST_SELECT1;

/** \brief ABIST select 2  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_ABIST_SELECT2_Bits B;
}TLF35584_ABIST_SELECT2;

/** \brief  Buck switching frequency change  */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_BCK_FREQ_CHANGE_Bits B;
}TLF35584_BCK_FREQ_CHANGE;

/** \brief  Buck Frequency spread   */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_BCK_FRE_SPREAD_Bits B;
}TLF35584_BCK_FRE_SPREAD;

/** \brief  Buck main PFM control   */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_BCK_MAIN_CTRL_Bits B;
}TLF35584_BCK_MAIN_CTRL;

/** \brief Buck PWM control 1 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_PWM_CTRL1_Bits B;
}TLF35584_PWM_CTRL1;

/** \brief Buck PWM control 2 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_PWM_CTRL2_Bits B;
}TLF35584_PWM_CTRL2;

/** \brief Buck PWM cycle control 1 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_PWM_CYC_CTRL1_Bits B;
}TLF35584_PWM_CYC_CTRL1;

/** \brief Buck PWM cycle control 2 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_PWM_CYC_CTRL2_Bits B;
}TLF35584_PWM_CYC_CTRL2;

/** \brief Buck PWM cycle control 3 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_PWM_CYC_CTRL3_Bits B;
}TLF35584_PWM_CYC_CTRL3;

/** \brief Buck trim */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_IDAC_TRIM_Bits B;
}TLF35584_IDAC_TRIM;

/** \brief Buck main PFM control */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_MAIN_CTRL_Bits B;
}TLF35584_MAIN_CTRL;

/** \brief Buck ADC control 1 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_ADC_CTRL1_Bits B;
}TLF35584_ADC_CTRL1;

/** \brief Buck ADC control 2 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_ADC_CTRL2_Bits B;
}TLF35584_ADC_CTRL2;

/** \brief Buck ADC control 3 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_ADC_CTRL3_Bits B;
}TLF35584_ADC_CTRL3;

/** \brief Buck ADC control 4 */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_ADC_CTRL4_Bits B;
}TLF35584_ADC_CTRL4;

/** \brief Testmode entry */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_TME_Bits B;
}TLF35584_TME;

/** \brief Global testmode */
typedef union
{
    /** \brief Unsigned access */
    unsigned char U;
    /** \brief Signed access */
    signed char I;
    /** \brief Bitfield access */
   TLF35584_GTM_Bits B;
}TLF35584_GTM;


/** \}  */
/******************************************************************************/
/******************************************************************************/
/** \addtogroup IfxLld_TLF35584_struct
 * \{  */
/******************************************************************************/
/** \name Object L0
 * \{  */

/** \brief  TLF35584 object */
typedef volatile struct _Ifx_TLF35584
{
   TLF35584_DEVCFG0 DEVCFG0;                   /**< \brief 00, Device configuration 0 *R2) 00H 08H */
   TLF35584_DEVCFG1 DEVCFG1;                   /**< \brief 00,   Device configuration 1 *R1) 01H 06H */
   TLF35584_DEVCFG2 DEVCFG2;                   /**< \brief 00,  Device configuration 2 *R2) 02H 00H */
   TLF35584_PROTCFG PROTCFG;                   /**< \brief 00,  Protection register *R2) 03H 00H */
   TLF35584_SYSPCFG0 SYSPCFG0;                 /**< \brief 00,   Protected System configuration request 0 *R1) 04H 0AH */
   TLF35584_SYSPCFG1 SYSPCFG1;                 /**< \brief 00,   Protected System configuration request 1 *R2) 05H 00H */
   TLF35584_WDCFG0 WDCFG0;                     /**< \brief 00,  Protected Watchdog configuration request 0 *R2) 06H 9BH */
   TLF35584_WDCFG1 WDCFG1;                     /**< \brief 00,  Protected Watchdog configuration request 1 *R2) 07H 09H */
   TLF35584_FWDCFG FWDCFG;                     /**< \brief 00,  Protected Functional watchdog configuration request *R2) 08H 0BH */
   TLF35584_WWDCFG0 WWDCFG0;                   /**< \brief 00,  Protected Window watchdog configuration request 0 *R2) 09H 06H */
   TLF35584_WWDCFG1 WWDCFG1;                   /**< \brief 00,  Protected Window watchdog configuration request 1 *R2) 0AH 0BH */
   TLF35584_RSYSPCFG0 RSYSPCFG0;               /**< \brief 00,   System configuration 0 status *R1) 0BH 0AH */
   TLF35584_RSYSPCFG1 RSYSPCFG1;               /**< \brief 00,  System configuration 1 status *R3) 0CH 00H */
   TLF35584_RWDCFG0 RWDCFG0;                   /**< \brief 00,  Watchdog configuration 0 status *R3) 0DH 9BH */
   TLF35584_RWDCFG1 RWDCFG1;                   /**< \brief 00,  Watchdog configuration 1 status *R3) 0EH 09H */
   TLF35584_RFWDCFG RFWDCFG;                   /**< \brief 00,  Functional watchdog configuration status *R3) 0FH 0BH */
   TLF35584_RWWDCFG0 RWWDCFG0;                 /**< \brief 00,  Window watchdog configuration 0 status *R3) 10H 06H */
   TLF35584_RWWDCFG1 RWWDCFG1;                 /**< \brief 00,  Window watchdog configuration 1 status *R3) 11H 09H */
   TLF35584_WKTIMCFG0 WKTIMCFG0;               /**< \brief 00,  Wake timer configuration 0 *R2) 12H 00H */
   TLF35584_WKTIMCFG1 WKTIMCFG1;               /**< \brief 00,  Wake timer configuration 1 *R2) 13H 00H */
   TLF35584_WKTIMCFG2 WKTIMCFG2;               /**< \brief 00,   Wake timer configuration 2 *R2) 14H 00H */
   TLF35584_DEVCTRL DEVCTRL;                   /**< \brief 00,   Device control request *R2) 15H 00H */
   TLF35584_DEVCTRLN DEVCTRLN;                 /**< \brief 00,   Device control inverted request *R2) 16H 00H */
   TLF35584_WWDSCMD WWDSCMD;                   /**< \brief 00,   Window watchdog service command *R2) 17H 00H */
   TLF35584_FWDRSP FWDRSP;                     /**< \brief 00,   Functional watchdog response command *R2) 18H 00H */
   TLF35584_FWDRSPSYNC FWDRSPSYNC;             /**< \brief 00,   Functional watchdog response command with synchronization *R2)  19H 00H */
   TLF35584_SYSFAIL SYSFAIL;                   /**< \brief 00,   Failure status flags *R1) 1AH 00H */
   TLF35584_INITERR INITERR;                   /**< \brief 00,   Init error status flags *R2) 1BH 00H */
   TLF35584_IF IF;                             /**< \brief 00,   Interrupt flags *R2) 1CH 00H */
   TLF35584_SYSSF SYSSF;                       /**< \brief 00,   System status flags *R2) 1DH 00H */
   TLF35584_WKSF WKSF;                         /**< \brief 00,   Wakeup status flags *R2) 1EH 00H */
   TLF35584_SPISF SPISF;                       /**< \brief 00,   SPI status flags *R2) 1FH 00H */
   TLF35584_MONSF0 MONSF0;                     /**< \brief 00,   Monitor status flags 0 *R1) 20H 00H */
   TLF35584_MONSF1 MONSF1;                     /**< \brief 00,   Monitor status flags 1 *R1) 21H 00H */
   TLF35584_MONSF2 MONSF2;                     /**< \brief 00,   Monitor status flags 2 *R2) 22H 00H */
   TLF35584_MONSF3 MONSF3;                     /**< \brief 00,   Monitor status flags 3 *R1) 23H 00H */
   TLF35584_OTFAIL OTFAIL;                     /**< \brief 00,   Over temperature failure status flags *R1) 24H 00H */
   TLF35584_OTWRNSF OTWRNSF;                   /**< \brief 00,   Over temperature warning status flags *R2) 25H 00H */
   TLF35584_VMONSTAT VMONSTAT;                 /**< \brief 00,   Voltage monitor status *R2) 26H 00H */
   TLF35584_DEVSTAT DEVSTAT;                   /**< \brief 00,   Device status *R2) 27H 00H */
   TLF35584_PROTSTAT PROTSTAT;                 /**< \brief 00,   Protection status *R2) 28H 01H */
   TLF35584_WWDSTAT WWDSTAT;                   /**< \brief 00,   Window watchdog status *R3) 29H 00H */
   TLF35584_FWDSTAT0 FWDSTAT0;                 /**< \brief 00,   Functional watchdog status 0 *R3) 2AH 30H */
   TLF35584_FWDSTAT1 FWDSTAT1;                 /**< \brief 00,   Functional watchdog status 1 *R3) 2BH 00H */
   TLF35584_ABIST_CTRL0 ABIST_CTRL0;           /**< \brief 00,   ABIST control0 *R2) 2CH 00H */
   TLF35584_ABIST_CTRL1 ABIST_CTRL1;           /**< \brief 00,   ABIST control1 *R2) 2DH 00H */
   TLF35584_ABIST_SELECT0 ABIST_SELECT0;       /**< \brief 00,   ABIST select 0 *R2) 2EH 00H */
   TLF35584_ABIST_SELECT1 ABIST_SELECT1;       /**< \brief 00,   ABIST select 1 *R2) 2FH 00H */
   TLF35584_ABIST_SELECT2 ABIST_SELECT2;       /**< \brief 00,   ABIST select 2 *R2) 30H 00H */
   TLF35584_BCK_FREQ_CHANGE BCK_FREQ_CHANGE;   /**< \brief 00,   Buck switching frequency change *R2) 31H 00H */
   TLF35584_BCK_FRE_SPREAD BCK_FRE_SPREAD;     /**< \brief 00,   Buck Frequency spread *R2) 32H 00H */
   TLF35584_BCK_MAIN_CTRL BCK_MAIN_CTRL;       /**< \brief 00,   Buck main PFM control *R2) 33H 00H */
    unsigned char reserved[11];
   TLF35584_GTM TME;                           /**< \brief 00,    testmode entry 3EH 00H */
   TLF35584_GTM GTM;                           /**< \brief 00,   Global testmode *R2) 3FH 02H */
}TLF35584;

enum TLF35584_REG_ADDR
{
    TLF35584_REG_DEVCFG0,                    // Device configuration 0 *R2) 00H 08H
    TLF35584_REG_DEVCFG1,                    // Device configuration 1 *R1) 01H 06H
    TLF35584_REG_DEVCFG2,                    // Device configuration 2 *R2) 02H 00H
    TLF35584_REG_PROTCFG,                    // Protection register *R2) 03H 00H
    TLF35584_REG_SYSPCFG0,                   // Protected System configuration request 0 *R1) 04H 0AH
    TLF35584_REG_SYSPCFG1,                   // Protected System configuration request 1 *R2) 05H 00H
    TLF35584_REG_WDCFG0,                     // Protected Watchdog configuration request 0 *R2) 06H 9BH
    TLF35584_REG_WDCFG1,                     // Protected Watchdog configuration request 1 *R2) 07H 09H
    TLF35584_REG_FWDCFG,                     // Protected Functional watchdog configuration  request *R2) 08H 0BH
    TLF35584_REG_WWDCFG0,                    // Protected Window watchdog configuration request 0 *R2) 09H 06H
    TLF35584_REG_WWDCFG1,                    // Protected Window watchdog configuration request 1 *R2) 0AH 0BH
    TLF35584_REG_RSYSPCFG0,                  // System configuration 0 status *R1) 0BH 0AH
    TLF35584_REG_RSYSPCFG1,                  // System configuration 1 status *R3) 0CH 00H
    TLF35584_REG_RWDCFG0,                    // Watchdog configuration 0 status *R3) 0DH 9BH
    TLF35584_REG_RWDCFG1,                    // Watchdog configuration 1 status *R3) 0EH 09H
    TLF35584_REG_RFWDCFG,                    // Functional watchdog configuration status *R3) 0FH 0BH
    TLF35584_REG_RWWDCFG0,                   // Window watchdog configuration 0 status *R3) 10H 06H
    TLF35584_REG_RWWDCFG1,                   // Window watchdog configuration 1 status *R3) 11H 09H
    TLF35584_REG_WKTIMCFG0,                  // Wake timer configuration 0 *R2) 12H 00H
    TLF35584_REG_WKTIMCFG1,                  // Wake timer configuration 1 *R2) 13H 00H
    TLF35584_REG_WKTIMCFG2,                  // Wake timer configuration 2 *R2) 14H 00H
    TLF35584_REG_DEVCTRL,                    // Device control request *R2) 15H 00H
    TLF35584_REG_DEVCTRLN,                   // Device control inverted request *R2) 16H 00H
    TLF35584_REG_WWDSCMD,                    // Window watchdog service command *R2) 17H 00H
    TLF35584_REG_FWDRSP,                     // Functional watchdog response command *R2) 18H 00H
    TLF35584_REG_FWDRSPSYNC,                 // Functional watchdog response command with   synchronization *R2)    19H 00H
    TLF35584_REG_SYSFAIL,                    // Failure status flags *R1) 1AH 00H
    TLF35584_REG_INITERR,                    // Init error status flags *R2) 1BH 00H
    TLF35584_REG_IF,                         // Interrupt flags *R2) 1CH 00H
    TLF35584_REG_SYSSF,                      // System status flags *R2) 1DH 00H
    TLF35584_REG_WKSF,                       // Wakeup status flags *R2) 1EH 00H
    TLF35584_REG_SPISF,                      // SPI status flags *R2) 1FH 00H
    TLF35584_REG_MONSF0,                     // Monitor status flags 0 *R1) 20H 00H
    TLF35584_REG_MONSF1,                     // Monitor status flags 1 *R1) 21H 00H
    TLF35584_REG_MONSF2,                     // Monitor status flags 2 *R2) 22H 00H
    TLF35584_REG_MONSF3,                     // Monitor status flags 3 *R1) 23H 00H
    TLF35584_REG_OTFAIL,                     // Over temperature failure status flags *R1) 24H 00H
    TLF35584_REG_OTWRNSF,                    // Over temperature warning status flags *R2) 25H 00H
    TLF35584_REG_VMONSTAT,                   // Voltage monitor status *R2) 26H 00H
    TLF35584_REG_DEVSTAT,                    // Device status *R2) 27H 00H
    TLF35584_REG_PROTSTAT,                   // Protection status *R2) 28H 01H
    TLF35584_REG_WWDSTAT,                    // Window watchdog status *R3) 29H 00H
    TLF35584_REG_FWDSTAT0,                   // Functional watchdog status 0 *R3) 2AH 30H
    TLF35584_REG_FWDSTAT1,                   // Functional watchdog status 1 *R3) 2BH 00H
    TLF35584_REG_ABIST_CTRL0,                // ABIST control0 *R2) 2CH 00H
    TLF35584_REG_ABIST_CTRL1,                // ABIST control1 *R2) 2DH 00H
    TLF35584_REG_ABIST_SELECT0,              // ABIST select 0 *R2) 2EH 00H
    TLF35584_REG_ABIST_SELECT1,              // ABIST select 1 *R2) 2FH 00H
    TLF35584_REG_ABIST_SELECT2,              // ABIST select 2 *R2) 30H 00H
    TLF35584_REG_BCK_FREQ_CHANGE,            // Buck switching frequency change *R2) 31H 00H
    TLF35584_REG_BCK_FRE_SPREAD,             // Buck Frequency spread *R2) 32H 00H
    TLF35584_REG_BCK_MAIN_CTRL,              // Buck main PFM control *R2) 33H 00H
    TLF35584_REG_TME = 0x3E,                 // Testmode entry 3EH
    TLF35584_REG_GTM,                        // Global testmode *R2) 3FH 02H
    TLF35584_MAX_REGS,
    /* Do not add members beyond this point */
};

#define TLF_READ_CMD     (0x0000U)
#define TLF_WRITE_CMD    (0x8000U)


#define WKTIMER_CYC_PERIOD_10us     (0U)
#define WKTIMER_CYC_PERIOD_10ms     (1U)

#define TLF35584_QUC_CURR_THRESHOLD_10mA    (0U)
#define TLF35584_QUC_CURR_THRESHOLD_30mA    (1U)
#define TLF35584_QUC_CURR_THRESHOLD_60mA    (2U)
#define TLF35584_QUC_CURR_THRESHOLD_100mA   (3U)

#define TLF35584_BOOT_FLAG_CHECK_ADDR   0x14000004U


void tlf35584_sm_init_to_normal(void);
void tlf35584_sm_normal_to_standby(void);
void tlf35584_sm_normal_to_sleep(void);
void tlf35584_sm_sleep_to_wake(void);
void tlf35584_wdt_control(uint8_t wwdt_en, uint8_t fwdt_en);
void tlf35584_clear_irqs(void);
void tlf33584_kick_wdt(void);
void tlf33584_err_service(void);
void tlf33584_kick_wdt_wwscmd(void);
void tlf35584_err_disable(void);
void tlf35584_get_wake_status(void);
void tlf35584_fwdt_cfg(uint8_t wdhbtp);
void tlf35584_verify_fwdcfg(uint8_t wdhbtp);
void tlf35584_kick_fwdt(void);
void tlf35584_move_to_normal(void);

void tlf35584_wktimer_val_config(uint8_t timval_low, uint8_t timval_mid, uint8_t timval_high);
void tlf35584_wktimer_cyc_config(uint8_t cyc_period);
void tlf35584_set_quc_curr_threshold(uint8_t current_threshold);

#endif /* DRV_TLF35584_H */

