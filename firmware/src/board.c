 /*
 * 
 * ULTRAVIOLETTE AUTOMOTIVE CONFIDENTIAL
 * ______________________________________
 * 
 * [2019] - [2020] Ultraviolette Automotive Pvt. Ltd.
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
 
#include "fw_common.h"
#include "clock_config.h"
#include "lpspi_master_driver.h"
#include "lpspi_slave_driver.h"
#ifndef USE_DEBUG_PRINTS
#include "debug_console.h"
#endif
#include "msdi.h"
#include "pin_mux.h"
#include "interrupt_manager.h"
#include "csec_driver.h"
#include "lpi2c_driver.h" 
#include "lpi2c_hw_access.h"
#include "drv_spi_legacy.h" 
#include "drv_led.h"
#include "bcm.h"
#include "lpit_driver.h" 
#include "flexcan_driver.h"
#include "S32K148_features.h"
#include "csec_driver.h"
#include "ftm_common.h"
#include "ftm_ic_driver.h"
#include "flash_driver.h"
#include "drv_mem.h"
#include "lpuart_driver.h"
#include "lte_task.h"
#include "bms_task.h"
#include "drv_loadswitches.h" 
#include "bms_can_if.h"
#include "dba_task.h"
#include "charger_task.h"
#include "vcu_can_comm_test_rx.h"
#include "mc_task.h"
#include "bms.h"

#define EDMA_CHN0_NUMBER                    (0U)
#define EDMA_CHN1_NUMBER                    (1U)
#define EDMA_CONFIGURED_CHANNELS_COUNT      (2U)

/* Target Id, Target Iq, Id, Iq */
#define MC_CAN_TPDO1_TID_TIQ_ID                 (0x101U)

/* Cutback Gain, throttle input voltage, velocity */
#define MC_CAN_TPDO2_CB_THRV_VEL                (0x102U)
#define MC_MAX_TPDOS                            (2U)
#define MOT_CONT_CAN_IF_MBX_FILTER         (0x500U)  
#define MOT_CONT_CAN_IF_GBL_MBX_FILTER     (0x500U)

static csec_state_t             csec_state;
static lpi2c_master_state_t     lpi2c1_master_state;
static lpspi_state_t            master_state_sja1105;
static lpspi_state_t    		master_state_cd1030;
static lpspi_state_t            master_state_icm30948;
static flexcan_state_t 			can_mc_state;
static flexcan_state_t 			can_abs_state;
static flexcan_state_t 			can_bcu_state;
static msdi_drv_config_t        msdi_drv_cfg;
static lpuart_state_t           lpuart0_state;

static edma_state_t dma_ctrl_state;
static edma_chn_state_t dma_ctrl_chn0_state;
static edma_chn_state_t dma_ctrl_chn1_state;

/* 
    Messages incoming on this bus.
*/
#define ABS_MAX_MSGS    (6U)
static const uint16_t abs_msg_ids[ABS_MAX_MSGS] = {
    DISP_CAN_ALS_MSG_ID,
    DISP_CAN_MAG_MSG_ID,
    ABS_CAN_SPEED_INFO_MSG_ID,
    CHG_MSG_CHG_BOOT_NTF_ID,
    CHG_MSG_CHRG_FAULTS_ID,			
    CAN_MSG_ANALOG_MEAS_ID
};

/* 
    New TPDOS need to be added here.
    Update MC_MAX_TPDOS in mc_task.h when adding/removing.
*/
static const uint16_t mc_tpdo_ids[MC_MAX_TPDOS] = {
    MC_CAN_TPDO1_TID_TIQ_ID,
    MC_CAN_TPDO2_CB_THRV_VEL,

};

static status_t can_if_edma_init(void)
{
    status_t ret = STATUS_SUCCESS;
    
    edma_chn_state_t *const edma_channel_states[] = {
        &dma_ctrl_chn0_state,
        &dma_ctrl_chn1_state
    };

    edma_channel_config_t dm_chn0_cfg = {
        .channelPriority = EDMA_CHN_PRIORITY_1,
        .virtChnConfig = EDMA_CHN0_NUMBER,
        .source = EDMA_REQ_FLEXCAN0,
        .callback = NULL,
        .callbackParam = NULL,
        .enableTrigger = false
    };
    
    edma_channel_config_t dm_chn1_cfg = {
        .channelPriority = EDMA_CHN_PRIORITY_0,
        .virtChnConfig = EDMA_CHN1_NUMBER,
        .source = EDMA_REQ_FLEXCAN2,
        .callback = NULL,
        .callbackParam = NULL,
        .enableTrigger = false
    };
    
    const edma_channel_config_t *const edma_chn_cfg_arr[] = {
        &dm_chn0_cfg,
        &dm_chn1_cfg
    };

    edma_user_config_t dma_ctrl_init_cfg = {
        .chnArbitration = EDMA_ARBITRATION_FIXED_PRIORITY,
        .haltOnError = false
    };
    
    ret = EDMA_DRV_Init(&dma_ctrl_state, &dma_ctrl_init_cfg, edma_channel_states, edma_chn_cfg_arr, EDMA_CONFIGURED_CHANNELS_COUNT);  
    
    return ret;
}

static void can_fd_if_bms_init(void)
{
    flexcan_data_info_t data_info =
    {
       .data_length = CAN_FD_MAX_LEN,
       .msg_id_type = FLEXCAN_MSG_ID_EXT,
       .enable_brs  = false,
       .fd_enable   = true,
       .fd_padding  = 0U
    };
    
        (void)FLEXCAN_DRV_ConfigRxMb(CAN_IF_BMS, CAN_BMS_RX_MAILBOX, &data_info, BMS_VCU_SLOT0_COMM_ID);
        FLEXCAN_DRV_SetRxMbGlobalMask(CAN_IF_BMS, FLEXCAN_MSG_ID_EXT, BMS_VCU_MBX_GBL_MASK_FILTER);
        (void)FLEXCAN_DRV_SetRxIndividualMask(CAN_IF_BMS, FLEXCAN_MSG_ID_EXT, CAN_BMS_RX_MAILBOX, BMS_VCU_SLOT0_COMM_ID);
        
		FLEXCAN_DRV_InstallEventCallback(CAN_IF_BMS, bms_can_callback, NULL);
#ifndef USE_FEATURE_CAN_BUS_TIMEOUT
        can_bus_if_timeout_timers[CANFD_LOGICAL_BUS0] = osif_timer_create(canfd_bus0_timeout_handler, osTimerOnce, NULL, NULL);
#endif
//        bms_create_tx_timer();
 
}

static void dba_config_can(void)
{
    uint16_t id_counter = 0U;
    flexcan_id_table_t filterTable[8];    
    uint16_t abs_msg = 0U;
    
    flexcan_data_info_t data_info =
    {
       .data_length = 8U,
       .msg_id_type = FLEXCAN_MSG_ID_STD,
       .enable_brs  = false,
       .fd_enable   = false,
       .fd_padding  = 0U
    };
    
    (void)FLEXCAN_DRV_ConfigRxMb(CAN_IF_ABS, CAN_DBA_RX_MAILBOX, &data_info, RX_ABS_MSG_ID);
    for(abs_msg = 0; abs_msg < ABS_MAX_MSGS; abs_msg++)
    {
        (void)FLEXCAN_DRV_ConfigRxMb(CAN_IF_MOTOR, CAN_DBA_RX_MAILBOX + abs_msg, &data_info, abs_msg_ids[abs_msg]);
        (void)FLEXCAN_DRV_SetRxIndividualMask(CAN_IF_MOTOR, FLEXCAN_MSG_ID_STD, CAN_DBA_RX_MAILBOX + abs_msg, abs_msg_ids[abs_msg]);
    }
    
	FLEXCAN_DRV_SetRxMbGlobalMask(CAN_IF_ABS, FLEXCAN_MSG_ID_STD, DBA_CAN_IF_MBX_FILTER);
    
	/* Fill id filter table */
	/* Fill id filter table */
	for(id_counter = 0U; id_counter < 8U; id_counter++)
	{
		filterTable[id_counter].isRemoteFrame = false;
		filterTable[id_counter].isExtendedFrame = 0U;
		filterTable[id_counter].id = 0;
	}
    
    /* Fill id filter table */
	for(abs_msg = 0; abs_msg < ABS_MAX_MSGS; abs_msg++)
	{
		filterTable[abs_msg].isRemoteFrame = false;
		filterTable[abs_msg].isExtendedFrame = 0U;
		filterTable[abs_msg].id = abs_msg_ids[abs_msg];
	}
    
	/* Configure RX FIFO ID filter table elements based on filter table defined above*/
	FLEXCAN_DRV_ConfigRxFifo(CAN_IF_ABS, FLEXCAN_RX_FIFO_ID_FORMAT_A, filterTable);
    
	/* set individual masking type */
	FLEXCAN_DRV_SetRxMaskType(CAN_IF_ABS, FLEXCAN_RX_MASK_INDIVIDUAL);
    
	/* rest of filter items are masked with RXFGMASK */
	FLEXCAN_DRV_SetRxFifoGlobalMask(CAN_IF_ABS, FLEXCAN_MSG_ID_STD, DBA_CAN_IF_GBL_MBX_FILTER);
    
	/* set mask affecting MB10 */
    (void)FLEXCAN_DRV_SetRxIndividualMask(CAN_IF_ABS, FLEXCAN_MSG_ID_STD, CAN_DBA_RX_MAILBOX, DBA_CAN_IF_MBX_FILTER);
    
    FLEXCAN_DRV_InstallEventCallback(CAN_IF_ABS, dba1_can_callback, NULL);
}

static status_t mc_config_can_mbx_fifo(uint8_t instance, uint8_t mb_idx)
{
	status_t read_status = STATUS_SUCCESS;
    uint16_t id_counter = 0U;
    flexcan_id_table_t filterTable[8];
    uint16_t mc_tpdo = 0U;
    
  //  UNUSED_PARAM(buff);
    
	flexcan_data_info_t data_info =
    {
       .data_length = 8U,
       .msg_id_type = FLEXCAN_MSG_ID_STD,
       .enable_brs  = false,
       .fd_enable   = false,
       .fd_padding  = 0U
    };

	/* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
    for(mc_tpdo = 0U; mc_tpdo < MC_MAX_TPDOS; mc_tpdo++)
    {
        (void)FLEXCAN_DRV_ConfigRxMb(instance, (uint8_t)(mb_idx + mc_tpdo), &data_info, mc_tpdo_ids[mc_tpdo]);
        (void)FLEXCAN_DRV_SetRxIndividualMask(instance, FLEXCAN_MSG_ID_STD, (uint8_t)(mb_idx + mc_tpdo), mc_tpdo_ids[mc_tpdo]);
    }

	FLEXCAN_DRV_SetRxMbGlobalMask(instance, FLEXCAN_MSG_ID_STD, MOT_CONT_CAN_IF_MBX_FILTER);

	/* Fill id filter table */
	for(id_counter = 0U; id_counter < 8U; id_counter++)
	{
		filterTable[id_counter].isRemoteFrame = false;
		filterTable[id_counter].isExtendedFrame = 0U;
		filterTable[id_counter].id = 0;
	}
    
    /* Fill id filter table */
	for(mc_tpdo = 0; mc_tpdo < MC_MAX_TPDOS; mc_tpdo++)
	{
		filterTable[mc_tpdo].isRemoteFrame = false;
		filterTable[mc_tpdo].isExtendedFrame = 0U;
		filterTable[mc_tpdo].id = mc_tpdo_ids[mc_tpdo];
	}
    
	/* Configure RX FIFO ID filter table elements based on filter table defined above*/
	FLEXCAN_DRV_ConfigRxFifo(instance, FLEXCAN_RX_FIFO_ID_FORMAT_A, filterTable);
    
	/* set individual masking type */
	FLEXCAN_DRV_SetRxMaskType(instance, FLEXCAN_RX_MASK_INDIVIDUAL); 

	/* rest of filter items are masked with RXFGMASK */
	FLEXCAN_DRV_SetRxFifoGlobalMask(instance, FLEXCAN_MSG_ID_STD, MOT_CONT_CAN_IF_GBL_MBX_FILTER);

    FLEXCAN_DRV_InstallEventCallback(instance, mc1_can_callback, NULL);
    
	return read_status;
}

static status_t can_if_bms_init(void)
{
    status_t ret = STATUS_SUCCESS;
    
    flexcan_user_config_t can_bms_cfg = 
    {
        .fd_enable = true,
        .pe_clock = FLEXCAN_CLK_SOURCE_PERIPH,
        .max_num_mb = 4,
        .num_id_filters = FLEXCAN_RX_FIFO_ID_FILTERS_8,
        .is_rx_fifo_needed = false,
        .flexcanMode = FLEXCAN_NORMAL_MODE,
        .payload = FLEXCAN_PAYLOAD_SIZE_64,
        .bitrate = {
            .propSeg    = 3U,
            .phaseSeg1  = 1U,
            .phaseSeg2  = 2U,
            .preDivider = 3U,
            .rJumpwidth = 1U
        },
        .bitrate_cbt = {
            .propSeg    = 3U,
            .phaseSeg1  = 1U,
            .phaseSeg2  = 2U,
            .preDivider = 3U,
            .rJumpwidth = 1U
        },
        .transfer_type = FLEXCAN_RXFIFO_USING_INTERRUPTS,
        .rxFifoDMAChannel = 0U
    };

    PINS_DRV_SetPins(BMS_CAN_EN_GPIO, (1U << BMS_CAN_EN_PIN));
    PINS_DRV_SetPins(BMS_STB_N_GPIO, (1U << BMS_STB_N_PIN));
    
    ret += FLEXCAN_DRV_Init(CAN_IF_BMS, &can_bcu_state, &can_bms_cfg);
    
    can_fd_if_bms_init();
    
	INT_SYS_SetPriority(CAN1_ORed_0_15_MB_IRQn, 2U);
	INT_SYS_SetPriority(CAN1_ORed_16_31_MB_IRQn, 2U);
	INT_SYS_SetPriority(CAN1_ORed_IRQn, 2U);
    
    return ret;
}
static status_t can_if_mc_init(void)
{
    status_t ret = STATUS_SUCCESS;
    
    flexcan_user_config_t can_mc_cfg = 
    {
        .fd_enable = false,
        .pe_clock = FLEXCAN_CLK_SOURCE_PERIPH,
        .max_num_mb = 16,
        .num_id_filters = FLEXCAN_RX_FIFO_ID_FILTERS_8,
        .is_rx_fifo_needed = true,
        .flexcanMode = FLEXCAN_NORMAL_MODE,
        .payload = FLEXCAN_PAYLOAD_SIZE_8,
        
        .bitrate = {
                .propSeg    = 3U,
                .phaseSeg1  = 1U,
                .phaseSeg2  = 2U,
                .preDivider = 7U,
                .rJumpwidth = 1U
            },
                .bitrate_cbt = {
                .propSeg    = 3U,
                .phaseSeg1  = 1U,
                .phaseSeg2  = 2U,
                .preDivider = 7U,
                .rJumpwidth = 1U
            },
                
        .transfer_type = FLEXCAN_RXFIFO_USING_DMA,
        .rxFifoDMAChannel = 0U
    };

    PINS_DRV_SetPins(MOTOR_CAN_EN_GPIO, (1U << MOTOR_CAN_EN_PIN));
    PINS_DRV_SetPins(MOTOR_STB_N_GPIO, (1U << MOTOR_STB_N_PIN));
    
    ret = FLEXCAN_DRV_Init(CAN_IF_MOTOR, &can_mc_state, &can_mc_cfg);
    return ret;
}

static status_t can_if_abs_init(void)
{
	status_t init_status = STATUS_SUCCESS;

    flexcan_user_config_t can_abs_cfg = 
    {
        .fd_enable = false,
        .pe_clock = FLEXCAN_CLK_SOURCE_PERIPH,
        .max_num_mb = 12,
        .num_id_filters = FLEXCAN_RX_FIFO_ID_FILTERS_16,
        .is_rx_fifo_needed = true,
        .flexcanMode = FLEXCAN_NORMAL_MODE,
        .payload = FLEXCAN_PAYLOAD_SIZE_8,
        
        .bitrate = {
            .propSeg    = 3U,
            .phaseSeg1  = 10U,
            .phaseSeg2  = 7U,
            .preDivider = 9U,
            .rJumpwidth = 1U
        },
        
        .bitrate_cbt = {
            .propSeg    = 4U,
            .phaseSeg1  = 9U,
            .phaseSeg2  = 1U,
            .preDivider = 9U,
            .rJumpwidth = 1U
        },

        .transfer_type = FLEXCAN_RXFIFO_USING_DMA,
        .rxFifoDMAChannel = 1U
    };
    
    
    PINS_DRV_SetPins(DBA_CAN_EN_GPIO, (1U << DBA_CAN_EN_PIN));
    PINS_DRV_SetPins(DBA_STB_N_GPIO, (1U << DBA_STB_N_PIN));

    init_status = FLEXCAN_DRV_Init(CAN_IF_ABS, &can_abs_state, &can_abs_cfg);

	INT_SYS_SetPriority(CAN2_ORed_0_15_MB_IRQn, 0U);
	INT_SYS_SetPriority(CAN2_ORed_16_31_MB_IRQn, 0U);
	INT_SYS_SetPriority(CAN2_ORed_IRQn, 0U);
    
    return init_status;
}

/* Initialising the all system clocks  */
static void sys_clock_init(void)
{
    status_t ret = STATUS_SUCCESS;
    
    ret = CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    DEV_ASSERT(ret == STATUS_SUCCESS);
    
    ret = CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);   
    if(ret != STATUS_SUCCESS)
    {
        /* External XTAL failure. Internal OSC */
        ret = CLOCK_SYS_UpdateConfiguration(1U, CLOCK_MANAGER_POLICY_AGREEMENT);   
        DEV_ASSERT(ret == STATUS_SUCCESS);
    }
}

static void sys_lpspi2_init(void)
{
    status_t ret = STATUS_SUCCESS;
    lpspi_master_config_t SPI_MasterConfig;

    SPI_MasterConfig.bitsPerSec         = 1000000U;
    SPI_MasterConfig.whichPcs           = LPSPI_PCS0;
    SPI_MasterConfig.pcsPolarity        = LPSPI_ACTIVE_LOW;
    SPI_MasterConfig.isPcsContinuous    = false;
    SPI_MasterConfig.bitcount           = 32U;
    SPI_MasterConfig.lpspiSrcClk        = 48000000U;
    SPI_MasterConfig.clkPhase           = LPSPI_CLOCK_PHASE_2ND_EDGE;
    SPI_MasterConfig.clkPolarity        = LPSPI_SCK_ACTIVE_HIGH;
    SPI_MasterConfig.lsbFirst           = false;
    SPI_MasterConfig.transferType       = LPSPI_USING_INTERRUPTS;
    SPI_MasterConfig.rxDMAChannel       = 255U;
    SPI_MasterConfig.txDMAChannel       = 255U;
    SPI_MasterConfig.callback           = NULL;
    SPI_MasterConfig.callbackParam      = NULL; 
    
    INT_SYS_DisableIRQ(LPSPI2_IRQn);
    
    ret |= LPSPI_DRV_MasterInit(2U, &master_state_sja1105, &SPI_MasterConfig);
    ret |= LPSPI_DRV_MasterSetDelay(2U, 5U, 0U, 0U);
    ERR_NOTIFY(ret == STATUS_SUCCESS, SYS_ERR_PERIPH_INIT_LPSPI2);
}

static void rtc_pac1921_i2c_init(void)
{
	status_t ret = STATUS_SUCCESS;
    lpi2c_master_user_config_t lpi2c1_MasterConfig;
    
    lpi2c1_MasterConfig.slaveAddress    = RTC_I2C_ADDR;
    lpi2c1_MasterConfig.is10bitAddr     = false;
    lpi2c1_MasterConfig.operatingMode   = LPI2C_STANDARD_MODE;
    lpi2c1_MasterConfig.baudRate        = RTC_I2C_BAUD;
    lpi2c1_MasterConfig.transferType    = LPI2C_USING_INTERRUPTS;
    lpi2c1_MasterConfig.dmaChannel      = 0U;
    lpi2c1_MasterConfig.masterCallback  = NULL;
    lpi2c1_MasterConfig.callbackParam   = NULL;

    ret = LPI2C_DRV_MasterInit(RTC_I2C_IF, &lpi2c1_MasterConfig, &lpi2c1_master_state);
	ERR_NOTIFY(ret == STATUS_SUCCESS, SYS_ERR_PERIPH_INIT_LPI2C1);
    
    LPI2C_Set_MasterBusIdleTimeout(RTC_IF_INST, 10U);  
    LPI2C_Set_MasterTimeoutConfig(RTC_IF_INST, LPI2C_TIMEOUT_SCL_OR_SDA);
    LPI2C_Set_MasterPinLowTimeout(RTC_IF_INST, 5U);
    
    INT_SYS_SetPriority(LPI2C0_Master_IRQn, 0U);
}

static void enable_port_clocks(void)
{
    CLOCK_DRV_SetModuleClock(PORTA_CLK, NULL);
    CLOCK_DRV_SetModuleClock(PORTB_CLK, NULL);
    CLOCK_DRV_SetModuleClock(PORTC_CLK, NULL);
    CLOCK_DRV_SetModuleClock(PORTD_CLK, NULL);
    CLOCK_DRV_SetModuleClock(PORTE_CLK, NULL);
}

static status_t init_crypto(void)
{
    status_t stat;
    
    CSEC_DRV_Init(&csec_state);
    stat = CSEC_DRV_InitRNG();
    
    return stat;
}

static void sys_lpspi0_init(void)
{
    status_t ret = STATUS_SUCCESS;
    
    lpspi_master_config_t SPI_MasterConfig = {0};
        
    SPI_MasterConfig.bitsPerSec         = 1000000U;
    SPI_MasterConfig.whichPcs           = LPSPI_PCS0;
    SPI_MasterConfig.pcsPolarity        = LPSPI_ACTIVE_LOW;
    SPI_MasterConfig.isPcsContinuous    = true;
    SPI_MasterConfig.bitcount           = 32U;
    SPI_MasterConfig.lpspiSrcClk        = 48000000U;
    SPI_MasterConfig.clkPhase           = LPSPI_CLOCK_PHASE_1ST_EDGE;
    SPI_MasterConfig.clkPolarity        = LPSPI_SCK_ACTIVE_HIGH;

    SPI_MasterConfig.lsbFirst           = false;
    SPI_MasterConfig.transferType       = LPSPI_USING_INTERRUPTS;
    SPI_MasterConfig.rxDMAChannel       = 255;
    SPI_MasterConfig.txDMAChannel       = 255;
    SPI_MasterConfig.callback           = NULL;
    SPI_MasterConfig.callbackParam      = NULL; 
   
    INT_SYS_DisableIRQ(LPSPI0_IRQn);
    ret |= LPSPI_DRV_MasterInit(0U, &master_state_cd1030, &SPI_MasterConfig);
    ret |= LPSPI_DRV_MasterSetDelay(0U, 5U, 0U, 0U);
    ERR_NOTIFY(ret == STATUS_SUCCESS, SYS_ERR_PERIPH_INIT_LPSPI0);
}

static void sys_lpspi1_init(void)
{
    status_t ret = STATUS_SUCCESS;
    lpspi_master_config_t SPI_MasterConfig;
        
    SPI_MasterConfig.bitsPerSec         = 1000000U;
    SPI_MasterConfig.whichPcs           = LPSPI_PCS0;
    SPI_MasterConfig.pcsPolarity        = LPSPI_ACTIVE_LOW;
    SPI_MasterConfig.isPcsContinuous    = false;
    SPI_MasterConfig.bitcount           = 8U;
    SPI_MasterConfig.lpspiSrcClk        = 48000000U;
    SPI_MasterConfig.clkPhase           = LPSPI_CLOCK_PHASE_2ND_EDGE;
    SPI_MasterConfig.clkPolarity        = LPSPI_SCK_ACTIVE_LOW;
    SPI_MasterConfig.lsbFirst           = false;
    SPI_MasterConfig.transferType       = LPSPI_USING_INTERRUPTS;
    SPI_MasterConfig.rxDMAChannel       = 255U;
    SPI_MasterConfig.txDMAChannel       = 255U;
    SPI_MasterConfig.callback           = NULL;
    SPI_MasterConfig.callbackParam      = NULL; 

    INT_SYS_DisableIRQ(LPSPI1_IRQn);
    
    ret |= LPSPI_DRV_MasterInit(1U, &master_state_icm30948, &SPI_MasterConfig);
    ret |= LPSPI_DRV_MasterSetDelay(1U, 1U, 0U, 0U);
	ERR_NOTIFY(ret == STATUS_SUCCESS, SYS_ERR_PERIPH_INIT_LPSPI1);
}

static status_t ec25_uart_config(void)
{
    status_t ret = STATUS_SUCCESS;
    
    lpuart_user_config_t lpuart1_cfg;
    
    lpuart1_cfg.transferType    = LPUART_USING_INTERRUPTS;
    lpuart1_cfg.baudRate        = 115200;
    lpuart1_cfg.parityMode      = LPUART_PARITY_DISABLED;
    lpuart1_cfg.stopBitCount    = LPUART_ONE_STOP_BIT;
    lpuart1_cfg.bitCountPerChar = LPUART_8_BITS_PER_CHAR;
    lpuart1_cfg.rxDMAChannel    = 0U;
    lpuart1_cfg.txDMAChannel    = 0U;
    
    ret = LPUART_DRV_Init(1U, &lpuart0_state, &lpuart1_cfg);
    
    return ret;
}

#ifdef USE_FEAT_EC25_TURN_ON_CHECK
static void wait_ec25_turnon(void)
{
    volatile uint32_t pins_gpiod = 0U; 
    volatile uint32_t timeout = 0U;
    
    pins_gpiod = PINS_DRV_ReadPins(LTE_IRQ_GPIO);
    
    while((pins_gpiod & (1U << LTE_IRQ_PIN)) == 0U)
    {
        sw_asm_delay_us(10000);        
        pins_gpiod = PINS_DRV_ReadPins(LTE_IRQ_GPIO);
    }
}
#endif /* USE_FEAT_EC25_TURN_ON_CHECK */

static void bcm_horn_diag_en(void)
{
    PINS_DRV_SetPins(DEN_GPIO, (1U << DEN_PIN));
}

void ec25_pwr_init(void)
{
    PINS_DRV_SetPins(VBAT_EN1_GPIO, (1U << VBAT_EN1_PIN));
    
    sw_asm_delay_us(100000U);
    PINS_DRV_ClearPins(RESET_MCU_GPIO, (1U << RESET_MCU_PIN));
    sw_asm_delay_us(100000U);
    
    /* Minimum spec. according to Quectel is 100ms or more */
    PINS_DRV_SetPins(ON_OFF_MCU_GPIO, (1U << ON_OFF_MCU_PIN));
    
    sw_asm_delay_us(150000U);
    
    PINS_DRV_ClearPins(ON_OFF_MCU_GPIO, (1U << ON_OFF_MCU_PIN));
}

msdi_status_t init_CD10x0(void)
{
    msdi_init_config_t init_cfg;
    volatile msdi_status_t status = MSDI_STATUS_SUCCESS;
    msdi_amux_cfg_t amux_cfg = { MSDI_AMUX_ASETT0_HIZ, MSDI_AMUX_ASEL_CD1030_NO_INPUT };

    msdi_drv_cfg.drvInstance = 0;
    msdi_drv_cfg.deviceType = MSDI_DEVICE_CD1030;

    MSDI_GetDefaultInitConfig(&msdi_drv_cfg, &init_cfg);

    init_cfg.triStateSg =
            MSDI_TRI_STATE_DIS(0) |
            MSDI_TRI_STATE_DIS(1) |
            MSDI_TRI_STATE_DIS(2) |
            MSDI_TRI_STATE_DIS(3) |
            MSDI_TRI_STATE_DIS(4) |
            MSDI_TRI_STATE_DIS(5) |
            MSDI_TRI_STATE_DIS(6) |
            MSDI_TRI_STATE_DIS(7) |
            MSDI_TRI_STATE_DIS(8) |
            MSDI_TRI_STATE_DIS(9) |
            MSDI_TRI_STATE_DIS(10) |
            MSDI_TRI_STATE_DIS(11) |
            MSDI_TRI_STATE_DIS(12) |
            MSDI_TRI_STATE_DIS(13);

    init_cfg.triStateSp =
            MSDI_TRI_STATE_DIS(0) |
            MSDI_TRI_STATE_EN(1)  |
            MSDI_TRI_STATE_DIS(2) |
            MSDI_TRI_STATE_DIS(3) |
            MSDI_TRI_STATE_DIS(4) |
            MSDI_TRI_STATE_DIS(5) |
            MSDI_TRI_STATE_DIS(6) |
            MSDI_TRI_STATE_EN(7)  |
            MSDI_TRI_STATE_DIS(8) |
            MSDI_TRI_STATE_EN(11);

    init_cfg.devConfig.spx =
            MSDI_CFG_SPX_BAT(0) |
            MSDI_CFG_SPX_BAT(1) |
            MSDI_CFG_SPX_BAT(2) |
            MSDI_CFG_SPX_BAT(3) |
            MSDI_CFG_SPX_BAT(4) |
            MSDI_CFG_SPX_BAT(5) |
            MSDI_CFG_SPX_BAT(6) |
            MSDI_CFG_SPX_BAT(7) |
            MSDI_CFG_SPX_BAT(10);

    (void)MSDI_ExitLPM(&msdi_drv_cfg);

    status |= MSDI_Init(&msdi_drv_cfg, &init_cfg);
    status |= MSDI_SetAMUXChannel(&msdi_drv_cfg, &amux_cfg);

    return status;
}

#ifdef USE_FEATURE_FAST_TURN_ON
static void board_pwr_on(void)
{
    bcm_control(BCM_AUX_CTRL, BCM_CTRL_STATE_ON);
    PINS_DRV_SetPins(AUX_5V_EN_GPIO, (1U << AUX_5V_EN_PIN));
    
    /* Enable power supply for i.MX */
    PINS_DRV_SetPins(IMX6_VDD_EN_GPIO, (1U << IMX6_VDD_EN_PIN));
    PINS_DRV_ClearPins(IMX6_PMIC_EN_GPIO, (1U << IMX6_PMIC_EN_PIN));

    /* Release reset for I.MX6 */
    PINS_DRV_ClearPins(IMX6_RESET_GPIO, (1U << IMX6_RESET_PIN));
    PINS_DRV_SetPins(MODE_GPIO_SW2_GPIO, (1U << MODE_GPIO_SW2_PIN));
    
    bcm_control(BCM_MCU_EN_CTRL, BCM_CTRL_STATE_ON);
}
#endif /* USE_FEATURE_FAST_TURN_ON */

status_t board_init(void)
{
    status_t init_status = STATUS_SUCCESS;
    
    INT_SYS_DisableIRQGlobal();

    sys_clock_init();
    SystemCoreClockUpdate();
    
    /* 
        Pin initialization only done in bootloader 
        init_status = PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
        DEV_ASSERT(init_status == STATUS_SUCCESS);
    
    */
    
    enable_port_clocks();
    
#ifdef USE_FEATURE_FAST_TURN_ON
    /* Turn ON EC25, i.MX */
    board_pwr_on();
    ec25_pwr_init();
#endif
    
    throttle_adc_init();
    init_status += init_crypto();
    
#ifdef USE_FEATURE_MPU
    mpu_config();
#endif 

#if 0
    /* If bootloader supports USE_FEATURE_FAST_WAKE */
    /* Undo bootloader configuration */
    (void)FLEXCAN_DRV_Deinit(CAN_IF_BMS);
#endif

    INT_SYS_EnableIRQGlobal();
    
    init_status += can_if_edma_init();
	init_status += can_if_mc_init();
    init_status += can_if_abs_init();
    init_status += can_if_bms_init();
//    init_status += ec25_uart_config();
    DEV_ASSERT(init_status == STATUS_SUCCESS);
    
	dba_config_can();
	(void)mc_config_can_mbx_fifo(CAN_IF_MOTOR, RX_MAILBOX);
	sys_lpspi2_init();
	sys_lpspi0_init();
    sys_lpspi1_init();
    rtc_pac1921_i2c_init();
	loadswitch_adc_init();
    
#ifndef USE_DEBUG_PRINTS
    DbgConsole_Init(SYS_DEBUG_LPUART_INTERFACE, SYS_DEBUG_LPUART_BAUD, DEBUG_CONSOLE_DEVICE_TYPE_LPUART);
#endif

    drv_led_init();
    bcm_horn_diag_en();
   
    return init_status;
}
