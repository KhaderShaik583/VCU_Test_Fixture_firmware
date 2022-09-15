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
 
#include <string.h> 
#include <stdlib.h>

#include "pins_driver.h"
#include "udp_task.h"
#include "osif.h"
#include "rtx_os.h"
#include "eth_task.h"
#include "shared_mem.h" 
#include "dba_task.h"
#include "flash_driver.h" 
#include "bcm.h"
#include "nvm.h"
#include "version.h"
#include "init_task.h" 
#include "odometer_task.h"
#include "bms_task.h"
#include "ringbuff.h" 
#include "bms_can_if.h" 
#include "mc_task.h"
#include "fota.h" 
#include "can_messenger_rx.h"
#include "wdt_task.h"
#include "swif_task.h"
#include "lsm_task.h"
#include "NXP_SJA1105P_resetGenerationUnit.h"
#include "NXP_SJA1105P_spi.h"
#include "common_utils.h"

typedef struct
{
	int32_t block_ofs;
	int32_t block_len;
    uint32_t crc;
}fw_header_info_t;

#define S32_IMX_UDP_PORT    (5556U)
#define S32_LTE_UDP_PORT    (5575U)
#define MAX_UDP_BUFFER_SZ   (ETH_MAX_FRAMELEN)

#define MSGQUEUE_OBJECTS (16)

#define VCU_FW_IDENT    (0x56U)
#define BMS_FW_IDENT    (0xB7U)

#define TNetConn struct netconn *
#define TNetBuf  struct netbuf  *

#if LWIP_NETCONN

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t udp_thread_imx_tcb;
#else
static thread_tcb_t udp_thread_imx_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

__attribute__((section("ARM_LIB_STACK")))
static uint64_t udp_thread_imx_stk[UDP_IMX_TASK_STACK_SIZE];

static thread_id_t thread_udp_imx;

static shmem_block_bms_t udp_resp;
static uint32_t imu_stall = 0U;
static osif_timer_id_t ethernet_link_up_timer;
static osif_timer_id_t ota_link_timer;
static uint32_t sim_det = 0U;
static uint32_t link_active = 0U;
static const uint32_t OTA_LINK_TIMER_TIMEOUT = 8000U;

static udp_rx_msg_t *umsg = NULL;
static uint32_t has_mc_tm_changed = 0U;
static volatile int32_t num_blocks = 0;
static volatile int32_t rx_blocks = 0;
static uint32_t cluster_state = 0U;

static const uint32_t LINK_UP_TIMER_TIMEOUT = 75000U;

static const thread_attrs_t udp_task_attr = {
    UDP_IMX_TASK_NAME,
    osThreadDetached,
    &udp_thread_imx_tcb,
    sizeof(udp_thread_imx_tcb),
    &udp_thread_imx_stk[0],
    UDP_IMX_TASK_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal1,
    0U,
    0U    
};
    
static TNetBuf rx_net_imx_buf;
static TNetBuf tx_net_imx_buf;
        
static TNetBuf rx_net_lte_buf;
static TNetBuf tx_net_lte_buf;

static volatile uint32_t mscm_ocmdr0_save = 0U;
static volatile uint32_t mscm_ocmdr1_save = 0U;
static volatile uint32_t ota_base_address = 0U;
static volatile uint32_t is_bms_fw = 0U;

static volatile uint32_t boot_fw_upd_sentinel = 0xFADEDEADU;
static volatile uint32_t boot_fw_upd_fail_sentinel = 0xDEADFADEU;

static flash_ssd_config_t flash_ssd_config;
static const flash_user_config_t flash_init_config = {
    .PFlashBase  = 0x00000000U,
    .PFlashSize  = 0x180000U,
    .DFlashBase  = 0x10000000U,
    .EERAMBase   = 0x14000000U,
    /* If using callback, any code reachable from this function must not be placed in a Flash block targeted for a program/erase operation.*/
    .CallBack    = NULL_CALLBACK
};

osif_msg_queue_id_t udp_msg_queue; 

static void ota_eth_switch_init(void)
{
    uint32_t prod_id[1] = {0x00000000U};
    volatile uint8_t sw_ret = 0U;
    
    SJA1105P_prodIdArgument_t sw_prod_id;

    /* Cold reset Switch */
    (void)SJA1105P_setResetCtrl(0x0004U, 1U);
    sw_asm_delay_us(100000U);
    
    /* Enable ACU register access */
    (void)SJA1105P_gpf_spiWrite32(1U, 1U, (uint32_t)0x100Bfd, prod_id);
    sw_asm_delay_us(500);
    
    /* Read the production ID -> Must read 0x9A86 */
    (void)SJA1105P_getProdId(&sw_prod_id, 1U);
    sw_asm_delay_us(500U);

    /* Initialize the switch */
    sw_ret += SJA1105P_initSwitch();
    sw_asm_delay_us(500U);
}

static void process_LTE_S32_UDP_FILE_UPD_START(udp_rx_msg_t *umesg)
{
    UNUSED_PARAM(umesg);
    
    set_status_bit(STAT_VCU_LOG_UPLOAD_RUNNING);
}

static void process_LTE_S32_UDP_FILE_UPD_STOP(udp_rx_msg_t *umesg)
{
    UNUSED_PARAM(umesg);
    
    clear_status_bit(STAT_VCU_LOG_UPLOAD_RUNNING);
}

static void process_LTE_S32_UDP_FILE_DL_STOP(const udp_rx_msg_t *umesg)
{
    sys_msg_queue_obj_t smq;
    
    UNUSED_PARAM(umesg);
    
    /* Once the file has been downloaded on the EC25 this command will be initiated */
    clear_status_bit(STAT_VCU_LOG_UPLOAD_RUNNING);
    if(abs_is_vehicle_stationary() == 1U)
    {        
        set_status_bit(STAT_VCU_FW_UPD_READY);  
        clear_status_bit(STAT_VCU_LOG_UPLOAD_RUNNING);
        
        smq.msg_id = MSG_ID_REBOOT_REQ;
        smq.msg_len = 0U;
        (void)osif_msg_queue_send(sys_msg_queue, &smq, 0U, 0U); 
    }
}

static void process_LTE_S32_UDP_MSG_FW_DL_START_NTF(const udp_rx_msg_t *umesg)
{
    status_t s = STATUS_SUCCESS;
    dba_msg_queue_obj_t dmq;
    udp_msg_queue_obj_t qmsg;
    bms_msg_queue_obj_t bmsg;
    
    volatile uint32_t fw_update_sentinel = 0U;
    int32_t lc = 0;
    int32_t n = 0;
    
    UNUSED_PARAM(umesg);
    
    set_status_bit(STAT_VCU_FW_DL_RUNNING);
    
    /* BMS data stream OFF & Motor & inhibit key control */
    bmsg.msg_id = BMS_MSG_KEEP_ALIVE_NO_DATA;
    bmsg.data = 0U;
    (void)osif_msg_queue_send(bms_msg_queue, &bmsg, 0U, 10U);
    
    INT_SYS_DisableIRQ(PORTC_IRQn);
    
    mc_set_gear(MC_GEAR_POS_NEUT_OFF);
    
    lc = osif_enter_critical();
    ota_eth_switch_init();
    (void)osif_exit_critical(lc);

#ifdef S32K148_SERIES
    mscm_ocmdr0_save = MSCM->OCMDR[0u];
    mscm_ocmdr1_save = MSCM->OCMDR[1u];
    
    MSCM->OCMDR[0u] |= MSCM_OCMDR_OCM1(0x3u);
    MSCM->OCMDR[1u] |= MSCM_OCMDR_OCM1(0x3u);
#endif /* S32K148_SERIES */
    
    imu_stall = 1U;
    
    nvm_read(FILE_SECT_BOOT_CONFIGURATION + FILE_SECT_BOOT_FW_UPD_OFFSET, (uint8_t *)&fw_update_sentinel, sizeof(uint32_t));

    if(umsg->len > 0U)
    {        
		num_blocks = (int32_t)(umsg->buffer[1] | ((uint32_t)umsg->buffer[2] << 8U) | ((uint32_t)umsg->buffer[3] << 16U) | ((uint32_t)umsg->buffer[4] << 24U));
        
        if(umsg->buffer[0] == BMS_FW_IDENT)
        {
            s =  FLASH_DRV_EraseSector(&flash_ssd_config, S32_BMS_FOTA_DL_ADDR, S32_BMS_FOTA_DL_SIZE);
            is_bms_fw = 1U;
            ota_base_address = S32_BMS_FOTA_DL_ADDR;
        }
        else
        {
            s =  FLASH_DRV_EraseSector(&flash_ssd_config, S32_VCU_FOTA_DL_ADDR, S32_VCU_FOTA_DL_SIZE);
            ota_base_address = S32_VCU_FOTA_DL_ADDR;
        }
    }
    
    DEV_ASSERT(STATUS_SUCCESS == s);
        
    dmq.msg_id = DBA_SEND_DISPLAY_MSG;
    dmq.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
    dmq.data[1] = (uint8_t)TT_SERVICE_INDICATOR;
    dmq.data[2] = DBA_DISPLAY_TELL_TALE_ON;
    (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);

    udp_resp.udp_msg.cmd = MSG_ID_DL_RESUME_S32_LTE;
    n = snprintf((char *)&udp_resp.buffer[0], SHMEM_BMS_BLK_SIZE, "OTA Last update state -> 0x%x\n", fw_update_sentinel);
    /* snprintf does not count null so +1 */
    udp_resp.udp_msg.len = (uint32_t)((n < 0) ? 0 : (n + 1));
    
    qmsg.msg = (void *)&udp_resp;
    qmsg.msg_id = udp_resp.udp_msg.cmd;
    qmsg.msg_len = udp_resp.udp_msg.len;
    (void)osif_msg_queue_send(udp_msg_queue, &qmsg, 0U, 10U);
    
    set_status_bit(STAT_VCU_FW_DL_RUNNING);
    (void)osif_timer_start(ota_link_timer, MSEC_TO_TICK(OTA_LINK_TIMER_TIMEOUT)); 
}

static void process_LTE_S32_UDP_MSG_FW_DL(const udp_rx_msg_t *umesg)
{
    volatile status_t s = STATUS_SUCCESS;
    volatile fw_header_info_t *fw_info = NULL;
    static volatile uint32_t prev_block = 0xFFFFFFFFU;
    volatile uint32_t current_block = 0U;
    volatile uint32_t crc = 0U;
    udp_msg_queue_obj_t qmsg;
    int32_t lc = 0;
    
    UNUSED_PARAM(umesg);

    lc = osif_enter_critical();
    fw_info = (fw_header_info_t *)umsg->buffer;
    current_block = (uint32_t)fw_info->block_ofs;
    
    crc = crc32(umsg->buffer + sizeof(fw_header_info_t), umsg->len);
    udp_resp.udp_msg.cmd = MSG_ID_DL_RESUME_S32_LTE;
    udp_resp.udp_msg.len = 1U;
    
    if((crc == fw_info->crc) &&
       (umsg->len > 0U))
    {
        s = FLASH_DRV_Program(&flash_ssd_config, ota_base_address + current_block, umsg->len, umsg->buffer + sizeof(fw_header_info_t));
        DEV_ASSERT(s == STATUS_SUCCESS);
        
        if(current_block != prev_block)
        {
            rx_blocks++;
        }
        
        prev_block = current_block;
        udp_resp.buffer[0] = 0x5A;
    }
    else
    {
        udp_resp.buffer[0] = 0x7C;
        osif_time_delay(500U);
        
        __NOP();
    }
        
    qmsg.msg = (void *)&udp_resp;
    qmsg.msg_id = udp_resp.udp_msg.cmd;
    qmsg.msg_len = udp_resp.udp_msg.len;
    (void)osif_msg_queue_send(udp_msg_queue, &qmsg, 0U, 1U);

    (void)osif_exit_critical(lc);

    (void)osif_timer_stop(ota_link_timer); 
    (void)osif_timer_start(ota_link_timer, MSEC_TO_TICK(OTA_LINK_TIMER_TIMEOUT));
}

static void process_LTE_S32_UDP_MSG_FW_DL_DONE(const udp_rx_msg_t *umesg)
{
    dba_msg_queue_obj_t dmq;
    sys_msg_queue_obj_t smq;
    bms_msg_queue_obj_t bmsg;
    int32_t lc = 0;
    
    UNUSED_PARAM(umesg);

    (void)osif_timer_stop(ota_link_timer);
    
    if(is_bms_fw == 0U)
    {
        dmq.msg_id = DBA_SEND_DISPLAY_MSG;
        dmq.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
        dmq.data[1] = TT_SERVICE_INDICATOR;
        dmq.data[2] = DBA_DISPLAY_TELL_TALE_OFF;
        (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
        
        NVIC_ClearPendingIRQ(PORTC_IRQn);
        INT_SYS_EnableIRQ(PORTC_IRQn);
        
        ota_base_address = 0U;
        
        lc = osif_enter_critical();
        (void)nvm_write(FILE_SECT_BOOT_CONFIGURATION + FILE_SECT_BOOT_FW_UPD_OFFSET, (uint8_t *)&boot_fw_upd_sentinel, 4U);
        (void)osif_exit_critical(lc);

        if(rx_blocks != num_blocks)
        {
            dmq.msg_id = DBA_SEND_DISPLAY_MSG;
            dmq.data[0] = DBA_DISPLAY_TELL_TALE_CTRL_MSG;
            dmq.data[1] = TT_ERROR_INDICATOR;
            dmq.data[2] = DBA_DISPLAY_TELL_TALE_ON;
            (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U); 
            
            lc = osif_enter_critical();
            (void)nvm_write(FILE_SECT_BOOT_CONFIGURATION + FILE_SECT_BOOT_FW_UPD_OFFSET, (uint8_t *)&boot_fw_upd_fail_sentinel, 4U);
            (void)osif_exit_critical(lc);
        }
        else
        {
            /* Clear the sentinel for torque maps. At next boot the NVM gets reloaded with the 
               values in the flash image.
            */
            mc_factory_reset_ride_tmaps();
        }

#ifdef S32K148_SERIES
        MSCM->OCMDR[0u] = mscm_ocmdr0_save;
        MSCM->OCMDR[1u] = mscm_ocmdr1_save;
#endif /* S32K148_SERIES */
        
        __DMB();        
        imu_stall = 0U;
            
        clear_status_bit(STAT_VCU_FW_DL_RUNNING); 
        sw_asm_delay_us(5000000U); 
        
        smq.msg_id = MSG_ID_REBOOT_REQ;
        smq.msg_len = 0U;
        (void)osif_msg_queue_send(sys_msg_queue, &smq, 0U, 10U);  
    }
    else
    {

#ifdef S32K148_SERIES
        MSCM->OCMDR[0u] = mscm_ocmdr0_save;
        MSCM->OCMDR[1u] = mscm_ocmdr1_save;
#endif /* S32K148_SERIES */
        
        /* At this point BMS fw is on VCU flash */
        clear_status_bit(STAT_VCU_FW_DL_RUNNING); 
        sw_asm_delay_us(3000000U); 
        
        bmsg.msg_id = BMS_MSG_POST_FW_UPD_EVENT;
        bmsg.data = 0U;
        (void)osif_msg_queue_send(bms_msg_queue, &bmsg, 0U, 0U); 
    }
}

static void process_LTE_S32_UDP_MSG_FW_VER_INFO(const udp_rx_msg_t *umesg)
{
    int32_t lc = 0;
    int32_t n = 0;
    volatile uint32_t fw_update_state = 0U;
    volatile uint8_t rcm_l = 0U;
    volatile uint8_t rcm_h = 0U;
    
    UNUSED_PARAM(umesg);
    
    volatile udp_msg_queue_obj_t qmsg;
    balancer_run_info_t bal_info;
    os_nvm_err_t os_err_info;
    
    if(umsg->len > 0U)
    {
        if(umsg->buffer[0] == 0U)
        {
            sim_det = 1U;
        }
    }
    
    bms_get_balancer_info(&bal_info);
    
    udp_resp.udp_msg.cmd = MSG_ID_FW_VER_S32_LTE;
    
    lc = osif_enter_critical();
    nvm_read(FILE_SECT_BOOT_CONFIGURATION + FILE_SECT_BOOT_FW_UPD_OFFSET, (uint8_t *)&fw_update_state, sizeof(uint32_t));
    nvm_read(FILE_SECT_OS_ERR_CTXT, (uint8_t *)&os_err_info, sizeof(os_nvm_err_t));
    if(os_err_info.os_err_code == 0xFFU)
    {
        /* This will happen for a factory boot as initial values in FlexNVM as all FFs */
        for(uint32_t i = 0U; i < 32U; i++)
        {
            os_err_info.thread[i] = '\0';
        }
    }
    
    rcm_l = (uint8_t)(RCM->SRS & 0x000000FFU);
    rcm_h = (uint8_t)((RCM->SRS & 0x0000FF00U) >> 8U);
    
    /* snprintf does not count null so +1 */
    /* Also piggybacking some additional info as there is a ton of space in the buffer */
    n = snprintf((char *)&udp_resp.buffer[0], SHMEM_BMS_BLK_SIZE, 
                                                   "VCU FW: v%d.%d. %s, %s\nBMS FW: %s\nBMS BLIN: %d,%d,%d\n"   \
                                                   "F/W Sentinel: 0x%x\nVCU RCM-H:%d, RCM-L:%d\nException:%s,%d,%x\n",
                                                    get_fw_max_ver(), 
                                                    get_fw_min_ver(), 
                                                    get_fw_date(), 
                                                    get_fw_timestamp(), 
                                                    bms_get_fw_version(),
                                                    bal_info.balancer_run_count,
                                                    bal_info.balancer_abort_cause,
                                                    bal_info.balancer_time_ticks,
                                                    fw_update_state,
                                                    rcm_h,
                                                    rcm_l,
                                                    os_err_info.thread,
                                                    os_err_info.os_err_code,
                                                    os_err_info.os_obj_id);
     
    udp_resp.udp_msg.len = (uint32_t)((n < 0) ? 0 : (n + 1));
    (void)osif_exit_critical(lc);
    
    qmsg.msg = (void *)&udp_resp;
    qmsg.msg_id = MSG_ID_FW_VER_S32_LTE;
    qmsg.msg_len = udp_resp.udp_msg.len; 
        
    (void)osif_msg_queue_send(udp_msg_queue, (void *)&qmsg, 0U, 5U);
    
    /* Clear sector to prevent false alarms */
    (void)memset((uint8_t *)&os_err_info, 0, sizeof(os_nvm_err_t));
    nvm_write(FILE_SECT_OS_ERR_CTXT, (uint8_t *)&os_err_info, sizeof(os_nvm_err_t));
    
    osif_time_delay(100);
}

static void process_LTE_S32_UDP_MSG_ODO_INFO(const udp_rx_msg_t *umesg)
{
    volatile udp_msg_queue_obj_t qmsg;
    int32_t odo_info_len = 0;
    UNUSED_PARAM(umesg);
    
    udp_resp.udp_msg.cmd = MSG_ID_ODO_INFO_S32_LTE;
    odo_info_len = odo_get_info(&udp_resp.buffer[0]);
    if(odo_info_len < 0)
    {
        udp_resp.buffer[0] = 'O';
        udp_resp.buffer[1] = 'D';
        udp_resp.buffer[2] = 'O';
        udp_resp.buffer[3] = '-';
        udp_resp.buffer[4] = 'E';
        udp_resp.buffer[5] = 'R';
        udp_resp.buffer[6] = 'R';
        udp_resp.buffer[7] = 'O';
        udp_resp.buffer[8] = 'R';
        udp_resp.buffer[9] = '\0';

        udp_resp.udp_msg.len = 10U;
    }
    else
    {
        udp_resp.udp_msg.len = (uint32_t)odo_info_len;
    }    
    
    qmsg.msg = (void *)&udp_resp;
    qmsg.msg_id = MSG_ID_ODO_INFO_S32_LTE;
    qmsg.msg_len = udp_resp.udp_msg.len;
        
    (void)osif_msg_queue_send(udp_msg_queue, (void *)&qmsg, 0U, 5U);  
}

static void process_LTE_S32_UDP_ODO_RESET(const udp_rx_msg_t *umesg)
{
    int32_t lc = 0;
    
    UNUSED_PARAM(umesg);
    
    if((abs_is_vehicle_stationary() == 1U) &&
       ((get_status() & ((uint64_t)1U << (uint32_t)STAT_VCU_MOTOR_CON_DIR_FWD)) ==  0U) &&
       ((get_status() & ((uint64_t)1U << (uint32_t)STAT_VCU_MOTOR_CON_DIR_REV)) ==  0U))
    {
        lc = osif_enter_critical();
        odo_reset();
        (void)osif_exit_critical(lc);
    }
}

static void process_LTE_S32_UDP_CALIB_IMU(const udp_rx_msg_t *umesg)
{
    imu_msg_queue_obj_t imq;
    UNUSED_PARAM(umesg);
    
    if((abs_is_vehicle_stationary() == 1U) &&
       ((get_status() & ((uint64_t)1U << (uint32_t)STAT_VCU_MOTOR_CON_DIR_FWD)) ==  0U) &&
       ((get_status() & ((uint64_t)1U << (uint32_t)STAT_VCU_MOTOR_CON_DIR_REV)) ==  0U))
    {
       imq.msg_id = IMU_MSG_CAL_START;
       imq.data = 0x0fU;
       (void)osif_msg_queue_send(imu_msg_queue, &imq, 0U, 10U);
    }
}    

static void process_LTE_S32_UDP_SET_TIME(const udp_rx_msg_t *umesg)
{
    int32_t lc = 0;
    datetime_t set_rtc_time_dec;
    static uint32_t update_in_progress = 0U;
    
    DEV_ASSERT(umesg != NULL);
    
    if((umesg->len == 6U) && (update_in_progress == 0U))
    {
        lc = osif_enter_critical();
        
        update_in_progress = 1U;
        set_rtc_time_dec.valid = 1U;
        set_rtc_time_dec.hour = umesg->buffer[0];
        set_rtc_time_dec.minute = umesg->buffer[1];
        set_rtc_time_dec.second = 1U + umesg->buffer[2]; /* account for the update time */
        set_rtc_time_dec.month = umesg->buffer[3];
        set_rtc_time_dec.year = (uint8_t)((1900U + umesg->buffer[4]) - 2000U);
        set_rtc_time_dec.weekday = umesg->buffer[5];
        set_rtc_time_dec.day = umesg->buffer[5];
        
        rtc_set_date_time_nw(&set_rtc_time_dec);
        (void)osif_exit_critical(lc);
        
        update_in_progress = 0U;
    }
}

static void process_time_cmd(const udp_rx_msg_t *umesg)
{
    datetime_t date_time;
    int32_t lc = 0;
    int32_t i = 0;
    char *csv_fields[8] = {0};
    char *token = NULL;
    const char *format = "dd,mm,yy,hh,mm,ss,wd";
    
    DEV_ASSERT(umesg != NULL);
    
    volatile uint32_t cmd_len = strlen((const char *)umesg->buffer);
    const uint32_t exp_len = strlen(format) + strlen("--s32_set_time") + 1U;

    if(cmd_len == exp_len)
    {
        lc = osif_enter_critical();
        token = strtok((char *)umesg->buffer, ",");
        while(token != NULL)
        {
            csv_fields[i] = token;
            i++;
            token = strtok(NULL, ",");
        }

        date_time.day       = (uint8_t)strtol(csv_fields[1], NULL, 10);
        date_time.month     = (uint8_t)strtol(csv_fields[2], NULL, 10);
        date_time.year      = (uint8_t)strtol(csv_fields[3], NULL, 10);
        date_time.hour      = (uint8_t)strtol(csv_fields[4], NULL, 10);
        date_time.minute    = (uint8_t)strtol(csv_fields[5], NULL, 10);
        date_time.second    = (uint8_t)strtol(csv_fields[6], NULL, 10);
        date_time.weekday   = (uint8_t)strtol(csv_fields[7], NULL, 10);
        
        rtc_set_date_time_nw(&date_time);
        (void)osif_exit_critical(lc);
    }
}

static void process_odo_rst_cmd(void)
{
    if((abs_is_vehicle_stationary() == 1U) &&
       ((get_status() & ((uint64_t)1U << (uint32_t)STAT_VCU_MOTOR_CON_DIR_FWD)) ==  0U) &&
       ((get_status() & ((uint64_t)1U << (uint32_t)STAT_VCU_MOTOR_CON_DIR_REV)) ==  0U))
    {
        odo_reset();
    }    
}

static void process_imu_calibration_cmd(uint32_t op)
{
    imu_msg_queue_obj_t imq;
    uint32_t valid = 1U;
    
    if((abs_is_vehicle_stationary() == 1U) &&
       ((get_status() & ((uint64_t)1U << (uint32_t)STAT_VCU_MOTOR_CON_DIR_FWD)) ==  0U) &&
       ((get_status() & ((uint64_t)1U << (uint32_t)STAT_VCU_MOTOR_CON_DIR_REV)) ==  0U))
    {
       switch(op)
       {
           case 0U:
               imq.msg_id = IMU_MSG_CAL_START;
               break;
           
           case 1U:
               imq.msg_id = IMU_MSG_CAL_RESET;
               break;
           
           default:
               valid = 0U;
               break;
       }

       if(valid == 1U)
       {
           imq.data = 0x0fU;
           (void)osif_msg_queue_send(imu_msg_queue, &imq, 0U, 10U);
       }
    }     
}

static void process_torque_map_calibration_cmd(const uint8_t *msg)
{
    lut_info_t *t = NULL;
    int32_t lc = 0;
    
    DEV_ASSERT(msg != NULL);

    lc = osif_enter_critical();
    t = (lut_info_t *)(msg + strlen("--s32_trq_map"));
   
    if(t->num_of_entries > 0)
    {
        mc_update_torque_map_signal(t);
        has_mc_tm_changed = 1U;
    }    
    (void)osif_exit_critical(lc);
}

static void process_torque_map_commit_cmd(const udp_rx_msg_t *umesg)
{
    int32_t lc = 0;
    
    UNUSED_PARAM(umesg);

    if(has_mc_tm_changed == 1U)
    {
        has_mc_tm_changed = 0U;
        lc = osif_enter_critical();
        mc_commit_torque_map_signal();
        (void)osif_exit_critical(lc);
    }
}

static void process_torque_map_factory_rst(const udp_rx_msg_t *umesg)
{
    UNUSED_PARAM(umesg);

    mc_factory_reset_ride_tmaps();
}

static void process_mc_ctxt_factory_rst(const udp_rx_msg_t *umesg)
{
    UNUSED_PARAM(umesg);
    
    mc_factory_reset();
}

static void process_torque_map_read(const udp_rx_msg_t *umesg)
{
    volatile udp_msg_queue_obj_t qmsg;
    UNUSED_PARAM(umesg);
    
    udp_resp.udp_msg.cmd = MSG_ID_TMAP_INFO_S32_LTE;
    udp_resp.udp_msg.len = mc_read_tmap_info(&udp_resp.buffer[0]);
    
    qmsg.msg = (void *)&udp_resp;
    qmsg.msg_id = MSG_ID_TMAP_INFO_S32_LTE;
    qmsg.msg_len = udp_resp.udp_msg.len;
        
    (void)osif_msg_queue_send(udp_msg_queue, (void *)&qmsg, 0U, 5U);      
}

static void process_hw_rst_fuelgauge(const udp_rx_msg_t *umesg)
{
    int32_t lc = 0;
    
    UNUSED_PARAM(umesg);
    
    lc = osif_enter_critical();
    bms_fg_reset();
    (void)osif_exit_critical(lc);
}

static void process_mem_read(const udp_rx_msg_t *umesg)
{
    volatile udp_msg_queue_obj_t qmsg;
    volatile uint32_t mem_addr = 0U;
    volatile uint32_t read_len = 0U;
    int32_t lc = 0;
    
    extern uint32_t Image$$EXE_REGION_RW_L$$Base[];
    extern uint32_t Image$$ARM_LIB_HEAP$$Base[];
    
    DEV_ASSERT(umesg != NULL);
    
    mem_addr = *(uint32_t *)&umesg->buffer[0 + strlen("--s32_mem_read")];
    if((mem_addr >= (uint32_t)Image$$EXE_REGION_RW_L$$Base) && 
       ((mem_addr <= ((uint32_t)Image$$ARM_LIB_HEAP$$Base - (uint32_t)Image$$EXE_REGION_RW_L$$Base) ||
       (mem_addr >= flash_init_config.EERAMBase)) &&
       (mem_addr <= 0x1400107fU)))
    {
        read_len = *(uint32_t *)&umesg->buffer[4 + strlen("--s32_mem_read")];
        
        /* Max 256 byte block allowed to be read */
        if(read_len < 256U)
        {
            udp_resp.udp_msg.cmd = MSG_ID_MEM_INFO_S32_LTE;
            udp_resp.udp_msg.len = read_len;
            
            lc = osif_enter_critical();
            memcpy(&udp_resp.buffer[0], (uint8_t *)mem_addr, read_len);
            (void)osif_exit_critical(lc);
        }
    }
    else
    {
        udp_resp.udp_msg.len = strlen("Invalid Memory Range.");
        lc = osif_enter_critical();
        (void)snprintf((char *)&udp_resp.buffer[0], udp_resp.udp_msg.len, "%s", "Invalid Memory Range.");       
        (void)osif_exit_critical(lc);     
    }
    
    qmsg.msg = (void *)&udp_resp;
    qmsg.msg_id = MSG_ID_MEM_INFO_S32_LTE;
    qmsg.msg_len = udp_resp.udp_msg.len;

    (void)osif_msg_queue_send(udp_msg_queue, (void *)&qmsg, 0U, 5U); 
    
}

static void process_tamper_clr(const udp_rx_msg_t *umesg)
{
    bms_msg_queue_obj_t bmsg;
    
    UNUSED_PARAM(umesg);

    bmsg.msg_id = BMS_MSG_CLEAR_TAMPER_STATE;
    bmsg.data = 0U;
    (void)osif_msg_queue_send(bms_msg_queue, &bmsg, 0U, 10U);
}

static void process_string_cmds(const udp_rx_msg_t *umesg)
{
    DEV_ASSERT(umesg != NULL);
    
    if(umesg->buffer[1] == '-')
    {
        if(strncmp((const char *)umesg->buffer, "--s32_set_time", strlen("--s32_set_time")) == 0)
        {
            process_time_cmd(umesg);
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_odo_reset", strlen("--s32_odo_reset")) == 0)
        {
            process_odo_rst_cmd();
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_calib_imu", strlen("--s32_calib_imu")) == 0)
        {
            process_imu_calibration_cmd(0);
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_imu_cal_reset", strlen("--s32_imu_cal_reset")) == 0)
        {
            process_imu_calibration_cmd(1); 
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_trq_map", strlen("--s32_trq_map")) == 0)
        {
            process_torque_map_calibration_cmd(umesg->buffer); 
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_commit_tm", strlen("--s32_commit_tm")) == 0)
        {
            process_torque_map_commit_cmd(umesg); 
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_factory_reset_tm", strlen("--s32_factory_reset_tm")) == 0)
        {
            process_torque_map_factory_rst(umesg); 
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_read_tm", strlen("--s32_read_tm")) == 0)
        {
            process_torque_map_read(umesg);
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_bms_hw_reset_fg", strlen("--s32_bms_hw_reset_fg")) == 0)
        {
            process_hw_rst_fuelgauge(umesg);
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_mem_read", strlen("--s32_mem_read")) == 0)
        {
            process_mem_read(umesg);
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_tamper_clear", strlen("--s32_tamper_clear")) == 0)
        {
            process_tamper_clr(umesg);
        }
        else if(strncmp((const char *)umesg->buffer, "--s32_factory_reset_mc", strlen("--s32_factory_reset_mc")) == 0)
        {
            process_mc_ctxt_factory_rst(umesg); 
        }
        else
        {
            __NOP();
        }
    }
    
}

static void process_LTE_S32_UDP_GET_RTC_TIME(const udp_rx_msg_t *umesg)
{
    datetime_t dt;
    volatile udp_msg_queue_obj_t qmsg;
    
    UNUSED_PARAM(umesg);
    
    rtc_read_local_time(&dt);
    udp_resp.udp_msg.cmd = MSG_ID_RTC_INFO_S32_LTE;
    udp_resp.udp_msg.len = sizeof(datetime_t);
    (void)memcpy(udp_resp.buffer, (uint8_t *)&dt, sizeof(datetime_t));
    
    qmsg.msg = (void *)&udp_resp;
    qmsg.msg_id = MSG_ID_RTC_INFO_S32_LTE;
    qmsg.msg_len = udp_resp.udp_msg.len;
        
    (void)osif_msg_queue_send(udp_msg_queue, (void *)&qmsg, 0U, 5U);      
}

static void process_udp_payload_lte(const uint8_t *payload, uint32_t len)
{
    DEV_ASSERT(payload != NULL);
    UNUSED_PARAM(len);
    
    umsg = (udp_rx_msg_t *)payload;
    
    switch(umsg->cmd)
    {
        case LTE_S32_UDP_FILE_UPD_START:
            process_LTE_S32_UDP_FILE_UPD_START(umsg);
            break;
        
        case LTE_S32_UDP_FILE_UPD_STOP:
            process_LTE_S32_UDP_FILE_UPD_STOP(umsg);
            break;
        
        case LTE_S32_UDP_MSG_FW_DL_START_NTF:
            process_LTE_S32_UDP_MSG_FW_DL_START_NTF(umsg);
            break;
        
        case LTE_S32_UDP_MSG_FW_DL:
            process_LTE_S32_UDP_MSG_FW_DL(umsg);
            break;
        
        case LTE_S32_UDP_MSG_FW_DL_DONE:
            process_LTE_S32_UDP_MSG_FW_DL_DONE(umsg);
            break;
        
        case LTE_S32_UDP_MSG_FW_VER_INFO:
            process_LTE_S32_UDP_MSG_FW_VER_INFO(umsg);
            break;
        
        case LTE_S32_UDP_MSG_ODO_INFO:
            process_LTE_S32_UDP_MSG_ODO_INFO(umsg);
            break;
        
        case LTE_S32_UDP_ODO_RESET:
            process_LTE_S32_UDP_ODO_RESET(umsg);
            break;
        
        case LTE_S32_UDP_CALIB_IMU:
            process_LTE_S32_UDP_CALIB_IMU(umsg);
            break;
        
        case LTE_S32_UDP_SET_TIME:
            process_LTE_S32_UDP_SET_TIME(umsg);
            break;
        
        case LTE_S32_UDP_FILE_DL_STOP:
            process_LTE_S32_UDP_FILE_DL_STOP(umsg);
            break;
        
        case LTE_S32_UDP_MISC:
            process_string_cmds(umsg);
            break;
        
        case LTE_S32_UDP_GET_RTC_TIME:
            process_LTE_S32_UDP_GET_RTC_TIME(umsg);
            break;
        
        default:
            __NOP();
            break;
    }
}

static void process_IMX_S32_UDP_MSG_BRIGHTNESS_CTRL(const udp_rx_msg_t *imx_msg)
{
    volatile dba_msg_queue_obj_t dmq;
    
    DEV_ASSERT(imx_msg != NULL);
    
    /* Brightness value received in uints of percent: 0 to 100 % */
    
    dmq.msg_id = DBA_SEND_DISPLAY_MSG;
    dmq.data[0] = DBA_DISP_BRIGHTNESS_CTRL;
    dmq.data[1] = imx_msg->buffer[0];
    
    (void)osif_msg_queue_send(dba_msg_queue, (void *)&dmq, 0U, 5U);
}

static void process_IMX_S32_UDP_MSG_ABS_MODE_CTRL(udp_rx_msg_t *imx_msg)
{
    volatile dba_msg_queue_obj_t dmq;
    
    DEV_ASSERT(imx_msg != NULL);
    
    /* ABS Mode received */
    if(mc_is_vehicle_stationary() == 1U)
    {
        dmq.msg_id = DBA_SEND_ABS_MSG;
        
        if((imx_msg->buffer[0] == ABS_LGC_MODE_SINGLE_CHANNEL) ||
           (imx_msg->buffer[0] == ABS_LGC_MODE_DUAL_CHANNEL))
        {
            dmq.data[0] = imx_msg->buffer[0];
        
            (void)osif_msg_queue_send(dba_msg_queue, (void *)&dmq, 0U, 5U);   
        }            
    }
}

static void process_IMX_S32_UDP_MSG_CLU_RDY(udp_rx_msg_t *imx_msg)
{   
    UNUSED_PARAM(imx_msg);
    
    abs_update_menu_state();
    chg_get_ui_init_state();
    
    mc_get_regen_level();
    
    cluster_state = 1U;
}

static void process_MSG_ID_TRIP_RESET_IMX_S32(udp_rx_msg_t *imx_msg)
{ 
    volatile uint32_t trip_num = 0U;
    DEV_ASSERT(imx_msg != NULL);
    
    trip_num = imx_msg->buffer[0];
    
    odo_trip_reset(trip_num);
}

static void process_MSG_ID_GET_REGEN_LEVEL_IMX_S32(udp_rx_msg_t *imx_msg)
{
    volatile uint32_t regen_level = 0U;
    DEV_ASSERT(imx_msg != NULL);
    
    regen_level = imx_msg->buffer[0];
    
    mc_set_regen_level(regen_level);
}

static void process_udp_payload_imx(uint8_t *payload, uint32_t len)
{
    udp_rx_msg_t *imx_msg = NULL;
    UNUSED_PARAM(len);
    
    imx_msg = (udp_rx_msg_t *)payload;
    DEV_ASSERT(imx_msg != NULL);
    
    switch(imx_msg->cmd)
    {
        case MSG_ID_BRIGHTNESS_CTRL_IMX_S32:
            process_IMX_S32_UDP_MSG_BRIGHTNESS_CTRL(imx_msg);
            break;
        
        case MSG_ID_ABS_MODE_CTRL_IMX_S32:
            process_IMX_S32_UDP_MSG_ABS_MODE_CTRL(imx_msg);
            break;
        
        case MSG_ID_CLU_RDY_IMX_S32:
            process_IMX_S32_UDP_MSG_CLU_RDY(imx_msg);
            break;
        
        case MSG_ID_TRIP_RESET_IMX_S32:
            process_MSG_ID_TRIP_RESET_IMX_S32(imx_msg);
            break;
        
        case MSG_ID_GET_REGEN_LEVEL_IMX_S32:
            process_MSG_ID_GET_REGEN_LEVEL_IMX_S32(imx_msg);
            break;        
        
        default:
            __NOP();
            break;
    }
}

static void udp_msg_router(udp_msg_queue_obj_t mq, TNetBuf tx_net_buf, TNetConn xUdpConn, route_e where)
{
    err_t xRes;
    ip4_addr_t dst_imx_ip;
    ip4_addr_t dst_lte_ip;
    
    IP4_ADDR(&dst_imx_ip,  192, 168, 10, 50);
    IP4_ADDR(&dst_lte_ip,  192, 168, 10, 52);
    
    /* netbuf_ref(BUFF_REFERENCE, QUEUE MSG, SIZEOF(cmd) + SIZEOF(len) + MESSAGE LEN); */
    (void)netbuf_ref(tx_net_buf, mq.msg, 4U + 4U + (uint16_t)mq.msg_len);
    
    switch(where)
    {
        case ROUTE_SRC_S32_DST_LTE:
            xRes = netconn_sendto(xUdpConn, tx_net_buf, &dst_lte_ip, S32_LTE_UDP_PORT);
            DEV_ASSERT(xRes == ERR_OK);   
            break;
        
        case ROUTE_SRC_S32_DST_IMX:
            xRes = netconn_sendto(xUdpConn, tx_net_buf, &dst_imx_ip, S32_IMX_UDP_PORT);
            DEV_ASSERT(xRes == ERR_OK);   
            break;
        
        default:
            __NOP();
            break;
    }     
}

#ifdef USE_FEATURE_IMU_CIRCULAR_BUFFER
static void udp_imu_lte_tx(ip4_addr_t dst_ip, TNetBuf tx_net_buf, TNetConn xUdpConn)
{
    err_t xRes;
    int32_t lc = 0;
    uint32_t ret = 0U;
    udp_msg_queue_obj_t msg;
    
    imu_get_msg_params(&msg);
    imu_udp_msg.msg.cmd = msg.msg_id;
    imu_udp_msg.msg.len = msg.msg_len;   
    
    ret = ringbuff_read((ringbuff_t *)msg.msg, (uint8_t *)&m, msg.msg_len);
    
    if(ret > 0U)
    {
        lc = osif_enter_critical();
        memcpy(imu_udp_msg.buffer, (uint8_t *)&m, sizeof(imu_data_16_t));
        (void)osif_exit_critical(lc);
        
        netbuf_ref(tx_net_buf, &imu_udp_msg, 4U + 4U + (uint16_t)msg.msg_len);
        xRes = netconn_sendto(xUdpConn, tx_net_buf, &dst_ip, S32_LTE_UDP_PORT);
        DEV_ASSERT(xRes == ERR_OK); 
    } 
}
#endif /* USE_FEATURE_IMU_CIRCULAR_BUFFER */

static void udp_task_rxtx(void *arg)
{
    ip4_addr_t src_s32_ip;
    ip4_addr_t dst_imx_ip;
    ip4_addr_t dst_lte_ip;

    err_t ret = ERR_OK;

    status_t status;
    udp_msg_queue_obj_t mq;
   
    UNUSED_PARAM(arg);

    ip4_addr_set_zero(&src_s32_ip);
    ip4_addr_set_zero(&dst_imx_ip);
    ip4_addr_set_zero(&dst_lte_ip);
   
    IP4_ADDR (&src_s32_ip, 192, 168, 10, 51);
    IP4_ADDR (&dst_imx_ip, 192, 168, 10, 50);
    IP4_ADDR (&dst_lte_ip, 192, 168, 10, 52);
   
    /* i.MX Bindings */
    TNetConn udp_conn_cluster = netconn_new(NETCONN_UDP);
    ret = netconn_bind(udp_conn_cluster, &src_s32_ip, S32_IMX_UDP_PORT);
    netconn_set_recvtimeout(udp_conn_cluster, 20U);
   
    /* EC25 bindings */
    TNetConn udp_conn_modem = netconn_new(NETCONN_UDP);
    ret = netconn_bind(udp_conn_modem, &src_s32_ip, S32_LTE_UDP_PORT);
    netconn_set_recvtimeout(udp_conn_modem, 50U);

    /* Start IMU */
    (void)osif_task_start(imu_task_get_id());
    
    if (ret == ERR_OK)
    {
        tx_net_lte_buf = netbuf_new();
        rx_net_lte_buf = netbuf_new();
        
        tx_net_imx_buf = netbuf_new();
        rx_net_imx_buf = netbuf_new();
        
        while(1)
        {
            ret = netconn_connect(udp_conn_modem, &dst_lte_ip, S32_LTE_UDP_PORT);
            ret = netconn_connect(udp_conn_cluster, &dst_imx_ip, S32_IMX_UDP_PORT);
           
            status = osif_msg_queue_recv(udp_msg_queue, &mq, NULL, 1U);
            if(status == osOK)  
            {
                switch(mq.msg_id)
                {
                    case MSG_ID_SWIF_S32_IMX:
                        udp_msg_router(mq, tx_net_imx_buf, udp_conn_cluster, ROUTE_SRC_S32_DST_IMX);
                        shmem_free_block(SHMEM_POOL_TYPE_SWIF, mq.msg);
                        break;
                    
                    case MSG_ID_IMX_PWR_OFF_S32_IMX:
                        udp_msg_router(mq, tx_net_imx_buf, udp_conn_cluster, ROUTE_SRC_S32_DST_IMX);
                        break;
                    
                    case MSG_ID_DBG_AUX_S32_IMX:
                        udp_msg_router(mq, tx_net_imx_buf, udp_conn_cluster, ROUTE_SRC_S32_DST_IMX);
                        shmem_free_block(SHMEM_POOL_TYPE_BMS, mq.msg);
                        break;
                    
                    case MSG_ID_DBG_S32_LTE:
                        udp_msg_router(mq, tx_net_lte_buf, udp_conn_modem, ROUTE_SRC_S32_DST_LTE);
                        shmem_free_block(SHMEM_POOL_TYPE_BMS, mq.msg);
                        break;
                    
                    case MSG_ID_DL_RESUME_S32_LTE:
                        udp_msg_router(mq, tx_net_lte_buf, udp_conn_modem, ROUTE_SRC_S32_DST_LTE);
                        break;
                    
                    case MSG_ID_FW_VER_S32_LTE:
                        link_active = 1U;
                        udp_msg_router(mq, tx_net_lte_buf, udp_conn_modem, ROUTE_SRC_S32_DST_LTE);
                        break;
                    
                    case MSG_ID_ODO_INFO_S32_LTE:
                        udp_msg_router(mq, tx_net_lte_buf, udp_conn_modem, ROUTE_SRC_S32_DST_LTE);
                        break;
                    
                    case MSG_ID_RTC_INFO_S32_LTE:
                        udp_msg_router(mq, tx_net_lte_buf, udp_conn_modem, ROUTE_SRC_S32_DST_LTE);
                        break;
                    
                    case MSG_ID_ABS_MODE_UPD_S32_IMX:
                        udp_msg_router(mq, tx_net_imx_buf, udp_conn_cluster, ROUTE_SRC_S32_DST_IMX);
                        shmem_free_block(SHMEM_POOL_TYPE_BMS, mq.msg);
                        break;
                    
                    case MSG_ID_ABS_MENU_CTRL_S32_IMX:
                        udp_msg_router(mq, tx_net_imx_buf, udp_conn_cluster, ROUTE_SRC_S32_DST_IMX);
                        break;
                    
                    case MSG_ID_GRACEFUL_SHUTDOWN_S32_LTE:
                        udp_msg_router(mq, tx_net_lte_buf, udp_conn_modem, ROUTE_SRC_S32_DST_LTE);
                        break;
                    
                    case MSG_ID_CHARGER_EVT_S32_IMX:
                        udp_msg_router(mq, tx_net_imx_buf, udp_conn_cluster, ROUTE_SRC_S32_DST_IMX);
                        break;
                    
                    case MSG_ID_TMAP_INFO_S32_LTE:
                        udp_msg_router(mq, tx_net_lte_buf, udp_conn_modem, ROUTE_SRC_S32_DST_LTE);
                        break;
                    
                    case MSG_ID_MEM_INFO_S32_LTE:
                        udp_msg_router(mq, tx_net_lte_buf, udp_conn_modem, ROUTE_SRC_S32_DST_LTE);
                        break;
                    
                    case MSG_ID_GET_REGEN_LEVEL_IMX_S32:
                        udp_msg_router(mq, tx_net_lte_buf, udp_conn_modem, ROUTE_SRC_S32_DST_IMX);
                        break;
                    
                    default:
                        DEV_ASSERT(false);
                        break;
                }
            }
            
            /* send IMU packets while waiting for others */
#ifdef USE_FEATURE_IMU_CIRCULAR_BUFFER
            if(imu_stall == 0U)
            {
                udp_imu_lte_tx(dst_lte_ip, tx_net_lte_buf, xUdpConn_lte);
            }
#endif /* USE_FEATURE_IMU_CIRCULAR_BUFFER */

            (void)netconn_disconnect(udp_conn_modem);
            (void)netconn_disconnect(udp_conn_cluster);
           
            /* Initiate rx on ec25 UDP */
            ret = netconn_recv(udp_conn_modem, &rx_net_lte_buf);

            /* Process the data packet & respond */
            if(ret == ERR_OK)
            {
                process_udp_payload_lte(rx_net_lte_buf->p->payload, rx_net_lte_buf->p->len);
                clear_status_bit(STAT_VCU_PHY_LINK_TIMEOUT);
            }

            if(rx_net_lte_buf != NULL)
            {
                netbuf_delete(rx_net_lte_buf);
            }

            /* Initiate rx on imx UDP */
            ret = netconn_recv(udp_conn_cluster, &rx_net_imx_buf);

            /* Process the data packet & respond */
            if(ret == ERR_OK)
            {
                process_udp_payload_imx(rx_net_imx_buf->p->payload, rx_net_imx_buf->p->len);
            }

            if(rx_net_imx_buf != NULL)
            {
                netbuf_delete(rx_net_imx_buf);
            }
        }
    }  
}

thread_id_t udp_task_imx_get_id(void)
{
    return thread_udp_imx;
}

#ifdef USE_FEATURE_FLUSH_UDP_QUEUE
void udp_flush_messages(void)
{
    /* Flush out messages if any in queue */
    uint32_t nm = 0U;
    
    osif_msg_queue_num_msgs(udp_msg_queue, &nm);
    while(nm > 0U)
    {
        __NOP();
        osif_time_delay(2);
        osif_msg_queue_num_msgs(udp_msg_queue, &nm);
    }  
}
#endif /* USE_FEATURE_FLUSH_UDP_QUEUE */

void udp_task_kill(void)
{
    (void)osif_task_end(thread_udp_imx);
}

static void ota_link_timeout_handler(void *arg)
{
    UNUSED_PARAM(arg);
    
    (void)nvm_write(FILE_SECT_BOOT_CONFIGURATION + FILE_SECT_BOOT_FW_UPD_OFFSET, (uint8_t *)&boot_fw_upd_fail_sentinel, 4U);
    
    SystemSoftwareReset();
}

static void link_up_timer_handler(void *arg)
{
    UNUSED_PARAM(arg);
    
    (void)osif_timer_stop(ethernet_link_up_timer);
    
    if(link_active == 0U)
    {      
        set_status_bit(STAT_VCU_PHY_LINK_TIMEOUT);
    }
    else
    {
        if(sim_det == 0U)
        {
            /* No SIM detected CPIN failed */
            set_status_bit(STAT_VCU_SIM_NOT_DETECTED);
        }
    }
}

void udp_task_imx_init(void)
{
    status_t s = STATUS_SUCCESS;
    uint32_t param = NULL;

    s = FLASH_DRV_Init(&flash_init_config, &flash_ssd_config);
    DEV_ASSERT(STATUS_SUCCESS == s);
    
    ethernet_link_up_timer = osif_timer_create(link_up_timer_handler, osTimerOnce, NULL, NULL);
    DEV_ASSERT(ethernet_link_up_timer != NULL);
    (void)osif_timer_start(ethernet_link_up_timer, MSEC_TO_TICK(LINK_UP_TIMER_TIMEOUT)); 
    
    ota_link_timer = osif_timer_create(ota_link_timeout_handler, osTimerOnce, NULL, NULL);
    DEV_ASSERT(ethernet_link_up_timer != NULL);
 
    udp_msg_queue = osif_msg_queue_create(MSGQUEUE_OBJECTS, sizeof(udp_msg_queue_obj_t));
    DEV_ASSERT(udp_msg_queue != NULL);

    thread_udp_imx = osif_thread_create(udp_task_rxtx, &param, &udp_task_attr);
    DEV_ASSERT(thread_udp_imx != NULL);
}

#endif /* LWIP_NETCONN */

