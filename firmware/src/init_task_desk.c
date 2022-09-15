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
 
#include "init_task_desk.h"
#ifdef USE_FEATURE_MPU
#include "drv_mem.h"
#endif
#include "osif.h"
#include "cd1030.h"
#include "drv_spi_legacy.h" 
#include "NXP_SJA1105P_resetGenerationUnit.h"
#include "NXP_SJA1105P_spi.h"
#include "eth_task.h"
#include "pins_driver.h"
#include "led_task.h"
#include "bms_task.h"
#include "swif_task.h"
#include "lsm_task.h"
#include "mc_task.h"
#include "rtc_task.h"
#include "odometer_task.h"
#include "pac1921_task.h"
#include "lte_task.h"
#include "bcm.h"
#include "eth_task.h"
#include "lte_uart_task.h"
#include "sem_common.h"
#include "dba_task.h"
#include "udp_task.h"
#include "shared_mem.h"
#include "imu.h"
#include "sleep.h"

#ifdef USE_FEATURE_VCU_ON_DESK

typedef enum
{
    VCU_STATE_INIT = 0U,
    VCU_STATE_EVAL_KEY,
    VCU_STATE_WAIT_KEY,
    VCU_STATE_RUN_TASKS,
    VCU_STATE_IDLE,
    VCU_STATE_POST_PWR_OFF,
    VCU_STATE_START_PWR_OFF_SEQ,   
    
    MAX_VCU_STATES
}vcu_states_e;

void osRtxIdleThread (void *argument);
uint32_t osRtxErrorNotify (uint32_t code, void *object_id);

#ifdef USE_FEATURE_RTX_VIEWER
__attribute__((section(".bss.os.thread.cb")))
thread_tcb_t init_thread_desk_tcb;
#else
static thread_tcb_t init_thread_desk_tcb;
#endif /* USE_FEATURE_RTX_VIEWER */

static thread_id_t thread_init_desk;
static uint32_t vcu_state = VCU_STATE_RUN_TASKS;

static uint32_t reboot_request = 0xF0773100U;

__attribute__((section("ARM_LIB_STACK")))
static uint64_t init_thread_desk_stk[INIT_TASK_STACK_SIZE];

#ifdef USE_FEATURE_VCU_ON_DESK
osif_msg_queue_id_t sys_msg_desk_queue; 
#else
osif_msg_queue_id_t sys_msg_queue; 
#endif
static const thread_attrs_t init_attr = {
    INIT_TASK_NAME,
    osThreadDetached,
    &init_thread_desk_tcb,
    sizeof(init_thread_desk_tcb),
    &init_thread_desk_stk[0],
    INIT_TASK_STACK_SIZE * sizeof(uint64_t),
    osPriorityNormal,
    0U,
    0U    
};

#define INIT_TASK_MSG_QUEUE_MAX_OBJS    (16) 

/* 10 ms loop time. 500 counts -> 5 seconds */
#define SLEEP_WAIT_TIME             (500U)
#define KEY_FLAG                    (0x0080U)
#define INIT_MOTOR_PWR_ENABLE_FLAG  (0x0800U)
#define SWIF_KEY_ON_EVENT_FLAG      (0x0020U)

static void init_sm_vcu_state_init(void);
static void init_sm_vcu_state_eval_key(void);
static void init_sm_vcu_state_wait_key(void);
static void init_sm_vcu_state_start_pwr_off(void);
static void init_sm_vcu_state_idle(void);
static void init_sm_vcu_state_run_tasks(void);
static void init_wait_loop(void);
static void init_sm_set_reboot_cause(uint32_t reboot_sentinel);

#if 0
static void eth_switch_init(void)
{
    uint32_t prod_id[1] = {0x00000000U};
    volatile uint8_t sw_ret = 0U;
#ifdef USE_FEATURE_SWITCH_DBG
    uint8_t pll_lock_status = 0U;
    SJA1105P_configurationFlagsArgument_t sw_cfg_flags;
#endif
    
    SJA1105P_prodIdArgument_t sw_prod_id;
    
#ifdef USE_FEATURE_SWITCH_DBG
    SJA1105P_cfgPadSpiArgument_t sw_spi_cfg;
    SJA1105P_cfgPadMiixArgument_t sw_mii_pad_cfg;
    SJA1105P_basicControlArgument_t sw_sgmii_cfg;
    SJA1105P_idivCControlRegisterArgument_t idivCControlRegister_rd;
    SJA1105P_idivCControlRegisterArgument_t idivCControlRegister_wr;
#endif
    
    /* Hard Reset ENET */
    
    /* Setup callbacks for R/W */
    SJA1105P_registerSpiRead32CB(lpspi_trancieve_sja_read32);
    SJA1105P_registerSpiWrite32CB(lpspi_trancieve_sja_write32);
    
    /* Cold reset Switch */
    SJA1105P_setResetCtrl(0x0004U, 1U);
    sw_asm_delay_us(100000U);
    
    /* Enable ACU register access */
    SJA1105P_gpf_spiWrite32(1U, 1U, (uint32_t)0x100Bfd, prod_id);
    sw_asm_delay_us(500);
    
    /* Read the production ID -> Must read 0x9A86 */
    SJA1105P_getProdId(&sw_prod_id, 1U);
    sw_asm_delay_us(500U);

#ifdef USE_FEATURE_SWITCH_DBG
    SJA1105P_getConfigurationFlags(&sw_cfg_flags, 1U);
    sw_asm_delay_us(500);
#endif

    /* Initialize the switch */
    sw_ret += SJA1105P_initSwitch();
    sw_asm_delay_us(500U);
    
#ifdef USE_FEATURE_SWITCH_DBG
    sw_ret += SJA1105P_getDeviceId(&dev_id, 1U);
    sw_asm_delay_us(500U);
    sw_ret += SJA1105P_getPllStatus(&pll_lock_status, 1U, 1U);   
    sw_asm_delay_us(500U);
    
    SJA1105P_getConfigurationFlags(&sw_cfg_flags, 1U);
    sw_asm_delay_us(500U);
    
    SJA1105P_getCfgPadSpi(&sw_spi_cfg, 1U);
    sw_asm_delay_us(500U);
    

    sw_ret += SJA1105P_getCfgPadMiix(&sw_mii_pad_cfg, 0U, SJA1105P_e_direction_TX, 1U);
    sw_mii_pad_cfg.d10Ih = SJA1105P_e_padInputHysteresis_SCHMITT;
    sw_mii_pad_cfg.d10Ipud = SJA1105P_e_padInputStageSelection_PULL_UP;
    sw_ret += SJA1105P_setCfgPadMiix(&sw_mii_pad_cfg, 0U, SJA1105P_e_direction_TX, 1U);
    sw_asm_delay_us(500U);
    

    sw_sgmii_cfg.autonegEnable = 1U;
    sw_sgmii_cfg.duplexMode = 1U;
    sw_sgmii_cfg.powerDown = 0U;
    sw_sgmii_cfg.speedSelect = SJA1105P_e_speed_100_MBPS;
    sw_sgmii_cfg.loopback = 0U;
    sw_ret += SJA1105P_setBasicControl(&sw_sgmii_cfg, 1U);
    
#endif /* USE_FEATURE_SWITCH_DBG */

}

static void process_sys_queue(sys_msg_queue_obj_t mq)
{
    switch(mq.msg_id)
    {
        case MSG_ID_REBOOT_REQ:
            init_sm_set_reboot_cause(0x0013770FU);
            break;
        case MSG_ID_SLEEP_REQ:
            vcu_state = VCU_STATE_START_PWR_OFF_SEQ;
            break;
        default:
            __NOP();
            break;
    }
}

void init_sm_set_reboot_cause(uint32_t reboot_sentinel)
{
    reboot_request = reboot_sentinel;
}
#endif /* 0 */

static void sys_init_hw(void)
{
    volatile msdi_status_t msdi_status = MSDI_STATUS_SUCCESS;

    /* Common semaphore for the I2C bus accesses */
    sem_common_init();
    
    /*drv_rtc_init();*/
    /*drv_PAC1921_init();*/
    
    NVIC_DisableIRQ(PORTC_IRQn);
	
	msdi_status |= init_CD10x0();
    ERR_NOTIFY(msdi_status == STATUS_SUCCESS, SYS_ERR_PERIPH_INIT_MSDI);
	
    osif_time_delay(100U);
	
    NVIC_ClearPendingIRQ(PORTC_IRQn);
    NVIC_EnableIRQ(PORTC_IRQn);
    
	cd1030_sem_init();	
    cd1030_timer_init();
    /*rtc_init();*/
    
    /*init_odometer();*/

    /*PINS_DRV_ClearPins(IMU_VDD_EN_GPIO, 1U << IMU_VDD_EN_PIN); */
    /*osif_time_delay(100);*/
    /*PINS_DRV_SetPins(IMU_VDD_EN_GPIO, 1U << IMU_VDD_EN_PIN);*/ 
    /*osif_time_delay(100);*/
    
    /*imu_init();*/

}

static void sys_init_tasks(void)
{
#ifndef USE_FEATURE_VCU_ON_DESK
    led_task_create();
    swif_task_create();
    lsm_task_create();
    mc_task_create();
    bms_task_create();
    dba_task_create();
    imu_task_create();
#else
    led_task_create();
    bms_task_create();
#endif /* USE_FEATURE_VCU_ON_DESK */
    
#ifdef USE_FEATURE_MPU
    mpu_protect_vector();
#endif 
    
}

static void sys_terminate_tasks(void)
{
#ifndef USE_FEATURE_VCU_ON_DESK
    (void)osif_task_end(led_task_get_id());
  	(void)osif_task_end(swif_task_get_id());
    
    (void)osif_task_end(lte_uart_task_get_id());
    (void)osif_task_end(mc_task_get_id());
    (void)osif_task_end(lsm_task_get_id());
    (void)osif_task_end(dba_task_get_id());
    (void)osif_task_end(imu_task_get_id());
    
    /* This task is a special case as it has init and startup in the lwip stack*/
    net_tasks_deinit();
#else
    (void)osif_task_end(led_task_get_id());
    (void)osif_task_end(bms_task_get_id());
#endif /* USE_FEATURE_VCU_ON_DESK */

}


static void sys_run_tasks(void)
{
#ifndef USE_FEATURE_VCU_ON_DESK
    (void)osif_task_start(led_task_get_id());
  	(void)osif_task_start(swif_task_get_id());
    
    /* Start BMS */
    (void)osif_task_start(bms_task_get_id());
    
    (void)osif_task_start(mc_task_get_id());
    (void)osif_task_start(lsm_task_get_id());
    (void)osif_task_start(dba_task_get_id());
    
    /* This task is a special case as it has init and startup in the lwip stack */
    net_tasks_init();
#else
    (void)osif_task_start(led_task_get_id());
    (void)osif_task_start(bms_task_get_id());
#endif /* USE_FEATURE_VCU_ON_DESK */
}


static void start_init_proc(void)
{
    bcm_control(BCM_AUX_CTRL, BCM_CTRL_STATE_ON);
    PINS_DRV_SetPins(AUX_5V_EN_GPIO, (1U << AUX_5V_EN_PIN));
    
#if 0
    /* Enable power supply for i.MX */
    PINS_DRV_SetPins(IMX6_VDD_EN_GPIO, (1U << IMX6_VDD_EN_PIN));
    PINS_DRV_ClearPins(IMX6_PMIC_EN_GPIO, (1U << IMX6_PMIC_EN_PIN));

    /* Release reset for I.MX6 */
    PINS_DRV_ClearPins(IMX6_RESET_GPIO, (1U << IMX6_RESET_PIN));
    PINS_DRV_SetPins(MODE_GPIO_SW2_GPIO, (1U << MODE_GPIO_SW2_PIN));
    
    eth_switch_init();

    bcm_control(BCM_MCU_EN_CTRL, BCM_CTRL_STATE_ON);
    bcm_control(BCM_HORN_VCC_CTRL, BCM_CTRL_STATE_ON);
    bcm_control(BCM_FWD_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_REV_CTRL, BCM_CTRL_STATE_OFF);

    bcm_control(BCM_ABS_EN_CTRL, BCM_CTRL_STATE_ON);
    bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_ON);
    bcm_control(BCM_TAIL_LAMP_CTRL, BCM_CTRL_STATE_ON);
#endif

}


void init_sm_vcu_state_run_tasks(void)
{
    start_init_proc();
    vcu_state = VCU_STATE_INIT;
}


void init_sm_vcu_state_init(void)
{
    /* Init hardware, Setup tasks, execute tasks */
    sys_init_hw();
    sys_init_tasks();
    sys_run_tasks(); 

    vcu_state = VCU_STATE_IDLE;  
}

static void wait_ec25_turnoff(void)
{
    volatile uint32_t pins_gpiod = 0U; 
    volatile uint32_t timeout = 0U;
    
    pins_gpiod = PINS_DRV_ReadPins(LTE_IRQ_GPIO);
    
    while((pins_gpiod & (1U << LTE_IRQ_PIN)) > 0U)
    {
        sw_asm_delay_us(10000);
        timeout++;
        if(timeout > EC25_SLEEP_TIMEOUT)
        {
            break;
        }
        
        pins_gpiod = PINS_DRV_ReadPins(LTE_IRQ_GPIO);
    }
}

static void start_shutdown_proc(void)
{
    dba_msg_queue_obj_t dmq;
    shmem_block_swif_t *msg = NULL;
    shmem_block_bms_t *shutdown_msg = NULL;
    udp_msg_queue_obj_t qmsg;
    udp_msg_queue_obj_t qmsg_shutdown;
    bms_msg_queue_obj_t bmsg;
    
    bcm_control(BCM_TAIL_LAMP_CTRL, BCM_CTRL_STATE_OFF);
    
    /* Turn OFF IMU */
    PINS_DRV_ClearPins(IMU_VDD_EN_GPIO, 1U << IMU_VDD_EN_PIN); 
    NVIC_ClearPendingIRQ(PORTD_IRQn);
    INT_SYS_DisableIRQ(PORTD_IRQn);

    /* Set motor to neutral */
    bcm_control(BCM_FWD_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_REV_CTRL, BCM_CTRL_STATE_OFF);
    
    /* Turn OFF Motor and ABS power */
    mc_set_motor_pwr_state(MC_MOTOR_PWR_OFF);
    bcm_control(BCM_ABS_EN_CTRL, BCM_CTRL_STATE_OFF);
    
    /* Send bms command to turn FETs OFF */
    bmsg.msg_id = BMS_MSG_ASYNC_FET_OFF;
    bmsg.data = 0U;
    (void)osif_msg_queue_send(bms_msg_queue, &bmsg, 0U, 10U);
    
    osif_time_delay(1000);
    
    /* Shutting down. The message content is irrelevant */
    shutdown_msg = (shmem_block_bms_t *)shmem_alloc_block(SHMEM_POOL_TYPE_BMS);
    shutdown_msg->udp_msg.cmd = MSG_ID_GRACEFUL_SHUTDOWN_S32_LTE;
    shutdown_msg->udp_msg.len = 1U;
    shutdown_msg->buffer[0] = 0xFU;
    
    qmsg_shutdown.msg = (void *)shutdown_msg;
    qmsg_shutdown.msg_id = shutdown_msg->udp_msg.cmd;
    qmsg_shutdown.msg_len = shutdown_msg->udp_msg.len;
        
    (void)osif_msg_queue_send(udp_msg_queue, (void *)&qmsg_shutdown, 0U, 0U);  
    
    /* Send shutdown command to EC25 */
    /* Minimum spec. according to Quectel is 650ms or more */
    PINS_DRV_SetPins(ON_OFF_MCU_GPIO, (1U << ON_OFF_MCU_PIN));
    osif_time_delay(850U);
    PINS_DRV_ClearPins(ON_OFF_MCU_GPIO, (1U << ON_OFF_MCU_PIN));

    /* Turn off HORN */
    bcm_control(BCM_HORN_VCC_CTRL, BCM_CTRL_STATE_OFF);
    
    dmq.msg_id = DBA_SEND_DISPLAY_MSG;
    dmq.data[0] = DBA_DISP_PWR_OFF;
    dmq.data[1] = 0U;
    dmq.data[2] = 0U;
    (void)osif_msg_queue_send(dba_msg_queue, &dmq, 0U, 10U);
    (void)osif_task_end(bms_task_get_id());
    
    osif_time_delay(500);
    
    update_odo_persistence();
    
    msg = (shmem_block_swif_t *)shmem_alloc_block(SHMEM_POOL_TYPE_SWIF);
    msg->udp_msg.cmd = MSG_ID_IMX_PWR_OFF_S32_IMX;
    msg->udp_msg.len = 1U;
    msg->sw_info = 0;
    
    qmsg.msg = (void *)msg;
    qmsg.msg_id = msg->udp_msg.cmd;
    qmsg.msg_len = msg->udp_msg.len;
    
    (void)osif_msg_queue_send(udp_msg_queue, &qmsg, 0U, 0U);
    
#ifdef USE_FEATURE_FLUSH_UDP_QUEUE
    udp_flush_messages();
#endif
    
    /* After this point there must be no messages published to UDP queue */
    
    /* Turn OFF VCU */
    /* Removed from hardware v2.0 */

    PINS_DRV_ClearPins(LED_GREEN_GPIO, (1U << LED_GREEN_PIN));
    PINS_DRV_ClearPins(LED_RED_GPIO, (1U << LED_RED_PIN));
    
    /* wait for EC25 to be powered off */
    wait_ec25_turnoff();
    
    bcm_control(BCM_DRL_LED_CTRL, BCM_CTRL_STATE_OFF);

    /* Wait 3 sec for all tasks to process their messages */
    osif_time_delay(3000U);

    /* Start task kill procedure */
    sys_terminate_tasks();
    udp_task_kill();
    
    /* Enter Sleep */
    sleep(reboot_request);
    
    vcu_state = VCU_STATE_EVAL_KEY;  
}

static uint32_t key_sw_state(void)
{
    volatile uint32_t key_sw = 0U;
    volatile uint32_t key_sw_st = 0U;

    key_sw = PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    
    if((key_sw & (1U << KEY_WAKE_SIG_PIN)) == 0U)
    {
        key_sw_st = 1U;
    }
    else
    {
        key_sw_st = 0U;
    }
    
    return key_sw_st;
}

static void init_sm_vcu_state_eval_key(void)
{
    uint32_t key_state = 0U;
    const uint32_t eval_key_outcome[2] = {VCU_STATE_WAIT_KEY, VCU_STATE_RUN_TASKS};
    
    key_state = key_sw_state();
    DEV_ASSERT(key_state < 2U);
    
    vcu_state = eval_key_outcome[key_state];
}

static void init_sm_vcu_state_wait_key(void)
{
    static uint32_t wait_count = 0U;
    
    if(key_sw_state() == 1U)
    {
        vcu_state = VCU_STATE_RUN_TASKS;
    }
    else
    {
        osif_time_delay(5U);
        
        /* Wait 5 seconds then enter sleep */
        /* A scenario where the lead acid battery is connected
           with the key in off state.
        */
        wait_count++;
        if(wait_count > SLEEP_WAIT_TIME)
        {
            wait_count = 0U;
            sleep_at_boot();
        }
        
        vcu_state = VCU_STATE_WAIT_KEY;
    }
}

#ifndef USE_FEATURE_VCU_ON_DESK
static void init_sm_vcu_state_idle(void)
{
    uint32_t tf = 0U;
    status_t status = STATUS_SUCCESS;
    
    if((key_sw_state() == 1U) && (reboot_request == 0xF0773100U))
    {
        vcu_state = VCU_STATE_IDLE;
        /* Key in ON position. Stay in same state */
    }
    else
    {
        vcu_state = VCU_STATE_START_PWR_OFF_SEQ;
    }
}
#else
static void init_sm_vcu_state_idle(void)
{
    __NOP();
}
#endif /* USE_FEATURE_VCU_ON_DESK */

static void init_sm_vcu_state_start_pwr_off(void)
{
    set_status_bit(STAT_VCU_VEHICLE_KEY_OFF);
    cd1030_enter_LPM();
    NVIC_DisableIRQ(PORTC_IRQn);
    NVIC_DisableIRQ(PORTA_IRQn);
    start_shutdown_proc();
}

static void init_sm_undefined_state(void)
{
    set_status_bit(STAT_VCU_SM_INVALID_STATE_ENTRY);
    vcu_state = VCU_STATE_IDLE;
}

__NO_RETURN static void sys_init_task(void *argument) 
{
    sys_msg_queue_obj_t mq;
    
    UNUSED_PARAM(argument);

    for(;; )
    {        
        switch(vcu_state)
        {
            case VCU_STATE_INIT:
                init_sm_vcu_state_init();
                break;
            
            case VCU_STATE_EVAL_KEY: 
                init_sm_vcu_state_eval_key();
                break;
                    
            case VCU_STATE_WAIT_KEY:
                init_sm_vcu_state_wait_key();
                break;
                
            case VCU_STATE_RUN_TASKS:

                PINS_DRV_SetPins(LED_GREEN_GPIO, 1U << LED_GREEN_PIN);

                init_sm_vcu_state_run_tasks();                            
                break;
            
            case VCU_STATE_IDLE:
                init_sm_vcu_state_idle();
#if 0
                if(STATUS_SUCCESS == osif_msg_queue_recv(sys_msg_queue, &mq, NULL, 1U))
                {
                    process_sys_queue(mq);
                }
#endif
                break;
                
            case VCU_STATE_POST_PWR_OFF:
                break;
                
            case VCU_STATE_START_PWR_OFF_SEQ: 
                init_sm_vcu_state_start_pwr_off();
                break;
                
            default:
                init_sm_undefined_state();
                break;
        }
    }
}

void init_task_desk_create(void)
{
    uint32_t param = NULL;
    
    sys_msg_queue = osif_msg_queue_create(INIT_TASK_MSG_QUEUE_MAX_OBJS, sizeof(sys_msg_queue_obj_t));
    DEV_ASSERT(sys_msg_queue != NULL);
    
    thread_init_desk = osif_thread_create(sys_init_task, &param, &init_attr);
    DEV_ASSERT(thread_init_desk != NULL);
    
}

thread_id_t init_task_desk_get_id(void)
{
    return thread_init_desk;
}

void osRtxIdleThread(void *argument)
{
    UNUSED_PARAM(argument);
    /* Dummy Implementation to prevent link error */
}

uint32_t osRtxErrorNotify(uint32_t code, void *object_id)
{
    (void)object_id;

    switch (code) 
    {
        case osRtxErrorStackUnderflow:
          /* Stack overflow detected for thread (thread_id=object_id) */
          break;
        case osRtxErrorISRQueueOverflow:
          /* ISR Queue overflow detected when inserting object (object_id) */
          break;
        case osRtxErrorTimerQueueOverflow:
          /* User Timer Callback Queue overflow detected for timer (timer_id=object_id) */
          break;
        case osRtxErrorClibSpace:
          /* Standard C/C++ library libspace not available: increase OS_THREAD_LIBSPACE_NUM */
          break;
        case osRtxErrorClibMutex:
          /* Standard C/C++ library mutex initialization failed */
          break;
        default:
          /* Reserved */
          break;
    }

    for(;; )
    {
        if(key_sw_state() == 0U)
        {
            /* If Key is OFF */
            SystemSoftwareReset();
        }
        else
        {
            /* TBD */
            /* Selective Startup */
            __NOP();
        }  
    }
}
#endif /* USE_FEATURE_VCU_ON_DESK */


