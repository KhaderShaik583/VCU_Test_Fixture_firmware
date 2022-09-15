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
 
#include "sleep.h"
#include "clock_config.h"
#ifdef USE_DEBUG_PRINTS
#include "debug_console.h"
#endif
#include "msdi.h"
#include "pin_mux.h"
#include "interrupt_manager.h"
#include "csec_driver.h"
#include "lpi2c_driver.h" 
#include "lpi2c_hw_access.h"
#include "drv_spi_legacy.h" 
#include "flexcan_driver.h"
#include "csec_driver.h"
#include "ftm_common.h"
#include "lpspi_master_driver.h"
#include "lpuart_driver.h"
#include "power_manager.h"
#include "bcm.h"
#include "cd1030.h"
#include "drv_mem.h"
#include "enet_driver.h" 
#include "wdog_hw_access.h"
#include "odometer_task.h"
#include "pins_gpio_hw_access.h" 
#include "pins_port_hw_access.h"
#include "dba_task.h"
#include "mc_task.h"

#define DBA_MAX_MSGS    (1U)

#ifdef USE_FEATURE_LP_SLEEP_MODE
static uint32_t pin_val = 0U;
#endif

static void lp_sleep(void);

static status_t run_to_vlpr(void)
{
    status_t s = STATUS_ERROR;
    
    power_manager_user_config_t pwrMan_InitConfig = {
        .powerMode = POWER_MANAGER_VLPR,                                 /*!< Power manager mode  */
        .sleepOnExitValue = false,                                       /*!< Sleep on exit value */
    }; 

    power_manager_user_config_t *powerConfigsArr[] = {
        &pwrMan_InitConfig,
    };    
    
    power_manager_callback_user_config_t *powerStaticCallbacksConfigsArr[] = {(void *)0};
    
    s = POWER_SYS_Init(&powerConfigsArr, 1U, &powerStaticCallbacksConfigsArr, 0);
    
    s = POWER_SYS_SetMode(0U, POWER_MANAGER_POLICY_AGREEMENT);
    
    return s;
}


static void disable_wdt_internal(void)
{
    /* Write of the WDOG unlock key to CNT register, must be done in order to allow any modifications*/
    WDOG->CNT = (uint32_t ) FEATURE_WDOG_UNLOCK_VALUE;
    /* The dummy read is used in order to make sure that the WDOG registers will be configured only
    * after the write of the unlock value was completed. */
    (void)WDOG->CNT;

    /* Initial write of WDOG configuration register:
     * enables support for 32-bit refresh/unlock command write words,
     * clock select from LPO, update enable, watchdog disabled */
    WDOG->CS  = (uint32_t)((1UL << WDOG_CS_CMD32EN_SHIFT)                     |
                           (FEATURE_WDOG_CLK_FROM_LPO << WDOG_CS_CLK_SHIFT)   |
                           (0U << WDOG_CS_EN_SHIFT)                           |
                           (1U << WDOG_CS_UPDATE_SHIFT)                       );

    /* Configure timeout */
    WDOG->TOVAL = (uint32_t)0xFFFF;
}

static void wdt_reinit(void)
{
	status_t ret = STATUS_SUCCESS;
    wdog_user_config_t wdt_cfg;
    
    /* 2 second internal WDT Timeout with prescaler enabled */
    wdt_cfg.clkSource       = WDOG_LPO_CLOCK;
    wdt_cfg.opMode.wait     = false; 
    wdt_cfg.opMode.stop     = false;
    wdt_cfg.opMode.debug    = false;
    wdt_cfg.updateEnable    = true;
    wdt_cfg.intEnable       = true;
    wdt_cfg.winEnable       = false;
    wdt_cfg.windowValue     = 0U;
    wdt_cfg.timeoutValue    = 2000U;
    wdt_cfg.prescalerEnable = true;
        
    ret = WDOG_Config(WDOG, &wdt_cfg);
	DEV_ASSERT(ret == STATUS_SUCCESS);
}

static void sleep_irq_disable(void)
{
    /* Disable external IRQs */
    NVIC_DisableIRQ(PORTA_IRQn);
    NVIC_DisableIRQ(PORTC_IRQn);
    NVIC_DisableIRQ(PORTD_IRQn);
    NVIC_DisableIRQ(PORTE_IRQn);
    NVIC_DisableIRQ(PORTB_IRQn);
    
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_ClearPendingIRQ(PORTC_IRQn);
    NVIC_ClearPendingIRQ(PORTD_IRQn);
    NVIC_ClearPendingIRQ(PORTE_IRQn);
    NVIC_ClearPendingIRQ(PORTB_IRQn);    
}

static void sleep_peripheral_disable(uint32_t at_boot)
{
    /* Abort any transfers initiated over SPI */
    (void)LPSPI_DRV_MasterAbortTransfer(0U);
    (void)LPSPI_DRV_MasterAbortTransfer(1U);
    (void)LPSPI_DRV_MasterAbortTransfer(2U);
    
    (void)LPSPI_DRV_MasterDeinit(0U);
    (void)LPSPI_DRV_MasterDeinit(1U);
    (void)LPSPI_DRV_MasterDeinit(2U);
    
    (void)LPI2C_DRV_MasterDeinit(RTC_I2C_IF);
    (void)FTM_DRV_Deinit(SYS_FTM_INST);
    (void)LPUART_DRV_Deinit(1U);   

    if(at_boot == 0U)
    {
        update_odo_persistence();
        mc_ctxt_update_persitence();
    }
    
    /* Write pending parameters to NVM */
    /* TBD: regen percentage, mode */
    
    /* Shutdown Crypto */
    CSEC_DRV_Deinit();
    
    (void)EDMA_DRV_Deinit();
    
    ENET_DRV_SetSleepMode(0U, 1U);
}

static void sleep_can_disable(void)
{
    (void)FLEXCAN_DRV_Deinit(CAN_IF_BMS);
    PINS_DRV_ClearPins(BMS_CAN_EN_GPIO, (1U << BMS_CAN_EN_PIN));
    PINS_DRV_ClearPins(BMS_STB_N_GPIO, (1U << BMS_STB_N_PIN));

    (void)FLEXCAN_DRV_AbortTransfer(CAN_IF_MOTOR, RX_MAILBOX);
    (void)FLEXCAN_DRV_AbortTransfer(CAN_IF_MOTOR, CAN_MC_TX_MAILBOX);
    (void)FLEXCAN_DRV_Deinit(CAN_IF_MOTOR);

    PINS_DRV_ClearPins(MOTOR_CAN_EN_GPIO, (1U << MOTOR_CAN_EN_PIN));
    PINS_DRV_ClearPins(MOTOR_STB_N_GPIO, (1U << MOTOR_STB_N_PIN));
    
    
    (void)FLEXCAN_DRV_AbortTransfer(CAN_IF_ABS, CAN_DBA_RX_MAILBOX);
    (void)FLEXCAN_DRV_AbortTransfer(CAN_IF_ABS, CAN_DBA_TX_MAILBOX);
    (void)FLEXCAN_DRV_Deinit(CAN_IF_ABS);

#if 0
    PINS_DRV_ClearPins(DBA_CAN_EN_GPIO, (1U << DBA_CAN_EN_PIN));
    PINS_DRV_ClearPins(DBA_STB_N_GPIO, (1U << DBA_STB_N_PIN));
#endif    
}

static void sys_scb_config(void)
{
    /* Enable DeepSleep */
    S32_SCB->SCR |= S32_SCB_SCR_SLEEPDEEP_MASK;
}

#ifdef USE_FEATURE_LP_SLEEP_MODE
static void dba_config_can(void)
{
    /* Configure CAN RX Pin as Input, Wake up on IRQ */
    PINS_SetMuxModeSel(DBA_CAN_RX_PORT, DBA_CAN_RX_PIN, PORT_MUX_AS_GPIO);
    PINS_DRV_SetPinDirection(DBA_CAN_RX_GPIO, DBA_CAN_RX_PIN, GPIO_INPUT_DIRECTION);
    PINS_DRV_SetPinIntSel(DBA_CAN_RX_PORT, DBA_CAN_RX_PIN, PORT_INT_FALLING_EDGE);

    NVIC_EnableIRQ(PORTB_IRQn);
}

static void can_if_sleep_init(void)
{
    PINS_DRV_SetPins(DBA_CAN_EN_GPIO, (1U << DBA_CAN_EN_PIN));
    PINS_DRV_SetPins(DBA_STB_N_GPIO, (1U << DBA_STB_N_PIN));
}
#endif /* USE_FEATURE_LP_SLEEP_MODE */

static void cluster_power_off(void)
{
    /* Enable power supply for i.MX */
    PINS_DRV_ClearPins(IMX6_VDD_EN_GPIO, (1U << IMX6_VDD_EN_PIN));
    PINS_DRV_SetPins(IMX6_PMIC_EN_GPIO, (1U << IMX6_PMIC_EN_PIN));

    /* Release reset for I.MX6 */
    PINS_DRV_SetPins(IMX6_RESET_GPIO, (1U << IMX6_RESET_PIN));
    PINS_DRV_ClearPins(MODE_GPIO_SW2_GPIO, (1U << MODE_GPIO_SW2_PIN));
}

void sleep_at_boot(void)
{
    (void)osif_kernel_suspend();
    
    disable_wdt_internal();
    mpu_unprotect_vector();
    
    sleep_irq_disable();

    sleep_can_disable();
    
    sleep_peripheral_disable(1U);
    
    cluster_power_off();
    
    /* Turn OFF AUX */

    bcm_control(BCM_AUX_CTRL, BCM_CTRL_STATE_OFF);
    bcm_control(BCM_NBR_PLATE_CTRL, BCM_CTRL_STATE_OFF);
    
    PINS_DRV_SetPins(RESET_MCU_GPIO, (1U << RESET_MCU_PIN));

    PINS_DRV_ClearPins(VBAT_EN1_GPIO, (1U << VBAT_EN1_PIN));
    PINS_DRV_SetPins(LED_GREEN_GPIO, (1U << LED_GREEN_PIN));
    PINS_DRV_SetPins(LED_RED_GPIO, (1U << LED_RED_PIN));
    
    /* Enable KEY IRQ */
    NVIC_EnableIRQ(PORTA_IRQn);
    mpu_protect_vector();
    
    /* Shutdown the Systick Timer & disable systick IRQ*/
    S32_SysTick->CSR = (uint32_t)S32_SysTick_CSR_ENABLE(0U);
    S32_SysTick->CSR = (uint32_t)S32_SysTick_CSR_ENABLE(0U) | S32_SysTick_CSR_TICKINT(0U);

    sys_scb_config();
    __DMB();
    
    /* No LED must turn ON beyond this point, unless woken up */
    DEV_ASSERT(run_to_vlpr() == STATUS_SUCCESS);
    __DSB();
    __ISB();
    
#ifndef USE_FEATURE_LP_SLEEP_MODE
    SystemCoreClockUpdate();
    
    __WFI();    
    __enable_irq(); 

    SystemSoftwareReset();
#else
    lp_sleep();
#endif /* USE_FEATURE_LP_SLEEP_MODE */
}

void sleep(uint32_t reboot_request)
{
    (void)osif_kernel_suspend();
    
    disable_wdt_internal();
    mpu_unprotect_vector();
    
    sleep_irq_disable();
    sleep_peripheral_disable(0U);
    sleep_can_disable();
    ENET_DRV_SetSleepMode(0U, 1U);
    
    /* Shutdown the Systick Timer & disable systick IRQ*/
    S32_SysTick->CSR = (uint32_t)S32_SysTick_CSR_ENABLE(0U);
    S32_SysTick->CSR = (uint32_t)S32_SysTick_CSR_ENABLE(0U) | S32_SysTick_CSR_TICKINT(0U); 
    
    /* Turn OFF power to i.MX */
    PINS_DRV_ClearPins(IMX6_PMIC_EN_GPIO, (1U << IMX6_PMIC_EN_PIN));
    PINS_DRV_ClearPins(IMX6_VDD_EN_GPIO, (1U << IMX6_VDD_EN_PIN));

    /* Turn OFF AUX */
    bcm_control(BCM_AUX_CTRL, BCM_CTRL_STATE_OFF);

    bcm_control(BCM_NBR_PLATE_CTRL, BCM_CTRL_STATE_OFF);
    
    PINS_DRV_SetPins(RESET_MCU_GPIO, (1U << RESET_MCU_PIN));
    PINS_DRV_ClearPins(VBAT_EN1_GPIO, (1U << VBAT_EN1_PIN));

    /* Enable KEY IRQ */
    NVIC_EnableIRQ(PORTA_IRQn);
    mpu_protect_vector();

    PINS_DRV_SetPins(LED_GREEN_GPIO, (1U << LED_GREEN_PIN));
    PINS_DRV_SetPins(LED_RED_GPIO, (1U << LED_RED_PIN));

    sys_scb_config();
    __DMB();
    
    if(reboot_request == 0x0013770FU)
    {
        SystemSoftwareReset();
    }
    
    /* No LED must turn ON beyond this point, unless woken up */
    DEV_ASSERT(run_to_vlpr() == STATUS_SUCCESS);
    __DSB();
    __ISB();
    
#ifndef USE_FEATURE_LP_SLEEP_MODE
    SystemCoreClockUpdate();
    
    __WFI();    
    __enable_irq(); 

    SystemSoftwareReset();
#else
    lp_sleep();
#endif /* USE_FEATURE_LP_SLEEP_MODE */
}

#ifdef USE_FEATURE_LP_SLEEP_MODE
__NO_RETURN static void lp_sleep(void)
{
    volatile uint32_t key_sw = 0U;     

    SystemCoreClockUpdate();
    can_if_sleep_init();
    dba_config_can();
    
    __DSB();
    __DMB();
    __ISB();
    
    /* Check key state before sleeping */
    key_sw = PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
    
    if((key_sw & (1U << KEY_WAKE_SIG_PIN)) == 0U)
    {
        SystemSoftwareReset();
    }

    for(;; )
    {     
        __WFI();    
        __enable_irq(); 

        key_sw = PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        key_sw |= PINS_DRV_ReadPins(KEY_WAKE_SIG_GPIO);
        
        if(((pin_val & (1U << DBA_CAN_RX_PIN)) > 0U) ||
           ((key_sw & (1U << KEY_WAKE_SIG_PIN)) == 0U))
        {
            SystemSoftwareReset();
        }
        
        NVIC_EnableIRQ(PORTB_IRQn);
    }    
}

void sleep_set_pin(uint32_t pins)
{
    pin_val = pins;
}

#endif /* USE_FEATURE_LP_SLEEP_MODE */
