#include "drv_led.h"
#include "pins_driver.h"
#include "board.h" 
#include "osif.h"

static const GPIO_Type *led_gpio_map[NUM_LEDS] = { LED_RED_GPIO, LED_GREEN_GPIO };
static const uint32_t led_pin_map[NUM_LEDS] = { LED_RED_PIN, LED_GREEN_PIN };

/* Driver semaphore */
static semaphore_t drv_led_sem_t;

void drv_led_init(void)
{
    status_t s;
    
    s = OSIF_SemaCreate(&drv_led_sem_t, 0U);
    
    DEV_ASSERT(s == STATUS_SUCCESS);
    
}

status_t drv_led_sem_acquire(uint32_t timeout)
{
    status_t s;
    s = OSIF_SemaWait(&drv_led_sem_t, timeout);
    return s;
}

status_t drv_led_sem_release(void)
{
    status_t s;
    s = OSIF_SemaPost(&drv_led_sem_t);
    return s;
}


void drv_led_on(uint32_t led_num)
{
    PINS_DRV_ClearPins((GPIO_Type *)led_gpio_map[led_num], 1U << led_pin_map[led_num]);
}

void drv_led_off(uint32_t led_num)
{
    PINS_DRV_SetPins((GPIO_Type *)led_gpio_map[led_num], 1U << led_pin_map[led_num]);
}

void drv_led_toggle(uint32_t led_num)
{
    PINS_DRV_TogglePins((GPIO_Type *)led_gpio_map[led_num], 1U << led_pin_map[led_num]);
}


