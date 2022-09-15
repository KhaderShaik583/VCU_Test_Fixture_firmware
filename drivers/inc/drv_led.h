#ifndef DRV_LED_H
#define DRV_LED_H

#include <stdint.h>
#include "fw_common.h"
#include "fw_features.h"

#define NUM_LEDS    (2U)
#define LED_RED     (0U)
#define LED_GREEN   (1U)

void drv_led_on(uint32_t led_num);
void drv_led_off(uint32_t led_num);
void drv_led_toggle(uint32_t led_num);

void drv_led_init(void);
status_t drv_led_sem_acquire(uint32_t timeout);
status_t drv_led_sem_release(void);

#endif /* DRV_LED_H */
