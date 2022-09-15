#ifndef PIN_MUX_H
#define PIN_MUX_H

/* MODULE pin_mux. */
#include "pins_driver.h"

/*! @brief User number of configured pins */
#define NUM_OF_CONFIGURED_PINS      (116U)

/*! @brief User configuration structure */
extern pin_settings_config_t g_pin_mux_InitConfigArr[NUM_OF_CONFIGURED_PINS];

/* END pin_mux. */

#endif /* PIN_MUX_H */
