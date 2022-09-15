#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include "fw_common.h"

#ifdef  USE_DEBUG_PRINTS
#define SYS_DEBUG_LPUART_INTERFACE   (0U)
#define SYS_DEBUG_LPUART_BAUD        (115200U)
#endif

/* Debug console type definition. */
#define DEBUG_CONSOLE_DEVICE_TYPE_NONE      0U   /*!< No debug console.             */
#define DEBUG_CONSOLE_DEVICE_TYPE_UART      1U   /*!< Debug console base on UART.   */
#define DEBUG_CONSOLE_DEVICE_TYPE_LPUART    2U /*!< Debug console base on LPUART. */

#ifndef SDK_DEBUGCONSOLE
#define SDK_DEBUGCONSOLE 1U
#endif

/*! @brief Definition to printf float number. */
#ifndef PRINTF_FLOAT_ENABLE
#define PRINTF_FLOAT_ENABLE 1U
#endif /* PRINTF_FLOAT_ENABLE */

/*! @brief Definition to scanf float number. */
#ifndef SCANF_FLOAT_ENABLE
#define SCANF_FLOAT_ENABLE 0U
#endif /* SCANF_FLOAT_ENABLE */

/*! @brief Definition to support advanced format specifier for printf. */
#ifndef PRINTF_ADVANCED_ENABLE
#define PRINTF_ADVANCED_ENABLE 1U
#endif /* PRINTF_ADVANCED_ENABLE */

/*! @brief Definition to support advanced format specifier for scanf. */
#ifndef SCANF_ADVANCED_ENABLE
#define SCANF_ADVANCED_ENABLE 0U
#endif /* SCANF_ADVANCED_ENABLE */

#if SDK_DEBUGCONSOLE /* Select printf, scanf, putchar, getchar of SDK version. */
#define PRINTF DbgConsole_Printf
#define SCANF DbgConsole_Scanf
#define PUTCHAR DbgConsole_Putchar
#define GETCHAR DbgConsole_Getchar
#else /* Select printf, scanf, putchar, getchar of toolchain. */
#define PRINTF printf
#define SCANF scanf
#define PUTCHAR putchar
#define GETCHAR getchar
#endif /* SDK_DEBUGCONSOLE */


int32_t DbgConsole_Printf(const char *fmt_s, ...);
status_t DbgConsole_Init(uint32_t instance, uint32_t baudRate, uint8_t device);

#ifdef USE_FEATURE_CORE_DUMP
void exception_printf_override(status_t (*LPUART_CB)(uint32_t instance, const uint8_t *buffer, uint32_t length, uint32_t timeout));
#endif

#ifdef USE_DEBUG_PRINTS
#define dbg_printf(x, ...)  PRINTF((x), ##__VA_ARGS__)
#else
#define dbg_printf(x, ...)
#endif


#endif /* DEBUG_CONSOLE_H */
