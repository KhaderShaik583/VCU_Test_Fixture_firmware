
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include "lpuart_driver.h"
#include "debug_console.h" 
#include "osif.h"

#define USE_BLOCKING_TRANSFER
#define IO_MAXLINE      20U

static lpuart_state_t lpuart1_State;
typedef int32_t (*PUTCHAR_FUNC)(int32_t a);

/* Forward declarations */
static int32_t DbgConsole_Putchar(int32_t ch);
static int32_t DbgConsole_PrintfFormattedData(PUTCHAR_FUNC func_ptr, const char *fmt, va_list ap);
static int32_t DbgConsole_ConvertRadixNumToString(char *numstr, void *nump, int32_t neg, int32_t radix, bool use_caps);
static void DbgConsole_PrintfPaddingCharacter(char c, int32_t curlen, int32_t width, int32_t *count, PUTCHAR_FUNC func_ptr);
static int32_t DbgConsole_ConvertFloatRadixNumToString(char *numstr, void *nump, int32_t radix, uint32_t precision_width);


#if PRINTF_ADVANCED_ENABLE
/*! @brief Specification modifier flags for printf. */
enum _debugconsole_printf_flag
{
    kPRINTF_Minus               = 0x01U,   /*!< Minus FLag. */
    kPRINTF_Plus                = 0x02U,   /*!< Plus Flag. */
    kPRINTF_Space               = 0x04U,   /*!< Space Flag. */
    kPRINTF_Zero                = 0x08U,   /*!< Zero Flag. */
    kPRINTF_Pound               = 0x10U,   /*!< Pound Flag. */
    kPRINTF_LengthChar          = 0x20U,   /*!< Length: Char Flag. */
    kPRINTF_LengthShortInt      = 0x40U,   /*!< Length: Short Int Flag. */
    kPRINTF_LengthLongInt       = 0x80U,   /*!< Length: Long Int Flag. */
    kPRINTF_LengthLongLongInt   = 0x100U,  /*!< Length: Long Long Int Flag. */
};
#endif /* PRINTF_ADVANCED_ENABLE */


/*! @brief Operation functions definitions for debug console. */
typedef struct DebugConsoleOperationFunctions
{
#ifdef USE_BLOCKING_TRANSFER
    union
    {
        status_t (*PutChar)(uint32_t instance, const uint8_t *buffer, size_t length, uint32_t timeout);
        status_t (*UART_PutChar)(uint32_t instance, const uint8_t *buffer, size_t length, uint32_t timeout);
        status_t (*LPUART_PutChar)(uint32_t instance, const uint8_t *buffer, uint32_t length, uint32_t timeout);
    } tx_union;
    
    union
    {
        status_t (*GetChar)(uint32_t instance, const uint8_t *buffer, size_t length, uint32_t timeout);
        status_t (*UART_GetChar)(uint32_t instance, uint8_t *buffer, size_t length, uint32_t timeout);
        status_t (*LPUART_GetChar)(uint32_t instance, uint8_t *buffer, uint32_t length, uint32_t timeout);
    } rx_union;
#else
    union
    {
        status_t (*PutChar)(uint32_t instance, const uint8_t *buffer, size_t length);
        status_t (*UART_PutChar)(uint32_t instance, const uint8_t *buffer, size_t length);
        status_t (*LPUART_PutChar)(uint32_t instance, const uint8_t *buffer, uint32_t length);
    } tx_union;
    
    union
    {
        status_t (*GetChar)(uint32_t instance, const uint8_t *buffer, size_t length);
        status_t (*UART_GetChar)(uint32_t instance, uint8_t *buffer, size_t length);
        status_t (*LPUART_GetChar)(uint32_t instance, uint8_t *buffer, uint32_t length);
    } rx_union;    
#endif
    
} debug_console_ops_t;

/*! @brief State structure storing debug console. */
typedef struct DebugConsoleState
{
    uint8_t type;            /*!< Indicator telling whether the debug console is initialized. */
    uint32_t instance;              /*!< Base of the IP register. */
    debug_console_ops_t ops; /*!< Operation function pointers for debug UART operations. */
} debug_console_state_t;


static debug_console_state_t s_debugConsole = {.type = DEBUG_CONSOLE_DEVICE_TYPE_NONE, .instance = 0U, .ops = {{0}, {0}}};

status_t DbgConsole_Init(uint32_t instance, uint32_t baudRate, uint8_t device)
{
    lpuart_user_config_t lpuart1_InitConfig0;
    
    if (s_debugConsole.type != DEBUG_CONSOLE_DEVICE_TYPE_NONE)
    {
        return STATUS_ERROR;
    }

    /* Set debug console to initialized to avoid duplicated initialized operation. */
    s_debugConsole.type = device;

    /* Switch between different device. */
    switch (device)
    {
        case DEBUG_CONSOLE_DEVICE_TYPE_LPUART:
        {
            lpuart1_InitConfig0.transferType = LPUART_USING_INTERRUPTS;
            lpuart1_InitConfig0.baudRate = baudRate;
            lpuart1_InitConfig0.parityMode = LPUART_PARITY_DISABLED;
            lpuart1_InitConfig0.stopBitCount = LPUART_ONE_STOP_BIT;
            lpuart1_InitConfig0.bitCountPerChar = LPUART_8_BITS_PER_CHAR;
            lpuart1_InitConfig0.rxDMAChannel = 0U;
            lpuart1_InitConfig0.txDMAChannel = 0U;
            
            s_debugConsole.instance = instance;
            
            LPUART_DRV_Init(s_debugConsole.instance, &lpuart1_State, &lpuart1_InitConfig0);
#ifdef USE_BLOCKING_TRANSFER
            /* Set the function pointer for send and receive for this kind of device. */
            s_debugConsole.ops.tx_union.LPUART_PutChar = LPUART_DRV_SendDataBlocking;
            s_debugConsole.ops.rx_union.LPUART_GetChar = LPUART_DRV_ReceiveDataBlocking;
#else
             /* Set the function pointer for send and receive for this kind of device. */
            s_debugConsole.ops.tx_union.LPUART_PutChar = LPUART_DRV_SendData;
            s_debugConsole.ops.rx_union.LPUART_GetChar = LPUART_DRV_ReceiveData;           
#endif
        }
        break;
    }

    return STATUS_SUCCESS;
}

/*!
 * @brief Converts a radix number to a string and return its length.
 *
 * @param[in] numstr    Converted string of the number.
 * @param[in] nump      Pointer to the number.
 * @param[in] neg       Polarity of the number.
 * @param[in] radix     The radix to be converted to.
 * @param[in] use_caps  Used to identify %x/X output format.

 * @return Length of the converted string.
 */
static int32_t DbgConsole_ConvertRadixNumToString(char *numstr, void *nump, int32_t neg, int32_t radix, bool use_caps)
{
#if PRINTF_ADVANCED_ENABLE
    int64_t a;
    int64_t b;
    int64_t c;

    uint64_t ua;
    uint64_t ub;
    uint64_t uc;
#else
    int32_t a;
    int32_t b;
    int32_t c;

    uint32_t ua;
    uint32_t ub;
    uint32_t uc;
#endif /* PRINTF_ADVANCED_ENABLE */

    int32_t nlen;
    char *nstrp;

    nlen = 0;
    nstrp = numstr;
    *nstrp++ = '\0';

    if (neg)
    {
#if PRINTF_ADVANCED_ENABLE
        a = *(int64_t *)nump;
#else
        a = *(int32_t *)nump;
#endif /* PRINTF_ADVANCED_ENABLE */
        if (a == 0)
        {
            *nstrp = '0';
            ++nlen;
            return nlen;
        }
        while (a != 0)
        {
#if PRINTF_ADVANCED_ENABLE
            b = (int64_t)a / (int64_t)radix;
            c = (int64_t)a - ((int64_t)b * (int64_t)radix);
            if (c < 0)
            {
                uc = (uint64_t)c;
                c = (int64_t)(~uc) + 1 + '0';
            }
#else
            b = a / radix;
            c = a - (b * radix);
            if (c < 0)
            {
                uc = (uint32_t)c;
                c = (uint32_t)(~uc) + 1 + '0';
            }
#endif /* PRINTF_ADVANCED_ENABLE */
            else
            {
                c = c + '0';
            }
            a = b;
            *nstrp++ = (char)c;
            ++nlen;
        }
    }
    else
    {
#if PRINTF_ADVANCED_ENABLE
        ua = *(uint64_t *)nump;
#else
        ua = *(uint32_t *)nump;
#endif /* PRINTF_ADVANCED_ENABLE */
        if (ua == 0)
        {
            *nstrp = '0';
            ++nlen;
            return nlen;
        }
        while (ua != 0)
        {
#if PRINTF_ADVANCED_ENABLE
            ub = (uint64_t)ua / (uint64_t)radix;
            uc = (uint64_t)ua - ((uint64_t)ub * (uint64_t)radix);
#else
            ub = ua / (uint32_t)radix;
            uc = ua - (ub * (uint32_t)radix);
#endif /* PRINTF_ADVANCED_ENABLE */

            if (uc < 10)
            {
                uc = uc + '0';
            }
            else
            {
                uc = uc - 10 + (use_caps ? 'A' : 'a');
            }
            ua = ub;
            *nstrp++ = (char)uc;
            ++nlen;
        }
    }
    return nlen;
}

/*!
 * @brief This function puts padding character.
 *
 * @param[in] c         Padding character.
 * @param[in] curlen    Length of current formatted string .
 * @param[in] width     Width of expected formatted string.
 * @param[in] count     Number of characters.
 * @param[in] func_ptr  Function to put character out.
 */
static void DbgConsole_PrintfPaddingCharacter(char c, int32_t curlen, int32_t width, int32_t *count, PUTCHAR_FUNC func_ptr)
{
    int32_t i;

    for (i = curlen; i < width; i++)
    {
        func_ptr(c);
        (*count)++;
    }
}


#if PRINTF_FLOAT_ENABLE
/*!
 * @brief Converts a floating radix number to a string and return its length.
 *
 * @param[in] numstr            Converted string of the number.
 * @param[in] nump              Pointer to the number.
 * @param[in] radix             The radix to be converted to.
 * @param[in] precision_width   Specify the precision width.

 * @return Length of the converted string.
 */
static int32_t DbgConsole_ConvertFloatRadixNumToString(char *numstr,
                                                       void *nump,
                                                       int32_t radix,
                                                       uint32_t precision_width)
{
    int32_t a;
    int32_t b;
    int32_t c;
    int32_t i;
    uint32_t uc;
    double fa;
    double dc;
    double fb;
    double r;
    double fractpart;
    double intpart;

    int32_t nlen;
    char *nstrp;
    nlen = 0;
    nstrp = numstr;
    *nstrp++ = '\0';
    r = *(double *)nump;
    if (!r)
    {
        *nstrp = '0';
        ++nlen;
        return nlen;
    }
    fractpart = modf((double)r, (double *)&intpart);
    /* Process fractional part. */
    for (i = 0; i < precision_width; i++)
    {
        fractpart *= radix;
    }
    if (r >= 0)
    {
        fa = fractpart + (double)0.5;
        if (fa >= pow(10, precision_width))
        {
            intpart++;
        }
    }
    else
    {
        fa = fractpart - (double)0.5;
        if (fa <= -pow(10, precision_width))
        {
            intpart--;
        }
    }
    for (i = 0; i < precision_width; i++)
    {
        fb = fa / (int32_t)radix;
        dc = (fa - (int64_t)fb * (int32_t)radix);
        c = (int32_t)dc;
        if (c < 0)
        {
            uc = (uint32_t)c;
            c = (int32_t)(~uc) + 1 + '0';
        }
        else
        {
            c = c + '0';
        }
        fa = fb;
        *nstrp++ = (char)c;
        ++nlen;
    }
    *nstrp++ = (char)'.';
    ++nlen;
    a = (int32_t)intpart;
    if (a == 0)
    {
        *nstrp++ = '0';
        ++nlen;
    }
    else
    {
        while (a != 0)
        {
            b = (int32_t)a / (int32_t)radix;
            c = (int32_t)a - ((int32_t)b * (int32_t)radix);
            if (c < 0)
            {
                uc = (uint32_t)c;
                c = (int32_t)(~uc) + 1 + '0';
            }
            else
            {
                c = c + '0';
            }
            a = b;
            *nstrp++ = (char)c;
            ++nlen;
        }
    }
    return nlen;
}
#endif /* PRINTF_FLOAT_ENABLE */

static int32_t DbgConsole_PrintfFormattedData(PUTCHAR_FUNC func_ptr, const char *fmt, va_list ap)
{
    /* va_list ap; */
    char *p;
    int32_t c;

    char vstr[33];
    char *vstrp = NULL;
    int32_t vlen = 0;

    int32_t done;
    int32_t count = 0;

    uint32_t field_width;
    uint32_t precision_width;
    char *sval;
    int32_t cval;
    bool use_caps;
    uint8_t radix = 0;

#if PRINTF_ADVANCED_ENABLE
    uint32_t flags_used;
    int32_t schar, dschar;
    int64_t ival;
    uint64_t uval = 0;
#else
    int32_t ival;
    uint32_t uval = 0;
#endif /* PRINTF_ADVANCED_ENABLE */

#if PRINTF_FLOAT_ENABLE
    double fval;
#endif /* PRINTF_FLOAT_ENABLE */

    /* Start parsing apart the format string and display appropriate formats and data. */
    for (p = (char *)fmt; (c = *p) != 0; p++)
    {
        /*
         * All formats begin with a '%' marker.  Special chars like
         * '\n' or '\t' are normally converted to the appropriate
         * character by the __compiler__.  Thus, no need for this
         * routine to account for the '\' character.
         */
        if (c != '%')
        {
            func_ptr(c);
            count++;
            /* By using 'continue', the next iteration of the loop is used, skipping the code that follows. */
            continue;
        }

        use_caps = true;

#if PRINTF_ADVANCED_ENABLE
        /* First check for specification modifier flags. */
        flags_used = 0;
        done = false;
        while (!done)
        {
            switch (*++p)
            {
                case '-':
                    flags_used |= kPRINTF_Minus;
                    break;
                case '+':
                    flags_used |= kPRINTF_Plus;
                    break;
                case ' ':
                    flags_used |= kPRINTF_Space;
                    break;
                case '0':
                    flags_used |= kPRINTF_Zero;
                    break;
                case '#':
                    flags_used |= kPRINTF_Pound;
                    break;
                default:
                    /* We've gone one char too far. */
                    --p;
                    done = true;
                    break;
            }
        }
#endif /* PRINTF_ADVANCED_ENABLE */

        /* Next check for minimum field width. */
        field_width = 0;
        done = false;
        while (!done)
        {
            c = *++p;
            if ((c >= '0') && (c <= '9'))
            {
                field_width = (field_width * 10) + (c - '0');
            }
            else
            {
                /* We've gone one char too far. */
                --p;
                done = true;
            }
        }
        /* Next check for the width and precision field separator. */
        precision_width = 6;
        if (*++p == '.')
        {
            /* Must get precision field width, if present. */
            precision_width = 0;
            done = false;
            while (!done)
            {
                c = *++p;
                if ((c >= '0') && (c <= '9'))
                {
                    precision_width = (precision_width * 10) + (c - '0');
                }
                else
                {
                    /* We've gone one char too far. */
                    --p;
                    done = true;
                }
            }
        }
        else
        {
            /* We've gone one char too far. */
            --p;
        }
#if PRINTF_ADVANCED_ENABLE
        /*
         * Check for the length modifier.
         */
        switch (/* c = */ *++p)
        {
            case 'h':
                if (*++p != 'h')
                {
                    flags_used |= kPRINTF_LengthShortInt;
                    --p;
                }
                else
                {
                    flags_used |= kPRINTF_LengthChar;
                }
                break;
            case 'l':
                if (*++p != 'l')
                {
                    flags_used |= kPRINTF_LengthLongInt;
                    --p;
                }
                else
                {
                    flags_used |= kPRINTF_LengthLongLongInt;
                }
                break;
            default:
                /* we've gone one char too far */
                --p;
                break;
        }
#endif /* PRINTF_ADVANCED_ENABLE */
        /* Now we're ready to examine the format. */
        c = *++p;
        {
            if ((c == 'd') || (c == 'i') || (c == 'f') || (c == 'F') || (c == 'x') || (c == 'X') || (c == 'o') ||
                (c == 'b') || (c == 'p') || (c == 'u'))
            {
                if ((c == 'd') || (c == 'i'))
                {
#if PRINTF_ADVANCED_ENABLE
                    if (flags_used & kPRINTF_LengthLongLongInt)
                    {
                        ival = (int64_t)va_arg(ap, int64_t);
                    }
                    else
#endif /* PRINTF_ADVANCED_ENABLE */
                    {
                        ival = (int32_t)va_arg(ap, int32_t);
                    }
                    vlen = DbgConsole_ConvertRadixNumToString(vstr, &ival, true, 10, use_caps);
                    vstrp = &vstr[vlen];
#if PRINTF_ADVANCED_ENABLE
                    if (ival < 0)
                    {
                        schar = '-';
                        ++vlen;
                    }
                    else
                    {
                        if (flags_used & kPRINTF_Plus)
                        {
                            schar = '+';
                            ++vlen;
                        }
                        else
                        {
                            if (flags_used & kPRINTF_Space)
                            {
                                schar = ' ';
                                ++vlen;
                            }
                            else
                            {
                                schar = 0;
                            }
                        }
                    }
                    dschar = false;
                    /* Do the ZERO pad. */
                    if (flags_used & kPRINTF_Zero)
                    {
                        if (schar)
                        {
                            func_ptr(schar);
                            count++;
                        }
                        dschar = true;

                        DbgConsole_PrintfPaddingCharacter('0', vlen, field_width, &count, func_ptr);
                        vlen = field_width;
                    }
                    else
                    {
                        if (!(flags_used & kPRINTF_Minus))
                        {
                            DbgConsole_PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr);
                            if (schar)
                            {
                                func_ptr(schar);
                                count++;
                            }
                            dschar = true;
                        }
                    }
                    /* The string was built in reverse order, now display in correct order. */
                    if ((!dschar) && schar)
                    {
                        func_ptr(schar);
                        count++;
                    }
#endif /* PRINTF_ADVANCED_ENABLE */
                }

#if PRINTF_FLOAT_ENABLE
                if ((c == 'f') || (c == 'F'))
                {
                    fval = (double)va_arg(ap, double);
                    vlen = DbgConsole_ConvertFloatRadixNumToString(vstr, &fval, 10, precision_width);
                    vstrp = &vstr[vlen];

#if PRINTF_ADVANCED_ENABLE
                    if (fval < 0)
                    {
                        schar = '-';
                        ++vlen;
                    }
                    else
                    {
                        if (flags_used & kPRINTF_Plus)
                        {
                            schar = '+';
                            ++vlen;
                        }
                        else
                        {
                            if (flags_used & kPRINTF_Space)
                            {
                                schar = ' ';
                                ++vlen;
                            }
                            else
                            {
                                schar = 0;
                            }
                        }
                    }
                    dschar = false;
                    if (flags_used & kPRINTF_Zero)
                    {
                        if (schar)
                        {
                            func_ptr(schar);
                            count++;
                        }
                        dschar = true;
                        DbgConsole_PrintfPaddingCharacter('0', vlen, field_width, &count, func_ptr);
                        vlen = field_width;
                    }
                    else
                    {
                        if (!(flags_used & kPRINTF_Minus))
                        {
                            DbgConsole_PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr);
                            if (schar)
                            {
                                func_ptr(schar);
                                count++;
                            }
                            dschar = true;
                        }
                    }
                    if ((!dschar) && schar)
                    {
                        func_ptr(schar);
                        count++;
                    }
#endif /* PRINTF_ADVANCED_ENABLE */
                }
#endif /* PRINTF_FLOAT_ENABLE */
                if ((c == 'X') || (c == 'x'))
                {
                    if (c == 'x')
                    {
                        use_caps = false;
                    }
#if PRINTF_ADVANCED_ENABLE
                    if (flags_used & kPRINTF_LengthLongLongInt)
                    {
                        uval = (uint64_t)va_arg(ap, uint64_t);
                    }
                    else
#endif /* PRINTF_ADVANCED_ENABLE */
                    {
                        uval = (uint32_t)va_arg(ap, uint32_t);
                    }
                    vlen = DbgConsole_ConvertRadixNumToString(vstr, &uval, false, 16, use_caps);
                    vstrp = &vstr[vlen];

#if PRINTF_ADVANCED_ENABLE
                    dschar = false;
                    if (flags_used & kPRINTF_Zero)
                    {
                        if (flags_used & kPRINTF_Pound)
                        {
                            func_ptr('0');
                            func_ptr((use_caps ? 'X' : 'x'));
                            count += 2;
                            /*vlen += 2;*/
                            dschar = true;
                        }
                        DbgConsole_PrintfPaddingCharacter('0', vlen, field_width, &count, func_ptr);
                        vlen = field_width;
                    }
                    else
                    {
                        if (!(flags_used & kPRINTF_Minus))
                        {
                            if (flags_used & kPRINTF_Pound)
                            {
                                vlen += 2;
                            }
                            DbgConsole_PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr);
                            if (flags_used & kPRINTF_Pound)
                            {
                                func_ptr('0');
                                func_ptr(use_caps ? 'X' : 'x');
                                count += 2;

                                dschar = true;
                            }
                        }
                    }

                    if ((flags_used & kPRINTF_Pound) && (!dschar))
                    {
                        func_ptr('0');
                        func_ptr(use_caps ? 'X' : 'x');
                        count += 2;
                        vlen += 2;
                    }
#endif /* PRINTF_ADVANCED_ENABLE */
                }
                if (c == 'o')
                {
                    uval = (uint32_t)va_arg(ap, uint32_t);
                    radix = 8;
                }
                if (c == 'b')
                {
                    uval = (uint32_t)va_arg(ap, uint32_t);
                    radix = 2;
                    vstrp = &vstr[vlen];
                }
                if (c == 'p')
                {
                    uval = (uint32_t)va_arg(ap, void *);
                    radix = 16;
                    vstrp = &vstr[vlen];
                }
                if (c == 'u')
                {
                    uval = (uint32_t)va_arg(ap, uint32_t);
                    radix = 10;
                    vstrp = &vstr[vlen];
                }
                if ((c == 'o') || (c == 'b') || (c == 'p') || (c == 'u'))
                {
                    vlen = DbgConsole_ConvertRadixNumToString(vstr, &uval, false, radix, use_caps);
                    vstrp = &vstr[vlen];
#if PRINTF_ADVANCED_ENABLE
                    if (flags_used & kPRINTF_Zero)
                    {
                        DbgConsole_PrintfPaddingCharacter('0', vlen, field_width, &count, func_ptr);
                        vlen = field_width;
                    }
                    else
                    {
                        if (!(flags_used & kPRINTF_Minus))
                        {
                            DbgConsole_PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr);
                        }
                    }
#endif /* PRINTF_ADVANCED_ENABLE */
                }
#if !PRINTF_ADVANCED_ENABLE
                DbgConsole_PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr);
#endif /* !PRINTF_ADVANCED_ENABLE */
                while (*vstrp)
                {
                    func_ptr(*vstrp--);
                    count++;
                }
#if PRINTF_ADVANCED_ENABLE
                if (flags_used & kPRINTF_Minus)
                {
                    DbgConsole_PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr);
                }
#endif /* PRINTF_ADVANCED_ENABLE */
            }
            else if (c == 'c')
            {
                cval = (char)va_arg(ap, uint32_t);
                func_ptr(cval);
                count++;
            }
            else if (c == 's')
            {
                sval = (char *)va_arg(ap, char *);
                if (sval)
                {
                    vlen = strlen(sval);
#if PRINTF_ADVANCED_ENABLE
                    if (!(flags_used & kPRINTF_Minus))
#endif /* PRINTF_ADVANCED_ENABLE */
                    {
                        DbgConsole_PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr);
                    }
                    while (*sval)
                    {
                        func_ptr(*sval++);
                        count++;
                    }
#if PRINTF_ADVANCED_ENABLE
                    if (flags_used & kPRINTF_Minus)
                    {
                        DbgConsole_PrintfPaddingCharacter(' ', vlen, field_width, &count, func_ptr);
                    }
#endif /* PRINTF_ADVANCED_ENABLE */
                }
            }
            else
            {
                func_ptr(c);
                count++;
            }
        }
    }
    return count;
}

static int32_t DbgConsole_Putchar(int32_t ch)
{
    /* Do nothing if the debug UART is not initialized. */
    if (s_debugConsole.type == DEBUG_CONSOLE_DEVICE_TYPE_NONE)
    {
        return -1;
    }
    
#ifdef USE_BLOCKING_TRANSFER
    s_debugConsole.ops.tx_union.PutChar(s_debugConsole.instance, (uint8_t *)(&ch), 1, 500U);
#else
    s_debugConsole.ops.tx_union.PutChar(s_debugConsole.instance, (uint8_t *)(&ch), 1);
#endif
    
    return 1;
}

#ifdef USE_FEATURE_CORE_DUMP
void exception_printf_override(status_t (*LPUART_CB)(uint32_t instance, const uint8_t *buffer, uint32_t length, uint32_t timeout))
{
    if(LPUART_CB != NULL)
    {
        INT_SYS_DisableIRQ(LPUART1_RxTx_IRQn);
        s_debugConsole.ops.tx_union.LPUART_PutChar = LPUART_CB;
    }
}
#endif 

int32_t DbgConsole_Printf(const char *fmt_s, ...)
{
    va_list ap;
    int result;

    int32_t lc = 0;
    lc = osif_enter_critical();  
    
    /* Do nothing if the debug UART is not initialized. */
    if (s_debugConsole.type == DEBUG_CONSOLE_DEVICE_TYPE_NONE)
    {
        return -1;
    }
    
    va_start(ap, fmt_s);
    
    result = DbgConsole_PrintfFormattedData(DbgConsole_Putchar, fmt_s, ap);
    
    va_end(ap);
    
    (void)osif_exit_critical(lc);

    return result;
}




