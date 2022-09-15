#ifndef DEVASSERT_H
#define DEVASSERT_H

#include <stdbool.h>
#include "fw_features.h"

#if defined (USE_FEATURE_CUSTOM_DEVASSERT)
    /* If the CUSTOM_DEVASSERT symbol is defined, then add the custom implementation */
    #include CUSTOM_DEVASSERT
#elif defined (USE_FEATURE_DEV_ERROR_DETECT)
    /* Implement default assert macro */
static inline void dev_assert(volatile bool x)
{
	if(x) { } else { __asm("BKPT #0\n\t"); for(;;) {} }
}
    #define DEV_ASSERT(x) dev_assert(x)
    #define DEV_ERROR_DETECT
#else
    /* Assert macro does nothing */
    #define DEV_ASSERT(x) ((void)0)
#endif

#endif /* DEVASSERT_H */

