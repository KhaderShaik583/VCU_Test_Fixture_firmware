#include "fw_common.h"
#include "rm_governor.h"

#define MAX_THERMAL_LUTS    (6U)
#define MAX_SOC_POINTS      (4U)

typedef struct 
{
    float_t low;
    float_t high;
    uint8_t allowed_mode;
}governer_lut_t;

typedef struct 
{
    uint32_t low;
    uint32_t high;
    uint8_t allowed_mode;
}soc_governer_lut_t;

typedef struct
{
    governer_lut_t *g;
    uint32_t sz;
}thermal_governer_t;

/*
 * The allowed_mode field is encoded as shown below
 * G - Glide
 * C - Combat
 * B - Ballistic
 * x - Mode not allowed
 
 * Decimal  Binary  Allowed Modes
 *       7     111            GCB
 *       6     110            GCx
 *       4     100            Gxx
 *       0     000            xxx

*/

static const governer_lut_t lg_thermal_lut[3] = {
    {10.0f,  70.0f,  7U},
    {70.0f,  80.0f,  0U},
    {-20.0f, 10.0f,  6U}
};

static const governer_lut_t mlc_thermal_lut[3] = {
    {0.0f,   60.0f,   7U},
    {60.0f,  70.0f,   0U},
    {-20.0f, -1.0f,   7U}
};

static const governer_lut_t bak_thermal_lut[6] = {
    {0.0f,   40.0f,   7U},
    {40.0f,  45.0f,   6U},
    {45.0f,  60.0f,   4U},
    {60.0f,  70.0f,   0U},
    {-20.0f, -10.0f,  4U},
    {-10.0f, -1.0f,   6U},
};

static const governer_lut_t eve_thermal_lut[6] = {
    {0.0f,   40.0f,   7U},
    {40.0f,  45.0f,   6U},
    {45.0f,  60.0f,   4U},
    {60.0f,  70.0f,   0U},
    {-20.0f, -10.0f,  4U},
    {-10.0f, -1.0f,   6U},
};

static const governer_lut_t sam_thermal_lut[6] = {
    {0.0f,   40.0f,   7U},
    {40.0f,  45.0f,   6U},
    {45.0f,  60.0f,   4U},
    {60.0f,  70.0f,   0U},
    {-20.0f, -10.0f,  4U},
    {-10.0f, -1.0f,   6U},
};

static const thermal_governer_t thermal_luts[MAX_THERMAL_LUTS] = {
    {NULL,                                0U},
    {(governer_lut_t *)lg_thermal_lut,    3U},
    {(governer_lut_t *)mlc_thermal_lut,   3U},
    {(governer_lut_t *)bak_thermal_lut,   6U},
    {(governer_lut_t *)eve_thermal_lut,   6U},
    {(governer_lut_t *)sam_thermal_lut,   6U}
};

static const soc_governer_lut_t soc_lut[MAX_SOC_POINTS] = {
    {0U,   4U,   4U},
    {5U,   19U,  4U},
    {20U,  69U,  6U},
    {70U,  100U, 7U}
};

static uint32_t rm_get_ride_mode_soc(uint32_t soc)
{
    uint32_t mode = 0U;
   
    for(uint32_t i = 0U; i < MAX_SOC_POINTS; i++)
    {
        if((soc >= soc_lut[i].low) &&
           (soc <= soc_lut[i].high))
        {
            mode = soc_lut[i].allowed_mode;
            break;
        }
    }
   
    return mode;
}

static uint32_t rm_get_ride_mode_thermal(uint32_t variant, float_t temperature)
{
    uint32_t mode = 0U;
    governer_lut_t *govn = NULL;
    
    DEV_ASSERT(variant < MAX_THERMAL_LUTS);
   
    govn = thermal_luts[variant].g;
    DEV_ASSERT(govn != NULL);
    
    for(uint32_t i = 0U; i < thermal_luts[variant].sz; i++)
    {
        if((temperature >= govn[i].low) &&
           (temperature < govn[i].high))
        {
            mode = govn[i].allowed_mode;
            break;
        }
    }
   
    return mode;
}

void rm_gov_eval_ride_mode(uint32_t soc, uint32_t variant, float_t temperature, uint32_t *mode)
{
    uint32_t m = 0U;
    volatile uint32_t mt = 0U;
    volatile uint32_t ms = 0U;
    
    DEV_ASSERT(mode != NULL);
    
    mt = rm_get_ride_mode_thermal(variant, temperature);
    ms = rm_get_ride_mode_soc(soc);
    
    m = mt & ms;
    
    *mode = m;
}
