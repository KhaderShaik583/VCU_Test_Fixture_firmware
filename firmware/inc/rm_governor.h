#ifndef RM_GOVERNOR_H
#define RM_GOVERNOR_H

#include <stdint.h>

void rm_gov_eval_ride_mode(uint32_t soc, uint32_t variant, float_t temperature, uint32_t *mode);

#endif /* RM_GOVERNOR_H */
