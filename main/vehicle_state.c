#include "vehicle_state.h"
#include <string.h>

static vehicle_state_t s_state;

void vehicle_state_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
}

vehicle_state_t *vehicle_state_get(void)
{
    return &s_state;
}
