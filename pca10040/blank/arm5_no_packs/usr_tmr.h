#ifndef _USR_TMR_H_
#define _USR_TMR_H_

// Timer
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


void timer_init (void);
void timer_event_handler(nrf_timer_event_t event_type, void* p_context);



#endif
