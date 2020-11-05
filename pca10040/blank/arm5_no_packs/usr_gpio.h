#ifndef _USR_GPIO_H_
#define _USR_GPIO_H_

// GPIO Interrupt
#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"



/* Interrupt pin number */
#define INT_PIN	2

void gpio_init(void);
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

#endif
