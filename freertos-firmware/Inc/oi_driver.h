#ifndef __OI_DRIVER_H
#define __OI_DRIVER_H

#include "stm32f3xx_hal.h"
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

void oi_driver_set(GPIO_TypeDef * gpioBank, int pin, int driverBit, int driverState);

#ifdef __cplusplus
}
#endif
#endif  // __OI_DRIVER_H
