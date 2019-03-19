#include "oi_driver.h"

void oi_driver_set(GPIO_TypeDef * gpioBank, int pin, int driverBit, int driverState){
      if(driverState & (1<<driverBit)){
        HAL_GPIO_WritePin(gpioBank, pin, GPIO_PIN_SET);
      }else{
        HAL_GPIO_WritePin(gpioBank, pin, GPIO_PIN_RESET);
      }
}
