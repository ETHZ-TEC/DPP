

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdlib.h>
#include <driverlib.h>
#include "../config.h"
#include "sensors/bme280.h"        /* Bosch BME280 sensor driver */
#include "sensors/sht30.h"         /* Sensirion SHT30 sensor driver */


#define SENSOR_I2C_INTERFACE    EUSCI_B3_BASE
#define SENSOR_I2C_TIMEOUT      ((uint32_t) 10000)

#define SENSOR_I2C_SCL_PORT     GPIO_PORT_P6
#define SENSOR_I2C_SCL_PIN      GPIO_PIN7
#define SENSOR_I2C_SDA_PORT     GPIO_PORT_P6
#define SENSOR_I2C_SDA_PIN      GPIO_PIN6
#define SENSOR_I2C_GPIO_AF      GPIO_SECONDARY_MODULE_FUNCTION


void sensor_init(void);

int8_t sensor_i2c_read(uint8_t, uint8_t, uint8_t*, uint16_t);
int8_t sensor_i2c_write(uint8_t, uint8_t, uint8_t*, uint16_t);
void sensor_delay_ms(uint32_t);


#endif /* __SENSOR_H__ */
