/******************************************************************************
        Teenage Engineering oplab_baremetal_usb_lite
        File: i2c.h
        Author: Ben Olayinka
        Date: 12 Jun 2019
        Brief:
******************************************************************************/
#ifndef I2C_I2C_H_
#define I2C_I2C_H_

/******************************************************************************
//std
******************************************************************************/
#include <stdbool.h>
#include <inttypes.h>

void I2CInit();
void I2CDeinit();
void I2CReadRegister(const uint8_t slave_addr, const uint8_t reg, uint8_t *data);
void I2CWriteRegister(const uint8_t slave_addr, const uint8_t reg, uint8_t data);
void I2CWriteBytes(const uint8_t slave_addr, const uint8_t *data, uint32_t length);

#endif /* I2C_I2C_H_ */
/* end of file */

