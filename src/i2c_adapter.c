/******************************************************************************
        Teenage Engineering oplab_baremetal_usb_lite
        File: i2c.c
        Author: Ben Olayinka
        Date: 11 Jun 2019
        Brief:
******************************************************************************/

/******************************************************************************
//std
******************************************************************************/
#include <stdbool.h>
#include <inttypes.h>
#include <sys/printk.h>

/******************************************************************************
//zephyr
******************************************************************************/
#include "i2c.h"

/******************************************************************************
//variables
******************************************************************************/
struct device *i2c_dev;
static bool is_initialized = false;

/******************************************************************************
//functions
******************************************************************************/
void I2CInit()
{
	i2c_dev = device_get_binding("I2C_0");
	i2c_configure(i2c_dev, I2C_MODE_MASTER | I2C_SPEED_SET(I2C_SPEED_FAST));
}

void I2CDeinit()
{
}

void I2CReadRegister(const uint8_t slave_addr, const uint8_t reg, uint8_t *data)
{
	i2c_reg_read_byte(i2c_dev, slave_addr,
				    reg, data);
}

void I2CWriteRegister(const uint8_t slave_addr, const uint8_t reg, const uint8_t data)
{
	i2c_reg_write_byte(i2c_dev, slave_addr,
				    reg, data);
}

void I2CWriteBytes(const uint8_t slave_addr, const uint8_t *data, uint32_t length)
{
	i2c_write(i2c_dev, data, 1, slave_addr);
}

/* end of file */