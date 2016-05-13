#ifndef MXC6225_H
#define MXC6225_H

#include <linux/ioctl.h>

#define MXC6225_I2C_SLAVE_WRITE_ADDR		0x2A
#define MXC6225_FIXED_DEVID			0x05
	
/* MXC6225 Register Map  (Please refer to MXC6225 Specifications) */
#define MXC6225_REG_DATAX0        0x00
#define MXC6225_REG_DEVID				0x08

#define MXC6225_BW_200HZ				0x0d
#define MXC6225_BW_100HZ				0x0c
#define MXC6225_BW_50HZ				0x0b
#define MXC6225_BW_25HZ				0x0a
	
#define MXC6225_REG_DETECTION		0x04	

#define MXC6225_RANGE_2G				0x03
#define MXC6225_RANGE_4G				0x05
#define MXC6225_RANGE_8G				0x08

#define MXC6225_MEASURE_MODE			0x80	
	
#define MXC6225_SUCCESS						0
#define MXC6225_ERR_I2C						-1
#define MXC6225_ERR_STATUS					-3
#define MXC6225_ERR_SETUP_FAILURE			-4
#define MXC6225_ERR_GETGSENSORDATA			-5
#define MXC6225_ERR_IDENTIFICATION			-6
	 
	 

#define MXC6225_BUFSIZE				256

#endif

