#include "imu.h"

/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#include "Invn/Devices/SerifHal.h"
// include to low level system driver
// #include "MyTarget/SPI.h"
#include "nrf_drv_twi.h"

extern const nrf_drv_twi_t m_twi;

extern ret_code_t i2c_read_bytes(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t * dest, uint8_t dest_count);
extern ret_code_t i2c_write_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, const uint8_t* data, uint32_t len, bool stop);
extern void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
#define ICM_20948_I2C_ADDRESS		0x69U


// forward declarations
int my_serif_open_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
int my_serif_open_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
// Exported instance of SerifHal object:
// A pointer to this structure shall be passed to the Device API,
// for the device driver to access to the SPI/I2C bus.
// The device will not modify the object, so it can be declared const
// The underlying HW serial interface must be up and running before calling any
// device methods
const inv_serif_hal_t my_serif_instance = {
        my_serif_open_read_reg,  /* callback to read_reg low level method */
        my_serif_open_write_reg, /* callback to read_reg low level method */
        256,                     /* maximum number of bytes allowed per read transaction,
                                    (limitation can come from internal buffer in the system driver) */
        256,                     /* maximum number of bytes allowed per write transaction,
                                    (limitation can come from internal buffer in the system driver) */
        INV_SERIF_HAL_TYPE_I2C,  /* type of the serial interface (between SPI or I2C) */
        (void *)0xDEAD           /* some context pointer passed to read_reg/write_reg callbacks */
};
int my_serif_open_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
        (void)context, (void)reg, (void)rbuffer, (void)rlen;
        // MyTarget_SPI_do_read_reg(&reg, 1, rbuffer, rlen);
	
				ret_code_t error = i2c_read_bytes( &m_twi, ICM_20948_I2C_ADDRESS, reg, rbuffer, rlen);
				if(error == NRF_SUCCESS)
				{
					return 0;
				}else{
					return -1;	// shall return a negative value on error
				}
}

int my_serif_open_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
        (void)context, (void)reg, (void)wbuffer, (void)wlen;
        // MyTarget_SPI_do_write_reg(&reg, 1, wbuffer, wlen);
	
//				for( int i=0; i<wlen; i++)
//				{
					ret_code_t error = i2c_write_byte( &m_twi, ICM_20948_I2C_ADDRESS, reg, wbuffer, wlen, false);
//					i++;
//				}
				// TODO: return value is now always 0
				if(error == NRF_SUCCESS)
				{
					return 0;
				}else{
					return -1;	// shall return a negative value on error
				}
//        return 0; // shall return a negative value on error
}
