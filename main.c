// nRF52 communication with ICM20948 IMU using DMP for signal processing

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "imu.h"


/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* I2C address of IMU */
#define ICM_20948_I2C_ADDRESS		0x69U
#define ICM_20948_WHOAMI				0x00U

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from IMU */
static uint8_t data[1];

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t data)
{
	NRF_LOG_INFO("Data: 0x%x", data);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(data[0]);
            }
            m_xfer_done = true;
            break;
				case NRF_DRV_TWI_EVT_ADDRESS_NACK:
						NRF_LOG_INFO("Address NACK");
						break;
				case NRF_DRV_TWI_EVT_DATA_NACK:
						NRF_LOG_INFO("Data NACK");
						break;
        default:
            break;
    }
}

/* I2C Initialisation */
/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_imu_config = {
       .scl                = ARDUINO_SCL_PIN,		// PIN 27
       .sda                = ARDUINO_SDA_PIN,		// PIN 26
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_imu_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


//write byte to register
ret_code_t i2c_write_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t* data, bool stop)
{   
		ret_code_t err_code;
		uint8_t data_write[2];
		
		data_write[0] = sub_address;
		data_write[1] = *data;
	
		m_xfer_done = false;
	
		err_code = nrf_drv_twi_tx(twi_handle, address, data_write, sizeof(data_write), stop);  
		APP_ERROR_CHECK(err_code);
	
		while (m_xfer_done == false);
	
		return err_code;
}

//reading byte or bytes from register            
ret_code_t i2c_write_forread_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address)
{   
		ret_code_t err_code;
		
		m_xfer_done = false;
	
		err_code = nrf_drv_twi_tx(twi_handle, address, &sub_address, sizeof(sub_address), true);  
		APP_ERROR_CHECK(err_code);
	
		while (m_xfer_done == false);
	
		return err_code;
}

        
ret_code_t i2c_read_bytes(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t * dest, uint8_t dest_count)
{   
		ret_code_t err_code;  
		
		// Write address to read from first
		err_code = i2c_write_forread_byte(twi_handle, address, sub_address);
		
	
		if (NRF_SUCCESS == err_code)
		{			
			m_xfer_done = false;
		
			// Now we can actually read the data
			err_code = nrf_drv_twi_rx(twi_handle, address, dest, dest_count);
			APP_ERROR_CHECK(err_code);
	
			while (m_xfer_done == false);
		}
	
		return err_code;
}



/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nSTART");
		NRF_LOG_FLUSH();
	
    twi_init();
		NRF_LOG_INFO("i2c init");
		NRF_LOG_FLUSH();
	
		uint8_t data1[1] = 0x00;
	
		i2c_write_byte(&m_twi, ICM_20948_I2C_ADDRESS, 0x06, data1, true);
	
    while (true)
    {
        nrf_delay_ms(100);
				NRF_LOG_FLUSH();

				i2c_read_bytes(&m_twi, ICM_20948_I2C_ADDRESS, 0x33, data, 1); // GYRO X H register	
    }
}

/** @} */
