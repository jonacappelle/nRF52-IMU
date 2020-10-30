
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* CHANGES */
#include "imu.h"


/* END CHANGES */

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define LM75B_ADDR          (0x90U >> 1)

#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U

/* Mode for LM75B. */
#define NORMAL_MODE 0U


/* CHANGES */
#define ICM_20948_I2C_ADDRESS		0x69U


/* END CHANGES */

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

static uint8_t value[1];

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
void LM75B_set_mode(void)
{
    ret_code_t err_code;

    /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
    uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /* Writing to pointer byte. */
    reg[0] = LM75B_REG_TEMP;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t* data)
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
                data_handler(&value[0]);
            }
            m_xfer_done = true;
            break;
				case NRF_DRV_TWI_EVT_ADDRESS_NACK:
						NRF_LOG_INFO("Address NACK");
				case NRF_DRV_TWI_EVT_DATA_NACK:
						NRF_LOG_INFO("Data NACK");
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

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data()
{
    m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}


//write byte to register
void write_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t *data, bool stop)
{   
		ret_code_t err_code;
		uint8_t data_write[2];
		
		data_write[0] = sub_address;
		data_write[1] = *data;
				
		err_code = nrf_drv_twi_tx(twi_handle, address, data_write, sizeof(data_write), stop);  
		APP_ERROR_CHECK(err_code);
}

//reading byte or bytes from register            
void write_forread_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address)
{   
		ret_code_t err_code;
		
		err_code = nrf_drv_twi_tx(twi_handle, address, &sub_address, sizeof(sub_address), true);  
		APP_ERROR_CHECK(err_code);
}
    
char read_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address)
{   
		ret_code_t err_code;
		uint8_t data; // `data` will store the register data     
		
		write_forread_byte(twi_handle, address, sub_address);
		
		err_code = nrf_drv_twi_rx(twi_handle, address, &data, sizeof(data));
		APP_ERROR_CHECK(err_code);
 
		return data; 
}
        
void read_bytes(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t * dest, uint8_t dest_count)
{   
		ret_code_t err_code;  
		
		write_forread_byte(twi_handle, address, sub_address);
		
		err_code = nrf_drv_twi_rx(twi_handle, address, dest, dest_count);
		APP_ERROR_CHECK(err_code);
}



/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
		NRF_LOG_FLUSH();
    twi_init();
		NRF_LOG_INFO("i2c init");
		NRF_LOG_FLUSH();
    //LM75B_set_mode();

	////////////////////////////////////////////
	
ret_code_t err_code;

uint8_t addr8 = 0x00U;
nrf_delay_ms(500);

	
    while (true)
    {
        nrf_delay_ms(100);

				if (err_code == NRF_SUCCESS)
				{
//					NRF_LOG_INFO("The Register read = 0x%x", value);
					NRF_LOG_FLUSH();
				}
			
//        do
//        {
//            __WFE();
//        }while (m_xfer_done == false);
				
        //read_sensor_data();
				m_xfer_done = false;
				err_code = nrf_drv_twi_tx(&m_twi, ICM_20948_I2C_ADDRESS, &addr8, sizeof(addr8), true);
				while (m_xfer_done == false);
				if (NRF_SUCCESS == err_code)
				{
					err_code = nrf_drv_twi_rx(&m_twi, ICM_20948_I2C_ADDRESS, value, sizeof(value));
				}
				APP_ERROR_CHECK(err_code);				
		
    }
}

/** @} */
