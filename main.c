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

#include "Invn/Devices/SerifHal.h"
#include "Invn/Devices/DeviceIcm20948.h"

#include "imu.h"

// Timer
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"



/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* I2C address of IMU */
#define ICM_20948_I2C_ADDRESS		0x69U
#define ICM_20948_WHOAMI				0x00U

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

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
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_imu_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

//write byte to register
ret_code_t i2c_write_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, const uint8_t* data, bool stop)
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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////////////////////////////////////////////////////////////



/* 
 * High resolution sleep implementation for Icm20948.
 * Used at initilization stage. ~100us is sufficient.
 */
void inv_icm20948_sleep_us(int us)
{
        /*
         * You may provide a sleep function that blocks the current programm
         * execution for the specified amount of us
         */
        (void)us;
	
				nrf_delay_us(us);
}
/*
 * Time implementation for Icm20948.
 */
uint64_t inv_icm20948_get_time_us(void)
{
        /*
         * You may provide a time function that return a monotonic timestamp in us
         */
				//return app_timer_cnt_get() / 32.768;
        return 0;
}
/*
 * Callback called upon sensor event reception
 * This function is called in the same function than inv_device_poll()
 */
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
        /* arg will contained the value provided at init time */
        (void)arg;
        
        (void)event;
        /* ... do something with event */
	
				//NRF_LOG_INFO("Sensor event!");
				//NRF_LOG_FLUSH();
				
}
/*
 * A listener onject will handle sensor events
 */
static inv_sensor_listener_t sensor_listener = {
        sensor_event_cb, /* callback that will receive sensor events */
        (void *)0xDEAD   /* some pointer passed to the callback */
};
/*
 * States for icm20948 device object
 */
static inv_device_icm20948_t device_icm20948; 
static uint8_t dmp3_image[] = {
	#include "Invn/Images/icm20948_img.dmp3a.h"
// #include "path/to/Icm30630Dmp3Image.h"
};
/*
 * serif_hal object that abstract low level serial interface between host and device
 */
extern int my_serif_open_read_reg(void * context, uint8_t reg, uint8_t * data, uint32_t len);
extern int my_serif_open_write_reg(void * context, uint8_t reg, const uint8_t * data, uint32_t len);
const inv_serif_hal_t serif_instance = {
    my_serif_open_read_reg,      /* user read_register() function that reads a register over the serial interface */
    my_serif_open_write_reg,     /* user write_register() function that writes a register over the serial interface */
    128,                    /* maximum number of bytes allowed per read transaction */
    1,                    /* maximum number of bytes allowed per write transaction */
    INV_SERIF_HAL_TYPE_I2C, /* type of the serial interface used between SPI or I2C */
    (void *)0xDEADBEEF      /* some context pointer passed to read/write callbacks */
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////



/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nSTART");
		NRF_LOG_FLUSH();
	
    

	
		uint8_t data1[1] = { 0x00 };
		
		
		
		
		
		
		/////////////////////////////////////////////////////////////////
				int rc = 0;
        
        inv_device_t * device; /* just a handy variable to keep the handle to device object */
        uint8_t whoami;
        
        /*
         * Open serial interface (SPI or I2C) before playing with the device
         */
        /* call low level drive initialization here... */
        twi_init();
				NRF_LOG_INFO("i2c init");
				NRF_LOG_FLUSH();
        /*
         * Create ICM20948 Device 
         * Pass to the driver:
         * - reference to serial interface object,
         * - reference to listener that will catch sensor events,
         */
        inv_device_icm20948_init2(&device_icm20948, &serif_instance, &sensor_listener, dmp3_image, sizeof(dmp3_image));
        
        /*
         * Simply get generic device handle from Icm20948 Device
         */
        device = inv_device_icm20948_get_base(&device_icm20948);
				
				
//				i2c_read_bytes(&m_twi, ICM_20948_I2C_ADDRESS, 0x00, data, 1); // whoami	
				
				
        /*
         * Just get the whoami
         */
        rc += inv_device_whoami(device, &whoami);
        /* ... do something with whoami */ 
				
				NRF_LOG_INFO("Data: 0x%x", whoami);
				NRF_LOG_FLUSH();
				
				
				while(1);
				////////////////////////////////////////////////////////////////
		
	
		i2c_write_byte(&m_twi, ICM_20948_I2C_ADDRESS, 0x06, data1, true);
	
    while (true)
    {
        nrf_delay_ms(100);
				NRF_LOG_FLUSH();

				i2c_read_bytes(&m_twi, ICM_20948_I2C_ADDRESS, 0x33, data, 1); // GYRO X H register	
    }
}

/** @} */
