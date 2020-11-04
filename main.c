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
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"

#include "imu.h"

#include "string.h"

// Timer
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"

// GPIO Interrupt
#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"


#include "Invn/EmbUtils/Message.h"


/* Define msg level */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG


/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* I2C address of IMU */
#define ICM_20948_I2C_ADDRESS		0x69U
#define ICM_20948_WHOAMI				0x00U

const nrf_drv_timer_t TIMER_MICROS = NRF_DRV_TIMER_INSTANCE(0);

const char * activityName(int act);

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from IMU */
static uint8_t data[1];


static void msg_printer(int level, const char * str, va_list ap);

/*
 * Printer function for IDD message facility
 */
static void msg_printer(int level, const char * str, va_list ap)
{
	NRF_LOG_INFO(str);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t data)
{
	//NRF_LOG_INFO("Data: 0x%x", data);
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
ret_code_t i2c_write_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, const uint8_t* data, uint32_t len, bool stop)
{   
		ret_code_t err_code;
		
		const uint8_t buf_len = len+1; // Register address + number of bytes
    uint8_t tx_buf[buf_len];

    tx_buf[0] = sub_address;
    
    memcpy(tx_buf+1, data, len); // Shift the data to make place for subaddress
	
		m_xfer_done = false;
	
		err_code = nrf_drv_twi_tx(twi_handle, address, tx_buf, buf_len, stop);  
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
//				NRF_LOG_INFO("us value requested");
	
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
	
	
				uint32_t time_us = nrf_drv_timer_capture(&TIMER_MICROS, NRF_TIMER_CC_CHANNEL0);
//				NRF_LOG_INFO("Timer value requested: %d", time_us);
				return time_us;
	
//        return 0;
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
	
	
	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {

		switch(INV_SENSOR_ID_TO_TYPE(event->sensor)) {
		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			NRF_LOG_INFO("data event %s (lsb): %llu %d %d %d", inv_sensor_str(event->sensor),
					(int)event->timestamp,
					(int)event->data.raw3d.vect[0],
					(int)event->data.raw3d.vect[1],
					(int)event->data.raw3d.vect[2]);
//					NRF_LOG_INFO("%d	%d	%d", event->data.raw3d.vect[0], event->data.raw3d.vect[1], event->data.raw3d.vect[2]);
			break;
		case INV_SENSOR_TYPE_ACCELEROMETER:
		case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		case INV_SENSOR_TYPE_GRAVITY:
			NRF_LOG_INFO("data event %s (mg): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.acc.vect[0]*1000),
					(int)(event->data.acc.vect[1]*1000),
					(int)(event->data.acc.vect[2]*1000),
					(int)(event->data.acc.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_GYROSCOPE:
			NRF_LOG_INFO("data event %s (mdps): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_MAGNETOMETER:
			NRF_LOG_INFO("data event %s (nT): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mdps): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.bias[0]*1000),
					(int)(event->data.gyr.bias[1]*1000),
					(int)(event->data.gyr.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (nT): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.bias[0]*1000),
					(int)(event->data.mag.bias[1]*1000),
					(int)(event->data.mag.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
			NRF_LOG_INFO("data event %s (e-3): %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.quaternion.quat[0]*1000),
					(int)(event->data.quaternion.quat[1]*1000),
					(int)(event->data.quaternion.quat[2]*1000),
					(int)(event->data.quaternion.quat[3]*1000),
					(int)(event->data.quaternion.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_ORIENTATION:
//			NRF_LOG_INFO("data event %s (e-3):, %d, %d, %d, Accuracy: %d ", inv_sensor_str(event->sensor),
//					(int)(event->data.orientation.x*1000),
//					(int)(event->data.orientation.y*1000),
//					(int)(event->data.orientation.z*1000),
//					(int)(event->data.orientation.accuracy_flag*1000)); // 0 - 3: not calibrated - fully calibrated
		NRF_LOG_INFO("%d, %d, %d, %d, %d, %d", // rewritten write funtion to allow easier plotting
					(int)(event->data.orientation.x),
					(int)(event->data.orientation.y),
					(int)(event->data.orientation.z),
//					(int)(event->data.orientation.accuracy_flag),
					(int)(event->data.gyr.accuracy_flag),
					(int)(event->data.acc.accuracy_flag),	
					(int)(event->data.mag.accuracy_flag)); // 0 - 3: not calibrated - fully calibrated
			break;
		case INV_SENSOR_TYPE_BAC:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : %d %s", inv_sensor_str(event->sensor),
					event->data.bac.event, activityName(event->data.bac.event));
			break;
		case INV_SENSOR_TYPE_STEP_COUNTER:
			NRF_LOG_INFO("data event %s : %lu", inv_sensor_str(event->sensor),
					(unsigned long)event->data.step.count);
			break;
		case INV_SENSOR_TYPE_PICK_UP_GESTURE:
		case INV_SENSOR_TYPE_STEP_DETECTOR:
		case INV_SENSOR_TYPE_SMD:
		case INV_SENSOR_TYPE_B2S:
		case INV_SENSOR_TYPE_TILT_DETECTOR:
		default:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : ...", inv_sensor_str(event->sensor));
			break;
	}
}
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
    256,                    /* maximum number of bytes allowed per read transaction */
    256,                    /* maximum number of bytes allowed per write transaction */
    INV_SERIF_HAL_TYPE_I2C, /* type of the serial interface used between SPI or I2C */
    (void *)0xDEADBEEF      /* some context pointer passed to read/write callbacks */
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////


static void check_rc(int rc)
{
	if(rc == -1) {
		NRF_LOG_INFO("BAD RC=%d", rc);
		NRF_LOG_FLUSH();
		//while(1);
	}
}



/*
 * Function to return activity name in printable char
 */
const char * activityName(int act)
{
	switch(act) {
	case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN:          return "BEGIN IN_VEHICLE";
	case INV_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN:             return "BEGIN WALKING";
	case INV_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN:             return "BEGIN RUNNING";
	case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN:          return "BEGIN ON_BICYCLE";
	case INV_SENSOR_BAC_EVENT_ACT_TILT_BEGIN:                return "BEGIN TILT";
	case INV_SENSOR_BAC_EVENT_ACT_STILL_BEGIN:               return "BEGIN STILL";
	case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_END:            return "END IN_VEHICLE";
	case INV_SENSOR_BAC_EVENT_ACT_WALKING_END:               return "END WALKING";
	case INV_SENSOR_BAC_EVENT_ACT_RUNNING_END:               return "END RUNNING";
	case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_END:            return "END ON_BICYCLE";
	case INV_SENSOR_BAC_EVENT_ACT_TILT_END:                  return "END TILT";
	case INV_SENSOR_BAC_EVENT_ACT_STILL_END:                 return "END STILL";
	default:                                                 return "unknown activity!";
	}
}

#define INT_PIN	2

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

//    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

//    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
//    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(INT_PIN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(INT_PIN, true);
}



/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("Timer finished");
            break;

        default:
            //Do nothing.
						NRF_LOG_INFO("Timer callback, timer cleared");
						nrf_drv_timer_clear(&TIMER_MICROS); // clear timer when it overflows
            break;
    }
}

void timer_init (void)
{
	// TODO: Timer will overflow in 1.19 hours
    ret_code_t err_code;

    const nrfx_timer_config_t timer_config = {
				.frequency = (nrf_timer_frequency_t)NRF_TIMER_FREQ_1MHz,      ///< Frequency 1 MHz
				.mode = (nrf_timer_mode_t)NRF_TIMER_MODE_TIMER,     ///< Mode of operation.
				.bit_width = (nrf_timer_bit_width_t)NRF_TIMER_BIT_WIDTH_32,   ///< Bit width 32 bits
				.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY, ///< Interrupt priority.
				.p_context = NULL          																	///< Context passed to interrupt handler.
    };

		err_code = nrf_drv_timer_init(&TIMER_MICROS, &timer_config, timer_led_event_handler);
    APP_ERROR_CHECK(err_code);

//		uint32_t time_ticks;
//		time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_MICROS, time_ms);
		
//		nrf_drv_timer_extended_compare(&TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
		
		nrf_drv_timer_enable(&TIMER_MICROS);
}

		bool interrupt = false;

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nSTART");
		NRF_LOG_FLUSH();
	
    	/*
	 * Setup message facility to see internal traces from IDD
	 */
	INV_MSG_SETUP(MSG_LEVEL, msg_printer);

	
		INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#          20948 example          #");
	INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
	NRF_LOG_FLUSH();
	
		uint8_t data1[4] = { 0x01, 0x02, 0x03, 0x04 };
		
		gpio_init();
		
		timer_init();
		
		uint32_t time_us;
		uint32_t time_ticks;
		
//		while(1)
//		{
//			time_ticks = nrf_drv_timer_capture(&TIMER_MICROS, NRF_TIMER_CC_CHANNEL0);
////			nrf_drv_timer_us_to_ticks(&TIMER_MICROS, time_us);
//			nrf_delay_ms(1000);
//			NRF_LOG_INFO("%d", time_ticks);
//			NRF_LOG_FLUSH();
//		}
		
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
		
		
//		i2c_write_byte(&m_twi, ICM_20948_I2C_ADDRESS, ICM_20948_WHOAMI, data1, 4, false);
		
		
		/*
		 * Create ICM20948 Device 
		 * Pass to the driver:
		 * - reference to serial interface object,
		 * - reference to listener that will catch sensor events,
		 */
		inv_device_icm20948_init2(&device_icm20948, &serif_instance, &sensor_listener, dmp3_image, sizeof(dmp3_image));
		NRF_LOG_FLUSH();
		/*
		 * Simply get generic device handle from Icm20948 Device
		 */
		device = inv_device_icm20948_get_base(&device_icm20948);
		NRF_LOG_FLUSH();
		/*
		 * Just get the whoami
		 */
		rc += inv_device_whoami(device, &whoami);
		check_rc(rc);
		/* ... do something with whoami */ 
		NRF_LOG_INFO("Data: 0x%x", whoami);
		NRF_LOG_FLUSH();
		
		
		/*
		 * Configure and initialize the Icm20948 device
		 */
		rc += inv_device_setup(device);
		check_rc(rc);
		NRF_LOG_FLUSH();
		
		rc += inv_device_load(device, NULL, dmp3_image, sizeof(dmp3_image), true /* verify */, NULL);
		check_rc(rc);
		NRF_LOG_FLUSH();
		
		nrf_delay_ms(2000);
		
//		rc += inv_device_set_sensor_period_us(device, INV_SENSOR_TYPE_RAW_GYROSCOPE, 50000);
//		check_rc(rc);
//		NRF_LOG_FLUSH();
		

		rc += inv_device_set_sensor_period_us(device, INV_SENSOR_TYPE_ROTATION_VECTOR, 50000); // 20 Hz
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);

		
		
		while(1)
		{
			NRF_LOG_FLUSH();
			if(interrupt)
			{
				inv_device_poll(device);
				interrupt = false;
			}

		}
		////////////////////////////////////////////////////////////////
		
	
//		i2c_write_byte(&m_twi, ICM_20948_I2C_ADDRESS, 0x06, data1, true);
	
    while (true)
    {
        nrf_delay_ms(100);
				NRF_LOG_FLUSH();

				i2c_read_bytes(&m_twi, ICM_20948_I2C_ADDRESS, 0x33, data, 1); // GYRO X H register	
    }
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    //NRF_LOG_INFO("Interrupt Occured!");
	
		interrupt = true;
}

/** @} */
