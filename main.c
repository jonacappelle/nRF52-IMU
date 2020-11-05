// nRF52 communication with ICM20948 IMU using DMP for signal processing

////////////////
//  INCLUDES  //
////////////////
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

// INV_MSG functionality
#include "Invn/EmbUtils/Message.h"

// Own includes
#include "imu.h"
#include "string.h"
#include "usr_twi.h"


////////////////
//  DEFINES   //
////////////////

/* Define msg level */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG



/* I2C address of IMU */
#define ICM_20948_I2C_ADDRESS		0x69U
#define ICM_20948_WHOAMI				0x00U

/* Interrupt pin number */
#define INT_PIN	2



/* Static variable to keep track if an interrupt has occurred in main loop */
static bool interrupt = false;

/* Timer instance for us timer (TIMER 0)*/
const nrf_drv_timer_t TIMER_MICROS = NRF_DRV_TIMER_INSTANCE(0);

const char * activityName(int act);


/* Interrupt pin handeler callback function */
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);


static void msg_printer(int level, const char * str, va_list ap);

/*
 * Printer function for IDD message facility
 */
static void msg_printer(int level, const char * str, va_list ap)
{
	NRF_LOG_INFO(str);
}



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
			NRF_LOG_INFO("RV: %d %d %d %d Accuracy: %d %d", //inv_sensor_str(event->sensor),
					(int)(event->data.quaternion.quat[0]*1000),
					(int)(event->data.quaternion.quat[1]*1000),
					(int)(event->data.quaternion.quat[2]*1000),
					(int)(event->data.quaternion.quat[3]*1000),
					(int)(event->data.quaternion.accuracy*1000),
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
		while(1);
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



/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

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
	// TODO: make timer 64 bits and run forever or reset timer when device goes to sleep if measurements are not longer than 1.19 hours
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

		/*
		 * Initialize GPIO pins
		 */
		gpio_init();
		
		/*
		 * Initialize us timer
		 */
		timer_init();

		int rc = 0;
		
		inv_device_t * device; /* just a handy variable to keep the handle to device object */
		uint8_t whoami;
		
		/*
		 * Open serial interface (SPI or I2C) before playing with the device
		 */
		twi_init();
		NRF_LOG_INFO("i2c init");
		NRF_LOG_FLUSH();
		
		/*
		 * Create ICM20948 Device 
		 * Pass to the driver:
		 * - reference to serial interface object,
		 * - reference to listener that will catch sensor events,
		 */
//		inv_device_icm20948_init(&device_icm20948, idd_io_hal_get_serif_instance_i2c(),&sensor_listener, dmp3_image, sizeof(dmp3_image));
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
		NRF_LOG_INFO("Data: 0x%x", whoami);
		NRF_LOG_FLUSH();
		
		nrf_delay_ms(500);
		
		/*
		 * Configure and initialize the Icm20948 device
		 */
		NRF_LOG_INFO("Setting up ICM20948");
		rc += inv_device_setup(device);
		check_rc(rc);
		NRF_LOG_FLUSH();
		
		// Load DMP
		NRF_LOG_INFO("Load DMP Image");
		rc += inv_device_load(device, NULL, dmp3_image, sizeof(dmp3_image), true /* verify */, NULL);
		check_rc(rc);
		NRF_LOG_FLUSH();
		
		// Test if sensor is available
		NRF_LOG_INFO("Ping sensor");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
		check_rc(rc);
		
		nrf_delay_ms(2000);
		
//		rc += inv_device_set_sensor_period_us(device, INV_SENSOR_TYPE_RAW_GYROSCOPE, 50000);
//		check_rc(rc);
//		NRF_LOG_FLUSH();
		
//		rc += inv_device_set_sensor_period_us(device, INV_SENSOR_TYPE_GYROSCOPE, 50000); // 20 Hz
//		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
//		rc += inv_device_set_sensor_period_us(device, INV_SENSOR_TYPE_ACCELEROMETER, 50000); // 20 Hz
//		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
//		rc += inv_device_set_sensor_period_us(device, INV_SENSOR_TYPE_MAGNETOMETER, 50000); // 20 Hz
//		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);


		// Start 9DoF quaternion output
		NRF_LOG_INFO("Start sensors");
		rc += inv_device_set_sensor_period_us(device, INV_SENSOR_TYPE_ROTATION_VECTOR, 50000); // 20 Hz
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
		check_rc(rc);
		
		nrf_delay_ms(1000);
		
		// Loop: IMU gives interrupt -> bool interrupt = true -> poll device for data
		////////////////////////////////////////////////////////////////		
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
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    //NRF_LOG_INFO("Interrupt Occured!");
		interrupt = true;
}

/** @} */
