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
#include "Invn/EmbUtils/Message.h"

// Own includes
#include "imu.h"
#include "string.h"
#include "usr_twi.h"
#include "usr_tmr.h"
#include "usr_gpio.h"

////////////////
//  DEFINES   //
////////////////

/* Define msg level */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG


/* Static variable to keep track if an interrupt has occurred in main loop */
static bool interrupt = false;

/* Activity classification */
const char * activityName(int act);

/* Declared in imu.c */
extern void msg_printer(int level, const char * str, va_list ap);

/*
 * States for icm20948 device object
 */
static inv_device_icm20948_t device_icm20948; 
static uint8_t dmp3_image[] = {
	#include "Invn/Images/icm20948_img.dmp3a.h"
};

/* Serif hal instances and listener */
extern const inv_serif_hal_t my_serif_instance;
extern inv_sensor_listener_t sensor_listener;


static void check_rc(int rc)
{
	if(rc == -1) {
		NRF_LOG_INFO("BAD RC=%d", rc);
		NRF_LOG_FLUSH();
		while(1);
	}
}

/* Interrupt pin handeler callback function */
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    //NRF_LOG_INFO("Interrupt Occured!");
		interrupt = true;
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

		/* Initialize GPIO pins */
		gpio_init();
		
		/* Initialize us timer */
		timer_init();

		/* To keep track of errors */
		int rc = 0;
		
		inv_device_t * device; /* just a handy variable to keep the handle to device object */
		uint8_t whoami;
		
		/* Open serial interface (SPI or I2C) before playing with the device */
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
		inv_device_icm20948_init2(&device_icm20948, &my_serif_instance, &sensor_listener, dmp3_image, sizeof(dmp3_image));
		NRF_LOG_FLUSH();
		
		/*
		 * Simply get generic device handle from Icm20948 Device
		 */
		device = inv_device_icm20948_get_base(&device_icm20948);
		NRF_LOG_FLUSH();
		
		/* Just get the whoami */
		rc += inv_device_whoami(device, &whoami);
		check_rc(rc);
		NRF_LOG_INFO("Data: 0x%x", whoami);
		NRF_LOG_FLUSH();
		
		nrf_delay_ms(500);
		
		/* Configure and initialize the Icm20948 device */
		NRF_LOG_INFO("Setting up ICM20948");
		rc += inv_device_setup(device);
		check_rc(rc);
		NRF_LOG_FLUSH();
		
		// Load DMP
		NRF_LOG_INFO("Load DMP Image");
		rc += inv_device_load(device, NULL, dmp3_image, sizeof(dmp3_image), true /* verify */, NULL);
		check_rc(rc);
		NRF_LOG_FLUSH();
		
		
		
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
//		NRF_LOG_INFO("Start sensors");
//		NRF_LOG_INFO("Ping sensor");
//		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
//		check_rc(rc);
//		rc += inv_device_set_sensor_period_us(device, INV_SENSOR_TYPE_ROTATION_VECTOR, 50000); // 200 Hz
//		check_rc(rc);
//		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
//		check_rc(rc);
		
		// Start 9DoF euler angles output
		NRF_LOG_INFO("Start sensors");
		NRF_LOG_INFO("Ping sensor");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR);
		check_rc(rc);
		rc += inv_device_set_sensor_period_us(device, INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR, 50000); // 200 Hz
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR);
		check_rc(rc);
		
		nrf_delay_ms(2000);
		
		/* Activity classification */
//		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_BAC);
//		check_rc(rc);
		
		/* Step Counter */
//		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_STEP_COUNTER);
//		check_rc(rc);
//		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_STEP_COUNTER);
//		check_rc(rc);
		
		
		nrf_delay_ms(1000);
		
		// Loop: IMU gives interrupt -> bool interrupt = true -> poll device for data
		////////////////////////////////////////////////////////////////		
		while(1)
		{
//			NRF_LOG_FLUSH();
			if(interrupt)
			{
				inv_device_poll(device);
				interrupt = false;
			}
			NRF_LOG_FLUSH();

		}
		////////////////////////////////////////////////////////////////
}

/** @} */
