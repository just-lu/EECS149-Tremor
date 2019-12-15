#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "app_pwm.h"

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"
#include "nrfx_twim.h"

#include "buckler.h"
#include "mpu9250.h"
#include "simple_logger.h"
#include "virtual_timer.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// global variables
static float x_rot = 0;
static float y_rot = 0;
static float z_rot = 0;

static volatile mpu9250_measurement_t acc_measurement;
static volatile mpu9250_measurement_t gyr_measurement;

static uint32_t poll_period = 500; // in ms
static volatile bool poll_flag;

// LED array
static uint8_t LEDS[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};

void poll() {
	// __disable_irq();
	printf("Timer fired\n");

	poll_flag = true;

	// get measurements
	// acc_measurement = mpu9250_read_accelerometer();
	// gyr_measurement = mpu9250_read_gyro();

	// // determine rotation from gyro
	// // gyros are messy, so only add value if it is of significant magnitude
	// // note that we are dividing by 10 since we are measuring over a tenth of a second
	// float x_rot_amount = gyr_measurement.x_axis * poll_period / 1000.00;
	// if (abs(x_rot_amount) > 0.005) {
	// 	x_rot += x_rot_amount;
	// }
	// float y_rot_amount = gyr_measurement.y_axis * poll_period / 1000.00;
	// if (abs(y_rot_amount) > 0.005) {
	// 	y_rot += y_rot_amount;
	// }
	// float z_rot_amount = gyr_measurement.z_axis * poll_period / 1000.00;
	// if (abs(z_rot_amount) > 0.005) {
	// 	z_rot += z_rot_amount;
	// }

	// printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
	// printf("Rot ammount  (degrees): %10.3f\t%10.3f\t%10.3f\n", x_rot, y_rot, z_rot);


	// print results
	// printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
	// printf("                  ----------\t----------\t----------\n");
	// printf("I2C IMU Acc  (g): %10.3f\t%10.3f\t%10.3f\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
	// printf("I2C IMU Gyro (g): %10.3f\t%10.3f\t%10.3f\n", gyr_measurement.x_axis, gyr_measurement.y_axis, gyr_measurement.z_axis);
	// printf("Angle  (degrees): %10.3f\t%10.3f\t%10.3f\n", x_rot, y_rot, z_rot);
	// printf("\n");
	// __enable_irq();
}

int main(void) {
	ret_code_t error_code = NRF_SUCCESS;

	// initialize RTT library
	error_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(error_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	printf("Board initialized!\n");

	// initialize GPIO driver, need to uncomment when app_pwm_init is not there
	if (!nrfx_gpiote_is_init()) {
		error_code = nrfx_gpiote_init();
	}
	APP_ERROR_CHECK(error_code);

	// configure leds
	// manually-controlled (simple) output, initially set
	nrfx_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
	for (int i=0; i<3; i++) {
		error_code = nrfx_gpiote_out_init(LEDS[i], &out_config);
		APP_ERROR_CHECK(error_code);
	}

	// initialize i2c master (two wire interface)
	nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
	i2c_config.scl = BUCKLER_SENSORS_SCL;
	i2c_config.sda = BUCKLER_SENSORS_SDA;
	i2c_config.frequency = NRF_TWIM_FREQ_100K;
	error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
	APP_ERROR_CHECK(error_code);

	// initialize MPU-9250 driver
	mpu9250_init(&twi_mngr_instance);
	printf("MPU-9250 initialized\n");

	// initialize timer library
	virtual_timer_init();
	nrf_delay_ms(1000);

	float initial_z = 100.0;
	float prev_z = 100.0;

	float initial_x = 100.0;
	float prev_x = 100.0;

	float output = 0.0;
	float x_output = 0.0;
	float input, input_start, input_end, output_start, output_end;

	int loop_index = 0;
	int recalibration_count = 0;
	int z_direction = 0;
	int x_direction = 0;
	int prev_z_direction = 100;
	int prev_x_direction = 100;

	int tremor_count = 0;
	int time_count = 0;

	int threshold = 4;
	int period_count = 50;
	bool flag = false;


	// start polling timer in microsec
	virtual_timer_start_repeated(poll_period * 1000, poll);

	// loop forever
	while(1) {
		if (poll_flag) {
			//get measurements
			printf("Time: %d\n\n", read_timer());
			acc_measurement = mpu9250_read_accelerometer();
			gyr_measurement = mpu9250_read_gyro();

			// determine rotation from gyro
			// gyros are messy, so only add value if it is of significant magnitude
			// note that we are dividing by 10 since we are measuring over a tenth of a second
			float x_rot_amount = gyr_measurement.x_axis * poll_period / 1000.00;
			if (x_rot_amount > 0.005 || x_rot_amount < 0.005) {
				x_rot += x_rot_amount;
			}
			float z_rot_amount = gyr_measurement.z_axis * poll_period / 1000.00;
			if (z_rot_amount > 0.005 || z_rot_amount < 0.005) {
				z_rot += z_rot_amount;
			}

			printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
			printf("Rot ammount  (degrees): %10.3f\t%10.3f\t%10.3f\n", x_rot, y_rot, z_rot);


			// print results
			printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
			printf("                  ----------\t----------\t----------\n");
			printf("I2C IMU Acc  (g): %10.3f\t%10.3f\t%10.3f\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
			printf("I2C IMU Gyro (g): %10.3f\t%10.3f\t%10.3f\n", gyr_measurement.x_axis, gyr_measurement.y_axis, gyr_measurement.z_axis);
			printf("Angle  (degrees): %10.3f\t%10.3f\t%10.3f\n", x_rot, y_rot, z_rot);
			printf("\n");

			poll_flag = false;
		}

		nrf_gpio_pin_toggle(LEDS[loop_index%3]);
		nrf_delay_ms(100);
		loop_index++;
	}
}