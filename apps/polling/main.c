#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "app_pwm.h"
#include "mpu9250.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "buckler.h"
#include "virtual_timer.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// global variables
float x_rot = 0;
float y_rot = 0;
float z_rot = 0;

mpu9250_measurement_t acc_measurement;
mpu9250_measurement_t gyr_measurement;

void poll() {
	printf("Polling IMU data\n");
		// print results
		printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
		printf("                  ----------\t----------\t----------\n");
		printf("I2C IMU Acc  (g): %10.3f\t%10.3f\t%10.3f\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
		printf("I2C IMU Gyro (g): %10.3f\t%10.3f\t%10.3f\n", gyr_measurement.x_axis, gyr_measurement.y_axis, gyr_measurement.z_axis);
		printf("Angle  (degrees): %10.3f\t%10.3f\t%10.3f\n", x_rot, y_rot, z_rot);
		printf("\n");
}

int main(void) {
	ret_code_t error_code = NRF_SUCCESS;

	// initialize RTT library
	error_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(error_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	printf("Board initialized!\n");

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


	// start polling timer
	virtual_timer_start_repeated(500000, poll);

	// loop forever
	while(1) {
		// get measurements
		acc_measurement = mpu9250_read_accelerometer();
		gyr_measurement = mpu9250_read_gyro();

		// determine rotation from gyro
		// gyros are messy, so only add value if it is of significant magnitude
		// note that we are dividing by 10 since we are measuring over a tenth of a second
		float x_rot_amount = gyr_measurement.x_axis/10;
		if (abs(x_rot_amount) > 0.5) {
			x_rot += x_rot_amount;
		}
		float y_rot_amount = gyr_measurement.y_axis/10;
		if (abs(y_rot_amount) > 0.5) {
			y_rot += y_rot_amount;
		}
		float z_rot_amount = gyr_measurement.z_axis/10;
		if (abs(z_rot_amount) > 0.5) {
			z_rot += z_rot_amount;
		}

		nrf_delay_ms(10);
	}
}