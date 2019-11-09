#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
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
#include "bno055.h"

// ADC channels
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

int main(void) {

  // initialize MPU-9250 driver
  mpu9250_init(&twi_mngr_instance);
  printf("BNO055 initialized\n");

  // loop forever
  int loop_index = 0;

  while (1) {

    // sample analog inputs
    nrf_saadc_value_t x_val = sample_value(X_CHANNEL);
    nrf_saadc_value_t y_val = sample_value(Y_CHANNEL);
    nrf_saadc_value_t z_val = sample_value(Z_CHANNEL);

    // get imu measurements
    bno055_measurement_t acc_measurement = bno055_read_accelerometer();
    bno055_measurement_t gyro_measurement = bno055_read_gyro();

    // print results
    printf("                        X-Axis\t    Y-Axis\t    Z-Axis\n");
    printf("                    ----------\t----------\t----------\n");
    printf("Analog Accel (raw): %10d\t%10d\t%10d\n", x_val, y_val, z_val);
    printf("I2C IMU Accel (g):  %10.3f\t%10.3f\t%10.3f\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
    printf("I2C IMU Gyro (g):  %10.3f\t%10.3f\t%10.3f\n", gyro_measurement.x_axis, gyro_measurement.y_axis, gyro_measurement.z_axis);
    printf("\n");

    // wait before continuing loop
    loop_index++;
    nrf_delay_ms(500);
  }
}