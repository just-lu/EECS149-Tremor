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

#include "mpu9250.h"

// LED array
static uint8_t LEDS[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize GPIO driver
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

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

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


  // loop forever
  int loop_index = 0;
  while (1) {
    // blink two LEDs
    nrf_gpio_pin_toggle(LEDS[loop_index%2]);

    // get measurements
    mpu9250_measurement_t acc_measurement = mpu9250_read_accelerometer();
    mpu9250_measurement_t gyr_measurement = mpu9250_read_gyro();

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

    // print results
    printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
    printf("                  ----------\t----------\t----------\n");
    printf("I2C IMU Acc (g): %10.3f\t%10.3f\t%10.3f\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
    printf("I2C IMU Gyro (g):  %10.3f\t%10.3f\t%10.3f\n", gyr_measurement.x_axis, gyr_measurement.y_axis, gyr_measurement.z_axis);
    printf("Angle  (degrees): %10.3f\t%10.3f\t%10.3f\n", x_rot, y_rot, z_rot);
    printf("\n");

    loop_index++;
    nrf_delay_ms(100);
  }
}