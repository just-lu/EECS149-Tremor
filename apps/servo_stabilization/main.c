#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


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

#include "app_error.h"
#include "app_pwm.h"

#include "mpu9250.h"

APP_PWM_INSTANCE(PWM2,2);                   // Create the instance "PWM1" using TIMER1.

static volatile bool ready_flag;            // A flag indicating PWM status.

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

// LED array
static uint8_t LEDS[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

int main(void) {
  // servo stuff
  ret_code_t err_code;

  // initializing servo pin number
  uint8_t SERVO_PIN = 4;

  /* 1-channel PWM, 50Hz, output on DK LED pins, 20ms period */
  app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_1CH(20000L, SERVO_PIN);

  //Switch the polarity of the first channel. 
  pwm2_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

  /* Initialize and enable PWM. */
  err_code = app_pwm_init(&PWM2,&pwm2_cfg,NULL);
  APP_ERROR_CHECK(err_code);
  app_pwm_enable(&PWM2);

  uint8_t servo_pos_max = 25;
  uint8_t servo_pos_min = 6.75;
  uint8_t servo_pos_angle_45 = 6;
  uint8_t servo_pos_angle_90 = 8;
  uint8_t servo_pos_angle_135 = 11;


  ret_code_t error_code = NRF_SUCCESS;

  // initialize GPIO driver
  // if (!nrfx_gpiote_is_init()) {
  //   error_code = nrfx_gpiote_init();
  // }
  // APP_ERROR_CHECK(error_code);

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


  /* Set the duty cycle - keep trying until PWM is ready... */
  // while (app_pwm_channel_duty_set(&PWM2, 0, servo_pos_min) == NRF_ERROR_BUSY);
  // nrf_delay_ms(100);

  // while (app_pwm_channel_duty_set(&PWM2, 0, servo_pos_angle_45) == NRF_ERROR_BUSY);
  // nrf_delay_ms(100);

  // while (app_pwm_channel_duty_set(&PWM2, 0, servo_pos_angle_90) == NRF_ERROR_BUSY);
  // nrf_delay_ms(100);

  // while (app_pwm_channel_duty_set(&PWM2, 0, servo_pos_angle_135) == NRF_ERROR_BUSY);
  // nrf_delay_ms(100);

  // while (app_pwm_channel_duty_set(&PWM2, 0, servo_pos_max) == NRF_ERROR_BUSY);
  // nrf_delay_ms(100);


  // loop forever
  float x_rot = 0;
  float y_rot = 0;
  float z_rot = 0;

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

    // mapping
    int input_start = 60;
    int input_end = -60;

    int output_start = 2;
    int output_end = 20;


    float input = gyr_measurement.z_axis;

    if (input < -60) {
      input = 60;
    } else if (input > 60) {
      input = 60;
    }

    int output;

    double slope = 1.0 * (output_end - output_start)/(input_end - input_start);
    output = output_start + round(slope * (input - input_start));


    /* Set the duty cycle - keep trying until PWM is ready... */
    // while (app_pwm_channel_duty_set(&PWM2, 0, output) == NRF_ERROR_BUSY);
    // nrf_delay_ms(100);

    while (app_pwm_channel_duty_set(&PWM2, 0, 8) == NRF_ERROR_BUSY);
    nrf_delay_ms(100);

    loop_index++;

    // if (loop_index >= 30) {
    //   loop_index = 0;
    //   x_rot = 0;
    //   y_rot = 0;
    //   z_rot = 0;
    // }

    nrf_delay_ms(100);
  }
}