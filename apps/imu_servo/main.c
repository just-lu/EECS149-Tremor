// Analog accelerometer app
//
// Reads data from the ADXL327 analog accelerometer

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

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

#include "buckler.h"

#include "bsp.h"
#include "app_pwm.h"

// ADC channels
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.

static volatile bool ready_flag;            // A flag indicating PWM status.

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

// callback for SAADC events
void saadc_callback (nrfx_saadc_evt_t const * p_event) {
  // don't care about adc callbacks
}

// sample a particular analog channel in blocking mode
nrf_saadc_value_t sample_value (uint8_t channel) {
  nrf_saadc_value_t val;
  ret_code_t error_code = nrfx_saadc_sample_convert(channel, &val);
  APP_ERROR_CHECK(error_code);
  return val;
}

int main (void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  // initialize analog to digital converter
  nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
  error_code = nrfx_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(error_code);

  // initialize analog inputs
  // configure with 0 as input pin for now
  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(0);
  channel_config.gain = NRF_SAADC_GAIN1_6; // input gain of 1/6 Volts/Volt, multiply incoming signal by (1/6)
  channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // 0.6 Volt reference, input after gain can be 0 to 0.6 Volts

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_X;
  error_code = nrfx_saadc_channel_init(X_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_Y;
  error_code = nrfx_saadc_channel_init(Y_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_Z;
  error_code = nrfx_saadc_channel_init(Z_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // initialization complete
  printf("Buckler initialized!\n");

  ret_code_t err_code;

  // initializing servo pin number
  uint8_t SERVO_PIN = 4;

  /* 1-channel PWM, 50Hz, output on DK LED pins, 20ms period */
  app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(20000L, SERVO_PIN);

  /* Switch the polarity of the first channel. */
  pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

  /* Initialize and enable PWM. */
  err_code = app_pwm_init(&PWM1,&pwm1_cfg,NULL);
  APP_ERROR_CHECK(err_code);
  app_pwm_enable(&PWM1);

  uint8_t servo_pos_max = 8;
  uint8_t servo_pos_min = 6.5;

  // loop forever
  while (1) {
    // sample analog inputs
    float x_val = sample_value(X_CHANNEL) * (3.6 / (float) (1 << 12));
    float y_val = sample_value(Y_CHANNEL) * (3.6 / (float) (1 << 12));
    float z_val = sample_value(Z_CHANNEL) * (3.6 / (float) (1 << 12));

    float x_val_g = (x_val - (2.85 / 2)) / ((2.85 / 3) * .420);
    float y_val_g = (y_val - (2.85 / 2)) / ((2.85 / 3) * .420);
    float z_val_g = (z_val - (2.85 / 2)) / ((2.85 / 3) * .420);


    float theta = atan2 (x_val_g, sqrt ((y_val_g * y_val_g) * (z_val_g * z_val_g)));
    float psi   = atan2 (y_val_g, sqrt ((x_val_g * x_val_g) * (z_val_g * z_val_g)));
    float phi   = atan2 (sqrt ((x_val_g * x_val_g) * (y_val_g * y_val_g)), z_val_g);
    // display results
    // printf("Voltage x: %f\tVoltage y: %f\tVoltage z:%f\n", x_val, y_val, z_val);
    // nrf_delay_ms(100);
    // printf("g-force x: %f\tg-force y: %f\tg-force z:%f\n", x_val_g, y_val_g, z_val_g);
    // nrf_delay_ms(100);
    printf("tilt-theta: %f\ttilt-psi: %f\ttilt-phi:%f\n", theta, psi, phi);
    //nrf_delay_ms(100);

    /* Set the duty cycle - keep trying until PWM is ready... */
    while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_max) == NRF_ERROR_BUSY);
    nrf_delay_ms(500);
    while (app_pwm_channel_duty_set(&PWM1, 0, servo_pos_min) == NRF_ERROR_BUSY);
    nrf_delay_ms(500);
  }
}


