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
#include "simple_logger.h"

#include "buckler.h"

#include "bsp.h"
#include "app_pwm.h"

// ADC channels
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2


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
  // printf("Buckler initialized!\n");


  // ret_code_t error_code = NRF_SUCCESS;

  printf("Started SD card demo app...\n");

  // Enable SoftDevice (used to get RTC running)
  nrf_sdh_enable_request();

  // Initialize GPIO driver
  if (!nrfx_gpiote_is_init()) {
    error_code = nrfx_gpiote_init();
  }
  APP_ERROR_CHECK(error_code);

  // Configure GPIOs
  nrf_gpio_cfg_output(BUCKLER_SD_ENABLE);
  nrf_gpio_cfg_output(BUCKLER_SD_CS);
  nrf_gpio_cfg_output(BUCKLER_SD_MOSI);
  nrf_gpio_cfg_output(BUCKLER_SD_SCLK);
  nrf_gpio_cfg_input(BUCKLER_SD_MISO, NRF_GPIO_PIN_NOPULL);

  nrf_gpio_pin_set(BUCKLER_SD_ENABLE);
  nrf_gpio_pin_set(BUCKLER_SD_CS);

  // Initialize SD card
  const char filename[] = "1mintest.txt";
  const char permissions[] = "a"; // w = write, a = append

  // Start file
  simple_logger_init(filename, permissions);

  // If no header, add it
  // simple_logger_log_header("HEADER for file \'%s\', written on %s \n", filename, "DATE");
  printf("Wrote header to SD card\n");


  // initialize logger
  // printf("%d", simple_logger_init("filename.txt", "w"));
  // simple_logger_update();
  simple_logger_log_header("%s,%s,%s\n","theta", "psi", "phi");

  // initialization complete
  printf("Buckler initialized after logger!\n");

  ret_code_t err_code;

  float x_val, y_val, z_val, x_val_g, y_val_g, z_val_g;
  float theta_prev, psi_prev, phi_prev, theta, psi, phi, theta_diff, psi_diff, phi_diff;
  theta_prev = 0;
  psi_prev   = 0;
  phi_prev   = 0;

  int data_num = 0;

  // loop forever
  while (data_num < 600) {
    // sample analog inputs
    x_val = sample_value(X_CHANNEL) * (3.6 / (float) (1 << 12));
    y_val = sample_value(Y_CHANNEL) * (3.6 / (float) (1 << 12));
    z_val = sample_value(Z_CHANNEL) * (3.6 / (float) (1 << 12));

    x_val_g = (x_val - (2.85 / 2)) / ((2.85 / 3) * .420);
    y_val_g = (y_val - (2.85 / 2)) / ((2.85 / 3) * .420);
    z_val_g = (z_val - (2.85 / 2)) / ((2.85 / 3) * .420);


    theta = atan2 (x_val_g, sqrt ((y_val_g * y_val_g) * (z_val_g * z_val_g)));
    psi   = atan2 (y_val_g, sqrt ((x_val_g * x_val_g) * (z_val_g * z_val_g)));
    phi   = atan2 (sqrt ((x_val_g * x_val_g) * (y_val_g * y_val_g)), z_val_g);

    theta_diff = theta - theta_prev;
    psi_diff   = psi - psi_prev;
    phi_diff   = phi - phi_prev;

    theta_prev = theta;
    psi_prev   = psi;
    phi_prev   = phi;

    // log data
    simple_logger_log("%f,%f,%f\n",theta, psi, phi);
    printf("tilt-theta: %f\ttilt-psi: %f\ttilt-phi:%f\n", theta, psi, phi);

    // using sprintf
    // float mybuffer[32];
    // sprintf (mybuffer, "%f;%f;%f\n", theta, psi, phi);
    // SEGGER_RTT_WriteString(0, mybuffer);
    // nrf_delay_ms(100);
    // printf("tilt-theta: %f\ttilt-psi: %f\ttilt-phi:%f\n", theta, psi, phi);
    nrf_delay_ms(100);

   	data_num++;
  }
}


