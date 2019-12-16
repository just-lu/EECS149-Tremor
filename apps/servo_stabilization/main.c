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

APP_PWM_INSTANCE(PWM2,2);                   // Create the instance "PWM1" using TIMER1.

static volatile bool ready_flag;            // A flag indicating PWM status.

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

// static volatile bool volun_flag; 
// void pin_change_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
//   if (nrfx_gpiote_in_is_set(BUCKLER_SWITCH0)) {
//     volun_flag = true;
//   } else {
//     volun_flag = false;
//   }
// }

// LED array
static uint8_t LEDS[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// flag to poll
static volatile bool poll_flag = false;

// polling callback function
void poll() {
  poll_flag = true;
}

int main(void) {
  // servo stuff
  ret_code_t err_code;

  // initializing servo pin number
  uint8_t SERVO_PIN = 3;
  uint8_t SERVO_PIN_TWO = 4;

  /* 1-channel PWM, 50Hz, output on DK LED pins, 20ms period */
  app_pwm_config_t pwm2_cfg = APP_PWM_DEFAULT_CONFIG_2CH(20000L, SERVO_PIN, SERVO_PIN_TWO);

  //Switch the polarity of the first channel. 
  pwm2_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
  pwm2_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

  /* Initialize and enable PWM. */
  err_code = app_pwm_init(&PWM2,&pwm2_cfg,NULL);
  APP_ERROR_CHECK(err_code);
  app_pwm_enable(&PWM2);

  ret_code_t error_code = NRF_SUCCESS;

  // initialize GPIO driver, need to uncomment when app_pwm_init is not there
  // if (!nrfx_gpiote_is_init()) {
  //   error_code = nrfx_gpiote_init();
  // }
  // APP_ERROR_CHECK(error_code);

  
  // // initializing printing to sd card
  // printf("Started SD card demo app...\n");

  // // Enable SoftDevice (used to get RTC running)
  // nrf_sdh_enable_request();

  // // Configure GPIOs
  // nrf_gpio_cfg_output(BUCKLER_SD_ENABLE);
  // nrf_gpio_cfg_output(BUCKLER_SD_CS);
  // nrf_gpio_cfg_output(BUCKLER_SD_MOSI);
  // nrf_gpio_cfg_output(BUCKLER_SD_SCLK);
  // nrf_gpio_cfg_input(BUCKLER_SD_MISO, NRF_GPIO_PIN_NOPULL);

  // nrf_gpio_pin_set(BUCKLER_SD_ENABLE);
  // nrf_gpio_pin_set(BUCKLER_SD_CS);

  // // Initialize SD card
  // const char filename[] = "test.txt";
  // const char permissions[] = "a"; // w = write, a = append

  // // Start file
  // simple_logger_init(filename, permissions);

  // // If no header, add it
  // simple_logger_log_header("HEADER for file \'%s\', written on %s \n", filename, "DATE");
  // printf("Wrote header to SD card\n");

  // configure leds
  // manually-controlled (simple) output, initially set
  nrfx_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
  for (int i=0; i<3; i++) {
    error_code = nrfx_gpiote_out_init(LEDS[i], &out_config);
    APP_ERROR_CHECK(error_code);
  }

  // nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
  // in_config.pull = NRF_GPIO_PIN_NOPULL;
  // error_code = nrfx_gpiote_in_init(BUCKLER_SWITCH0, &in_config, pin_change_handler);
  // nrfx_gpiote_in_event_enable(BUCKLER_SWITCH0, true);

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

  // initialize timer library
  virtual_timer_init();
  nrf_delay_ms(1000);

  // polling period
  uint32_t poll_period = 20; // in ms

  // loop forever
  // float x_rot = 0;
  // float y_rot = 0;
  float z_rot = 0;

  float initial_z = 100.0;
  float prev_z = 100.0;

  // float initial_x = 100.0;
  // float prev_x = 100.0;

  float output = 0.0;
  // float x_output = 0.0;
  float input, input_start, input_end, output_start, output_end;

  int loop_index = 0;
  int recalibration_count = 0;
  int z_direction = 0;
  // int x_direction = 0;
  int prev_z_direction = 100;
  // int prev_x_direction = 100;

  int tremor_count = 0;
  int time_count = 0;
  int same_direction_count = 0;

  int threshold = 4;
  int period_count = 50;
  bool volun_flag = false;

  // start polling timer in microsec
  virtual_timer_start_repeated(poll_period * 1000, poll);

  while (1) {     // (loop_index < 100000) for certain time
    // blink two LEDs
    nrf_gpio_pin_toggle(LEDS[loop_index%3]);

    if (poll_flag) {
      // get measurements
      mpu9250_measurement_t acc_measurement = mpu9250_read_accelerometer();
      mpu9250_measurement_t gyr_measurement = mpu9250_read_gyro();

      // determine rotation from gyro
      // gyros are messy, so only add value if it is of significant magnitude
      // note that we are dividing by 10 since we are measuring over a tenth of a second
      // float x_rot_amount = gyr_measurement.x_axis * poll_period / 1000.00;
      // if (x_rot_amount > 0.05 || x_rot_amount < -0.05) {
      //   x_rot += x_rot_amount;
      // }
      // float y_rot_amount = gyr_measurement.y_axis * poll_period / 1000.00;
      // if (y_rot_amount > 0.05 || y_rot_amount < -0.05) {
      //   y_rot += y_rot_amount;
      // }
      float z_rot_amount = gyr_measurement.z_axis * poll_period / 1000.00;
      if (z_rot_amount > 0.1 || z_rot_amount < -0.1) {
        z_rot += z_rot_amount;
      }

      // print results
      printf("                      Z-Axis\n");
      printf("                  ----------\n");
      printf("I2C IMU Acc  (g): %10.3f\n", acc_measurement.z_axis);
      printf("I2C IMU Gyro (g): %10.3f\n", gyr_measurement.z_axis);
      printf("Angle  (degrees): %10.3f\n", z_rot);
      printf("\n");
      // printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
      // printf("                  ----------\t----------\t----------\n");
      // printf("I2C IMU Acc (g): %10.3f\t%10.3f\t%10.3f\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
      // printf("I2C IMU Gyro (g):  %10.3f\t%10.3f\t%10.3f\n", gyr_measurement.x_axis, gyr_measurement.y_axis, gyr_measurement.z_axis);
      // printf("Angle  (degrees): %10.3f\t%10.3f\t%10.3f\n", x_rot, y_rot, z_rot);
      // printf("\n");

      // simple_logger_log("Acc,%f,%f,%f\n",acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);
      // simple_logger_log("Gyro,%f,%f,%f\n",gyr_measurement.x_axis, gyr_measurement.y_axis, gyr_measurement.z_axis);
      // simple_logger_log("Angle,%f,%f,%f\n",x_rot, y_rot, z_rot);

      
      if (loop_index <= 5) {
        initial_z = z_rot;
        prev_z = z_rot;
        // initial_x = x_rot;
        // prev_x = x_rot;
      }

      // if (prev_z != 100.0 && fabsf(prev_z - z_rot) < 5.0) {
      //   recalibration_count += 1;
      // }

      // if (recalibration_count > 25 && tremor_count < 5) {
      //   initial_z = z_rot;
      //   initial_x = x_rot;   
      //   recalibration_count = 0;
      // }


      // if (initial_z - z_rot > 40.0) { //cw
      //   output = 10.0;
      // } else if (z_rot - initial_z > 40.0) { //ccw
      //   output = 5.0;
      // } else if (initial_z - z_rot > 0) { //cw
      //   input = z_rot - initial_z;
      //   input_start = 0;
      //   input_end = 40;
      //   output_start = 7.5;
      //   output_end = 10;
      //   float slope = 1.0 * (output_end - output_start)/(input_end - input_start);
      //   output = output_start + slope * (input - input_start);
      // } else if (z_rot - initial_z > 0) { //ccw
      //   input = initial_z - z_rot;
      //   input_start = 0;   
      //   input_end = 40;
      //   output_start = 7.5;
      //   output_end = 5;
      //   float slope = 1.0 * (output_end - output_start)/(input_end - input_start);
      //   output = output_start + slope * (input - input_start);
      // } else {
      //   output = 0.0;
      // }

      // printf("Mapping: %f", output);
      
      if (initial_z != 100.0 && prev_z_direction == 100) {
        if (z_rot - initial_z < 0) {
          prev_z_direction = 1;  // CW
        } else if (z_rot - initial_z > 0) {
          prev_z_direction = 2;  // CCW
        }
      }

      // z_direction = 0;
      // if (initial_z != 100.0) {
      //   if (z_rot - initial_z < 0) {
      //     z_direction = 1;
      //   } else if (z_rot - initial_z > 0) {
      //     z_direction = 2;  
      //   }
      // }

      z_direction = 0;
      if (initial_z != 100.0) {
        if (z_rot - prev_z < -20.0) { //cw
          output = 7.8;
        } else if (z_rot - prev_z > 20.0) { //ccw
          output = 7.3;
        } else if (z_rot - prev_z < -1.0) { //cw
          input = prev_z - z_rot;
          if (input < 4) {
            output = 7.57;
          } else {
            input_start = 2;
            input_end = 20;
            output_start = 7.57;
            output_end = 7.8;
            float slope = 1.0 * (output_end - output_start)/(input_end - input_start);
            output = output_start + slope * (input - input_start);
          }
          z_direction = 1;
        } else if (z_rot - prev_z > 1.0) { //ccw
          input = z_rot - prev_z;
          if (input < 4) {
            output = 7.555;
          } else {
            input_start = 2;
            input_end = 20;
            output_start = 7.54;
            output_end = 7.3;
            float slope = 1.0 * (output_end - output_start)/(input_end - input_start);
            output = output_start + slope * (input - input_start);
          }
          z_direction = 2;
        } 
      }

      // z_direction = 0;
      // float diff = z_rot - prev_z;
      // if (initial_z != 100.0) {
      //   if (diff < -20 || diff > 20) {
      //     if (diff > 0)
      //   }
      // }

      // printf("z_direction: %d\n", z_direction);    
      // printf("prev_z_direction: %d\n", prev_z_direction); 
      // printf("difference: %f\n", z_rot-prev_z); 
      // printf("output: %f\n", output);


      if ((z_direction == 1 && prev_z_direction != 1)) {
        if (tremor_count < 10) {
          tremor_count++;
        }
      }

      if ((z_direction == 1 && prev_z_direction == 1) || (z_direction == 2 && prev_z_direction == 2)) {
        same_direction_count++;
      } else {
        same_direction_count = 0;
      }

      if (same_direction_count > 50) {
        tremor_count = 0;
        same_direction_count = 0;
        volun_flag = false;
      }

      // if (fabsf(initial_z - z_rot) > 100.0) {
      //   tremor_count = 0;
      //   recalibration_count = 0;

      // }
      if (tremor_count >= threshold && time_count < period_count) {
        volun_flag = true;
      } else if (tremor_count < threshold && time_count >= period_count) {
        volun_flag = false;
        time_count = 0;
        tremor_count = 0;
        
      }

      if (time_count >= period_count) {
        time_count = 0;
        // tremor_count = 0;
        if (tremor_count - 3 >= 0) {
          tremor_count = tremor_count - 3;
        } else {
          tremor_count = 0;
        }
      }

      // printf("tremor_count: %d\n", tremor_count);    
      // printf("time_count: %d\n", time_count); 
      // printf("volun_flag: %d\n", volun_flag); 
      
      poll_flag = false;
    }

    printf("Z: %x\n", z_direction);
    // printf("Prev Z: %x\n", prev_z_direction);
    if (z_direction == 1) { // microservo is between 5 and 10
      while (app_pwm_channel_duty_set(&PWM2, 0, 7.65) == NRF_ERROR_BUSY);
      nrf_delay_ms(2);
      printf("Direction 111111111111111111111");
    } else if (z_direction == 2) {
      while (app_pwm_channel_duty_set(&PWM2, 7.45, output) == NRF_ERROR_BUSY);
      nrf_delay_ms(2);
      printf("Direction 2222222222222222222222");
    } else {
      while (app_pwm_channel_duty_set(&PWM2, 0, 0) == NRF_ERROR_BUSY);
      nrf_delay_ms(2);
    }
    printf("Time: %d\n\n", read_timer());


    // if (initial_x != 100.0 && prev_x_direction == 100) {
    //   if (x_rot - initial_x < 0) {
    //     prev_x_direction = 1;
    //   } else if (x_rot - initial_x > 0) {
    //     prev_x_direction = 2;
    //   }
    // }

    // x_direction = 0;
    // if (initial_x != 100.0) {
    //   if (x_rot - initial_x < 0) {
    //     x_direction = 1;
    //   } else if (x_rot - initial_x > 0) {
    //     x_direction = 2;
    //   }
    // }

    // if (initial_x - x_rot > 40.0) { //cw
    //   x_output = 10.0;
    // } else if (x_rot - initial_x > 40.0) { //ccw
    //   x_output = 5.0;
    // } else if (initial_x - x_rot > 0) { //cw
    //   input = x_rot - initial_x;
    //   input_start = 0;
    //   input_end = 40;
    //   output_start = 7.5;
    //   output_end = 10;
    //   float slope = 1.0 * (output_end - output_start)/(input_end - input_start);
    //   x_output = output_start + slope * (input - input_start);
    // } else if (x_rot - initial_x > 0) { //ccw
    //   input = initial_x - x_rot;
    //   input_start = 0;   
    //   input_end = 40;
    //   output_start = 7.5;
    //   output_end = 5;
    //   float slope = 1.0 * (output_end - output_start)/(input_end - input_start);
    //   x_output = output_start + slope * (input - input_start);
    // } else {
    //   x_output = 0.0;
    // }

    // // printf("X: %x\n", x_direction);
    // // printf("Prev X: %x\n", prev_x_direction);
    // if (x_output != 0.0 && volun_flag == true) {
    //   while (app_pwm_channel_duty_set(&PWM2, 1, x_output) == NRF_ERROR_BUSY);
    //   nrf_delay_ms(10);
    
    // // else if (x_direction == 2) {
    // //   while (app_pwm_channel_duty_set(&PWM2, 1, 7.9) == NRF_ERROR_BUSY);
    // //   nrf_delay_ms(10);
    // } else {
    //   while (app_pwm_channel_duty_set(&PWM2, 1, 0) == NRF_ERROR_BUSY);
    //   nrf_delay_ms(10);
    // }

    time_count++;
    // prev_x_direction = x_direction;
    prev_z_direction = z_direction;
    prev_z = z_rot; 
    loop_index++;
  }
}