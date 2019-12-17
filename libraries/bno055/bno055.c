// MPU-9250 driver
//
// Read from MPU-9250 3-axis accelerometer/gyro/magnetometer over I2C
// Register documentation: http://www.invensense.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf
//
// Based on https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/blob/master/src/util/inv_mpu.c

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nrf_twi_mngr.h"

#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"

#include "bno055.h"

static const nrf_drv_spi_t* spi_instance;
static nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

void mpu9250_init_spi(const nrf_drv_spi_t* instance) {
  // LCD screen
  // #define BUCKLER_LCD_SCLK NRF_GPIO_PIN_MAP(0,17)
  // #define BUCKLER_LCD_MISO NRF_GPIO_PIN_MAP(0,16)
  // #define BUCKLER_LCD_MOSI NRF_GPIO_PIN_MAP(0,15)
  // #define BUCKLER_LCD_CS   NRF_GPIO_PIN_MAP(0,18)

  //  nrf_drv_timer_config_t timer_cfg = {
  //   .frequency          = NRF_TIMER_FREQ_1MHz,
  //   .mode               = NRF_TIMER_MODE_TIMER,
  //   .bit_width          = NRF_TIMER_BIT_WIDTH_32,
  //   .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
  //   .p_context          = NULL,
  // };
  spi_instance = instance;

  spi_config.sck_pin    = BUCKLER_LCD_SCLK;
  spi_config.miso_pin   = BUCKLER_LCD_MISO;
  spi_config.mosi_pin   = BUCKLER_LCD_MOSI;
  spi_config.ss_pin     = BUCKLER_LCD_CS;
  spi_config.frequency  = NRF_DRV_SPI_FREQ_1M;
  spi_config.mode       = NRF_DRV_SPI_MODE_0;
  spi_config.bit_order  = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

  // nrf_gpio_cfg_output(RTC_WDI);
  // nrf_gpio_pin_set(RTC_WDI);
}

void mpu9250_read_reg(uint8_t reg, uint8_t* read_buf, size_t len){
  if (len > 256) return;
  uint8_t readreg = reg;
  uint8_t buf[257];

  nrf_drv_spi_init(spi_instance, &spi_config, NULL, NULL);
  nrf_drv_spi_transfer(spi_instance, &readreg, 1, buf, len+1);
  nrf_drv_spi_uninit(spi_instance);

  memcpy(read_buf, buf+1, len);
}

void mpu9250_write_reg(uint8_t reg, uint8_t* write_buf, size_t len) {
  if (len > 256) return;
  uint8_t buf[257];
  buf[0] = 0x80 | reg;
  memcpy(buf+1, write_buf, len);

  nrf_drv_spi_init(spi_instance, &spi_config, NULL, NULL);
  nrf_drv_spi_transfer(spi_instance, buf, len+1, NULL, 0);
  nrf_drv_spi_uninit(spi_instance);
}

void mpu9250_get_gyro(mpu9250_measurement_t* gyr) {
  uint8_t read[10];

  mpu9250_read_reg(MPU9250_GYRO_XOUT_H, read, 8);

  printf("GYRO: %x\n", read[0]);
}

void mpu9250_get_acc(mpu9250_measurement_t* acc) {
  uint8_t read[10];

  mpu9250_read_reg(MPU9250_ACCEL_XOUT_H, read, 8);
  printf("ACCEL: %x\n", read[0]);
}

