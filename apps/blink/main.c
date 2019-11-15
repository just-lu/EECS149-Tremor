// Blink app
//
// Blinks an LED

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

// Pin definitions for DK
#define LED NRF_GPIO_PIN_MAP(0,7)

// Test pin 15
#define TEST NRF_GPIO_PIN_MAP(0,15)
#define TEST1 NRF_GPIO_PIN_MAP(0,14)
#define TEST2 NRF_GPIO_PIN_MAP(0,13)
#define TEST3 NRF_GPIO_PIN_MAP(0,12)
#define TEST4 NRF_GPIO_PIN_MAP(0,11)
#define TEST16 NRF_GPIO_PIN_MAP(0,16)
#define TEST17 NRF_GPIO_PIN_MAP(0,17)
#define TEST18 NRF_GPIO_PIN_MAP(0,18)
#define TEST19 NRF_GPIO_PIN_MAP(0,19)

int main(void) {

  // Initialize.
  nrf_gpio_cfg_output(LED);
  nrf_gpio_cfg_output(TEST);
  nrf_gpio_cfg_output(TEST1);
  nrf_gpio_cfg_output(TEST2);
  nrf_gpio_cfg_output(TEST3);
  nrf_gpio_cfg_output(TEST4);
  nrf_gpio_cfg_output(TEST16);
  nrf_gpio_cfg_output(TEST17);
  nrf_gpio_cfg_output(TEST18);
  nrf_gpio_cfg_output(TEST19);

  // Enter main loop.
  while (1) {
    // nrf_gpio_pin_set(LED);
    nrf_gpio_pin_toggle(LED);
    nrf_gpio_pin_toggle(TEST);
    nrf_gpio_pin_toggle(TEST1);
    nrf_gpio_pin_toggle(TEST2);
    nrf_gpio_pin_toggle(TEST3);
    nrf_gpio_pin_toggle(TEST4);
    nrf_gpio_pin_toggle(TEST16);
    nrf_gpio_pin_toggle(TEST17);
    nrf_gpio_pin_toggle(TEST18);
    nrf_gpio_pin_toggle(TEST19);
    nrf_delay_ms(250);
  }
}
