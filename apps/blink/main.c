// Blink app
//
// Blinks an LED

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

// Pin definitions for DK
#define LED NRF_GPIO_PIN_MAP(0,17)

// Test pin 15
#define TEST NRF_GPIO_PIN_MAP(0,15)

int main(void) {

  // Initialize.
  nrf_gpio_cfg_output(LED);
  nrf_gpio_cfg_output(TEST);

  // Enter main loop.
  while (1) {
    // nrf_gpio_pin_set(LED);
    nrf_gpio_pin_toggle(LED);
    nrf_gpio_pin_toggle(TEST);
    nrf_delay_ms(500);
  }
}
