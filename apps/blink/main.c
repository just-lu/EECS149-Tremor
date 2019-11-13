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

int main(void) {

  // Initialize.
  nrf_gpio_cfg_output(LED);

  // Enter main loop.
  while (1) {
    nrf_gpio_pin_clear(LED);
    // nrf_gpio_pin_toggle(LED);
    // nrf_delay_ms(500);
  }
}