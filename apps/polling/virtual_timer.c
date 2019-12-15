// Virtual timer implementation

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "nrf.h"

#include "mpu9250.h"
#include "virtual_timer.h"
#include "virtual_timer_linked_list.h"

// This is the interrupt handler that fires on a compare event
void TIMER4_IRQHandler(void) {
  // This should always be the first line of the interrupt handler!
  // It clears the event so that it doesn't happen again
  NRF_TIMER4->EVENTS_COMPARE[0] = 0;

  uint32_t start_time = read_timer();
  node_t * temp = list_get_first();
  while(temp != NULL && temp->timer_value <= read_timer()) {
  	  // __disable_irq();
    temp->cb();
    printf("fail 3\n");
    list_remove_first();

	  if(temp->repeated) {
	  	temp->timer_value = start_time + temp->incr;
	  	list_insert_sorted(temp);
	  } else {
	  	free(temp);
	  }
	  if(list_get_first()) {
	  	NRF_TIMER4->CC[0] = list_get_first() -> timer_value;
	  }
    temp = list_get_first();
    printf("fail 4\n");
	  // __enable_irq();
  }


}

// Read the current value of the timer counter
uint32_t read_timer(void) {

  // Should return the value of the internal counter for TIMER4
	//6.2.2
	NRF_TIMER4->TASKS_CAPTURE[1] = 1;
  return NRF_TIMER4->CC[1];
}

// Initialize TIMER4 as a free running timer
// 1) Set to be a 32 bit timer
// 2) Set to count at 1MHz
// 3) Enable the timer peripheral interrupt (look carefully at the INTENSET register!)
// 4) Clear the timer
// 5) Start the timer
void virtual_timer_init(void) {
  // Place your timer initialization code here

	NRF_TIMER4->PRESCALER = 0x4;
	NRF_TIMER4->BITMODE = 0x3;
	NRF_TIMER4->TASKS_CLEAR = 1;

	NRF_TIMER4->INTENSET |= 1<<16;
	NVIC_EnableIRQ(TIMER4_IRQn);

  NRF_TIMER4->TASKS_START = 1;
}

// Start a timer. This function is called for both one-shot and repeated timers
// To start a timer:
// 1) Create a linked list node (This requires `malloc()`. Don't forget to free later)
// 2) Setup the linked list node with the correct information
//      - You will need to modify the `node_t` struct in "virtual_timer_linked_list.h"!
// 3) Place the node in the linked list
// 4) Setup the compare register so that the timer fires at the right time
// 5) Return a timer ID
//
// Your implementation will also have to take special precautions to make sure that
//  - You do not miss any timers
//  - You do not cause consistency issues in the linked list (hint: you may need the `__disable_irq()` and `__enable_irq()` functions).
//
// Follow the lab manual and start with simple cases first, building complexity and
// testing it over time.
static uint32_t timer_start(uint32_t microseconds, virtual_timer_callback_t cb, bool repeated) {

    __disable_irq();
  	node_t* timer_node = malloc(sizeof(node_t));
  	timer_node->timer_value = microseconds + read_timer();
  	timer_node->cb = cb;
  	timer_node->incr = microseconds;
  	timer_node->repeated = repeated;
  	list_insert_sorted(timer_node);
  	NRF_TIMER4->CC[0] = list_get_first()->timer_value;
    __enable_irq();
  	return timer_node;

}

// You do not need to modify this function
// Instead, implement timer_start
uint32_t virtual_timer_start(uint32_t microseconds, virtual_timer_callback_t cb) {
  return timer_start(microseconds, cb, false);
}

// You do not need to modify this function
// Instead, implement timer_start
uint32_t virtual_timer_start_repeated(uint32_t microseconds, virtual_timer_callback_t cb) {
  return timer_start(microseconds, cb, true);
}

// Remove a timer by ID.
// Make sure you don't cause linked list consistency issues!
// Do not forget to free removed timers.
void virtual_timer_cancel(uint32_t timer_id) {
  list_remove((node_t*) timer_id);
  free((node_t*) timer_id);
  if(list_get_first() != NULL){
      NRF_TIMER4->CC[2] = list_get_first()->timer_value;
  }
  else{
      NRF_TIMER4->CC[2] = 0;
  }
}

