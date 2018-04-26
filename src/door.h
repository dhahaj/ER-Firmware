/*
 * door.h -> Door type definitions
 *
 * Created: 7/2/2014
 * Modified: 8/4/2015 3:48 PM
 * Author: DMH
 *
 */

#ifndef DOOR_H_
#define DOOR_H_

#include <stdint.h>
#include <stdbool.h>


typedef struct {
	volatile bool toggle_mode;
	volatile bool toggled, is_active;
	volatile uint8_t active_high;
	const uint8_t pin;
	const uint8_t port;
} Door;



#endif /* DOOR_H_ */