/*
* main.h
*
* Created: 1/23/2013 11:54:48 AM
* Modified: 8/4/2015 3:48 PM
* Author: DMH
*/

// #pragma once

// #include <stdint.h>
// #include <stdbool.h>
// #include <avr/pgmspace.h>

// #ifndef Door
// #include "door.h"
// #endif

// /*
// DOOR I/O :		*/
// #define OUTPUT_PORT		PORTB	// Output port for Door1/2
// #define DOOR_PIN_PORT	PINB	// Door 1/2 Pins
// #define OUTPUT_DDR		DDRB	// Door 1/2 Data Direction Register
// #define DR1_OUT			PB0		// Door 1 Output Pin
// #define DR2_OUT			PB1		// Door 2 Output Pin

// /*
// RELAY I/O:		*/
// #define RELAY_PORT		PORTA	// Relays 1/2 Output port
// #define RELAY_PIN_PORT	PINA	// Relays 1/2 Input Pins
// #define RELAY_DDR		DDRA	// Relays 1/2 Data Direction Register
// #define RELAY1_PIN		PA0		// Relay 1 Output Pin
// #define RELAY2_PIN		PA1		// Relay 2 Output Pin

// /*
// DOOR TRIGGER INPUTS:		*/
// #define INPUT_PIN_PORT	PIND	// Inputs 1/2 Input Port
// #define INPUT_PORT		PORTD	// Inputs 1/2 Output Port
// #define INPUT_DDR		DDRD	// Inputs 1/2 Data Direction Register

// /*
// DIP SWITCHES:		*/
// #define DR1_BUTTON			PIND2	// Door 1 Input Pin
// #define DR2_BUTTON			PIND3	// Door 2 Input Pin
// #define MODE_PIN			PIND0	// Mode selection pin
// #define ACTIVE_MODE1_PIN	PIND4	// Active Mode Pin for Door 1
// #define TOGGLE1_PIN			PIND1	// Toggle Mode pin for Door 1
// #define ACTIVE_MODE2_PIN	PIND7	// Active Mode Pin for Door 2
// #define TOGGLE2_PIN			PIND6	// Toggle Mode pin for Door 2

// /*
// TIMING SWITCHES:			*/
// #define DS_PORT				PORTC	// Digital Switch Input Port
// #define DS_PIN_PORT			PINC	// Digital Switch Input Pin Port
// #define DS_DDR				DDRC	// Data Direction Register for DS_PORT
// #define DS1_PIN				PC0		// DS1 Control Output Pin
// #define DS2_PIN				PC1		// DS2 Control Output Pin
// #define INPUT_MASK		(BIT(MODE_PIN) | BIT(TOGGLE1_PIN) | BIT(TOGGLE2_PIN) | BIT(DR1_BUTTON) | BIT(DR2_BUTTON) | BIT(ACTIVE_MODE1_PIN) | BIT(ACTIVE_MODE2_PIN))

// /*
// VARIOUS DEFINITIONS:		*/
// #define DIFF_DELAY		300UL	// Differential Delay Time between door retractions(ms)
// #define SIG_DELAY		100UL	// Delay Time between activating the relay signals(ms)
// #define EExER_DELAY		500UL	// Delay for the EExER build option
// #define DEPENDENT		0x1		// Operating Mode Types
// #define INDEPENDENT		0x0
// #define ACTIVE_HIGH		0x1		// Output Mode Types
// #define ACTIVE_LOW		0x0
// #define DEBOUNCE_TIME	50		// Input Switch Debounce Time (us)

// #ifndef __bool_true_false_are_defined
// #define true 1
// #define false 0
// #endif

// typedef uint8_t ui8;
// typedef uint8_t byte;

// enum DoorMode
// {
  // ModeDependent   = 0,
  // ModeIndependent = 1,
// };

// enum OutputMode
// {
  // ActiveLow  = 0,
  // ActiveHigh = 1
// };

// enum ToggleMode
// {
  // ToggleOff = 0,
  // ToggleOn  = 1
// };

// struct Settings {
  // Settings(DoorMode _doorMode = ModeDependent, OutputMode _outMode = ActiveHigh, ToggleMode _toggleMode = ToggleOff): doorMode(_doorMode), outputMode(_outputMode), toggleMode(_toggleMode) {}
  
  // DoorMode doorMode;
  // OutputMode outputMode;
  // ToggleMode toggleMode;
// };

// enum DelayTime
// {
    // delay_1s   = 1,
    // delay_2s   = 2,
    // delay_3s   = 3,
    // delay_5s   = 5,
    // delay_7s   = 7,
    // delay_10s  = 10,
    // delay_15s  = 15,
    // delay_20s  = 20,
    // delay_25s  = 25,
    // delay_30s  = 30,
    // delay_35s  = 35,
    // delay_40s  = 40,
    // delay_45s  = 45,
    // delay_50s  = 50,
    // delay_55s  = 55,
    // delay_60s  = 60
// };

// enum RotaryPositions
// {
  // position0 = 0b0000,
  // position1 = 0b0001,
  // position2 = 0b0010,
  // position3 = 0b0011,
  // position4 = 0b0100,
  // position5 = 0b0101,
  // position6 = 0b0110,
  // position7 = 0b0111,
  // position8 = 0b1000,
  // position9 = 0b1001,
  // positionA = 0b1010,
  // positionB = 0b1011,
  // positionC = 0b1100,
  // positionD = 0b1101,
  // positionE = 0b1110,
  // positionF = 0b1111,
// };

// struct Delay {
  // Delay(DelayTime _delay1 = delay_1s, DelayTime _delay2 = delay_1s): delay1(_delay1), delay2(_delay2) {}

  // DelayTime delay1;
  // DelayTime delay2;
// };

// Door dr1(), dr2();

// typedef struct {
  // volatile bool mode;
  // volatile ui8 prev_values;
  // volatile bool retracting;
// } Inputs;

// Inputs    dip_switch;


// void dependentRetract(void);

// void activateDoor1(bool activate);
// void activateDoor2(bool activate);
// void readDip(void);

// unsigned long door_timer(bool reset);
// bool buttonPressed(volatile ui8 *PIN, ui8 BUTTON_BIT);
// bool doorActive(const Door *door);
// void doorPinWrite(const Door *door, bool activate);

// void setSettings(const Settings& settings);
// const Settings& getSettings();
// void setDelay(const Delay &delay);
// const Delay& getDelay();

// #endif /* MAIN_H_ */

// /*
// * main.h
// *
// * Created: 1/23/2013 11:54:48 AM
// * Modified: 8/4/2015 3:48 PM
// * Author: DMH
// */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <stdbool.h>
#include <avr/pgmspace.h>

#ifndef Door
#include "door.h"
#endif

/*
DOOR I/O :		*/
#define OUTPUT_PORT		PORTB	// Output port for Door1/2
#define DOOR_PIN_PORT	PINB	// Door 1/2 Pins
#define OUTPUT_DDR		DDRB	// Door 1/2 Data Direction Register
#define DR1_OUT			PB0		// Door 1 Output Pin
#define DR2_OUT			PB1		// Door 2 Output Pin

/*
RELAY I/O:		*/
#define RELAY_PORT		PORTA	// Relays 1/2 Output port
#define RELAY_PIN_PORT	PINA	// Relays 1/2 Input Pins
#define RELAY_DDR		DDRA	// Relays 1/2 Data Direction Register
#define RELAY1_PIN		PA0		// Relay 1 Output Pin
#define RELAY2_PIN		PA1		// Relay 2 Output Pin

/*
DOOR TRIGGER INPUTS:		*/
#define INPUT_PIN_PORT	PIND	// Inputs 1/2 Input Port
#define INPUT_PORT		PORTD	// Inputs 1/2 Output Port
#define INPUT_DDR		DDRD	// Inputs 1/2 Data Direction Register

/*
DIP SWITCHES:		*/
#define DR1_BUTTON			PIND2	// Door 1 Input Pin
#define DR2_BUTTON			PIND3	// Door 2 Input Pin
#define MODE_PIN			PIND0	// Mode selection pin
#define ACTIVE_MODE1_PIN	PIND4	// Active Mode Pin for Door 1
#define TOGGLE1_PIN			PIND1	// Toggle Mode pin for Door 1
#define ACTIVE_MODE2_PIN	PIND7	// Active Mode Pin for Door 2
#define TOGGLE2_PIN			PIND6	// Toggle Mode pin for Door 2

/*
TIMING SWITCHES:			*/
#define DS_PORT				PORTC	// Digital Switch Input Port
#define DS_PIN_PORT			PINC	// Digital Switch Input Pin Port
#define DS_DDR				DDRC	// Data Direction Register for DS_PORT
#define DS1_PIN				PC0		// DS1 Control Output Pin
#define DS2_PIN				PC1		// DS2 Control Output Pin
#define INPUT_MASK			(BIT(MODE_PIN) | BIT(TOGGLE1_PIN) | BIT(TOGGLE2_PIN) | BIT(DR1_BUTTON) | BIT(DR2_BUTTON) | BIT(ACTIVE_MODE1_PIN) | BIT(ACTIVE_MODE2_PIN))

/*
VARIOUS DEFINITIONS:		*/
#define DIFF_DELAY			300UL	// Differential Delay Time between door retractions(ms)
#define SIG_DELAY			100UL	// Delay Time between activating the relay signals(ms)
#define EExER_DELAY			500UL	// Delay for the EExER build option
#define DEPENDENT			0x1		// Operating Mode Types
#define INDEPENDENT			0x0
#define ACTIVE_HIGH			0x1		// Output Mode Types
#define ACTIVE_LOW			0x0
#define DEBOUNCE_TIME		50		// Input Switch Debounce Time (us)

#ifndef __bool_true_false_are_defined
#define true				1
#define false 				0
#endif

#ifdef __cplusplus
extern "C"{
#endif

typedef uint8_t ui8;
typedef uint8_t byte;

extern Door dr1, dr2;

typedef struct {
	volatile bool mode;
	volatile ui8 prev_values;
	volatile bool retracting;
} Inputs;

extern Inputs 	dip_switch;
void 	dependentRetract(void);

extern void activateDoor1(bool activate);
extern void activateDoor2(bool activate);
extern void readDip(void);

unsigned long 	door_timer(bool reset);
bool 			buttonPressed(volatile ui8 *PIN, ui8 BUTTON_BIT);
bool 			doorActive(const Door *door);
void 			doorPinWrite(const Door *door, bool activate);

#ifdef __cplusplus
} // extern "C"
#endif

enum DoorMode
{
  ModeDependent   = 0,
  ModeIndependent = 1,
};

enum OutputMode
{
  ActiveLow  = 0,
  ActiveHigh = 1
};

enum ToggleMode
{
  ToggleOff = 0,
  ToggleOn  = 1
};

// struct Settings {
  // Settings(DoorMode _doorMode = ModeDependent, OutputMode _outMode = ActiveHigh, ToggleMode _toggleMode = ToggleOff): doorMode(_doorMode), outputMode(_outputMode), toggleMode(_toggleMode) {}
  
  // DoorMode doorMode;
  // OutputMode outputMode;
  // ToggleMode toggleMode;
// };


enum RotaryPositions
{
  position0 = 0b0000,
  position1 = 0b0001,
  position2 = 0b0010,
  position3 = 0b0011,
  position4 = 0b0100,
  position5 = 0b0101,
  position6 = 0b0110,
  position7 = 0b0111,
  position8 = 0b1000,
  position9 = 0b1001,
  positionA = 0b1010,
  positionB = 0b1011,
  positionC = 0b1100,
  positionD = 0b1101,
  positionE = 0b1110,
  positionF = 0b1111,
};


enum DelayTime
{
    delay_1s   = 1,
    delay_2s   = 2,
    delay_3s   = 3,
    delay_5s   = 5,
    delay_7s   = 7,
    delay_10s  = 10,
    delay_15s  = 15,
    delay_20s  = 20,
    delay_25s  = 25,
    delay_30s  = 30,
    delay_35s  = 35,
    delay_40s  = 40,
    delay_45s  = 45,
    delay_50s  = 50,
    delay_55s  = 55,
    delay_60s  = 60
};

// struct Delay {
	// Delay(DelayTime _delay1 = delay_1s, DelayTime _delay2 = delay_1s): delay1(_delay1), delay2(_delay2) {}

  // DelayTime delay1;
  // DelayTime delay2;
// };

#endif /* MAIN_H_ */
