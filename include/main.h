#ifndef main_h // ifndef main_h to prevent double declaration of any identifiers such as types, enums and static variables
#define main_h // ifndef main_h 

// debug setup
//#define DEBUG_VAR
#define DEBUG_ACT
//#define DEBUG_FCT
#define DEBUG_STATE

#ifdef DEBUG_VAR
  #define DEBUG_PRINT_VAR(x) Serial.print(x)
  #define DEBUG_PRINTLN_VAR(x) Serial.println(x)
#else
  #define DEBUG_PRINTLN_VAR(x)
  #define DEBUG_PRINT_VAR(x) 
#endif

#ifdef DEBUG_ACT
  #define DEBUG_PRINT_ACT(x) Serial.println(x)
  #define DEBUG_PRINTLN_ACT(x) Serial.println(x)
#else
  #define DEBUG_PRINT_ACT(x)
  #define DEBUG_PRINTLN_ACT(x)
#endif

#ifdef DEBUG_FCT
  #define DEBUG_PRINTLN_FCT(x) Serial.println(x)
#else
  #define DEBUG_PRINTLN_FCT(x)
#endif

#ifdef DEBUG_STATE
  #define DEBUG_PRINTLN_STATE(x) Serial.print("Machine State: "); Serial.println(x)
#else
  #define DEBUG_PRINTLN_STATE(x)
#endif

// include software header files
#include <Arduino.h>

//Pin Settings and hardware header files

// NeoPixel LED pins and header
#include "Adafruit_NeoPixel.h"
#define NEOPIXEL_PIN 23
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800); //setup Neopixel

// ezButton pins and header
#include "ezButton.h"
#define button_debounce_time 100 
ezButton button_command(4); 
ezButton button_left(16);
ezButton button_backwards(19);
ezButton button_forwards(17);
ezButton button_right(18);
#define button_stop 5

// States
#define VOID_ST 0
#define INIT_ST 1
#define START_EXEC_ST 2
#define EXEC_ST 3
#define READ_COMM_ST 4
#define STOP_ST 5
#define FORWARD_ST 6
#define TURN_RIGHT_ST 7
#define TURN_LEFT_ST 8
#define BACK_ST 9

// Movement Commands
#define MAX_NR_COMMANDS 20
#define STOP 0
#define TURN_LEFT 1
#define TURN_RIGHT 3
#define FORWARD 2
#define BACKWARD 4
#define ROTATION_TICKS 1920

// commands
int nr_comm;
int comm_index;         // the index of the action that is being executed...
int recorded_button[MAX_NR_COMMANDS];
int button_index = 0;
int mov;                // Programed data from buttons
unsigned long button_command_count;  // Nr. of times command button is pressed

int on_execute_test_st; // state control variable
int on_execute_comm_st; // state control variable

// TODO - state in movement
int machine_state;
int last_machine_state;
int stop_next_state;

#endif // ifndef main_h