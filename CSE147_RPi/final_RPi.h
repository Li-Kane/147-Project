
#ifndef _ASSIGNMENT_BODY_
#define _ASSIGNMENT_BODY_

#include <stdint.h>

// Macros
#define TURN_ON(pin) digitalWrite(pin, 1)
#define TURN_OFF(pin) digitalWrite(pin, 0)

#define READ(pin) digitalRead(pin)
#define WRITE(pin, x) digitalWrite(pin, x)

// Constants
#define RUNNING 3
#define PAUSE 2

#define LOW 0
#define HIGH 1

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

#define DETECT_SOUND 1
#define NO_SOUND 0

// A. Pin number definitions (DO NOT MODIFY)
// We use 8 sensors.
//
// 1. Button
#define PIN_BUTTON 0

// 2. Infrared motion sensor
#define PIN_MOTION 4

// 3. Rotary encoder
#define PIN_ROTARY_CLK 5
#define PIN_ROTARY_DT 6

// 4. Microphone sound sensor
#define PIN_SOUND 7

// 5. DIP two-color LED (Dual In-line Package)
#define PIN_DIP_RED 8
#define PIN_DIP_GRN 9

// 6. SMD RGB LED (Surface Mount Device)
#define PIN_SMD_RED 27
#define PIN_SMD_GRN 28
#define PIN_SMD_BLU 29

// 7. Auto-flash LED
#define PIN_ALED 12

// 8. Passive Buzzer
#define PIN_BUZZER 13

#define PIN_LASER 26 //THINK THIS IS RIGHT
#define PIN_SONIC_ECHO 5
#define PIN_SONIC_TRIG 6

// B. Shared structure
// All thread functions get a shared variable of the structure
// as the function parameter.
// If needed, you can add anything in this structure.
typedef struct shared_variable {
    int bProgramExit; // Once set to 1, the program will terminate.
    // You can add more variables if needed.
    int state;
    //int ROTstate; //rotation state (using CLOCKWISE and COUNTERCLOCKWISE)
} SharedVariable;

// C. Functions
// You need to implement the following functions.
// Do not change any function name here.
void init_shared_variable(SharedVariable* sv);
void init_sensors(SharedVariable* sv);
void body_button(SharedVariable* sv);     // Button
//void body_encoder(SharedVariable* sv);    // Rotary encoder
void body_twocolor(SharedVariable* sv);   // DIP two-color LED
void body_rgbcolor(SharedVariable* sv);   // SMD RGB LED
void body_laser(SharedVariable* sv); 
void body_sonic(SharedVariable* sv); 

#endif
