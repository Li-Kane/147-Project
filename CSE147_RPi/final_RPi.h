#ifndef _ASSIGNMENT_BODY_
#define _ASSIGNMENT_BODY_

#include <stdint.h>

// Constants
#define RUNNING 3
#define PAUSE 2

#define LOW 0
#define HIGH 1

#define CLOSE 1
#define FAR 0

// Pin number definitions
// We use 7 sensors.
//
// 1. Button
#define PIN_BUTTON 0

// 2. DIP two-color LED
#define PIN_DIP_RED 8
#define PIN_DIP_GRN 9

// 3. SMD RGB LED
#define PIN_SMD_RED 27
#define PIN_SMD_GRN 28
#define PIN_SMD_BLU 29

// 4. Active Buzzer
#define PIN_BUZZER 13

// 5. Laser Emitter
#define PIN_LASER 26

// 6. UltraSonic Sensor
#define PIN_SONIC_ECHO 5
#define PIN_SONIC_TRIG 6

// 7. Jetson Nano Communication
#define PIN_NANO 2

// B. Shared structure
// All thread functions get a shared variable of the structure
// as the function parameter.
// If needed, you can add anything in this structure.
typedef struct shared_variable {
    int bProgramExit; // Once set to 1, the program will terminate.
    // You can add more variables if needed.
    int state;
    int nanoState;
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
void body_buzzer(SharedVariable* sv);
void body_nano(SharedVariable* sv);

#endif
