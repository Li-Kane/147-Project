//Threads file with ultrasonic sensor for testing distance, used similar implementation to individual project

#include <stdio.h>
#include <pthread.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "final_RPi.h"

// Thread declaration macros
#define thread_decl(NAME) \
void* thread_##NAME(void* param) { \
	SharedVariable* pV = (SharedVariable*) param; \
	body_##NAME(pV); \
	return NULL; }

// Declare 7 thread, 1 for each sensor
thread_decl(button)
thread_decl(twocolor)
thread_decl(rgbcolor)
thread_decl(buzzer)
thread_decl(laser)
thread_decl(sonic)
thread_decl(nano)

// Thread creation and joining macros
#define thread_create(NAME) pthread_create(&t_##NAME, NULL, thread_##NAME, &v);
#define thread_join(NAME) pthread_join(t_##NAME, NULL);

int main(int argc, char* argv[]) {
	// Initialize shared variable
	SharedVariable v;

	// Initialize WiringPi library
	if (wiringPiSetup() == -1) {
		printf("Failed to setup wiringPi.\n");
		return 1; 
	}

	// Initialize shared variable and sensors
	init_shared_variable(&v);
	init_sensors(&v);

	// Thread identifiers
	pthread_t t_button,
		  t_twocolor,
		  t_buzzer,
	          t_rgbcolor,
		  t_laser,
		  t_nano,
		  t_sonic;

	// Main program loop
	while (v.bProgramExit != 1) {
		// Create sensing threads
		thread_create(button);
		thread_create(buzzer);
		thread_create(twocolor);
		thread_create(rgbcolor);
		thread_create(laser);
		thread_create(sonic);
		thread_create(nano);

		// Wait for all threads to finish
		thread_join(button);
		thread_join(buzzer);
		thread_join(twocolor);
		thread_join(rgbcolor);
		thread_join(laser);
		thread_join(sonic);
		thread_join(nano);

		// Add a slight delay between iterations
		delay(10);
	}

	printf("Program finished.\n");

	return 0;
}
