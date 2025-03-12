# 147-Project
Bryce Blair, b1blair@ucsd.edu  
Kane Li, kal036@ucsd.edu

This is the 2025 Cse 147 final project repository for Bryce Blair and Kane Li, for the project BeamSense. BeamSense is a pointing direction tracker that seeks to track the distance from the tip of a user's finger to the nearest object. It does so by utilizing a red and green led for finger direction, a [ZED2 stereo camera](https://www.stereolabs.com/products/zed-2) for depth estimation, an [NVIDIA Jetson Orin Nano](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/) to handle finger tracking, and a [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/) to control the leds, on/off button, and ultrasonic sensor. All the files inside [CSE147_Jetson](./CSE147_Jetson/) are to be run on the Jetson Orin Nano, and files inside [CSE147_RPi](./CSE147_RPi/) are to be run on the raspberry pi 5.

## Nvidia Jetson Orin Nano
To begin running on a Jetson Orin Nano, make sure to flash the Jetpack 6.0 SDK with an Ubuntu OS. Then, install the ZED2 SDK on Linux and Python. You will also need to install python dependencies, mainly OpenCV, Numpy, Jetson.GPIO, and ZED2 python bindings. 

After all the dependencies are installed, you need to configure Jetson GPIO pins for output. This is due to bug in Jetpack 6.0 where pins are no longer able to output correctly. To fix this, you can use devmem to directly write to the register that corresponds to physical pin 31 on the board and modify its bits to enable output. Here are the commands:
```
# print what's current in the register
sudo busybox devmem 0x02430070
# write the register with 0x408
sudo busybox devmem 0x02430070 w 0x408
# verify the register has been written to
sudo busybox devmem 0x02430070
```
Since this can be a dangerous operation, it is recommended to read through the documentation yourself to configure the pinmux [(link)](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonAgxOrinSeries.html?highlight=gpio#changing-the-pinmux). Once this is run, just make sure to connect pin 31 from the 40 pin header to the Raspberry Pi 5.

From here, you will be able to start the script by plugging in the ZED2 camera to a USB port and simply running `python3 ./CSE147_Jetson/main.py`.

## Raspberry Pi 5
To begin running on the Raspberry Pi 5, you will need to flash the sd card with the Ubunutu custom kernel that was provided in Individual Project 1. Once you have set up your Raspberry Pi with the correct kernel you can begin to download independecies. This includes WiringPi which can be downloaded from this repo: https://github.com/WiringPi/WiringPi Once you have downloaded WiringPi by following the step in the github link, you should be good to start running the .c and .h files on the Raspberry Pi 5. 
When we want to run we first should run `make clean`. After running clean, we now have to choose if we want to compile for just using the glove, or if we want to compile using the ultrasonic sensor as well for testing distances. If we just want to compile we can simply run `make`. If we want to compile with the ultrasonic sensor, we run `make withSonic`. Both of these will compile into a file called, "final_main". We can then run this by calling `sudo ./final_main`.
Upon running the LEDs and laser emitter shoukd turn on. Then we want to start the Jetson Nano code, which will allow for the GPIO pins of the two to communicate via one GPIO connection.

## Ordering
First compile both files, then run the file on the RPi5 first, which should then be followed by running the file on the Jetson Nano.