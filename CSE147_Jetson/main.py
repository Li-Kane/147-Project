import pyzed.sl as sl
import cv2
from zed import Zed
import Jetson.GPIO as GPIO

'''
Main program to be run Jetson Orin Nano that tracks  a red and green led,
visualizing a pointing direction from green to red. It then outputs the 
distance from the red led to the nearest object, and sends an output to the
GPIO pin if the distance is less than 250mm. 
'''

# GPIO BCM pin 6 is board pin 31
output_pin = 6

# setup gpio so we can communicate with rPi
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

# create a camera object from the ZED2 API
def setup_camera():
    camera = sl.Camera()
    init_params = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                                coordinate_units=sl.UNIT.MILLIMETER)

    # Open the camera
    status = camera.open(init_params)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 10)
    if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
        print("Camera Open : "+repr(status)+". Exit program.")
        return None
    return camera

def main():
    camera = setup_camera()
    if (camera == None):
        exit()

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    zed = Zed(camera=camera)

    # setup gpio
    setup_gpio()
    curr_out = GPIO.LOW

    # config cv2 windows for visualization
    cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("camera", 800, 600)
    try:
        # in a loop, detect the distance of nearest
        # object pointed to from green to red
        while True:
            if camera.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                zed.update_image()
                zed.find_leds()
                zed.get_direction()
                distance = zed.raymarch()
                if(distance and distance > 50):
                    print("distance: ", distance)
                    
                    # buzz if we detect a nearby object
                    if (curr_out == GPIO.LOW and distance < 250):
                        curr_out = GPIO.HIGH
                    else:
                        curr_out = GPIO.LOW
                    GPIO.output(output_pin, curr_out)
                zed.draw()
                cv2.imshow("camera", zed.cv2_img)
                cv2.waitKey(10)
    finally:
        cv2.destroyAllWindows()
        camera.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()