########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import pyzed.sl as sl
import cv2
from zed import Zed
import Jetson.GPIO as GPIO

# BCM pin 6 is board pin 31
output_pin = 6

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

def setup_camera():
    # Create and configure a Camera object
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
    setup_gpio()
    camera = setup_camera()
    if (camera == None):
        exit()

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    zed = Zed(camera=camera)

    # config cv2 windows
    cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("camera", 800, 600)
    try:
        while True:
            # A new image is available if grab() returns SUCCESS
            if camera.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:

                # Retrieve left image and measure its depth
                zed.update_image()
                zed.find_leds()
                
                # get a direction from green -> red
                zed.get_direction()
                distance = zed.raymarch()
                if(distance):
                    print("distance: ", distance)
                
                # draw to visualize the calculations
                zed.draw()
                cv2.imshow("camera", zed.cv2_img)
                cv2.waitKey(10)
    finally:
        cv2.destroyAllWindows()
        camera.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()