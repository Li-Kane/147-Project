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
import math
import cv2
import numpy as np


def find_leds(img: np.ndarray):
    green_led = (-1, -1)
    red_led = (-1, -1)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # find green led
    lower_green = np.array([36, 150, 150])
    upper_green = np.array([86, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    contours, _ = cv2.findContours(green_mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        green_led = (int(x + w/2), int(y + h/2))
        cv2.circle(img, green_led, 5, (255, 0, 0), 3)
    
    # find red led

    return (green_led, red_led)


def main():
    # Create and configure a Camera object
    zed = sl.Camera()
    init_params = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                                coordinate_units=sl.UNIT.MILLIMETER)

    # Open the camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
        print("Camera Open : "+repr(status)+". Exit program.")
        exit()

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    image = sl.Mat()
    depth = sl.Mat()

    # config cv2 windows
    cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("camera", 800, 600)
    cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("mask", 800, 600)

    try:
        while True:
            # A new image is available if grab() returns SUCCESS
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left image and measure its depth
                zed.retrieve_image(image, sl.VIEW.LEFT)
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

                # Get and print distance value in mm at the center of the image
                # x = round(image.get_width() / 2)
                # y = round(image.get_height() / 2)
                # err, distance = depth.get_value(x,y)

                # detect leds
                cv2_img = image.get_data()
                green_led, red_led = find_leds(cv2_img)
                
                # get distances to the leds
                if (green_led != (-1, -1)):
                    err, green_dist = depth.get_value(green_led[0], green_led[1])
                    if (math.isfinite(green_dist)):
                        print(f"Distance to green led: {green_dist}")
                    else : 
                        print(f"The distance can not be computed for green led")

                cv2.imshow("camera", cv2_img)
                cv2.waitKey(10)
    finally:
        cv2.destroyAllWindows()
        zed.close()

if __name__ == "__main__":
    main()