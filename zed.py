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
import math
import cv2

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

    try:
        while True:
            # A new image is available if grab() returns SUCCESS
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left image and measure its depth
                zed.retrieve_image(image, sl.VIEW.LEFT)
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

                # Get and print distance value in mm at the center of the image
                x = round(image.get_width() / 2)
                y = round(image.get_height() / 2)
                err, distance = depth.get_value(x,y)

                # detect leds
                

                # draw circle
                cv2_img = image.get_data()
                cv2.circle(cv2_img, (x, y), 5, (255, 0, 0), 3)
                cv2.imshow("left camera", cv2_img)
                cv2.waitKey(10)

                if math.isfinite(distance):
                    print(f"Distance to Camera at {{{x};{y}}}: {distance}")
                else : 
                    print(f"The distance can not be computed at {{{x};{y}}}")
    finally:
        cv2.destroyAllWindows()
        zed.close()

if __name__ == "__main__":
    main()