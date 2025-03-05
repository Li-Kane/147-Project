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
import numpy as np

def get_cam_intrinsics(zed: sl.Camera):
    params = zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
    intrinsic = np.array([[params.fx, 0, params.cx],
                    [0, params.fy, params.cy],
                    [0, 0, 1]])
    distortion = params.disto
    return intrinsic, distortion

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
    lower_red1 = np.array([0, 150, 200])
    upper_red1 = np.array([20, 255, 255])
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    lower_red2 = np.array([160, 150, 200])
    upper_red2 = np.array([180, 255, 255])
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        red_led = (int(x + w/2), int(y + h/2))
        cv2.circle(img, red_led, 5, (255, 0, 0), 3)

    return (green_led, red_led)


def main():
    # Create and configure a Camera object
    zed = sl.Camera()
    init_params = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                                coordinate_units=sl.UNIT.MILLIMETER)

    # Open the camera
    status = zed.open(init_params)
    zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 20)
    if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
        print("Camera Open : "+repr(status)+". Exit program.")
        exit()

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    image = sl.Mat()
    points = sl.Mat()

    # Get camera intrinsics
    CAMERA_INTRINSICS, DISTORTION = get_cam_intrinsics(zed)

    # config cv2 windows
    cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("camera", 800, 600)
    try:
        while True:
            # A new image is available if grab() returns SUCCESS
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Retrieve left image and measure its depth
                zed.retrieve_image(image, sl.VIEW.LEFT)
                zed.retrieve_measure(points, sl.MEASURE.XYZ)

                # detect leds
                cv2_img = image.get_data()
                green_led, red_led = find_leds(cv2_img)
                
                # get points of the leds
                if (green_led != (-1, -1) and red_led != (-1, -1)):
                    _, green_point = points.get_value(green_led[0], green_led[1])
                    _, red_point = points.get_value(red_led[0], red_led[1])
                    # interpolate a point 305 mm from green -> red -> point
                    if (not np.any(np.isnan(green_point)) and not np.any(np.isnan(red_point))):
                        direction = red_point - green_point
                        direction = direction / np.linalg.norm(direction)
                        new_point = direction * 305 + red_point
                        new_point = new_point[:-1]
                        # project point cloud to image x, y
                        rotation = sl.Rotation().get_rotation_vector()
                        translation = sl.Translation().get()
                        img_pts, _ = cv2.projectPoints(new_point, rotation, translation, CAMERA_INTRINSICS, DISTORTION)
                        if(not np.any(np.isnan(img_pts[0][0]))):
                            origin = (int(img_pts[0][0][0]), int(img_pts[0][0][1]))
                            cv2.circle(cv2_img, origin, 5, [255, 255, 255], -1)
                        

                cv2.imshow("camera", cv2_img)
                cv2.waitKey(10)
    finally:
        cv2.destroyAllWindows()
        zed.close()

if __name__ == "__main__":
    main()