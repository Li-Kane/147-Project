import cv2
import numpy as np
import pyzed.sl as sl

'''
Helper class to main.py that manages calculations for determining 
a finger pointing direction and where it's pointing in 3D space
'''
class Zed:
    # maximum march distance in mm
    MAX_DISTANCE = 10000
    # determines smoothing factor 0 < x < 1
    SMOOTHNESS = 0.5

    def __init__(self, camera: sl.Camera):
        self.camera = camera
        params = camera.get_camera_information().camera_configuration.calibration_parameters.left_cam
        self.CAM_INTRINSICS = np.array([[params.fx, 0,         params.cx],
                                        [0,         params.fy, params.cy],
                                        [0,         0,         1]])
        self.DISTORTION = params.disto
        self.image = sl.Mat()
        self.points = sl.Mat()
        self.green_led = (-1, -1)
        self.red_led = (-1, -1)
        self.green_pt = np.ndarray(shape=(4))
        self.red_pt = np.ndarray(shape=(4))
        self.direction = np.ndarray(shape=(4))
        self.end = (-1, -1)
        self.cv2_img = None

    # get left camera's image and a depth per pixel
    def update_image(self):
        self.camera.retrieve_image(self.image, sl.VIEW.LEFT)
        self.camera.retrieve_measure(self.points, sl.MEASURE.XYZ)
        self.cv2_img = self.image.get_data()

    # use exponential moving average (EMA) algorithm to
    # more smoothly track led_pos across video frames
    def smooth_position(self, old_pos, new_pos):
        if old_pos is None:
            return new_pos
        return (
            int(self.SMOOTHNESS * new_pos[0] + (1 - self.SMOOTHNESS) * old_pos[0]),
            int(self.SMOOTHNESS * new_pos[1] + (1 - self.SMOOTHNESS) * old_pos[1]),
        )

    # using opencv image processing, determine a (x, y) point
    # for the center of the green and red led
    def find_leds(self):
        hsv = cv2.cvtColor(self.cv2_img, cv2.COLOR_BGR2HSV)

        # find green led
        lower_green = np.array([36, 150, 150])
        upper_green = np.array([86, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        contours, _ = cv2.findContours(green_mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            new_green_led = (int(x + w/2), int(y + h/2))
            self.green_led = self.smooth_position(self.green_led, new_green_led)
        
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
            new_red_led = (int(x + w/2), int(y + h/2))
            self.red_led = self.smooth_position(self.red_led, new_red_led)

    # calculate a normalized direction vector in 3D
    # space from the green led to the red led
    def get_direction(self):
        if (self.green_led != (-1, -1) and self.red_led != (-1, -1)):
            _, self.green_pt = self.points.get_value(self.green_led[0], self.green_led[1])
            _, self.red_pt = self.points.get_value(self.red_led[0], self.red_led[1])
            if (not np.any(np.isnan(self.green_pt)) and not np.any(np.isnan(self.red_pt))):
                self.direction = self.red_pt - self.green_pt
                self.direction = self.direction / np.linalg.norm(self.direction)


    # from our red led, march in the calculated direction
    # until we collide with an object, then return the distance
    # to that object
    def raymarch(self):
        expected = (-1, -1, -1, -1)
        curr_point = np.copy(self.red_pt)
        dist_traveled = 0
        while(dist_traveled < self.MAX_DISTANCE):
            # march
            step_size = max(5, dist_traveled * 0.1) 
            dist_traveled += step_size
            curr_point += self.direction * step_size

            # determine where our marched point would be on the camera
            img_pt, _ = cv2.projectPoints(curr_point[:-1], np.ndarray(shape=(3)), np.ndarray(shape=(3)), self.CAM_INTRINSICS, self.DISTORTION)
            x, y, = img_pt[0][0]
            if (
                np.any(np.isnan(x)) or np.any(np.isnan(y)) or
                x < 0 or x > self.points.get_width() or
                y < 0 or y > self.points.get_height()
            ):
                self.end = (-1, -1)
                return
            
            # if the camera detects an object where our marched point is, there is a collision
            _, expected = self.points.get_value(int(img_pt[0][0][0]), int(img_pt[0][0][1]))
            difference = np.linalg.norm(curr_point - expected)
            cv2.circle(self.cv2_img, (int(img_pt[0][0][0]), int(img_pt[0][0][1])), 5, (0, 255, 0), 3)
            if difference < max(40, 0.07 * dist_traveled): 
                self.end = (int(img_pt[0][0][0]), int(img_pt[0][0][1]))
                distance = np.linalg.norm(expected - self.red_pt)
                return distance

    # visualize leds and our pointing direction
    def draw(self):
        cv2.circle(self.cv2_img, self.green_led, 5, (255, 0, 0), 3)
        cv2.circle(self.cv2_img, self.red_led, 5, (255, 0, 0), 3)
        if (self.end != (-1, -1)):
            cv2.line(self.cv2_img, self.red_led, self.end, [255, 255, 255], 3)