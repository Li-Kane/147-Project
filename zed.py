import cv2
import numpy as np
import pyzed.sl as sl

class Zed:
    image = sl.Mat()
    points = sl.Mat()
    green_led = (-1, -1)
    red_led = (-1, -1)
    green_pt = np.ndarray(shape=(4))
    red_pt = np.ndarray(shape=(4))
    direction = np.ndarray(shape=(4))
    end = (-1, -1)
    cv2_img = None
    # maximum march distance in mm
    MAX_DISTANCE = 2000
    STEP_SIZE = 100

    def __init__(self, camera: sl.Camera):
        self.camera = camera
        params = camera.get_camera_information().camera_configuration.calibration_parameters.left_cam
        self.CAM_INTRINSICS = np.array([[params.fx, 0,         params.cx],
                                        [0,         params.fy, params.cy],
                                        [0,         0,         1]])
        self.DISTORTION = params.disto

    def update_image(self):
        self.camera.retrieve_image(self.image, sl.VIEW.LEFT)
        self.camera.retrieve_measure(self.points, sl.MEASURE.XYZ)
        self.cv2_img = self.image.get_data()

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
            self.green_led = (int(x + w/2), int(y + h/2))
        
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
            self.red_led = (int(x + w/2), int(y + h/2))

    def get_direction(self):
        if (self.green_led != (-1, -1) and self.red_led != (-1, -1)):
            _, self.green_pt = self.points.get_value(self.green_led[0], self.green_led[1])
            _, self.red_pt = self.points.get_value(self.red_led[0], self.red_led[1])
            if (not np.any(np.isnan(self.green_pt)) and not np.any(np.isnan(self.red_pt))):
                self.direction = self.red_pt - self.green_pt
                self.direction = self.direction / np.linalg.norm(self.direction)

    def raymarch(self):
        expected = (-1, -1, -1, -1)
        curr_point = self.red_pt
        for i in range(0, self.MAX_DISTANCE, self.STEP_SIZE):
            curr_point += self.direction * self.STEP_SIZE
            img_pt, _ = cv2.projectPoints(curr_point[:-1], np.ndarray(shape=(3)), np.ndarray(shape=(3)), self.CAM_INTRINSICS, self.DISTORTION)
            if(np.any(np.isnan(img_pt[0][0])) or img_pt[0][0][0] < 0 or img_pt[0][0][0] > self.points.get_width() or img_pt[0][0][1] < 0 or img_pt[0][0][1] > self.points.get_height()):
                self.end = (-1, -1)
                return
            _, expected = self.points.get_value(int(img_pt[0][0][0]), int(img_pt[0][0][1]))
            difference = np.linalg.norm(curr_point - expected)
            if(np.any(np.isnan(difference)) or difference < 200):
                continue
            # if our ray march has a background (that causes distance) to be further than 200
            # we will draw a line earlier than we are supposed to
            self.end = (int(img_pt[0][0][0]), int(img_pt[0][0][1]))
            distance = np.linalg.norm(expected - self.red_pt)
            print("distance: ", distance)


    def draw(self):
        cv2.circle(self.cv2_img, self.green_led, 5, (255, 0, 0), 3)
        cv2.circle(self.cv2_img, self.red_led, 5, (255, 0, 0), 3)
        if (self.end != (-1, -1)):
            cv2.line(self.cv2_img, self.red_led, self.end, [255, 255, 255], 3)