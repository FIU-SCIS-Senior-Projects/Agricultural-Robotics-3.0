import cv2
import numpy as np
from threading import Thread

class Camera:
    def __init__(self, drone, width, height, event):
        # Constant camera vals
        print ">>> AR Drone 2.0 Camera Viewer"
        self.__drone = drone
        self.cam_thread = None # public to join in gui object
        self.__CAM_EVENT = event

        # These can be reconfigured in Drone object if needed
        self.__PROTOCOL = "tcp"
        self.__VID_IP = "192.168.1.1"
        self.__VID_PORT = "5555"

        # These can be toggled/changed mid-flight
        print ">>> Initializing capture settings..."
        self.__drone.frontCam()
        self.__drone.midVideo()
        self.__drone.sdVideo()

        # Configure camera settings
        print ">>> Beginning video capture..."
        self.__capture = cv2.VideoCapture(
                "{}://{}:{}".format(self.__PROTOCOL, self.__VID_IP, self.__VID_PORT))

        # OpenCV3 constants - errors here mean you're using OpenCV2
        self.__capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.__capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Configure computer vision
        print ">>> Configuring computer vision options..."
        self.__colors = False
        self.__color_def = [    # BGR vals
                np.uint8([[[255,   0,   0]]]),  # blue
                ]
        self.__color_ranges = []
        self.__get_hsv()

        # Stored Frame
        self.__currentFrame = None

        # Done initializing
        print ">>> CAMERA READY"

    def __get_hsv(self):
        """Converts BGR values in constructor to min/max of HSV"""
        for color in self.__color_def:
            # Colors defined as OpenCV-standard BGR values in constructor
            hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

            # Ranges to accomodate changes in lighting around a 3d object
            hue_low, hue_high = hsv[0][0][0] - 10, hsv[0][0][0] + 10

            # Low for dark/shady lighting, high for light/bright
            self.__color_ranges.append((
                np.array([hue_low, 75, 75]),
                np.array([hue_high, 255, 255])))

    def __make_colors(self, img):
        """Derives threshold image (black&white) using HSV values.
           Uses threshold image to isolate shapes of given HSVs.
           Draws a contour around each shape on original image."""
        # Declare mask var of correct shape to which new masks will be 'added'
        mask_acc = np.zeros((img.shape[0], img.shape[1]), np.uint8)

        # Convert incoming image to HSV for comparison to calculated HSV ranges
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for hsv_range in self.__color_ranges:
            # Identify colors and add their pixel positions to mask accumulator
            mask_cur = cv2.inRange(img_hsv, hsv_range[0], hsv_range[1])
            mask_acc = cv2.add(mask_acc, mask_cur)

        # Identify contours of shapes in mask accumulator
        blurred = cv2.GaussianBlur(mask_acc, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

        # Draw a border around every contour onto original image
        for c in cnts:
            hull = cv2.convexHull(c)
            tar_range = len(hull)
            for i in range(tar_range):
                px, py = list(hull[i][0]), list(hull[(i + 1) % tar_range][0])
                cv2.line(img, (px[0], px[1]), (py[0], py[1]), (255, 255, 255), 2)

        # Return modified image
        return img

    def __updateFrame(self):
        """Threaded function to continuously query drone for a new
           frame of its camera sensor, and do a basic check of
           presence to avoid errors propagating in GUI"""
        while(not self.__CAM_EVENT.is_set()):
            ret, frame = self.__capture.read()
            while not ret: ret, frame = self.__capture.read()
            try: frame[:,:]  # some erroneous frame detection in
            except: continue #  case a bad image gets through
            self.__currentFrame = frame

    def getFrame(self):
        """Public function to retrieve the last good frame from the
           threaded updateFrame function and perform any image
           modification before returning it."""
        out_image = self.__currentFrame

        # Checking if color flag is toggled 'on'
        if self.__colors: out_image = self.__make_colors(out_image)

        # Return image in requested form
        return cv2.cvtColor(out_image, cv2.COLOR_BGR2RGB)

    def start(self):
        """Begin the threaded frame update function"""
        self.cam_thread = Thread(target=self.__updateFrame, args=())
        self.cam_thread.start()

    def release(self):
        """Stops the video capture. Must be used before the drone
           is shut down."""
        return self.__capture.release()

    def tog_colors(self):
        """Toggle highlighting contours of colors given in constructor."""
        self.__colors = not self.__colors

