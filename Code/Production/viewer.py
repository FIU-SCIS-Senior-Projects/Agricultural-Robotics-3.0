import cv2, time
import numpy as np
from threading import Thread

class Camera:
    def __init__(self, drone, width, height, event):
        # Constant camera vals
        print ">>> AR Drone 2.0 Camera Viewer"
        self.__CAM_WIDTH = width
        self.__CAM_HEIGHT = height
        self.__CAM_EVENT = event
        self.__PROTOCOL = "tcp"
        self.__VID_IP = "192.168.1.1"
        self.__VID_PORT = "5555"
        self.__FRONT_CAM = True
        self.cam_thread = None

        # Configure drone video
        print ">>> Initializing video capture..."
        self.__drone = drone
        self.__drone.frontCam()
        self.__drone.midVideo()
        self.__drone.sdVideo()

        # Configure camera settings
        print ">>> Initializing capture settings..."
        self.__capture = cv2.VideoCapture(
                "{}://{}:{}".format(self.__PROTOCOL, self.__VID_IP, self.__VID_PORT))
        self.__capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.__CAM_WIDTH)
        self.__capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.__CAM_HEIGHT)

        # Configure computer vision
        print ">>> Configuring computer vision options..."
        self.__colors = True
        self.__color_def = [    # BGR vals
                np.uint8([[[255,   0,   0]]]),  # blue
                #np.uint8([[[  0, 255,   0]]]),  # green
                #np.uint8([[[  0,   0, 255]]]),  # red
                ]
        self.__color_ranges = []
        self.__get_hsv()

        # Stored Frames
        self.__currentFrame = None
        self.__currentGrayFrame = None

        # Done initializing
        print ">>> CAMERA READY"

    def __get_hsv(self):
        for color in self.__color_def:
            hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
            hue_low, hue_high = hsv[0][0][0] - 10, hsv[0][0][0] + 10
            self.__color_ranges.append((
                np.array([hue_low, 75, 75]),
                np.array([hue_high, 255, 255])))

    def __make_colors(self, img):
        mask_acc = np.zeros((img.shape[0], img.shape[1]), np.uint8)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for hsv_range in self.__color_ranges:
            mask_cur = cv2.inRange(img_hsv, hsv_range[0], hsv_range[1])
            mask_acc = cv2.add(mask_acc, mask_cur)
        blurred = cv2.GaussianBlur(mask_acc, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
        for c in cnts:
            hull = cv2.convexHull(c)
            tar_range = len(hull)
            for i in range(tar_range):
                px, py = list(hull[i][0]), list(hull[(i + 1) % tar_range][0])
                cv2.line(img, (px[0], px[1]), (py[0], py[1]), (255, 255, 255), 2)
        return img

    def __make_bounding_box(self, img, cnts):
        for cnt in cnts:
            rect_rot = cv2.minAreaRect(cnt)
            rect_pts = cv2.boxPoints(rect_rot)
            for i in range(4):
                p1 = [int(x) for x in rect_pts[i]]
                p2 = [int(x) for x in rect_pts[(i + 1) % 4]]
                cv2.line(img,
                        (p1[0], p1[1]),
                        (p2[0], p2[1]),
                        (255, 255, 255), 2)

    def __updateFrame(self):
        while(not self.__CAM_EVENT.is_set()):
            ret, frame = self.__capture.read()
            while not ret: ret, frame = self.__capture.read()
            try: frame[:,:]  # some erroneous frame detection
            except: continue # in case a bad image gets through
            self.__currentFrame = frame

    def getFrame(self):
        out_image = self.__currentFrame
        if self.__colors: out_image = self.__make_colors(out_image)
        return cv2.cvtColor(out_image, cv2.COLOR_BGR2RGB)

    def start(self):
        self.cam_thread = Thread(target=self.__updateFrame, args=())
        self.cam_thread.start()

    def release(self):
        return self.__capture.release()

    def tog_colors(self): self.__colors = not self.__colors

