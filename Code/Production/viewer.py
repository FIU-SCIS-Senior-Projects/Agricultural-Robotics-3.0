import cv2, time
import numpy as np
from threading import Thread

class Camera:
    def __init__(self, drone, width, height):
        # Constant camera vals
        print ">>> AR Drone 2.0 Camera Viewer"
        self.__CAM_WIDTH = width
        self.__CAM_HEIGHT = height
        self.__PROTOCOL = "tcp"
        self.__VID_IP = "192.168.1.1"
        self.__VID_PORT = "5555"
        self.__FRONT_CAM = True
        

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
        self.__edges = False
        self.__corners = False
        self.__colors = False
        self.__shapes = False
        self.__processing = [
                self.__edges,
                self.__corners,
                self.__colors,
                self.__shapes,
                ]
        self.__color_def = [    # BGR vals
                np.uint8([[[255,   0,   0]]]),  # blue
                np.uint8([[[  0, 255,   0]]]),  # green
                np.uint8([[[  0,   0, 255]]]),  # red
                ]
        self.__color_ranges = []
        self.__CANNY_MIN = 100
        self.__CANNY_MAX = 200
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
                np.array([hue_low, 100, 100]),
                np.array([hue_high, 255, 255])))

    def __make_colors(self, img):
        #print img.shape # (360,640,3)
        mask_acc = np.zeros((img.shape[0], img.shape[1]), np.uint8)
        #print mask_acc.shape # (360,640)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for hsv_range in self.__color_ranges:
            mask_cur = cv2.inRange(img_hsv, hsv_range[0], hsv_range[1])
            #print mask_cur.shape # (360, 640)
            mask_acc = cv2.add(mask_acc, mask_cur)

        out_img = cv2.bitwise_and(img, img, mask = mask_acc)
        out_img = cv2.cvtColor(out_img, cv2.COLOR_BGR2RGB)
        return out_img

    def __make_edges(self, img):
        gray = self.__currentGrayFrame
        edges = cv2.Canny(gray, self.__CANNY_MIN, self.__CANNY_MAX)
        edges = cv2.GaussianBlur(edges, (5, 5), 0)
        out_img = cv2.max(img, edges)
        return out_img

    def __make_corners(self, img):
        return img

    def __make_shapes(self, img):
        # obtain thresholded b&w image
        blurred = cv2.GaussianBlur(self.__currentGrayFrame, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
        
        # identify centroids
        for c in cnts:
            M = cv2.moments(c)
            # avoid divide by zeros by skipping that loop
            try:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            except ZeroDivisionError: continue
        
            # draw contours
            cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
            cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(img, "center", (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return img

    def __updateFrame(self):
        while(True):
            ret, frame = self.__capture.read()
            while not ret: ret, frame = self.__capture.read()
            try: frame[:,:]  # some erroneous frame detection
            except: continue # in case a bad image gets through
            self.__currentFrame = frame

    def start(self):
        cam = Thread(target=self.__updateFrame, args=())
        cam.daemon = True
        cam.start()

    def release(self):
        return self.__capture.release()

    def getFrame(self):
        out_image = self.__currentFrame
        if any(self.__processing):
            self.__currentGrayFrame = cv2.cvtColor(out_image, cv2.COLOR_BGR2GRAY)
        if self.__edges: out_image = self.__make_edges(out_image)
        if self.__corners: out_image = self.__make_corners(out_image)
        if self.__colors: out_image = self.__make_colors(out_image)
        if self.__shapes: out_image = self.__make_shapes(out_image)
        return out_image

    def tog_edges(self):
        if self.__edges: self.__edges = False
        else: self.__edges = True

    def tog_corners(self):
        if self.__corners: self.__corners = False
        else: self.__corners = True

    def tog_colors(self):
        if self.__colors: self.__colors = False
        else: self.__colors = True

    def tog_shapes(self):
        if self.__shapes: self.__shapes = False
        else: self.__shapes = True

