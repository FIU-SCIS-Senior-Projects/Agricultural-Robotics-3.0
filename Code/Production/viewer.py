import cv2, time
from threading import Thread

class Camera:
    def __init__(self, drone):
        # Configure drone video
        self.__drone = drone
        self.__drone.frontCam()
        self.__drone.midVideo()
        self.__drone.sdVideo()

        # Configure camera settings
        self.__currentFrame = None
        self.__CAMERA_WIDTH = 640
        self.__CAMERA_HEIGHT = 360
        self.__capture = cv2.VideoCapture('tcp://192.168.1.1:5555')
        self.__capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,self.__CAMERA_WIDTH)
        self.__capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,self.__CAMERA_HEIGHT)

        # Configure computer vision
        self.__mono = False
        self.__edges = False
        self.__corners = False
        self.__colors = False

    def start(self):
        cam = Thread(target=self.__updateFrame, args=())
        cam.daemon = True
        cam.start()

    def __updateFrame(self):
        while(True):
            ret, self.__currentFrame = self.__capture.read()
            while not ret: ret, frame = self.__capture.read()

    def __make_mono(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def getFrame(self):
        out_image = self.__currentFrame
        if self.__mono: out_image = self.__make_mono(out_image)
        if self.__edges: out_image = self.__make_edges(out_image)
        if self.__corners: out_image = self.__make_corners(out_image)
        if self.__colors: out_image = self.__make_colors(out_image)
        return out_image

    def release(self):
        return self.__capture.release()

    def tog_mono(self):
        if self.__mono: self.__mono = False
        else: self.__mono = True

    def tog_edges(self):
        if self.__edges: self.__edges = False
        else: self.__edges = True

    def tog_corners(self):
        if self.__corners: self.__corners = False
        else: self.__corners = True

    def tog_colors(self):
        if self.__colors: self.__colors = False
        else: self.__colors = True

