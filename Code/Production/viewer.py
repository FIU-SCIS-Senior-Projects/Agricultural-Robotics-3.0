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
        self.__capture.set(cv2.CAP_PROP_FRAME_WIDTH,self.__CAMERA_WIDTH)
        self.__capture.set(cv2.CAP_PROP_FRAME_HEIGHT,self.__CAMERA_HEIGHT)

    def start(self):
        cam = Thread(target=self.__updateFrame, args=())
        cam.daemon = True
        cam.start()

    def __updateFrame(self):
        while(True):
            ret, self.__currentFrame = self.__capture.read()
            while not ret: ret, frame = self.__capture.read()

    def getFrame(self):
        return self.__currentFrame

    def release(self):
        return self.__capture.release()
