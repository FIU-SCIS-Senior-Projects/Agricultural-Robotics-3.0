import cv2, time
from threading import Thread

class Camera:
    def __init__(self, drone):
        # Configure drone video
        self.drone = drone
        self.drone.frontCam()
        self.drone.midVideo()
        self.drone.sdVideo()
        self.drone.startVideo()

        # Configure camera settings
        self.currentFrame = None
        self.CAMERA_WIDTH = 640
        self.CAMERA_HEIGHT = 360
        self.CAMERA_NUM = 0
        self.capture = cv2.VideoCapture('tcp://192.168.1.1:5555')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,self.CAMERA_WIDTH)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,self.CAMERA_HEIGHT)

    def start(self):
        cam = Thread(target=self.updateFrame, args=())
        cam.daemon = True
        cam.start()

    def updateFrame(self):
        while(True):
            ret, self.currentFrame = self.capture.read()
            while not ret: ret, frame = self.capture.read()

    def getFrame(self):
        return self.currentFrame

    def release(self):
        return self.capture.release()
