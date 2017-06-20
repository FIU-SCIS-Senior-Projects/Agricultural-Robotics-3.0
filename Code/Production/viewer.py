import cv2, time
from threading import Thread

class Camera:
    def __init__(self, drone):
        # Constant camera vals
        self.__CAMERA_WIDTH = 640
        self.__CAMERA_HEIGHT = 360
        self.__PROTOCOL = "tcp"
        self.__VID_IP = "192.168.1.1"
        self.__VID_PORT = "5555"
        self.__FRONT_CAM = True
        

        # Configure drone video
        self.__drone = drone
        self.__drone.frontCam()
        self.__drone.midVideo()
        self.__drone.sdVideo()

        # Configure camera settings
        self.__capture = cv2.VideoCapture(
                "{}://{}:{}".format(self.__PROTOCOL, self.__VID_IP, self.__VID_PORT))
        self.__capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.__CAMERA_WIDTH)
        self.__capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.__CAMERA_HEIGHT)

        # Configure computer vision
        self.__edges = True
        self.__corners = False
        self.__colors = False
        self.__shapes = False
        self.__processing = [
                self.__edges,
                self.__corners,
                self.__colors,
                self.__shapes,
                ]
        self.__CANNY_MIN = 100
        self.__CANNY_MAX = 200

        # Stored Frames
        self.__currentFrame = None
        self.__currentGrayFrame = None

    def __make_edges(self, img):
        gray = self.__currentGrayFrame
        edges = cv2.Canny(gray, self.__CANNY_MIN, self.__CANNY_MAX)

        out_img = img + edges
        return out_img

    def __make_corners(self, img):
        return img

    def __make_colors(self, img):
        return img

    def __make_shapes(self, img):
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

