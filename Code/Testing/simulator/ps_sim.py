from threading import Thread, Event
from math import cos, radians, sin
import numpy as np
import time

class Drone(object):
    def __init__(self):
        # dummy vals
        print ">>> AR Drone 2.0 Simulator"
        self.__home_gps = [25.758983, -80.374478, 0.0]
        self.__status = ["landed", "hovering", "moving"]
        self.__curr_status = self.__status[0]
        self.__speeds = [0.0, 0.0, 0.0, 0.0]
        self.__nav_update_spd = 1.0 / 200 # original navdata speed
        self.__speed_def = 1.0

        # accessed values from navigator
        print ">>> Initializing NavData"
        self.NavDataCount = 0
        self.NavData = {
                "altitude":[0.0],
                "demo":[0.0, 0.0, [0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0]],
                "gps":self.__home_gps,
                "magneto":[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "raw_measures":[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                }

        # accessed values from ui
        self.NoNavData = False

        # shutdown event - called when closing app
        self.__shutdown = Event()

        # still event - called to close position updater
        self.__still = Event()

        # navlock - isolates reads and writes to navdata
        self.__navlock = Event()
        self.__navlock.set()

        # position over time thread
        self.__still.set()
        self.__movement = Thread(
                target = self.__pos_update,
                args = ())

        # navdatacount update thread
        print ">>> Beginning NavDataCount Incrementer"
        self.__navdata = Thread(
                target = self.__nav_data_inc,
                args = ())

        self.__navdata.start()
        self.__movement.start()
        print ">>> Simulator intialization complete"

    def __nav_data_inc(self):
        print ">>> NavDataCount Incrementer Begun"
        while not self.__shutdown.is_set():
            time.sleep(self.__nav_update_spd)
            self.NavDataCount += 1

    def __pos_update(self):
        print ">>> Position Updater Begun"
        while not self.__shutdown.is_set():
            hdg = radians(self.NavData["magneto"][0])
            rot = np.matrix([
                    [ cos(hdg), sin(hdg), 0.0],
                    [-sin(hdg), cos(hdg), 0.0],
                    [      0.0,      0.0, 1.0]])
            dz  = self.__speeds[2] * self.__nav_update_spd
            dt = self.__speeds[3] * self.__nav_update_spd
            mov = np.matrix([
                [self.__speeds[1] * self.__nav_update_spd],
                [self.__speeds[0] * self.__nav_update_spd],
                [                                      dt]])
            dxy = rot * mov

            if not self.__still.is_set() and self.__navlock.wait(self.__nav_update_spd):
                self.__navlock.clear()
                
                # adjusting altitude
                self.NavData["altitude"][0] += dz * 1000

                # adjusting heading
                currMag = self.NavData["magneto"][0]
                self.NavData["magneto"][0] = ((dt * 50) + currMag + 360) % 360

                # adjusting latitude
                self.NavData["gps"][0] += float(dxy[1] / 111132)

                # adjusting longitude
                self.NavData["gps"][1] += float(dxy[0] / 111132)

                # wake up waiting threads
                self.__navlock.set()
                time.sleep(self.__nav_update_spd)

            time.sleep(self.__nav_update_spd)

    def __get_nav(self):
        self.__navlock.wait()
        self.__navlock.clear()
        nav = self.NavData
        self.__navlock.set()
        return nav

    def hover(self):
        if self.__curr_status == self.__status[0]: return False
        time.sleep(0.1)
        self.__still.set()
        self.__curr_status = self.__status[1]
        return True

    def takeoff(self):
        if self.__curr_status != self.__status[0]: return False
        self.__curr_status = self.__status[2]
        while self.__get_nav()["altitude"][0] < 0.5:
            self.moveUp()
            time.sleep(self.__nav_update_spd)
        self.hover()
        return True

    def land(self):
        if self.__curr_status == self.__status[0]: return False
        while self.__get_nav()["altitude"][0] > 0:
            self.moveDown()
            time.sleep(self.__nav_update_spd)
        self.hover()
        self.__curr_status = self.__status[0]
        return True

    def shutdown(self):
        if self.__curr_status != self.__status[0]: self.land()
        self.__shutdown.set()
        self.__navdata.join()
        self.__movement.join()
        return True

    def move(self, leftright, backwardforward, downup, turnleftright): 
        if self.__curr_status == self.__status[0]: return False
        self.__speeds = [-leftright, backwardforward, downup, turnleftright]
        self.__curr_status = self.__status[2]
        self.__still.clear()
        return True

    def moveForward(self, *args):
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, speed, 0.0, 0.0)

    def moveBackward(self, *args):
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, -speed, 0.0, 0.0)

    def moveLeft(self, *args):
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(speed, 0.0, 0.0, 0.0)

    def moveRight(self, *args):
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(-speed, 0.0, 0.0, 0.0)

    def moveUp(self, *args):
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, 0.0, speed, 0.0)

    def moveDown(self, *args):
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, 0.0, -speed, 0.0)

    def turnLeft(self, *args):
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, 0.0, 0.0, speed)

    def turnRight(self, *args):
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, 0.0, 0.0, -speed)

    # from original ps_drone
    def angleDiff(self, base, value):
        adiff = ((base+180)-(value+180)) %360
        if adiff>180: adiff-=360
        return adiff

    # dummy functions to reduce code removal
    def useDemoMode(self, *args): return True
    def getNDpackage(self, *args): return True
    def trim(self, *args): return True
    def mtrim(self, *args): return True
    def reconnectNavData(self, *args): return True
    def getBattery(self, *args): return ["100", "OK"]
    def reset(self, *args): return True
    def startup(self, *args): return True

