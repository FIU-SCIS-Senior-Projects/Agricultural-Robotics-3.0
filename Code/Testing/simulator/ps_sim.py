from threading import Thread, Event
import math, time

class Drone(object):
    def __init__(self):
        # dummy vals
        print ">>> AR Drone 2.0 Simulator"
        self.__home_gps = [25.758983, -80.374478, 0.0]
        self.__status = ["landed", "hovering", "moving"]
        self.__curr_status = self.__status[0]
        self.__speeds = [0.0, 0.0, 0.0, 0.0]
        #self.__nav_update_spd = 1.0 / 200 # original navdata speed
        self.__nav_update_spd = 1.0 # test navdata speed
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

        print ">>> Simulator intialization complete"

    def startup(self):
        self.__navdata.start()
        self.__movement.start()
        return True

    def __nav_data_inc(self):
        print ">>> NavDataCount Incrementer Begun"
        while not self.__shutdown.is_set():
            time.sleep(self.__nav_update_spd)
            self.NavDataCount += 1

    def __pos_update(self):
        print ">>> Position Updater Begun"
        while not self.__shutdown.is_set():
            if not self.__still.is_set():
                hdg = self.NavData["magneto"][0]
                dx = self.__speeds[1] * self.__nav_update_spd
                dy = self.__speeds[0] * self.__nav_update_spd
                dz = self.__speeds[2] * self.__nav_update_spd
                
                # adjusting altitude
                self.NavData["altitude"][0] += dz

                # adjusting latitude
                self.NavData["gps"][0] += math.sin(math.radians(hdg)) * dx / 111132
                self.NavData["gps"][0] += math.cos(math.radians(hdg)) * dy / 111132

                # adjusting longitude
                self.NavData["gps"][1] += math.sin(math.radians(hdg)) * dy / 111132
                self.NavData["gps"][1] += math.cos(math.radians(hdg)) * dx / 111132

            time.sleep(self.__nav_update_spd)

    def turnAngle(self, ndir, speed, *args):
        print ">>> turnAngle() invoked"
        if self.__curr_status == self.__status[0]: return False
        target = self.NavData["magneto"][0] + ndir
        if ndir > 0: turn = ndir
        else: turn = -ndir
        self.__curr_status = self.__status[2]
        while self.NavData["magneto"][0] != target:
            self.NavData["magneto"][0] += turn * self.__nav_update_spd * speed
            time.sleep(self.__nav_update_spd)
        self.__curr_status = self.__status[1]
        return True

    def hover(self):
        print ">>> hover() invoked"
        if self.__curr_status == self.__status[0]: return False
        time.sleep(0.1)
        self.__still.set()
        self.__curr_status = self.__status[1]
        return True

    def takeoff(self):
        print ">>> takeoff() invoked"
        if self.__curr_status != self.__status[0]: return False
        self.__curr_status = self.__status[2]
        while self.NavData["altitude"][0] < 0.25:
            print self.NavData["altitude"][0]
            print "moveUp(): {}".format(self.moveUp())
            time.sleep(self.__nav_update_spd)
        print "hover(): {}".format(self.hover())
        return True

    def land(self):
        print ">>> land() invoked"
        if self.__curr_status == self.__status[0]: return False
        while self.NavData["altitude"][0] > 0:
            print self.NavData["altitude"][0]
            print "moveDown(): {}".format(self.moveDown())
            time.sleep(self.__nav_update_spd)
        self.__curr_status = self.__status[0]
        return True

    def shutdown(self):
        print ">>> shutdown() invoked"
        if self.__curr_status != self.__status[0]: self.land()
        self.__shutdown.set()
        self.__navdata.join()
        self.__movement.join()
        return True

    def move(self, leftright, backwardforward, downup, turnleftright): 
        print ">>> move() invoked"
        if self.__curr_status == self.__status[0]: return False
        self.__speeds = [leftright, backwardforward, downup, turnleftright]
        self.__curr_status = self.__status[2]
        self.__still.clear()
        return True

    def moveForward(self, *args):
        print ">>> moveForward() invoked"
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, speed, 0.0, 0.0)

    def moveBackward(self, *args):
        print ">>> moveBackward() invoked"
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, -speed, 0.0, 0.0)

    def moveLeft(self, *args):
        print ">>> moveLeft() invoked"
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(-speed, 0.0, 0.0, 0.0)

    def moveRight(self, *args):
        print ">>> moveRight() invoked"
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(speed, 0.0, 0.0, 0.0)

    def moveUp(self, *args):
        print ">>> moveUp() invoked"
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, 0.0, speed, 0.0)

    def moveDown(self, *args):
        print ">>> moveDown() invoked"
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, 0.0, -speed, 0.0)

    def turnLeft(self, *args):
        print ">>> turnLeft() invoked"
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, 0.0, 0.0, -speed)

    def turnRight(self, *args):
        print ">>> turnRight() invoked"
        try: speed = args[0]
        except: speed = self.__speed_def
        return self.move(0.0, 0.0, 0.0, speed)

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

