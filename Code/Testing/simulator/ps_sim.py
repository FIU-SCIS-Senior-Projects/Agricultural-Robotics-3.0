from threading import Thread, Event
import time

class Drone(object):
    def __init__(self):
        # dummy vals
        print ">>> AR Drone 2.0 Simulator"
        self.__home_gps = [25.758983, -80.374478, 0.0]
        self.__statuses = ["landed", "hovering", "moving"]
        self.__curr_status = self.__statuses[0]
        self.__speeds = [0.0, 0.0, 0.0, 0.0]
        #self.__nav_update_spd = 1.0 / 200
        self.__nav_update_spd = 1.0

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

        # navdatacount update thread
        print ">>> Beginning NavDataCount Incrementer"
        self.__shutdown = Event()
        self.__navdata = Thread(
                target = self.__nav_data_inc,
                args = ())
        self.__navdata.start()

        # position over time thread
        self.__still = Event()
        self.__still.set()
        self.__movement = Thread(
                target = self.__pos_update,
                args = ())

        # status watchdog thread
        print ">>> Beginning Movement Watchdog"
        self.__watchdog = Thread(
                target = self.__watchdog,
                args = ())
        self.__watchdog.start()

    def __watchdog(self):
        print ">>> Watchdog Begun"
        while not self.__shutdown.is_set():
            if self.__curr_status == self.__statuses[2]:
                self.__still.clear()
                self.__movement.start()
                print ">>> drone moving"
            else:
                self.__still.set()
                self.__movement.join()
                print ">>> drone still"
            time.sleep(self.__nav_update_spd)

    def __nav_data_inc(self):
        print ">>> NavDataCount Incrementer Begun"
        while not self.__shutdown.is_set():
            time.sleep(self.__nav_update_spd)
            self.NavDataCount += 1

    def __pos_update(self):
        print ">>> Position Updater Begun"
        while not self.__still.is_set():
            hdg = self.NavData["mag"][0]
            dx = self.__speeds[1] * self.__nav_update_spd
            dy = self.__speeds[0] * self.__nav_update_spd
            dz = self.__speeds[2] * self.__nav_update_spd
            
            # adjusting altitude
            self.NavData["altitude"][0] += dz * 1000

            # adjusting latitude
            self.NavData["gps"][0] += math.sin(math.radians(hdg)) * dx / 111132
            self.NavData["gps"][0] += math.cos(math.radians(hdg)) * dy / 111132

            # adjusting longitude
            self.NavData["gps"][1] += math.sin(math.radians(hdg)) * dy / 111132
            self.NavData["gps"][1] += math.cos(math.radians(hdg)) * dx / 111132

            time.sleep(self.__nav_update_spd)

    def turnAngle(self, ndir, speed, *args):
        print ">>> turnAngle() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        target = self.NavData["magneto"][0] + ndir
        if ndir > 0: turn = ndir
        else: turn = -ndir
        self.__curr_status = self.__statuses[2]
        while self.NavData["magneto"][0] != target:
            self.NavData["magneto"][0] += turn * self.__nav_update_spd * speed
            time.sleep(self.__nav_update_spd)
        self.__curr_status = self.__statuses[1]
        return True

    def hover(self):
        print ">>> hover() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        time.sleep(0.1)
        self.__curr_status = self.__statuses[1]
        return True

    def takeoff(self):
        print ">>> takeoff() invoked"
        if self.__curr_status != self.__statuses[0]: return False
        self.__curr_status = self.__statuses[2]
        while self.NavData["altitude"][0] < 250:
            print self.NavData["altitude"][0]
            print "moveUp(): {}".format(self.moveUp())
            time.sleep(self.__nav_update_spd)
        print "hover(): {}".format(self.hover())
        return True

    def land(self):
        print ">>> land() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        time.sleep(1)
        self.__curr_status = self.__statuses[0]
        return True

    def shutdown(self):
        print ">>> shutdown() invoked"
        self.__curr_status = self.__statuses[0]
        self.__shutdown.set()
        self.__navdata.join()
        self.__watchdog.join()
        return True

    def move(self, leftright, backwardforward, downup, turnleftright): 
        print ">>> move() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        self.__speeds = [leftright, backwardforward, downup, turnleftright]
        self.__curr_status = self.__statuses[2]
        return True

    def moveForward(self, *args):
        print ">>> moveForward() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        try: speed = args[0]
        except: speed = 1.0
        return self.move(0.0, speed, 0.0, 0.0)

    def moveBackward(self, *args):
        print ">>> moveBackward() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        try: speed = args[0]
        except: speed = 1.0
        return self.move(0.0, -speed, 0.0, 0.0)

    def moveLeft(self, *args):
        print ">>> moveLeft() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        try: speed = args[0]
        except: speed = 1.0
        return self.move(-speed, 0.0, 0.0, 0.0)

    def moveRight(self, *args):
        print ">>> moveRight() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        try: speed = args[0]
        except: speed = 1.0
        return self.move(speed, 0.0, 0.0, 0.0)

    def moveUp(self, *args):
        print ">>> moveUp() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        try: speed = args[0]
        except: speed = 1.0
        return self.move(0.0, 0.0, speed, 0.0)

    def moveDown(self, *args):
        print ">>> moveDown() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        try: speed = args[0]
        except: speed = 1.0
        return self.move(0.0, 0.0, -speed, 0.0)

    def turnLeft(self, *args):
        print ">>> turnLeft() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        try: speed = args[0]
        except: speed = 1.0
        return self.move(0.0, 0.0, 0.0, -speed)

    def turnRight(self, *args):
        print ">>> turnRight() invoked"
        if self.__curr_status == self.__statuses[0]: return False
        try: speed = args[0]
        except: speed = 1.0
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

