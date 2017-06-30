import itertools, ps_sim, time, math
from threading import Thread
from collections import deque
import numpy as np
np.seterr(divide='ignore', invalid='ignore')

class Navigator:
    """Navigator interface of an AR Drone 2.0"""

    def __init__(self, drone):
        """Initialize drone navigation variables"""
        # Constants
        print ">>> AR Drone 2.0 Navigator"
        self.__REQ_PACKS = ["altitude", "demo", "gps", "magneto", "raw_measures"]
        self.__SOFT_TURN = 0.1
        self.__HARD_TURN = 0.3
        self.__DEF_SPD   = 0.3
        self.__SAMP_NUM  = 150
        self.__SAMP_TIME = 0.005

        # Default (invalid) field values
        self.__mag_avg = [-14, 13] # Manually calculated
        self.__mag_acc = 6  # Points to record during calibration
        self.__samples = deque(maxlen = self.__SAMP_NUM) # Sample queue
        self.__targets = [] # Target list
        self.__waypoints = deque() # Waypoint Queue
        self.__tar_gps = None # Next target's gps coordinate
        self.__tar_dist = 0.0
        self.__tar_angle = 0.0
        self.__stats = {}   # Stats dict

        # Initialize sensor data transmissions
        print ">>> Initializing NavData"
        self.__drone = drone
        self.__drone.useDemoMode(False)
        self.__drone.getNDpackage(self.__REQ_PACKS)
        time.sleep(0.1)

        # Start taking sensor data
        print ">>> Populating data queue..."
        self.__sensors = Thread(target=self.__sensors_collect, args=())
        self.__sensors.daemon = True
        self.__sensors.start()
        time.sleep(self.__SAMP_TIME * self.__SAMP_NUM * 1.5)

        # Get current GPS for "home" location
        print ">>> Obtaining Home coordinate"
        self.__set_stats()
        self.__home = self.__stats["gps"]

        # Done initializing
        print ">>> NAVIGATOR READY"

    def __sensors_collect(self):
        """Continuously collects sensor data"""
        while True:
            self.__samples.append(self.__get_stats())
            time.sleep(self.__SAMP_TIME)

    def __set_stats(self):
        """Preprocessing of stats queue to reduce variation"""
        # 1-to-1 lists used in for loops
        vel = []
        acc, gyr, gps = [], [], []
        alt, mag, deg = [], [], []
        pry, mfu, out = [], [], []
        stat_names = ["vel", "acc", "gyr", "gps", "alt", "mag", "deg", "pry", "mfu"]
        stat_lists = [ vel,   acc,   gyr,   gps,   alt,   mag,   deg ,  pry ,  mfu ]

        # Build lists to be analyzed
        for item in list(self.__samples):
            for i in range(len(stat_names)):
                stat_lists[i].append(item[stat_names[i]])

        # Remove outliers
        for stat in stat_lists:
            out.append(list(itertools.compress(
                stat, self.__is_outlier(np.array(stat)))))

        # Check that lists are populated
        for i in range(len(stat_lists)):
            if out[i]: stat_lists[i] = out[i]

        # Average the remainder of the lists
        for i in range(len(stat_lists)):
            try:
                self.__stats[stat_names[i]] = reduce(
                        lambda x, y: x + y, np.array(stat_lists[i])
                        ) / len(stat_lists[i])
            except TypeError:
                self.__stats[stat_names[i]] = float('nan')

    def __is_outlier(self, points, thresh=3.5):
        """
            Boris Iglewicz and David Hoaglin (1993), "Volume 16: How to Detect and
            Handle Outliers", The ASQC Basic References in Quality Control:
            Statistical Techniques, Edward F. Mykytka, Ph.D., Editor. 
        """
        if len(points.shape) == 1:
            points = points[:,None]
        median = np.median(points, axis=0)
        diff = np.sum((points - median)**2, axis=-1)
        diff = np.sqrt(diff)
        med_abs_deviation = np.median(diff)
        modified_z_score = 0.6745 * diff / med_abs_deviation
    
        return modified_z_score > thresh

    def __get_stats(self):
        """Get stats list with human-readable sensor data."""
        stats = {}
        # Get fresh NavData
        NDC = self.__drone.NavDataCount
        while self.__drone.NavDataCount == NDC or not all(
                package in self.__drone.NavData for package in self.__REQ_PACKS):
            time.sleep(0.01)

        # Straightforward data
        stats["acc"] = self.__drone.NavData["raw_measures"][0]
        stats["gyr"] = self.__drone.NavData["raw_measures"][1]
        stats["gps"] = self.__drone.NavData["gps"][:-1] # not using altitude value
        stats["pry"] = self.__drone.NavData["demo"][2] # pitch roll yaw
        stats["mfu"] = self.__drone.NavData["magneto"][6]
        stats["vel"] = self.__drone.NavData["demo"][4] # xyz velocity mm/s

        # Convert altitude to meters
        stats["alt"] = self.__drone.NavData["altitude"][0] / 1000.0
    
        # Turn magnetometer data into heading (degrees)
        #stats["mag"] = self.__drone.NavData["magneto"][0][:-1] # not using z value
        #for i in range(len(stats["mag"])): stats["mag"][i] -= self.__mag_avg[i]
        #stats["deg"] = (360 + (-1 * (math.atan2(
        #    stats["mag"][1], stats["mag"][0]) * 180) / math.pi)) % 360

        #SIMULATION NOTE: MAGNETOMETER WILL ALWAYS CONTAIN HEADING IN DEGREES
        stats["deg"] = self.__drone.NavData["magneto"][0]
        stats["mag"] = self.__drone.NavData["magneto"][0]

        # Set new stats
        return stats

    def __calc_waypoints(self):
        """Takes target list, adds shortest route in order to waypoint queue"""
        # Current position is the first point of the path
        if self.__tar_gps == None:
            self.__set_stats()
            start = self.__stats["gps"]
        else: start = self.__tar_gps
        temp_start = start

        # Use NN to find shortest paths, queue targets as they're found
        while self.__targets:
            shortest = 1.0e999 # infinity
            for tar in self.__targets:
                dist = self.__calc_distance(temp_start, tar)
                if dist < shortest: shortest, start = dist, tar
            self.__waypoints.append(start)
            self.__targets.remove(start)
            temp_start = start

    def __next_tar(self):
        """Pop the next coordinate from the queue to current target"""
        try: self.__tar_gps = self.__waypoints.popleft()
        except IndexError: self.__tar_gps = self.__home
    
    def __calc_distance(self, start, finish):
        """Calculate distance to target"""
        r = 6371e3  # earth's radius in m
        x, y = start, finish

        # Convert GPS degrees to radians
        phi1 = math.radians(x[0])
        phi2 = math.radians(y[0])
        dphi = math.radians(y[0] - x[0])
        dlam = math.radians(y[1] - x[1])
    
        # 'Great circle' distance between two GPS coords
        a = math.sin(dphi / 2) * math.sin(dphi / 2)
        a += math.cos(phi1) * math.cos(phi2)
        a *= (math.sin(dlam / 2) * math.sin(dlam / 2))
        return 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    def __calc_heading(self, start, finish):
        """Calculate necessary heading for straight flight to target"""
        x, y = start, finish

        # Initial heading required for 'Great circle' traversal
        q = math.sin(y[1] - x[1]) * math.cos(y[0])
        p = math.cos(x[0]) * math.sin(y[0])
        p -= math.sin(x[0]) * math.cos(y[0]) * math.cos(y[1] - x[1])
        b = math.atan2(q, p) * 180.0 / math.pi
        return (b + 360.0) % 360.0
    
    def __calc_mag(self):
        """Rotates the drone to acquire mag data to use in normalization."""
        mag_x, mag_y = [], []
        for i in range(self.__mag_acc):
            NDC = self.__drone.NavDataCount
            while self.__drone.NavDataCount == NDC: time.sleep(0.01)
            mag = self.__drone.NavData["magneto"]
            mag_x.append(mag[0])
            mag_y.append(mag[1])
            self.__drone.turnAngle(-(360.0 / self.__mag_acc), 1.0)
            self.__drone.hover()
            time.sleep(2)
        self.__mag_avg[0] = np.mean(np.array(mag_x))
        self.__mag_avg[1] = np.mean(np.array(mag_y))
    
    def calibrate_drone(self, *mag):
        """Basic gyroscope and magnetometer recalibration."""
        # Requires 10-15 seconds of hovering flight.
        self.__drone.trim()
        time.sleep(5)
        self.__drone.takeoff()
        time.sleep(5)
        self.__drone.mtrim()
        time.sleep(5)
        if mag:
            self.__calc_mag()
            print self.__mag_avg
        self.__drone.land()

    def get_move(self):
        """Perform calculations to get arguments for a drone move"""
        if self.__tar_gps == None: return ([0.0, 0.0, 0.0, 0.0], 0.0)
        self.__set_stats()

        # Get angle of required turn
        self.__tar_angle = self.__calc_heading(self.__stats["gps"], self.__tar_gps)
        self.__tar_dist = self.__calc_distance(self.__stats["gps"], self.__tar_gps)
        angle_diff = self.__drone.angleDiff(
                self.__stats["deg"], self.__tar_angle)

        # If drastic turn is needed, only perform that turn
        if   angle_diff >  10.0:
            move_speed, turn_speed = 0.0,           -self.__HARD_TURN
        elif angle_diff < -10.0:
            move_speed, turn_speed = 0.0,            self.__HARD_TURN
        elif angle_diff > 0:
            move_speed, turn_speed = self.__DEF_SPD,  -self.__SOFT_TURN
        elif angle_diff < 0:
            move_speed, turn_speed = self.__DEF_SPD,   self.__SOFT_TURN
        else:
            move_speed, turn_speed = self.__DEF_SPD,   0.0

        # Return movement list and distance to target
        return ([0.0, move_speed, 0.0, turn_speed], self.__tar_dist)

    def mod_waypoints(self, waypoints, reset = False, interrupt = False):
        """ waypoints: list of iterables, [0]:lat [1]:lon

            Adds new waypoints, then recalculates route to reach
            all current waypoints.
            
            Setting "reset" to True will clear current waypoints
            before adding new ones.
            
            Setting "interrupt" to True will clear the current
            target, forcing a recalculation of next target.
        """
        if reset: del self.__targets[:]
        if interrupt: self.__tar_gps = None
        for waypoint in waypoints: self.__targets.append(waypoint)
        self.__calc_waypoints()

    def get_nav(self):
        self.__set_stats()
        return self.__stats

    def get_home(self):
        return self.__home

    def set_home(self, new_home):
        self.__home = new_home

    def set_target(self, new_target):
        """If the drone is already moving toward a target,
           move current target to waypoint queue and move
           to new target. """
        old_target = self.__tar_gps
        self.__tar_gps = new_target
        if old_target != None:
            self.mod_waypoints([old_target])



