import itertools, ps_drone, time, math
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
        self.__SOFT_TURN = 0.1
        self.__HARD_TURN = 0.3
        self.__DEF_SPD   = 0.3
        self.__SAMP_NUM  = 150
        self.__SAMP_TIME = 0.005
        self.__mag_avg = [-14, 13] # Manually calculated normalization of magnetometer x, y
        self.__mag_acc = 6  # Points to record during calibration
        self.waypoints = deque() # public for gui route drawing
        self.__samples = deque(maxlen = self.__SAMP_NUM) # Sampling queue

        # NavData packages required by the whole program, even if
        #  not the Navigator object itself.
        self.__REQ_PACKS = ["altitude", "demo", "gps", "magneto", "raw_measures"]

        # Default (invalid) field values
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

    # Sensor Data Calculation Functions
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
                stat, self.__is_not_outlier(np.array(stat)))))

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
        
        # Set flight status
        self.__set_stus()

    def __set_stus(self):
        """Determines current mode of flight. Noted in ps_drone
           library that 'HOVERING' should be taken as 'LANDING'"""
        dem_0 = self.__drone.NavData["demo"][0]
        h_bit, f_bit, l_bit = not dem_0[2], not dem_0[3], not dem_0[4]
        if   h_bit and f_bit: stus = "HOVERING"
        elif h_bit and l_bit: stus = "FLYING"
        elif f_bit and l_bit: stus = "LANDED"
        else: stus = "NONE"
        self.__stats["stus"] = stus

    def __is_not_outlier(self, points, thresh=3.5):
        """Identifies outliers and returns list of booleans identifying
           their locations with 'False' to reject them."""
        if len(points.shape) == 1:
            points = points[:, None]
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
        stats["mag"] = self.__drone.NavData["magneto"][0][:-1] # not using z value
        for i in range(len(stats["mag"])): stats["mag"][i] -= self.__mag_avg[i]
        stats["deg"] = (360 + (-1 * (math.atan2(
            stats["mag"][1], stats["mag"][0]) * 180) / math.pi)) % 360

        # Set new stats
        return stats

    # Calibration functions
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

    # Autonomous flight functions
    def next_tar(self):
        """Pop the next coordinate from the queue to current target"""
        try: self.__tar_gps = self.waypoints.popleft()
        except IndexError: self.__tar_gps = None

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
        if interrupt: self.__tar_gps = None
        for waypoint in waypoints: self.waypoints.append(waypoint)

    def get_nav(self):
        self.__set_stats()
        return self.__stats

    def set_target(self, new_target):
        """If the drone is already moving toward a target,
           move current target to waypoint queue and move
           to new target. """
        old_target = self.__tar_gps
        self.__tar_gps = new_target
        if old_target != None:
            self.mod_waypoints([old_target])

    def __traverse_box_ew(self, max_vrtcs_lon,
            g_range, gen_waypnts_arr, temp_lat,
            rec_vrts_3, rec_vrts_1, rec_vrts_2,
            temp_lon):
        for vrtx in range(g_range):
            #start vertex -> shortest length -> longest length -> shortest length etc.
            new_tempvrtx = temp_lon - max_vrtcs_lon
            if(vrtx % 2 == 0 and vrtx < g_range - 1):
                gen_waypnts_arr.append([temp_lat, new_tempvrtx])
                temp_lat = rec_vrts_3[0]
                gen_waypnts_arr.append([temp_lat, new_tempvrtx])
                temp_lon = new_tempvrtx
                new_tempvrtx = 0
            elif(vrtx % 2 == 1 and vrtx < g_range -1):
                gen_waypnts_arr.append([temp_lat, new_tempvrtx])
                temp_lat = rec_vrts_1[0]
                gen_waypnts_arr.append([temp_lat, new_tempvrtx])
                temp_lon = new_tempvrtx
                new_tempvrtx = 0
            elif(vrtx % 2 == 1 and vrtx == g_range-1):
                gen_waypnts_arr.append([temp_lat, new_tempvrtx])
                temp_lat = rec_vrts_3[0]
                gen_waypnts_arr.append([temp_lat, rec_vrts_3[1]])
                new_tempvrtx = 0
                break
            elif(vrtx % 2 == 0 and vrtx == g_range-1):
                gen_waypnts_arr.append([temp_lat, new_tempvrtx])
                temp_lat = rec_vrts_2[0]
                gen_waypnts_arr.append([temp_lat, rec_vrts_2[1]])
                new_tempvrtx = 0
                break

    def gen_waypnts(self, gps_coors):
        """Using list of GPS coordinates indicating a rectangle,
           calculates a route within the rectangle and populates
           waypoint list with results."""
        if not gps_coors: return True

        g_range = 6
        # GPS path coordinates
        gen_waypnts_arr = deque()

        rec_vrts_1 = gps_coors[0]
        rec_vrts_2 = gps_coors[1]
        rec_vrts_3 = gps_coors[2]
        rec_vrts_4 = gps_coors[3]

        # East to West path orientation
        if(abs(rec_vrts_1[1] - rec_vrts_3[1]) > abs(rec_vrts_1[0]-rec_vrts_3[0])
                                and rec_vrts_1[1] > rec_vrts_3[1]):
            # Generate test waypoints longitudinally
            max_vrtcs_lon = abs(rec_vrts_1[1] - rec_vrts_3[1])/g_range
            temp_lon = rec_vrts_1[1]
            temp_lat = rec_vrts_1[0]

            gen_waypnts_arr.append([temp_lat, temp_lon])

            self.__traverse_box_ew(max_vrtcs_lon,
                    g_range, gen_waypnts_arr, temp_lat,
                    rec_vrts_3, rec_vrts_1, rec_vrts_2,
                    temp_lon)
            #for vrtx in range(g_range):
            #    #start vertex -> shortest length -> longest length -> shortest length etc.
            #    new_tempvrtx = temp_lon - max_vrtcs_lon
            #    if(vrtx % 2 == 0 and vrtx < g_range - 1):
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lat = rec_vrts_3[0]
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lon = new_tempvrtx
            #        new_tempvrtx = 0
            #    elif(vrtx % 2 == 1 and vrtx < g_range -1):
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lat = rec_vrts_1[0]
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lon = new_tempvrtx
            #        new_tempvrtx = 0
            #    elif(vrtx % 2 == 1 and vrtx == g_range-1):
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lat = rec_vrts_3[0]
            #        gen_waypnts_arr.append([temp_lat, rec_vrts_3[1]])
            #        new_tempvrtx = 0
            #        break
            #    elif(vrtx % 2 == 0 and vrtx == g_range-1):
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lat = rec_vrts_2[0]
            #        gen_waypnts_arr.append([temp_lat, rec_vrts_2[1]])
            #        new_tempvrtx = 0
            #        break
        #West to East path orientation
        elif(abs(rec_vrts_1[1] - rec_vrts_3[1]) > abs(rec_vrts_1[0]-rec_vrts_3[0])
                                and rec_vrts_1[1] < rec_vrts_3[1]):
            # Generate test waypoints longitudinally
            max_vrtcs_lon = abs(rec_vrts_1[1] - rec_vrts_3[1])/g_range
            temp_lon = rec_vrts_1[1]
            temp_lat = rec_vrts_1[0]

            gen_waypnts_arr.append([temp_lat, temp_lon])

            self.__traverse_box_ew(max_vrtcs_lon,
                    g_range, gen_waypnts_arr, temp_lat,
                    rec_vrts_3, rec_vrts_1, rec_vrts_2,
                    temp_lon)
            #for vrtx in range(g_range):
            #    #start vertex -> shortest length -> longest length -> shortest length etc.
            #    new_tempvrtx = temp_lon + max_vrtcs_lon
            #    if(vrtx % 2 == 0 and vrtx < g_range - 1):
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lat = rec_vrts_3[0]
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lon = new_tempvrtx
            #        new_tempvrtx = 0
            #    elif(vrtx % 2 == 1 and vrtx < g_range -1):
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lat = rec_vrts_1[0]
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lon = new_tempvrtx
            #        new_tempvrtx = 0
            #    elif(vrtx % 2 == 1 and vrtx == g_range-1):
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lat = rec_vrts_3[0]
            #        gen_waypnts_arr.append([temp_lat, rec_vrts_3[1]])
            #        new_tempvrtx = 0
            #        break
            #    elif(vrtx % 2 == 0 and vrtx == g_range-1):
            #        gen_waypnts_arr.append([temp_lat, new_tempvrtx])
            #        temp_lat = rec_vrts_2[0]
            #        gen_waypnts_arr.append([temp_lat, rec_vrts_2[1]])
            #        new_tempvrtx = 0
            #        break
        # South to North path orientation
        elif(abs(rec_vrts_1[1] - rec_vrts_3[1]) < abs(rec_vrts_1[0]-rec_vrts_3[0])
                                and rec_vrts_1[0] > rec_vrts_3[0]):
            # Generate test waypoints longitudinally
            self.max_vrtcs_lat = abs(rec_vrts_1[0] - rec_vrts_3[0])/g_range
            temp_lon = rec_vrts_1[1]
            temp_lat = rec_vrts_1[0]

            gen_waypnts_arr.append([temp_lat, temp_lon])
            for vrtx in range(g_range):
                #start vertex -> shortest length -> longest length -> shortest length etc.
                new_tempvrtx = temp_lat - self.max_vrtcs_lat
                if(vrtx % 2 == 0 and vrtx < g_range - 1):
                    gen_waypnts_arr.append([new_tempvrtx, temp_lon])
                    temp_lon = rec_vrts_3[1]
                    gen_waypnts_arr.append([new_tempvrtx, temp_lon])
                    temp_lat = new_tempvrtx
                    new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx < g_range -1):
                    gen_waypnts_arr.append([new_tempvrtx, temp_lon])
                    temp_lon = rec_vrts_1[1]
                    gen_waypnts_arr.append([new_tempvrtx, temp_lon])
                    temp_lat = new_tempvrtx
                    new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx == g_range-1):
                    gen_waypnts_arr.append([new_tempvrtx, temp_lon])
                    temp_lon = rec_vrts_3[1]
                    gen_waypnts_arr.append([rec_vrts_3[0], temp_lon])
                    new_tempvrtx = 0
                    break
                elif(vrtx % 2 == 0 and vrtx == g_range-1):
                    gen_waypnts_arr.append([rec_vrts_3[0], temp_lon])
                    new_tempvrtx = 0
                    break
        # North to South path orientation
        elif(abs(rec_vrts_1[1] - rec_vrts_3[1]) < abs(rec_vrts_1[0]-rec_vrts_3[0])
                                and rec_vrts_1[0] < rec_vrts_3[0]):
            # Generate test waypoints longitudinally
            self.max_vrtcs_lat = abs(rec_vrts_1[0] - rec_vrts_3[0])/g_range
            temp_lon = rec_vrts_1[1]
            temp_lat = rec_vrts_1[0]

            gen_waypnts_arr.append([temp_lat, temp_lon])
            for vrtx in range(g_range):
                #start vertex -> shortest length -> longest length -> shortest length etc.
                new_tempvrtx = temp_lat + self.max_vrtcs_lat
                if(vrtx % 2 == 0 and vrtx < g_range - 1):
                    gen_waypnts_arr.append([new_tempvrtx, temp_lon])
                    temp_lon = rec_vrts_3[1]
                    gen_waypnts_arr.append([new_tempvrtx, temp_lon])
                    temp_lat = new_tempvrtx
                    new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx < g_range -1):
                    gen_waypnts_arr.append([new_tempvrtx, temp_lon])
                    temp_lon = rec_vrts_1[1]
                    gen_waypnts_arr.append([new_tempvrtx, temp_lon])
                    temp_lat = new_tempvrtx
                    new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx == g_range-1):
                    gen_waypnts_arr.append([new_tempvrtx, temp_lon])
                    temp_lon = rec_vrts_3[1]
                    gen_waypnts_arr.append([rec_vrts_3[0], temp_lon])
                    new_tempvrtx = 0
                    break
                elif(vrtx % 2 == 0 and vrtx == g_range-1):
                    gen_waypnts_arr.append([rec_vrts_3[0], temp_lon])
                    new_tempvrtx = 0
                    break
        self.waypoints.clear()

        for waypoint in gen_waypnts_arr:
            self.waypoints.append(waypoint)
