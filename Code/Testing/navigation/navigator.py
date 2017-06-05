import itertools, ps_drone, time, math
from threading import Thread
import numpy as np
np.seterr(divide='ignore', invalid='ignore')

class Navigator:
    """Navigator interface of an AR Drone 2.0"""
    SOFT_TURN = 0.2
    HARD_TURN = 1.0

    def __init__(self, drone):
        """Initialize drone navigation variables"""
        req_packages = ["altitude", "gps", "magneto", "raw_measures"]

        # Default (invalid) field values
        self.__mag_avg = [-14, 13] # Manually calculated
        self.__mag_acc = 6
        self.__samples = []
        self.__tar_gps = [0.0, 0.0]
        self.__tar_dist = 0.0
        self.__tar_angle = 0.0
        self.__stats = {}

        # Initialize sensor data transmissions
        self.__drone = drone
        self.__drone.useDemoMode(False)
        self.__drone.getNDpackage(req_packages)
        time.sleep(0.1)

        # Start taking sensor data
        self.__sensors = Thread(target=self.__sensors_collect, args=())
        self.__sensors.daemon = True
        self.__sensors.start()
        time.sleep(3)

        # Get current GPS for "home" location
        self.__set_stats()
        self.__home = self.__stats["gps"]

    def __sensors_collect(self):
        """Continuously collects sensor data"""
        # Initialize samples
        for i in range(4):
            self.__samples.append(self.__get_stats())
            time.sleep(0.5)

        while True:
            # If there are too many samples, delete some
            while len(self.__samples) >= 5: del self.__samples[0]

            # Add new sensor data
            self.__samples.append(self.__get_stats())

            # Wait for next sampling
            time.sleep(0.5)

    def __set_stats(self):
        """Preprocessing of stats queue to reduce variation"""
        acc, gyr, gps, alt, mag, deg = [], [], [], [], [], []
        for item in self.__samples:
            acc.append(item["acc"])
            gyr.append(item["gyr"])
            gps.append(item["gps"])
            alt.append(item["alt"])
            mag.append(item["mag"])
            deg.append(item["deg"])

        # Remove outliers
        acc_proc = list(itertools.compress(
            acc, self.__is_outlier(np.array(acc))))
        gyr_proc = list(itertools.compress(gyr,
            self.__is_outlier(np.array(gyr))))
        gps_proc = list(itertools.compress(gps,
            self.__is_outlier(np.array(gps))))
        alt_proc = list(itertools.compress(alt,
            self.__is_outlier(np.array(alt))))
        mag_proc = list(itertools.compress(mag,
            self.__is_outlier(np.array(mag))))
        deg_proc = list(itertools.compress(deg,
            self.__is_outlier(np.array(deg))))

        # Check that lists are populated
        if not acc_proc: acc_proc = acc
        if not gyr_proc: gyr_proc = gyr
        if not gps_proc: gps_proc = gps
        if not alt_proc: alt_proc = alt
        if not mag_proc: mag_proc = mag
        if not deg_proc: deg_proc = deg

        # Average the remainder of the lists
        self.__stats["acc"] = reduce(lambda x, y: x + y, np.array(acc_proc)) / len(acc_proc)
        self.__stats["gyr"] = reduce(lambda x, y: x + y, np.array(gyr_proc)) / len(gyr_proc)
        self.__stats["gps"] = reduce(lambda x, y: x + y, np.array(gps_proc)) / len(gps_proc)
        self.__stats["alt"] = reduce(lambda x, y: x + y, np.array(alt_proc)) / len(alt_proc)
        self.__stats["mag"] = reduce(lambda x, y: x + y, np.array(mag_proc)) / len(mag_proc)
        self.__stats["deg"] = reduce(lambda x, y: x + y, np.array(deg_proc)) / len(deg_proc)

    def __is_outlier(self, points, thresh=3.5):
        """
        Returns a boolean array with True if points are outliers and False 
        otherwise.
    
        Parameters:
        -----------
            points : An numobservations by numdimensions array of observations
            thresh : The modified z-score to use as a threshold. Observations with
                a modified z-score (based on the median absolute deviation) greater
                than this value will be classified as outliers.
    
        Returns:
        --------
            mask : A numobservations-length boolean array.
    
        References:
        ----------
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
    
        try: modified_z_score = 0.6745 * diff / med_abs_deviation
        except: pass
    
        return modified_z_score > thresh

    def __get_stats(self):
        """Get stats list with human-readable sensor data."""
        stats = {}
        # Get fresh NavData
        NDC = self.__drone.NavDataCount
        while self.__drone.NavDataCount == NDC: time.sleep(0.01)
    
        # Straightforward data
        stats["acc"] = self.__drone.NavData["raw_measures"][0]
        stats["gyr"] = self.__drone.NavData["raw_measures"][1]
        stats["gps"] = self.__drone.NavData["gps"][:-1] # not using altitude value

        # Convert altitude to meters
        stats["alt"] = self.__drone.NavData["altitude"][0] / 1000.0
    
        # Turn magnetometer data into heading (degrees)
        stats["mag"] = self.__drone.NavData["magneto"][0][:-1] # not using z value
        for i in range(len(stats["mag"])): stats["mag"][i] -= self.__mag_avg[i]
        stats["deg"] = (360 + (-1 * (math.atan2(
            stats["mag"][1], stats["mag"][0]) * 180) / math.pi)) % 360

        # Set new stats
        return stats
    
    def __calc_distance(self):
        """Calculate distance to target"""
        r = 6371e3  # earth's radius in m
        x = self.__stats["gps"]
        y = self.__tar_gps

        # Convert GPS degrees to radians
        phi1 = math.radians(x[0])
        phi2 = math.radians(y[0])
        dphi = math.radians(y[0] - x[0])
        dlam = math.radians(y[1] - x[1])
    
        # 'Great circle' distance between two GPS coords
        a = math.sin(dphi / 2) * math.sin(dphi / 2)
        a += math.cos(phi1) * math.cos(phi2)
        a *= (math.sin(dlam / 2) * math.sin(dlam / 2))
        self.__tar_dist = 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    def __calc_heading(self):
        """Calculate necessary heading for straight flight to target"""
        x = self.__stats["gps"]
        y = self.__tar_gps

        # Initial heading required for 'Great circle' traversal
        q = math.sin(y[1] - x[1]) * math.cos(y[0])
        p = math.cos(x[0]) * math.sin(y[0])
        p -= math.sin(x[0]) * math.cos(y[0]) * math.cos(y[1] - x[1])
        b = math.atan2(q, p) * 180.0 / math.pi
        self.__tar_angle = (b + 360.0) % 360.0
    
    def __calc_mag(self):
        """Rotates the drone to acquire mag data to use in normalization."""
        mag_x, mag_y = [], []
        for i in range(self.__mag_acc):
            NDC = self.__drone.NavDataCount
            while self.__drone.NavDataCount == NDC: time.sleep(0.01)
            mag = self.__drone.NavData["magneto"]
            mag_x.append(mag[0])
            mag_y.append(mag[1])
            self.__drone.turnAngle((360.0 / self.__mag_acc), 1.0)
            time.sleep(3)
        self.__mag_avg[0] = reduce(lambda x, y: x + y, mag_x) / len(mag_x)
        self.__mag_avg[1] = reduce(lambda x, y: x + y, mag_y) / len(mag_y)
    
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
        self.__set_stats()

        # Get angle of required turn
        self.__calc_heading()
        self.__calc_distance()
        angle_diff = self.__stats["deg"] - self.__tar_angle
        if angle_diff > 180: angle_turn = angle_diff - 360
        else: angle_turn = angle_diff

        # If drastic turn is needed, only perform that turn
        if   angle_turn >  10.0: move_speed, turn_speed = 0.0, -HARD_TURN
        elif angle_turn < -10.0: move_speed, turn_speed = 0.0,  HARD_TURN
        elif angle_turn > 0: move_speed, turn_speed = 0.5, -SOFT_TURN
        elif angle_turn < 0: move_speed, turn_speed = 0.5,  SOFT_TURN
        else: move_speed, turn_speed = 0.5, 0.0

        # Return movement list and distance to target
        return ([move_speed, 0.0, 0.0, turn_speed], self.__tar_dist)

    def set_target(self, new_target):
        self.__tar_gps = new_target


    # Diagnostic functions
    def get_home(self):
        return self.__home

    def get_mag(self):
        self.__set_stats()
        return self.__drone.NavData["magneto"][0]

    def get_deg(self):
        self.__set_stats()
        return self.__stats["deg"]

    def get_all(self):
        self.__set_stats()
        return self.__stats


