import ps_drone, time, math

class Navigator:
    """Navigator interface of an AR Drone 2.0"""

    def __init__(self, drone):
        """Initialize drone navigation variables"""
        req_packages = ["altitude", "gps", "magneto", "raw_measures"]
        self.__mag_avg = [-14, 13] # Manually calculated
        self.__mag_acc = 6
        self.__tar_gps = [0.0, 0.0]
        self.__tar_dist = 0.0
        self.__tar_angle = 0.0
        self.__stats = {}

        self.__drone = drone
        self.__drone.useDemoMode(False)
        self.__drone.getNDpackage(req_packages)
        self.__set_stats()
        self.__home = self.__stats["gps"]

    def __calc_distance(self):
        """Calculate distance to target"""
        r = 6371e3  # earth's radius in m
        x = self.__stats["gps"]
        y = self.__tar_gps
        phi1 = math.radians(x[0])
        phi2 = math.radians(y[0])
        dphi = math.radians(y[0] - x[0])
        dlam = math.radians(y[1] - x[1])
    
        a = math.sin(dphi / 2) * math.sin(dphi / 2)
        a += math.cos(phi1) * math.cos(phi2)
        a *= (math.sin(dlam / 2) * math.sin(dlam / 2))
        self.__tar_dist = 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    def __calc_heading(self):
        """Calculate necessary heading for straight flight to target"""
        x = self.__stats["gps"]
        y = self.__tar_gps
        q = math.sin(y[1] - x[1]) * math.cos(y[0])
        p = math.cos(x[0]) * math.sin(y[0])
        p -= math.sin(x[0]) * math.cos(y[0]) * math.cos(y[1] - x[1])
        b = math.atan2(q, p) * 180.0 / math.pi
        self.__tar_angle = (b + 360.0) % 360.0
    
    def __set_stats(self):
        """Set stats list with human-readable sensor data."""
        stats = {}
        # Get fresh NavData
        NDC = self.__drone.NavDataCount
        while self.__drone.NavDataCount == NDC: time.sleep(0.01)
    
        # Straightforward data
        stats["acc"] = self.__drone.NavData["raw_measures"][0]
        stats["gyr"] = self.__drone.NavData["raw_measures"][1]
        stats["gps"] = self.__drone.NavData["gps"][:-1] # not using altitude value

        # Convert altitude to meters
        alt = self.__drone.NavData["altitude"][0] # mm
        stats["alt"] = alt / 1000.0 # m
    
        # Turn magnetometer data into heading (degrees)
        stats["mag"] = self.__drone.NavData["magneto"][0][:-1] # not using z value
        for i in range(len(stats["mag"])): stats["mag"][i] -= self.__mag_avg[i]
        stats["deg"] = -1 * (math.atan2(
            stats["mag"][1], stats["mag"][0]) * 180) / math.pi
        if stats["deg"] < 0: stats["deg"] = stats["deg"] + 360
        self.__stats = stats
    
    def __calc_mag(self):
        """Rotates the drone to acquire mag data to normalize."""
        # UNTESTED
        mag_x, mag_y = [], []
        for i in range(self.__mag_acc):
            NDC = self.__drone.NavDataCount
            while self.__drone.NavDataCount == NDC: time.sleep(0.01)
            mag = self.__drone.NavData["magneto"]
            mag_x.append(mag[0])
            mag_y.append(mag[1])
            self.__drone.turnAngle((360.0 / self.__mag_acc), 0.5)
            time.sleep(2)
        self.__mag_avg[0] = reduce(lambda x, y: x + y, mag_x) / len(mag_x)
        self.__mag_avg[1] = reduce(lambda x, y: x + y, mag_y) / len(mag_y)
    
    def calibrate_drone(self, *mag):
        """Basic gyroscope and magnetometer recalibration."""
        # Requires 10 seconds of hovering flight.
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

        # Check if drone needs drastic turn
        self.__calc_heading()
        self.__calc_distance()
        angle_diff = self.__stats["deg"] - self.__tar_angle
        if angle_diff > 180: angle_turn = angle_diff - 360
        else: angle_turn = angle_diff

        if   angle_turn >  10.0: return [0.0, 0.0, 0.0,  0.5, self.__tar_dist]
        elif angle_turn < -10.0: return [0.0, 0.0, 0.0, -0.5, self.__tar_dist]
        elif angle_turn > 0: turn_speed =  0.2
        elif angle_turn < 0: turn_speed = -0.2
        else: turn_speed = 0.0

        # Otherwise move forward with small turn, return distance
        return [0.25, 0.0, 0.0, turn_speed, self.__tar_dist]

    def get_home(self):
        return self.__home

    def get_mag(self):
        self.__set_stats()
        return self.__drone.NavData["magneto"][0]

    def get_deg(self):
        self.__set_stats()
        return self.__stats["deg"]

    def set_target(self, new_target):
        self.__tar_gps = new_target


