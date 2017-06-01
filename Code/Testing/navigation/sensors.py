import ps_drone, time, math

class Navigator:
    """Navigator interface of an AR Drone 2.0"""

    def __init__(self, drone):
        """Initialize drone navigation variables"""
        req_packages = ["altitude", "gps", "magneto", "raw_measures"]
        self.__mag_avg = [-24.285714, -35.047619] # Manually calculated
        self.__mag_acc = 6
        self.__tar_gps = [0.0, 0.0]
        self.__tar_dist = 0.0
        self.__tar_angle = 0.0
        self.__home = []

        self.__drone = drone
        self.__drone.useDemoMode(False)
        self.__drone.getNDpackage(req_packages)
        self.__set_stats

    def __calc_dist(self, x, y):
        """Calculate distance to target"""
        r = 6371e3  # earth's radius in m
        phi1 = math.radians(x[0])
        phi2 = math.radians(y[0])
        dphi = math.radians(y[0] - x[0])
        dlam = math.radians(y[1] - x[1])
    
        a = math.sin(dphi / 2) * math.sin(dphi / 2)
        a += math.cos(phi1) * math.cos(phi2)
        a *= (math.sin(dlam / 2) * math.sin(dlam / 2))
        self.__tar_dist = 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    def __calc_angle(self, x, y):
        """Calculate necessary heading for straight flight to target"""
        q = math.sin(y[1] - x[1]) * math.cos(y[0])
        p = math.cos(x[0]) * math.sin(y[0])
        p -= math.sin(x[0]) * math.cos(y[0]) * math.cos(y[1] - x[1])
        b = math.atan2(q, p) * 180.0 / math.pi
        self.__tar_angle = (b + 360.0) % 360.0
    
    def __calc_forward(self, x, y):
        """Calculate speed to move forward"""
        return 0
    
    def __set_stats(self):
        """Set stats list with human-readable sensor data."""
        # Get fresh NavData
        NDC = self.__drone.NavDataCount
        while self.__drone.NavDataCount == NDC: time.sleep(0.01)
    
        # Straightforward data
        acc = self.__drone.NavData["raw_measures"][0]
        gyr = self.__drone.NavData["raw_measures"][1]
        gps = self.__drone.NavData["gps"][:-1]      # not using altitude value

        # Convert altitude to meters
        alt = self.__drone.NavData["altitude"][0]   # mm
        alt = alt / 1000.0              # m
    
        # Turn magnetometer data into heading (degrees)
        mag = self.__drone.NavData["magneto"][0][:-1] # not using z value
        for i in range(len(mag)): mag[i] -= self.__mag_avg[i]
        deg = -1 * (math.atan2(mag[1], mag[0]) * 180) / math.pi
    
        # Store sensor data
        self.__stats = [acc, gyr, mag, deg, gps, alt]
    
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
    
    def calibrate_drone(self):
        """Basic gyroscope and magnetometer recalibration."""
        # Requires 10 seconds of hovering flight.
        self.__drone.trim()
        time.sleep(5)
        self.__drone.takeoff()
        time.sleep(5)
        self.__drone.mtrim()
        time.sleep(5)
        self.__drone.land()

    def get_home(self):
        return self.__home

    def get_curr(self):
        self.__set_stats()
        return self.__curr

    def get_move(self):
        """Perform calculations to get arguments for a drone move"""
        return 0

