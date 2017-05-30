import ps_drone, time, math
ACC = 6 # magnetometer normalization accuracy; higher = better

class Sensor:
    """Sensor suit of an AR Drone 2.0"""

    def __init__(self, drone):
        self.drone = drone
        self.mag_avg = [-24.285714, -35.047619] # Manually calculated
        self.packages = ["altitude", "gps", "magneto", "raw_measures"]

    def get_nav(self, package_list):
        # Poll for new NavData until all requested packages
        # are present in a single transmission.
        while any(package not in self.drone.NavData for package in package_list):
            NDC = self.drone.NavDataCount
            self.drone.getNDpackage(package_list)
            while self.drone.NavDataCount == NDC: time.sleep(0.001)
        return self.drone.NavData
    
    def get_stat(self):
        # Return list of human-readable sensor data.
        nav_data = self.get_nav(self.packages)
    
        # Straightforward data
        acc = nav_data["raw_measures"][0]
        gyr = nav_data["raw_measures"][1]
        gps = nav_data["gps"][:-1]      # not using altitude value

        # Convert altitude to meters
        alt = nav_data["altitude"][0]   # mm
        alt = alt / 1000.0              # m
    
        # Turn magnetometer data into heading (degrees)
        mag = nav_data["magneto"][0][:-1] # not using z value
        for i in range(len(mag)): mag[i] -= self.mag_avg[i]
        deg = -1 * (math.atan2(mag[1], mag[0]) * 180) / math.pi
    
        return [acc, gyr, mag, deg, alt]
    
    def get_mag(self):
        # Rotates the drone to acquire mag data to normalize.
        mag_x, mag_y = [], []
        for i in range(ACC):
            mag = self.get_nav(["magneto"])["magneto"]
            mag_x.append(mag[0])
            mag_y.append(mag[1])
            self.drone.turnAngle((360.0 / ACC), 0.5)
            time.sleep(2)
        self.mag_avg[0] = reduce(lambda x, y: x + y, mag_x) / len(mag_x)
        self.mag_avg[1] = reduce(lambda x, y: x + y, mag_y) / len(mag_y)
        return self.mag_avg
    
    def drone_cal(self):
        # Basic gyroscope and magnetometer recalibration.
        # Requires 10 seconds of hovering flight.
        self.drone.trim()
        time.sleep(5)
        self.drone.takeoff()
        time.sleep(5)
        self.drone.mtrim()
        time.sleep(5)
        self.drone.land()


