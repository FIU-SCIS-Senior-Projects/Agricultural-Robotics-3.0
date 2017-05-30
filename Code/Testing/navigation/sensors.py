import ps_drone, time, math
ACC = 6 # magnetometer normalization accuracy; higher = better

def get_stat(drone):
    # Return list of human-readable sensor data.
    packages = ["gps", "magneto", "raw_measures", "altitude"]
    nav_data = get_nav(drone, packages)

    # Straightforward data
    acc = nav_data["raw_measures"][0]
    gyr = nav_data["raw_measures"][1]
    gps = nav_data["gps"]
    alt = nav_data["altitude"][0] #mm

    # Turn magnetometer data into heading (degrees)
    mag = nav_data["magneto"][0]
    #mag_avg = [-28, -13, -43]
    #for i in range(len(mag)): mag[i] -= mag_avg[i]
    #deg = (math.atan2(mag[1], mag[0]) * 180) / math.pi
    deg = math.atan2(mag[1], mag[0])

    return [acc, gyr, mag, deg, gps[:-1], (alt/1000.0)]

def get_nav(drone, packages):
    # Poll for new NavData until all requested packages
    # are present in a single transmission.
    #while any(package not in drone.NavData for package in packages) or not drone.NoNavData:
    while any(package not in drone.NavData for package in packages):
        NDC = drone.NavDataCount
        drone.getNDpackage(packages)
        while drone.NavDataCount == NDC: time.sleep(0.001)
    return drone.NavData

def get_mag(drone):
    # Rotates the drone to acquire mag data to normalize.
    mag_x, mag_y, mag_z, mag_avg = [], [], [], []
    for i in range(ACC):
        mag = get_nav(["magneto"])["magneto"]
        mag_x.append(mag[0])
        mag_y.append(mag[1])
        mag_z.append(mag[2])
        drone.turnAngle((360.0 / ACC), 0.5)
        time.sleep(2)
    mag_avg.append(reduce(lambda x, y: x + y, mag_x) / len(mag_x))
    mag_avg.append(reduce(lambda x, y: x + y, mag_y) / len(mag_y))
    mag_avg.append(reduce(lambda x, y: x + y, mag_z) / len(mag_z))
    return mag_avg

def drone_cal(drone):
    # Basic gyroscope and magnetometer recalibration.
    # Requires 10 seconds of hovering flight.
    drone.trim()
    time.sleep(5)
    drone.takeoff()
    time.sleep(5)
    drone.mtrim()
    time.sleep(5)
    drone.land()

