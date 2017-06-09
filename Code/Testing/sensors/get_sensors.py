import ps_drone, time

# initialize drone
drone = ps_drone.Drone()
drone.startup()
drone.reset()
drone.useDemoMode(False)

# print starting battery
timer = time.time()
while drone.getBattery()[0] == -1: time.sleep(0.1)
battery = drone.getBattery()
drone.printBlue("Battery: {}% {}".format(battery[0], battery[1]))

# gps, magnetometer, wifi connection
packages = ["gps", "magneto", "wifi", "raw_measures", "altitude"]
try:
    while True:
        while any(package not in drone.NavData for package in packages):
            NDC = drone.NavDataCount
            drone.getNDpackage(["all"])
            while drone.NavDataCount == NDC: time.sleep(0.001)
        
        acc = drone.NavData["raw_measures"][0]
        gyr = drone.NavData["raw_measures"][1]
        mag = drone.NavData["magneto"][0]
        gps = drone.NavData["gps"]
        alt = drone.NavData["altitude"][0] #mm
        wif = drone.NavData["wifi"]
        print "acc: {}\ngyros: {}\nmag: {}\ngps: {}\nalt: {}\nwifi: {}".format(
                acc, gyr, mag, gps, alt,  wif)
        time.sleep(0.5)


except: pass
drone.shutdown()
