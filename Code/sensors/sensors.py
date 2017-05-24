import ps_drone, time, sys

# initialize drone
drone = ps_drone.Drone()
drone.startup()
drone.reset()
drone.useDemoMode(False)

# print starting battery
timer = time.time()
while drone.getBattery()[0] == -1:
    time.sleep(0.1)
    timer = time.time()
    if timer > 2:
        print "bleh"
        sys.exit()
battery = drone.getBattery()
drone.printBlue("Battery: {}% {}".format(battery[0], battery[1]))

# gps, magnetometer, wifi connection
#while (package for package in ["gps","magneto","wifi"]) not in drone.NavData:
while any(package not in drone.NavData for package in ["gps","magneto","wifi"]):
    NDC = drone.NavDataCount
    drone.getNDpackage(["all"])
    while drone.NavDataCount == NDC: time.sleep(0.001)

mag = drone.NavData["magneto"][0]
gps = drone.NavData["gps"]
wif = drone.NavData["wifi"]

print "{}\n{}\n{}".format(mag, gps, wif)




drone.shutdown()
