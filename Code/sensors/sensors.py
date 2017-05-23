import ps_drone, time

drone = ps_drone.Drone()
drone.startup()
drone.reset()
while drone.getBattery()[0] == -1: time.sleep(0.1)
time.sleep(0.5)
battery = drone.getBattery()
drone.printBlue("Battery: {}% {}".format(battery[0], battery[1]))

drone.useDemoMode(False)
CDC = drone.ConfigDataCount
drone.getConfig()
while CDC == drone.ConfigDataCount: time.sleep(0.001)
lat, lon, alt = 0, 0, 0
for conf in drone.ConfigData:
    if conf[0] == "gps:latitude":
        lat = conf[1]
    elif conf[0] == "gps:longitude":
        lon = conf[1]
    elif conf[0] == "gps:altitude":
        alt = conf[1]

print "{}\n{}\n{}".format(lat, lon, alt)


