import ps_drone, time

drone = ps_drone.Drone()
drone.startup()
drone.reset()
drone.useDemoMode(False)

while drone.getBattery()[0] == -1: time.sleep(0.1)
battery = drone.getBattery()
drone.printBlue("Battery: {}% {}".format(battery[0], battery[1]))

while "gps" not in drone.NavData:
    NDC = drone.NavDataCount
    drone.getNDpackage(["all"])
    while drone.NavDataCount == NDC: time.sleep(0.001)

print drone.NavData["gps"]

drone.shutdown()
