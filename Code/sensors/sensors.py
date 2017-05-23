import ps_drone, time

drone = ps_drone.Drone()
drone.startup()
drone.reset()
while drone.getBattery()[0] == -1: time.sleep(0.1)
time.sleep(0.5)
battery = drone.getBattery()
drone.printBlue("Battery: {}% {}".format(battery[0], battery[1]))


