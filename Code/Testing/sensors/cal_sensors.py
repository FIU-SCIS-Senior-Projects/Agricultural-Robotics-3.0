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

# calibrate sensors
drone.trim()
time.sleep(5)
drone.takeoff()
time.sleep(5)
drone.mtrim()
time.sleep(5)

# shutdown
drone.shutdown()
