import ps_drone, sensors, time

stopping = False

drone = ps_drone.Drone()
drone.startup()
drone.reset()
drone.useDemoMode(False)
sensor = sensors.Sensor(drone)

while not stopping:
    data = sensor.get_stat()
    print "{}".format(data[2])
    if raw_input() == "z": stopping = True

drone.shutdown()

