import math, select, sys, time
from ps_drone import Drone
from sensors import Sensor

home = [25.758984, -80.373743]
gps_target = [25.758847, -80.374646]

def get_dist(x, y):
    r = 6371e3
    phi1 = math.radians(x[0])
    phi2 = math.radians(y[0])
    dphi = math.radians(y[0] - x[0])
    dlam = math.radians(y[1] - x[1])

    a = math.sin(dphi / 2) * math.sin(dphi / 2)
    a += math.cos(phi1) * math.cos(phi2) * (math.sin(dlam / 2) * math.sin(dlam / 2))
    return 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def goto(drone, sensors, target):
    at_target = False
    while not at_target:
        curr_pos = sensors.get_nav()["gps"][:-1]
        tar_dist = get_dist(curr_pos, target)
    #    tar_angle = get_angle()
    #
    #    if tar_dist < tar_threshold:
    #        drone.hover()
    #        drone.land()
    #        at_target = True
    #
    #    else:
    #        forward = compute_forward()
    #        turning = compute_turn()
    #
    #    drone.move(forward, turning)

def drone_act(drone, in_list, com):
    # Check character 'com' for valid command,
    # otherwise ignore it.
    global home, gps_target
    if com == 'z':
        in_list.remove(in_list[0])
        drone.shutdown()
    elif com == 't':
        get_dist(home, gps_target)
    return in_list

def print_bat(drone):
    # Poll for valid battery status and print it.
    battery = drone.getBattery()
    while battery[0] == -1:
        time.sleep(0.1)
        battery = drone.getBattery()
    drone.printBlue("Battery: {}% {}".format(battery[0], battery[1]))

def drone_init(drone):
    # Drone startup
    drone.startup()
    drone.reset()

    # NavData packages
    drone.useDemoMode(False)
    drone.getNDpackage(["altitude", "gps", "magneto", "raw_measures"])

    # Print battery
    print_bat(drone)

def main():
    # Initialize drone and sensors
    global home
    drone = Drone()
    drone_init(drone)
    sensors = Sensor(drone)

    # Get home GPS coordinates.
    #home = sensors.get_nav()["gps"][:-1]
    print "Home: {}".format(home)
    
    # Begin watching input for flight commands.
    read_list = [sys.stdin]
    while read_list:
        # get keypress and act
        ready = select.select(read_list, [], [], 0.1)[0]
        if ready:
            for f in ready:
                read_list = drone_act(drone, read_list, f.readline()[0])

    # Done, print battery
    print_bat(drone)
    
main()
