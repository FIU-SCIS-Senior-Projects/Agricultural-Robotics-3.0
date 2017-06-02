import math, select, sys, time
from ps_drone import Drone
from navigator import Navigator
from threading import Thread

#home = [25.758984, -80.373743]
gps_target = [25.758889, -80.374701]
tar_threshold = 2.0
landing = False


def goto(drone, navigator):
    global landing
    at_target = False
    while not landing:
        print "getting move"
        move = navigator.get_move()
        movement = move[0]
        tar_dist = move[1]
        print "dist: {}".format(tar_dist)

        if tar_dist < tar_threshold:
            drone.hover()
            drone.land()
            landing = True
        else:
            print "moving: {}".format(movement)
            motion = [3].append(movement)
            drone.at("PCMD", motion)
            time.sleep(2)
    print "landed"

def drone_act(drone, navigator, in_list, com):
    # Check character 'com' for valid command,
    # otherwise ignore it.
    #global home, home_heading
    global landing
    if com == 'z':
        landing = True
        in_list.remove(in_list[0])
        drone.shutdown()
    elif com == 't':
        drone.takeoff()
    elif com == 'd':
        print navigator.get_deg()
    elif com == 'c':
        navigator.calibrate_drone()
    elif com == 'm':
        print navigator.get_mag()
    elif com == 'g':
        moving = Thread(target=goto, args=(drone, navigator))
        moving.daemon = True
        moving.start()
    return in_list

def battery(drone):
    # Poll for valid battery status and return string
    battery = drone.getBattery()
    while battery[0] == -1:
        time.sleep(0.1)
        battery = drone.getBattery()
    return "Battery: {}% {}".format(battery[0], battery[1])

def drone_init(drone):
    # Drone startup
    drone.startup()
    drone.reset()

    # Print battery
    print battery(drone)

def main():
    # Initialize drone and navigator
    global home
    drone = Drone()
    print "initializing"
    drone_init(drone)
    print "setting up nav"
    navigator = Navigator(drone)
    print "setting target"
    navigator.set_target(gps_target)

    # Get home GPS coordinates.
    print "Home: {}".format(navigator.get_home())
    
    # Begin watching input for flight commands.
    read_list = [sys.stdin]
    while read_list:
        # get keypress and act
        ready = select.select(read_list, [], [], 0.1)[0]
        if ready:
            for f in ready:
                read_list = drone_act(drone, navigator, read_list, f.readline()[0])

    # Done, print battery
    print battery(drone)
    
main()
