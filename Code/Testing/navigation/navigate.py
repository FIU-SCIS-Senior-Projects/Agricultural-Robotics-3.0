import math, select, sys, time
from ps_drone import Drone
from navigator import Navigator
from threading import Thread

#TEST_HOME = [25.758995, -80.373743]
#gps_target = [25.758536, -80.374548] # south ecs parking lot
gps_target = [25.757582, -80.373888] # library entrance
#gps_target = [25.758633, -80.372067] # physics lecture
#gps_target = [25.759387, -80.376163] # roundabout

tar_threshold = 2.0
landing = False


def weeble(drone):
    # Test if movements must be 'cancelled out'
    # forward, stop, right, stop, up, stop, lturn, stop
    moves = [[ 0.0,  0.2,  0.0,  0.0],
             [ 0.0, -0.2,  0.0,  0.0],
             [ 0.2,  0.0,  0.0,  0.0],
             [-0.2,  0.0,  0.0,  0.0],
             [ 0.0,  0.0,  1.0,  0.0],
             [ 0.0,  0.0, -1.0,  0.0],
             [ 0.0,  0.0,  0.0,  1.0],
             [ 0.0,  0.0,  0.0, -1.0],
             ]
    drone.takeoff()
    time.sleep(3)
    for move in moves:
        print "moving {}".format(move)
        drone.move(*move)
        time.sleep(2)
    drone.hover()
    time.sleep(1)
    drone.land()
    

def goto(drone, navigator):
    global landing
    while not landing:
        move = navigator.get_move()
        movement = move[0]
        tar_dist = move[1]
        print "dist: {}".format(tar_dist)

        if tar_dist < tar_threshold:
            print "landing"
            drone.hover()
            drone.land()
            landing = True
        else:
            print "moving: {}".format(movement)
            #drone.move(*movement)
            time.sleep(1.5)

def drone_act(drone, navigator, in_list, com):
    # Check character 'com' for valid command,
    # otherwise ignore it.
    global landing
    if com == 'z':
        landing = True
        in_list.remove(in_list[0])
        drone.shutdown()
    elif com == 'g':
        moving = Thread(target=goto, args=(drone, navigator))
        moving.daemon = True
        moving.start()
    elif com == 't':
        drone.takeoff()
    elif com == 'C':
        navigator.calibrate_drone(True)
    elif com == 'c':
        navigator.calibrate_drone()
    elif com == 'a':
        print navigator.get_all()
    elif com == 'w':
        moving = Thread(target=weeble, args=(drone,))
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
    drone.startup()
    drone.reset()

def main():
    # Initialize drone and navigator
    drone = Drone()
    drone_init(drone)
    print battery(drone)
    navigator = Navigator(drone)
    time.sleep(0.5)
    navigator.set_target(gps_target)
    
    # Begin watching input for flight commands.
    read_list = [sys.stdin]
    while read_list:
        # get keypress and act
        ready = select.select(read_list, [], [], 0.1)[0]
        if ready:
            for f in ready:
                read_list = drone_act(drone, navigator, read_list, f.readline()[0])

main()
