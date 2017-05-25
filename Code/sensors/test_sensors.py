import os, ps_drone, select, sys, time

def get_nav(drone, packages):
    while any(package not in drone.NavData for package in packages):
        NDC = drone.NavDataCount
        drone.getNDpackage(packages)
        while drone.NavDataCount == NDC: time.sleep(0.001)
    return drone.NavData

def print_bat(drone):
    # print starting battery
    while drone.getBattery()[0] == -1: time.sleep(0.1)
    battery = drone.getBattery()
    drone.printBlue("Battery: {}% {}".format(battery[0], battery[1]))

def print_stat(drone):
    packages = ["gps", "magneto", "wifi", "raw_measures", "altitude"]
    nav_data = get_nav(drone, packages)
    acc = nav_data["raw_measures"][0]
    gyr = nav_data["raw_measures"][1]
    mag = nav_data["magneto"][0]
    gps = nav_data["gps"]
    alt = nav_data["altitude"][0] #mm
    #wif = nav_data["wifi"]
    print "acc: {}\ngyros: {}\nmag: {}\ngps x,y: {}\naltitude: {}m".format(
            acc, gyr, mag, gps[:-1], alt/1000.0)

def simple_flight(drone):
    drone.takeoff()
    time.sleep(5)
    drone.setSpeed(0.2)
    time.sleep(1)
    drone.moveForward()
    time.sleep(2)
    drone.stop()
    drone.land()

def drone_act(drone, in_list, com):
    if com == 'z':
        drone.land()
        in_list.remove(file)
    elif com == 's':
        simple_flight(drone)
    elif com == 'd':
        hover_alt(drone)
    return in_list

def drone_init(drone):
    drone.startup()
    drone.reset()
    time.sleep(0.1)
    drone.useDemoMode(False)
    #drone.useDemoMode(True)
    drone.setConfigAllID()
    #drone.groundCam()
    drone.frontCam()
    drone.midVideo()
    drone.sdVideo()
    drone.addNDpackage(['altitude'])
    CDC, NDC = drone.ConfigDataCount, drone.NavDataCount
    while CDC == drone.ConfigDataCount: time.sleep(0.0001)
    while NDC == drone.NavDataCount: time.sleep(0.0001)
    while drone.getBattery()[0] == -1: time.sleep(0.01)
    print_bat(drone)

def drone_cal(drone):
    drone.trim()
    time.sleep(5)
    drone.takeoff()
    time.sleep(5)
    drone.mtrim()
    time.sleep(5)
    drone.land()


# script start
drone = ps_drone.Drone()
drone_init(drone)
home = get_nav(drone, ["gps"])[:-1]
drone_cal(drone)

# piloted flight
stop, read_list, timer = False, [sys.stdin], time.time()
try:
    while read_list:
        # get keypress and act
        ready = select.select(read_list, [], [], 0.1)[0]
        if ready:
            for file in ready:
                read_list = drone_act(drone, read_list, file.readline()[0])
        if (time.time() - timer) >= 1:
            print_stat(drone)
            timer = time.time()
        if (time.time() - timer) >= 15:
            print_bat(drone)
            timer = time.time()

except KeyboardInterrupt as e:
    print e
    
# drone has shut down, print battery and status
print_bat(drone)
