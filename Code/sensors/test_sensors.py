import os, ps_drone, select, sys, time, math
from threading import Thread
quitting = False
dry_run = False

def get_nav(drone, packages):
    # Poll for new NavData until all requested packages
    # are present in a single transmission.
    while any(package not in drone.NavData for package in packages):
        NDC = drone.NavDataCount
        drone.getNDpackage(packages)
        while drone.NavDataCount == NDC: time.sleep(0.001)
    return drone.NavData

def drone_cal(drone):
    # Basic gyroscope and magnetometer recalibration.
    # Requires 10 seconds of hovering flight.
    if dry_run: print "drone.trim()"
    else: drone.trim()
    time.sleep(5)
    if dry_run: print "drone.takeoff()"
    else: drone.takeoff()
    time.sleep(5)
    if dry_run: print "drone.mtrim()"
    else: drone.mtrim()
    time.sleep(5)
    if dry_run: print "drone.land()"
    else: drone.land()

def print_bat(drone):
    # Poll for valid battery status and print it.
    battery = drone.getBattery()
    while battery[0] == -1:
        time.sleep(0.1)
        battery = drone.getBattery()
    drone.printBlue("Battery: {}% {}".format(battery[0], battery[1]))

def get_stat(drone):
    # Return list of human-readable sensor data.
    packages = ["gps", "magneto", "raw_measures", "altitude"]
    nav_data = get_nav(drone, packages)
    acc = nav_data["raw_measures"][0]
    gyr = nav_data["raw_measures"][1]
    mag = nav_data["magneto"][0]
    gps = nav_data["gps"]
    alt = nav_data["altitude"][0] #mm
    #mag_avg = [-28, -13, -43]
    #for i in range(len(mag)): mag[i] -= mag_avg[i]
    #deg = (math.atan2(mag[1], mag[0]) * 180) / math.pi
    deg = math.atan2(mag[1], mag[0])
    return [acc, gyr, mag, deg, gps[:-1], (alt/1000.0)]

def time_stat(drone):
    # Timed printing of sensor data for testing.
    # This is only used in a threaded process.
    global quitting
    old_stats = [0, 0, 0, 0, 0, 0]
    counter, timer = 0, time.time()
    stat_timer, old_timer = timer, timer
    while not quitting:
        curr_time = time.time()
        if (curr_time - stat_timer > 1):
            stats = get_stat(drone)
            print "acc: {}\ngyros: {}\nmag: {}\ndeg: {}\ngps x,y: {}\naltitude: {}m".format(
                    stats[0], stats[1], stats[2], stats[3], stats[4], stats[5])
            if stats == old_stats:
                if curr_time - old_timer < 10:
                    drone.shutdown()
                    print "Sensor error"
                    return 1
                if dry_run: print "drone.land()"
                else: drone.land()
                if dry_run: print "time.sleep(10)"
                else: time.sleep(10)
                drone_cal(drone)
                old_timer = curr_time
            stat_timer = curr_time
            old_stats = stats
    return 0

def simple_flight(drone):
    # An 8-second flight where it moves forward then lands.
    if dry_run: print "drone.takeoff()"
    else: drone.takeoff()
    time.sleep(5)
    if dry_run: print "drone.setSpeed(0.2)"
    else: drone.setSpeed(0.2)
    time.sleep(1)
    if dry_run: print "drone.moveForward()"
    else: drone.moveForward()
    time.sleep(2)
    if dry_run: print "drone.stop()"
    else: drone.stop()
    if dry_run: print "drone.land()"
    else: drone.land()

def drone_act(drone, in_list, com):
    # Check character 'com' for valid command,
    # otherwise ignore it.
    if com == 'z':
        in_list.remove(in_list[0])
        drone.shutdown()
    elif com == 'f':
        Thread(target=simple_flight, args=([drone])).start()
    elif com == 'w':
        if dry_run: print "Thread(target=drone.moveForward, args=()).start()"
        else: Thread(target=drone.moveForward, args=()).start()
    elif com == 'a':
        if dry_run: print "Thread(target=drone.moveLeft, args=()).start()"
        else: Thread(target=drone.moveLeft, args=()).start()
    elif com == 's':
        if dry_run: print "Thread(target=drone.moveBackward, args=()).start()"
        else: Thread(target=drone.moveBackward, args=()).start()
    elif com == 'd':
        if dry_run: print "Thread(target=drone.moveRight, args=()).start()"
        else: Thread(target=drone.moveRight, args=()).start()
    elif com == 'x':
        if dry_run: print "Thread(target=drone.hover, args=()).start()"
        else: Thread(target=drone.hover, args=()).start()
    return in_list

def drone_init(drone):
    # Drone initialization steps
    drone.startup()
    drone.reset()
    drone.useDemoMode(False)
    print_bat(drone)

def main():
    global quitting
    drone = ps_drone.Drone()
    drone_init(drone)

    # Get home GPS coordinates.
    home = get_nav(drone, ["gps"])["gps"][:-1]
    print "Home: {}".format(home)
    drone_cal(drone)
    
    # Call sensor printing thread.
    stats = Thread(target=time_stat, args=([drone]))
    stats.setDaemon(True)
    stats.start()
    
    # Begin watching input for flight commands.
    read_list = [sys.stdin]
    while read_list:
        # get keypress and act
        ready = select.select(read_list, [], [], 0.1)[0]
        if ready:
            for f in ready:
                read_list = drone_act(drone, read_list, f.readline()[0])
    
    # Clean shutdown, close printing thread, print battery.
    quitting = True
    print_bat(drone)

main()
