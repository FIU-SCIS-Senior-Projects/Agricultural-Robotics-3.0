import os, ps_drone, select, sys, time, math, thread
from threading import Thread

ACC = 6             # magnetometer normalization accuracy; higher = better
P_TIME = 0.1        # time to wait between sensor data readings
DRY_RUN = True      # debug mode, prints flight commands rather than fly
quitting = False    # global for printing thread

def get_stat(drone):
    # Return list of human-readable sensor data.
    nav_data = get_nav(drone)
    mag_avg = [-24.285, -35.047] # Manually calculated

    # Straightforward data
    acc = nav_data["raw_measures"][0]
    gyr = nav_data["raw_measures"][1]
    gps = nav_data["gps"][:-1]      # not using altitude value

    # Convert altitude to meters
    alt = nav_data["altitude"][0]   # mm
    alt = alt / 1000.0              # m

    # Turn magnetometer data into heading (degrees)
    mag = nav_data["magneto"][0][:-1] # not using z value
    for i in range(len(mag)): mag[i] -= mag_avg[i]
    deg = -1 * (math.atan2(mag[1], mag[0]) * 180) / math.pi

    return [acc, gyr, mag, deg, gps, alt]

def get_nav(drone):
    NDC = drone.NavDataCount
    while drone.NavDataCount == NDC: time.sleep(0.01)
    return drone.NavData

def cal_mag(drone):
    # Rotates the drone to acquire mag data to normalize.
    # NOT TESTED
    mag_x, mag_y, mag_avg = [], [], []
    for i in range(ACC):
        mag = get_nav(drone)["magneto"]
        mag_x.append(mag[0])
        mag_y.append(mag[1])
        drone.turnAngle((360.0 / ACC), 0.5)
        time.sleep(2)
    mag_avg.append(reduce(lambda x, y: x + y, mag_x) / len(mag_x))
    mag_avg.append(reduce(lambda x, y: x + y, mag_y) / len(mag_y))
    return mag_avg

def cal_drone(drone):
    # Basic gyroscope and magnetometer recalibration.
    # Requires 10 seconds of hovering flight.
    if DRY_RUN: print "drone.trim()"
    else: drone.trim()
    time.sleep(5)
    if DRY_RUN: print "drone.takeoff()"
    else: drone.takeoff()
    time.sleep(5)
    if DRY_RUN: print "drone.mtrim()"
    else: drone.mtrim()
    time.sleep(5)
    if DRY_RUN: print "drone.land()"
    else: drone.land()

def print_bat(drone):
    # Poll for valid battery status and print it.
    battery = drone.getBattery()
    while battery[0] == -1:
        time.sleep(0.1)
        battery = drone.getBattery()
    drone.printBlue("Battery: {}% {}".format(battery[0], battery[1]))

def time_stat(drone):
    # Timed printing of sensor data for testing.
    # This is only used in a threaded process.
    global quitting
    old_stats = [0, 0, 0, 0, 0, 0]
    timer, counter = time.time(), 0
    stat_timer, old_time = timer, timer
    while not quitting:
        curr_time = time.time()

        # Print new sensor data every P_TIME seconds
        if (curr_time - stat_timer > P_TIME):
            curr_stats = get_stat(drone)
            print "counter: {}".format(counter)
            print "acc: {}\ngyr: {}\nmag: {}".format(
                    curr_stats[0], curr_stats[1], curr_stats[2])
            print "deg: {}\ngps: {}\nalt: {}m\n\n".format(
                    curr_stats[3], curr_stats[4], curr_stats[5])

            # Check for bad magnetometer and shut down if so.
            if curr_stats[2] == old_stats[2]:
                if curr_time - old_time > 10:
                    drone.shutdown()
                    thread.interrupt_main()
                old_time = curr_time
            stat_timer = curr_time
            old_stats = curr_stats
            counter += 1

def simple_flight(drone):
    # An 8-second flight where it moves forward then lands.
    if DRY_RUN: print "drone.takeoff()"
    else: drone.takeoff()
    time.sleep(5)
    if DRY_RUN: print "drone.setSpeed(0.2)"
    else: drone.setSpeed(0.2)
    time.sleep(1)
    if DRY_RUN: print "drone.moveForward()"
    else: drone.moveForward()
    time.sleep(2)
    if DRY_RUN: print "drone.stop()"
    else: drone.stop()
    if DRY_RUN: print "drone.land()"
    else: drone.land()

def drone_act(drone, in_list, com):
    # Check character 'com' for valid command,
    # otherwise ignore it.
    if com == 'z':
        in_list.remove(in_list[0])
        drone.shutdown()
    elif com == 'f':
        Thread(target=simple_flight, args=(drone,)).start()
    elif com == 'w':
        if DRY_RUN: print "Thread(target=drone.moveForward, args=()).start()"
        else: Thread(target=drone.moveForward, args=()).start()
    elif com == 'a':
        if DRY_RUN: print "Thread(target=drone.moveLeft, args=()).start()"
        else: Thread(target=drone.moveLeft, args=()).start()
    elif com == 's':
        if DRY_RUN: print "Thread(target=drone.moveBackward, args=()).start()"
        else: Thread(target=drone.moveBackward, args=()).start()
    elif com == 'd':
        if DRY_RUN: print "Thread(target=drone.moveRight, args=()).start()"
        else: Thread(target=drone.moveRight, args=()).start()
    elif com == 'x':
        if DRY_RUN: print "Thread(target=drone.hover, args=()).start()"
        else: Thread(target=drone.hover, args=()).start()
    elif com == 'c':
        if DRY_RUN: print "Thread(target=cal_drone, args=(drone,)).start()"
        else: Thread(target=cal_drone, args=(drone,)).start()
    return in_list

def drone_init(drone):
    # Drone initialization steps
    drone.startup()
    drone.reset()
    drone.useDemoMode(False)
    drone.getNDpackage(["altitude", "gps", "magneto", "raw_measures"])
    print_bat(drone)

def main():
    global quitting
    drone = ps_drone.Drone()
    drone_init(drone)

    # Get home GPS coordinates.
    home = get_nav(drone)["gps"][:-1]
    print "Home: {}".format(home)
    
    # Call sensor printing thread.
    stats = Thread(target=time_stat, args=(drone,))
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
    
    # Clean shutdown, close printing thread
    quitting = True

main()
