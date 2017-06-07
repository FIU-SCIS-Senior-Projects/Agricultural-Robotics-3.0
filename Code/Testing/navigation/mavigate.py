from ps_drone import Drone
from navigator import Navigator
from threading import Thread
from dronekit import connect, VehicleMode
import dronekit_sitl

sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()
vehicle = connect(connection_string, wait_ready = True)

print "Making drone"
drone = Drone()

print "Startup"
drone.startup()

print "Reset"
drone.reset()

print "Navigator"
navigator = Navigator(drone)

print "Shutting down..."
drone.shutdown()
vehicle.close()
sitl.stop()

print "done"

