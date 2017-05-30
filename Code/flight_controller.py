import Tkinter as tk
from Tkinter import *
import ps_drone
import time
import sys
import ScrolledText


'''
	D.F.C.-Drone Flight Controller

	Read stdin from user input, place in buffer.
	Read from buffer the commands to control drone
	Drone movements will be executed using Absolute
	movement function from PS_DRONE API.
	
	Function definition:
		def move(self, leftright, backwardforward, downup, turnleftright)
	Values between 0 - 1.0 either positive or negative.

	First iteration: 
		Instantiate drone object to communicate with drone(Functional throughout all iterations)
		Controller window: 
			Start button (initial flight startup)
			Stop button (land, shutdown)


'''


class DroneController(object):

	#def stop_moving():
	#	print "Stop moving"

	#def move_forward():
	#	print "Move forward"

	#def move_backward():
	#	print "Move backward"

	#def move_left():
	#	print "Move left"

	#def move_right():
	#	print "Move right"

	#def turn_left():
	#	print "Turn left"

	#def turn_right():
	#	print "Turn right"


	def __init__(self,root,drone):

		#Drone 
		self.drone = drone
		print self.drone
		self.drone.startup()
		self.drone.reset()

		self.drone.useDemoMode(False)
		time.sleep(0.5)

		#GUI 
		self.root = root
		self.root.title("D.F.C. - Drone Flight Controller")

		self.frame = tk.Frame(root, width=100)
		self.frame.grid(row=2, column=0, columnspan=30, rowspan=30)

		self.label = tk.Label(root, text="Drone Controller")
		self.label.grid(row=0, column=1)

		
		self.batt_status = tk.Button(self.root, text="Battery", command=self.batt_status)
		self.batt_status.grid(row=1, column=1)

		self.take_off = tk.Button(self.root, text="Launch", command=self.take_off)
		self.take_off.grid(row=2, column=0)

		self.take_off = tk.Button(self.root, text="Manuever1", command=self.man_1)
		self.take_off.grid(row=3, column=0)

		self.shutdown = tk.Button(self.root, text="Shutdown", command=self.shutdown)
		self.shutdown.grid(row=2, column=2)

		self.quit_button = tk.Button(self.root, text="Quit GUI", command=root.quit)
		self.quit_button.grid(row=4,column=1)
		

	def batt_status(self):
		self.drone.printBlue("Battery: "+str(self.drone.getBattery()[0])+"%  "+str(self.drone.getBattery()[1]))	# Gives a battery status

	def take_off(self):
		print "Launch in process..."
		self.drone.takeoff()

	def man_1(self):
		print "scripted maneuver 1"

	def shutdown(self):
		print "Shutdown in process..."
		self.drone.land()
		self.drone.shutdown()

	def quit_GUI(self):
		print "Exiting GUI"

def main():
	drone = ps_drone.Drone()		#Drone object
	root = tk.Tk()
	drone_GUI = DroneController(root, drone)
	root.mainloop()

main()


		

	
