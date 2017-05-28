
import ps_drone			#imports PS_DRONE API
from Tkinter import *


# Drone flight_controller

class Controller():
	
	def take_off(event):
		print "Clicked takeoff"

	def stop_moving(event):
		print "Stop moving"

	def move_forward(event):
		print "Move forward"

	def move_backward(event):
		print "Move backward"

	def move_left(event):
		print "Move left"

	def move_right(event):
		print "Move right"

	def turn_left(event):
		print "Turn left"

	def turn_right(event):
		print "Turn right"



	main = Tk()
	main.title("D.F.C. - Drone Flight Controller")

	frame = Frame(main, width=100, height=100)
	frame.bind("<Key>", a)
	frame.pack()


	main.mainloop()

class DroneINIT():
	drone = ps_drone.Drone()
	#drone.startup()

	Controller()



if __name__ == "__main__":
	DroneINIT()	
