from Tkinter import *
import ps_drone

# Drone flight_controller

class Controller():
	
	def take_off():
		print "Clicked takeoff"

	def stop_moving():
		print "Stop moving"

	def move_forward():
		print "Move forward"

	def move_backward():
		print "Move backward"

	def move_left():
		print "Move left"

	def move_right():
		print "Move right"

	def turn_left():
		print "Turn left"

	def turn_right():
		print "Turn right"



	main = Tk()
	main.title("D.F.C. - Drone Flight Controller")
	frame = Frame(main)
	frame.pack()	

	main.bind('<space>', take_off())
	main.bind('0', stop_moving())
	main.bind('w', move_forward())
	main.bind('s', move_backward())
	main.bind('a', move_left())
	main.bind('d', move_right())
	main.bind('q', turn_left())
	main.bind('e', turn_right())


	main.mainloop()

class DroneINIT():
	drone = ps_drone.Drone()
	#drone.startup()

	Controller()



if __name__ == "__main__":
	DroneINIT()	
