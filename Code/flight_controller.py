from Tkinter import *
import ps_drone
import time


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


class DroneController():
	
	



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


	def __init__(self,master):
		self.master = master
		master.title("D.F.C. - Drone Flight Controller")

		self.label = Label(master, text="Simple Controller!")
		self.label.pack()

		self.take_off = Button(master, text="Launch", bg='green', command=self.take_off)
		self.take_off.pack()

		self.close_button = Button(master, text="Shutdown", command=master.quit)
		self.close_button.pack()

	def take_off(self):
		print "Clicked takeoff"


main = Tk()
drone_control = DroneController(main)
main.mainloop()


if __name__ == "__main__":
	drone = ps_drone.Drone()
	#drone.startup()

	#drone.reset()
	#drone.useDemoMode(False)
	time.sleep(0.5)
	print "Drone Controller accessing..."
		

	
