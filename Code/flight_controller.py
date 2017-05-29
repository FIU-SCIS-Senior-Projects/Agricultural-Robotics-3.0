import Tkinter as tk
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


	def __init__(self,root):
		self.root = root
		self.root.title("D.F.C. - Drone Flight Controller")

		self.label = tk.Label(root, text="Drone Controller")
		self.label.pack()

		self.take_off = tk.Button(self.root, text="Launch", command=self.take_off)
		self.take_off.pack()

		self.shutdown = tk.Button(self.root, text="Shutdown", command=self.shutdown)
		self.shutdown.pack()

		self.quit_button = tk.Button(self.root, text="Quit GUI", command=root.quit)
		self.quit_button.pack()

	def take_off(self):
		print "Launch in process..."

	def shutdown(self):
		print "Shutdown in process..."

	def quit_GUI(self):
		print "Exiting GUI"

def main():
	root = tk.Tk()
	drone_GUI = DroneController(root)
	root.mainloop()

main()


		

	
