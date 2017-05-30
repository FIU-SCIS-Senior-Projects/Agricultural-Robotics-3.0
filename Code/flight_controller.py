from Tkinter import *
import ps_drone
import time
import sys
import ScrolledText import *
from itertools import islice
from subprocess import Popen, PIPE
from textwrap import dedent
from threading import Thread 


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


	stdout to gui window code borrowed author: Sebastian, J.F., "https://stackoverflow.com/questions/665566/redirect-command-line-results-to-a-tkinter-gui"

'''

try:
	import Tkinter as tk		#python 2 support
	from Queue import Queue, Empty	#python 2 support
except ImportError:
	import tkinter as tk 		#python 3 support
	from queue import Queue, Empty	#python 3 support


def iter_except(function,exception):
	try:
		while True:
			yield function()
	except	exception:
		return


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
		
		#generic process to generate output. stdout to gui
		#self.process = Popen([sys.executable, "-u", "-c", dedent("""
		#	import itertools, time
		#
		#	for i in itertools.count():
		#		print("%d.%d" % divmod(i,10))
		#		time.sleep(0.1)
		#	""")], stdout=PIPE)

		self.process = Popen(['python', /Agricultural-Robotics-3.0/Code/flight_controller.py], stdout=self.process.PIPE,stderr=self.process.STDOUT,)	
		for line in iter(self.process.stdout.readline, ''):	
			print("%s") % line	

		q = Queue(maxsize=1024)
		t = Thread(target=self.reader_thread, args=[q])
		t.daemon = True
		t.start()


		self.frame = tk.Frame(self.root, width=500)
		self.frame.grid(row=2, column=0, columnspan=30, rowspan=30)

		self.label = tk.Label(self.root, text="Drone Controller")
		self.label.grid(row=0, column=2)

		self.label = tk.Label(self.root, text= "  ", font=(None, 12))	#stdout to gui
		self.label.grid(row=1, column=2)				#stdout to gui
		
		#Text box for flight commands list###
		self.textpad = tk.Frame(self.frame)
		self.text = tk.Text(textpad, height=50, width=90)

		self.scroll= tk.ScrollBar(textpad)
		self.text.configure(yscrollcommand=yscroll.set)

		self.text.pack(side=LEFT)
		self.scroll.pack(side=RIGHT,fill=Y)
		self.textpad.pack(side=TOP)
		#####################################		

		self.batt_status = tk.Button(self.root, text="Battery", command=self.batt_status)
		self.batt_status.grid(row=1, column=1)
		
		self.testcomm = tk.Button(self.root, text="Test LED", command=self.testcomm)
		self.testcomm.grid(row=4, column=0)

		self.take_off = tk.Button(self.root, text="Launch", command=self.take_off)
		self.take_off.grid(row=2, column=0)

		self.take_off = tk.Button(self.root, text="Manuever1", command=self.man_1)
		self.take_off.grid(row=3, column=0)

		self.shutdown = tk.Button(self.root, text="Shutdown", command=self.shutdown)
		self.shutdown.grid(row=2, column=3)

		self.quit_button = tk.Button(self.root, text="Quit GUI", command=root.quit)
		self.quit_button.grid(row=3,column=3)

		self.update(q)	#stdout to gui
		
	#stdout to gui 
	def reader_thread(self, q):
		try:
			with self.process.stdout as pipe:
				for line in iter(pipe.readline, b''):
					q.put(line)
		finally:
			q.put(None)


	#stdout to gui
	def update(self, q):
		for line in iter_except(q.get_nowait, Empty):
			if line is None:
				self.quit()
				return
			else:
				self.label['text'] = line
				break
		self.root.after(40, self.update, q)



	#stdout to gui
	def quit(self):
		self.process.kill()
		self.root.destroy()


	def testcomm(self):
		self.drone.led(20,5,3)


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

	def quit(self):
		print "Exiting GUI"



def main():
	drone = ps_drone.Drone()		#Drone object
	root = tk.Tk()				#New stdout to gui window
	root.geometry("800x600")
	drone_GUI = DroneController(root, drone)
	root.protocol("WM_DELETE_WINDOW", drone_GUI.quit)
	root.mainloop()

main()


		

	
