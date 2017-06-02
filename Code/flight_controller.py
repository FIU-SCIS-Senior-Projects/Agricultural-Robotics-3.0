from Tkinter import *
import ps_drone
import time
import sys
from itertools import islice
from subprocess import Popen, PIPE
from textwrap import dedent
from threading import Thread  
import ScrolledText as tkst


'''
D.F.C.-Drone Flight Controller:

GUI that connects to Parrot AR 2.0 Drone and gives the user
control of the drone eliminating the need to run different 
script through the CLI.


Cited sources:

stdout to gui window code borrowed: 
Sebastian, J.F., "https://stackoverflow.com/questions/665566/redirect-command-line-results-to-a-tkinter-gui"

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
	
	
class dcMainApp(object):


	def __init__(self,root,drone):

		#Drone 
		self.drone = drone
		#self.drone.startup()
		#self.drone.reset()
		#self.drone.useDemoMode(False)
		time.sleep(0.5)
			

		#GUI 
		self.root = root
		self.root.title("D.F.C. - Drone Flight Controller")

	
		#Main frame
		self.frame = tk.Frame(self.root, background="lightgrey")
		self.frame.grid(row=0, column=0, columnspan=30, rowspan=30)
		self.frame.config(width=900, height=600)

		

		userIn = StringVar()		# input variable for command keys
		controltext='''Type key into textbox and press enter.\nControl keys: \n"7": turnAngle(-10,1) \n"9": turnAngle( 10,1)\n"4": turnAngle(-45,1) \n"6": turnAngle( 45,1) \n"1": turnAngle(-90,1) \n"3": turnAngle( 90,1) \n"*": doggyHop() \n"+": doggyNod() \n"-": doggyWag()'''


		self.battlabel = tk.Label(self.root, text="Battery: ", bg="lightgrey")
		self.battlabel.grid(row=0, column=0)
		
		self.take_off = tk.Button(self.root, text="Launch", highlightbackground="lightgrey", command=self.take_off)
		self.take_off.config(width=7)
		self.take_off.grid(row=4, column=3)

		self.drone_land = tk.Button(self.root, text="Land", highlightbackground="lightgrey", command=self.d_land)
		self.drone_land.config(width=7)
		self.drone_land.grid(row=5, column=3)

		self.shutdown = tk.Button(self.root, text="Shutdown", highlightbackground="lightgrey", command=self.shutdown)
		self.shutdown.config(width=7)
		self.shutdown.grid(row=6, column=3)

		self.quit_button = tk.Button(self.root, text="Quit GUI", highlightbackground="lightgrey", command=root.quit)
		self.quit_button.config(width=7)
		self.quit_button.grid(row=7,column=3)

		# Control pad
		self.drone_hover = tk.Button(self.root, text="Hover", highlightbackground="lightgrey", command=self.d_hover)
		self.drone_hover.config(width=12)
		self.drone_hover.grid(row=5, column=18)

		self.drone_forward = tk.Button(self.root, text="Forward", highlightbackground="lightgrey", command=self.d_forward)
		self.drone_forward.config(width=10)
		self.drone_forward.grid(row=4, column=18)

		self.drone_back = tk.Button(self.root, text="Backward", highlightbackground="lightgrey", command=self.d_backward)
		self.drone_back.config(width=10)
		self.drone_back.grid(row=6, column=18)

		self.drone_left = tk.Button(self.root, text="Left ", highlightbackground="lightgrey", command=self.d_left)
		self.drone_left.config(width=4)
		self.drone_left.grid(row=5,column=17)		

		self.drone_right = tk.Button(self.root, text="Right", highlightbackground="lightgrey", command=self.d_right)
		self.drone_right.config(width=4)
		self.drone_right.grid(row=5,column=19)

		self.drone_turnleft = tk.Button(self.root, text="Turn Left ", highlightbackground="lightgrey", command=self.d_turnleft)
		self.drone_turnleft.grid(row=5, column=16)

		self.drone_turnright = tk.Button(self.root, text="Turn Right", highlightbackground="lightgrey", command=self.d_turnright)
		self.drone_turnright.grid(row=5, column=20)

		self.drone_moveup = tk.Button(self.root, text="Move Up", highlightbackground="lightgrey", command=self.d_moveup)
		self.drone_moveup.config(width=8)
		self.drone_moveup.grid(row=3,column=18)		

		self.drone_movedown = tk.Button(self.root, text="Move Down", highlightbackground="lightgrey", command=self.d_movedown)
		self.drone_movedown.config(width=8)
		self.drone_movedown.grid(row=7,column=18)	


		#Text input
		self.user_Entry = tk.Entry(self.root, textvariable=userIn, highlightbackground="lightgrey") 
		self.user_Entry.grid(row=9,column=0)

		self.ueButton = StringVar()
		self.ueButton.set("Send Command")
		
		self.button = tk.Button(self.root, textvariable=self.ueButton, highlightbackground="lightgrey", command=self.commandinput)
		self.button.grid(row=10, column=0)
		

		#self.canvastext = tk.Label(self.frame,text=controltext, font=("Helvetica",10), fg="red", anchor=W, justify=LEFT)
		#self.canvastext.grid(row=11, column=0)
		'''
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

	'''
	def commandinput(self):
		inputComm = self.user_Entry.get()
		self.user_Entry.delete(0, END)
		
		#ps_drone.py "Playground" algorithm
		key = inputComm			
		
		if   key == "0":	self.drone.hover()
		elif key == "w":	self.drone.moveForward()
		elif key == "s":	self.drone.moveBackward()
		elif key == "a":	self.drone.moveLeft()
		elif key == "d":	self.drone.moveRight()
		elif key == "q":	self.drone.turnLeft()
		elif key == "e":	self.drone.turnRight()
		elif key == "7":	self.drone.turnAngle(-10,1)
		elif key == "9":	self.drone.turnAngle( 10,1)
		elif key == "4":	self.drone.turnAngle(-45,1)
		elif key == "6":	self.drone.turnAngle( 45,1)
		elif key == "1":	self.drone.turnAngle(-90,1)
		elif key == "3":	self.drone.turnAngle( 90,1)
		elif key == "8":	self.drone.moveUp()
		elif key == "2":	self.drone.moveDown()
		elif key == "*":	self.drone.doggyHop()
		elif key == "+":	self.drone.doggyNod()
		elif key == "-":	self.drone.doggyWag()
		else: key = ""

	

	def batt_status(self):
		while (self.drone.getBattery()[0]==-1):
			time.sleep(0.1)
		self.drone.printBlue("Battery: "+str(self.drone.getBattery()[0])+"%  "+str(self.drone.getBattery()[1]))	# Gives a battery status
		if self.drone.getBattery()[1]=="empty":
			sys.exit()

	def take_off(self):
		self.drone.takeoff()

	def d_land(self):
		self.drone.land()

	def shutdown(self):
		self.drone.shutdown()
	
	def d_hover(self):
		self.drone.hover()

	def d_forward(self):
		self.drone.moveforward()

	def d_backward(self):
		self.drone.movebackward()

	def d_left(self):
		self.drone.moveleft()

	def d_right(self):
		self.drone.moveright()

	def d_turnright(self):
		self.drone.turnright()

	def d_turnleft(self):
		self.drone.turnleft()

	def d_moveup(self):
		self.drone.moveup()

	def d_movedown(self):
		self.drone.movedown()

	def quit(self):
		self.drone.shutdown
		print "Exiting GUI"





def main():
	drone = ps_drone.Drone()		#Drone object
	root = tk.Tk()				
	root.geometry("900x600")		#GUI window dimensions
	drone_GUI = dcMainApp(root, drone)
	#root.protocol("WM_DELETE_WINDOW", drone_GUI.quit)
	root.mainloop()


if __name__ == "__main__":
	main()


		

	
