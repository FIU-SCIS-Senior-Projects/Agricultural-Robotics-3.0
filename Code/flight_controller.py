from Tkinter import *
import ps_drone
import time
import sys
from itertools import islice
from subprocess import Popen, PIPE
from textwrap import dedent
from threading import Thread  



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

		######################################Variables########################################################	
		battstat = StringVar()		# battery variable for battery status display
		battstat.set(' ')		

		test = BooleanVar()
		test.set(False)

		userIn = StringVar()		# input variable for command keys
		#controltext='''Control keys: \n"7": turnAngle(-10,1) \n"9": turnAngle( 10,1)\n"4": turnAngle(-45,1) \n"6": turnAngle( 45,1) \n"1": turnAngle(-90,1) \n"3": turnAngle( 90,1) \n"*": doggyHop() \n"+": doggyNod() \n"-": doggyWag()'''

		#########################################Drone Object##################################################
		self.drone = drone
		#self.drone.startup()
		#self.drone.reset()

		#self.drone.useDemoMode(False)
		#self.drone.getNDpackage(["demo"])
		time.sleep(0.5)


		#GUI Object instance 
		self.root = root
		self.root.title("D.F.C. - Drone Flight Controller")

	
		######################################Main GUI Window##################################################
		self.startupside1 = tk.Frame(self.root)
		self.startupside1.grid(row=0, column=0, columnspan=2, rowspan=30)
		self.startupside1.config(width=400, height=600, background="lightgrey")

		self.controllerside = tk.Frame(self.root, background="lightslategrey")
		self.controllerside.config(width=500, height=600)
		self.controllerside.grid(row=0, column=3, columnspan=30, rowspan=30)


		###################################Drone startup/shutdown##############################################	
		
		self.takeoff = tk.Button(self.root, text="Launch", highlightbackground="lightslategrey", command=self.take_off)
		self.takeoff.config(width=9,font=('Arial',12,'bold'))
		self.takeoff.grid(row=4, column=6)

		self.drone_land = tk.Button(self.root, text="Land", highlightbackground="lightslategrey",command=self.d_land)
		self.drone_land.config(width=9,font=('Arial',12,'bold'))
		self.drone_land.grid(row=4, column=12)

		self.shutdown = tk.Button(self.root, text="Shutdown", highlightbackground="lightslategrey", command=self.shutdown)
		self.shutdown.config(width=9,font=('Arial',12,'bold'))
		self.shutdown.grid(row=8, column=6)

		self.battlabel = tk.Button(self.root,text="Battery",highlightbackground="lightslategrey", command=self.batt_status)
		self.battlabel.config(width=9,font=('Arial',12,'bold'))
		self.battlabel.grid(row=8, column=12)

		self.quit_button = tk.Button(self.root, text="Quit GUI", highlightbackground="lightslategrey", command=self.quit)
		self.quit_button.config(width=9, font=('Arial',12,'bold'))
		self.quit_button.grid(row=26,column=12)
		
		####################################Control pad########################################################

		####################################Forward/Backward Controls##########################################
		
		self.dr_Control = tk.Label(self.root, text="Flight\nController", bg="lightslategrey")
		self.dr_Control.config(font=('Arial',24,'bold'), fg='black')
		self.dr_Control.grid(row=1, column=10)

		self.drone_moveup = tk.Button(self.root, text="Move Up", highlightbackground="lightslategrey",command=self.d_moveup)
		self.drone_moveup.config(width=10,font=('Arial',12,'bold'))
		self.drone_moveup.grid(row=4,column=10)	

		self.drone_forward = tk.Button(self.root, text="Forward", highlightbackground="lightslategrey",command=self.d_forward)
		self.drone_forward.config(width=10,font=('Arial',12,'bold'))
		self.drone_forward.grid(row=5, column=10)

		self.drone_hover = tk.Button(self.root, text="Hover", highlightbackground="lightslategrey",command=self.d_hover)
		self.drone_hover.config(width=10,font=('Arial',12,'bold'))
		self.drone_hover.grid(row=6, column=10)

		self.drone_back = tk.Button(self.root, text="Backward", highlightbackground="lightslategrey",command=self.d_backward)
		self.drone_back.config(width=10,font=('Arial',12,'bold'))
		self.drone_back.grid(row=7, column=10)
	
		self.drone_movedown = tk.Button(self.root, text="MoveDown", highlightbackground="lightslategrey",command=self.d_movedown)
		self.drone_movedown.config(width=10,font=('Arial',12,'bold'))
		self.drone_movedown.grid(row=8,column=10)

		####################################Left/Right Controls################################################

		self.drone_turnleft = tk.Button(self.root, text="TurnLeft ", highlightbackground="lightslategrey",command=self.d_turnleft)
		self.drone_turnleft.config(width=9,font=('Arial',12,'bold'))
		self.drone_turnleft.grid(row=6, column=6)
		
		self.drone_left = tk.Button(self.root, text="Left ", highlightbackground="lightslategrey",command=self.d_left)
		self.drone_left.config(width=4,font=('Arial',12,'bold'))
		self.drone_left.grid(row=6,column=7)		

		self.drone_right = tk.Button(self.root, text="Right", highlightbackground="lightslategrey",command=self.d_right)
		self.drone_right.config(width=4,font=('Arial',12,'bold'))
		self.drone_right.grid(row=6,column=11)

		self.drone_turnright = tk.Button(self.root, text="TurnRight", highlightbackground="lightslategrey",command=self.d_turnright)
		self.drone_turnright.config(width=9,font=('Arial',12,'bold'))
		self.drone_turnright.grid(row=6, column=12)
		
		#######################################Drone Video Feed################################################

		self.dr_VFLabel1 = tk.Label(self.root, text="Video\nFeed", bg="lightgrey", anchor='w')
		self.dr_VFLabel1.config(font=('Arial',24,'bold'), fg="black")
		self.dr_VFLabel1.grid(row=1, column=1)	

		self.dr_VFLabel1 = tk.Canvas(self.root, width=300, height=200, bg="black")
		self.dr_VFLabel1.grid(row=2, column=1)	


		self.vs_on = tk.Button(self.root, text="Video On", highlightbackground="lightgrey")
		self.vs_on.config(width=9,font=('Arial',12,'bold'))
		self.vs_on.grid(row=3, column=1)

		self.vs_off = tk.Button(self.root, text="Video Off", highlightbackground="lightgrey")
		self.vs_off.config(width=9,font=('Arial',12,'bold'))
		self.vs_off.grid(row=4, column=1)

		#######################################################################################################

		#Text input

		self.user_Entry = tk.Label(self.root, text="Drone\nCommand", bg="lightgrey")
		self.user_Entry.grid(row=10, column=1)
		self.user_Entry.config(font=('Arial',14, 'bold'))
		self.user_EntryBox = tk.Entry(self.root, textvariable=userIn, highlightbackground="lightgrey")
		self.user_EntryBox.config(width=10) 
		self.user_EntryBox.grid(row=11,column=1)

#		self.ueButton = StringVar()
#		self.ueButton.set("Send Command")
		
#		self.button = tk.Button(self.root, textvariable=self.ueButton, highlightbackground="lightgrey", command=self.commandinput)
#		self.button.grid(row=10, column=0)
		

		#self.canvastext = tk.Label(self.frame,text=controltext, font=("Helvetica",10), fg="red", anchor=W, justify=LEFT)
		#self.canvastext.grid(row=11, column=0)
		
		#generic process to generate output. stdout to gui
		#self.process = Popen([sys.executable, "-u", "-c", dedent("""
		#	import itertools, time
		#
		#	for i in itertools.count():
		#		print("%d.%d" % divmod(i,10))
		#		time.sleep(0.1)
		#	""")], stdout=PIPE)
		
#		self.process = Popen(['python', /Agricultural-Robotics-3.0/Code/flight_controller.py], stdout=self.process.PIPE,stderr=self.process.STDOUT,)	
#		for line in iter(self.process.stdout.readline, ''):	
#			print("%s") % line	

#		q = Queue(maxsize=1024)
#		t = Thread(target=self.reader_thread, args=[q])
#		t.daemon = True
#		t.start()
	
		#stdout to gui 
#	def reader_thread(self, q):
#		try:
#			with self.process.stdout as pipe:
#				for line in iter(pipe.readline, b''):
#					q.put(line)
#		finally:
#			q.put(None)


#		#stdout to gui
#	def update(self, q):
#		for line in iter_except(q.get_nowait, Empty):
#			if line is None:
#				self.quit()
#				return
#			else:
#				self.label['text'] = line
#				break
#		self.root.after(40, self.update, q)
#
#	#stdout to gui
#	def quit(self):
#		self.process.kill()
#		self.root.destroy()
#
#	
#	def commandinput(self):
#		inputComm = self.user_Entry.get()
#		self.user_Entry.delete(0, END)
#		
#		#ps_drone.py "Playground" algorithm
#		key = inputComm			

#		elif key == "7":	self.drone.turnAngle(-10,1)
#		elif key == "9":	self.drone.turnAngle( 10,1)
#		elif key == "4":	self.drone.turnAngle(-45,1)
#		elif key == "6":	self.drone.turnAngle( 45,1)
#		elif key == "1":	self.drone.turnAngle(-90,1)
#		elif key == "3":	self.drone.turnAngle( 90,1)
#		elif key == "*":	self.drone.doggyHop()
#		elif key == "+":	self.drone.doggyNod()
#		elif key == "-":	self.drone.doggyWag()
#		else: key = ""
		

	###################################GUI Drone button functions##################################################
	def batt_status(self):
		#print "In battery function"
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
		self.drone.moveForward()

	def d_backward(self):
		self.drone.moveBackward()

	def d_left(self):
		self.drone.moveLeft()

	def d_right(self):
		self.drone.moveRight()

	def d_turnright(self):
		xr = 0.50
		self.drone.turnRight(xr)

	def d_turnleft(self):
		xl = 0.50
		self.drone.turnLeft(xl)

	def d_moveup(self):
		self.drone.moveUp()

	def d_movedown(self):
		self.drone.moveDown()

	def quit(self):
		self.drone.shutdown	# Land drone and discard drone object
		self.root.destroy()	# Discard Main window object
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


		

	
