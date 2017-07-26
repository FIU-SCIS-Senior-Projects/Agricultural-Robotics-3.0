from Tkinter import *
import ps_drone
import time
#import sys
#from itertools import islice
#from subprocess import Popen, PIPE
#from textwrap import dedent
#import threading



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

class DCMainApp(object):

	def __init__(self,root,drone):

		self.drone = drone		#Drone object
		self.root = root
		self.root.title("D.F.C. - Drone Flight Controller")

		self.startupside1 = tk.Frame(self.root)					#Mainwindow left
		self.startupside1.grid(row=0, column=0, columnspan=3, rowspan=30)
		self.startupside1.config(width=200, height=600, background="lightgrey")

		self.controllerside = tk.Frame(self.root, background="lightslategrey")	#Mainwindow right
		self.controllerside.config(width=700, height=600)
		self.controllerside.grid(row=0, column=3, columnspan=30, rowspan=30)


		######################################Batt/Alt/Vel##################################################
		self.battdis = tk.Label(self.root, text=" ", fg='blue', bg='lightgrey')
		self.battdis.config(font=('Arial',15,'bold'))
		self.battdis.grid(row=2, column=1)

		self.altdis = tk.Label(self.root, text=" ", fg='blue', bg='lightgrey')
		self.altdis.config(font=('Arial',15,'bold'))
		self.altdis.grid(row=4, column=1)

		self.veldis = tk.Label(self.root, text=" ")
		self.veldis.config(font=('Arial', 15, 'bold'), fg='blue', bg='lightgrey')
		self.veldis.grid(row=6, column=1)

		self.gpsdis = tk.Label(self.root, text=" ")
		self.gpsdis.config(font=('Arial', 15, 'bold'), fg='blue', bg='lightgrey')
		self.gpsdis.grid(row=8, column=1)

		self.pushbat = tk.Button(self.root, text="Activate Sensors Display", highlightbackground="lightgrey", command=self.senActivate)
		self.pushbat.grid(row=1, column=1)
		###################################Drone startup/shutdown##############################################

		self.dronetakeoff = tk.Button(self.root, text="Launch", highlightbackground="lightslategrey", command=self.take_off)
		self.dronetakeoff.config(width=9,font=('Arial',12,'bold'))
		self.dronetakeoff.grid(row=4, column=6)

		self.droneland = tk.Button(self.root, text="Land", highlightbackground="lightslategrey",command=self.d_land)
		self.droneland.config(width=9,font=('Arial',12,'bold'))
		self.droneland.grid(row=4, column=12)

		self.droneshutdown = tk.Button(self.root, text="Shutdown", highlightbackground="lightslategrey", command=self.shutdown)
		self.droneshutdown.config(width=9,font=('Arial',12,'bold'))
		self.droneshutdown.grid(row=8, column=6)

		self.GUIquit = tk.Button(self.root, text="Quit GUI", highlightbackground="lightslategrey", command=self.quit)
		self.GUIquit.config(width=9, font=('Arial',12,'bold'))
		self.GUIquit.grid(row=26,column=12)

		####################################Control pad:Forward/Backward Controls##############################

		self.dr_Control = tk.Label(self.root, text="Flight\nController", bg="lightslategrey")
		self.dr_Control.config(font=('Arial',24,'bold'), fg='black')
		self.dr_Control.grid(row=1, column=10)

		self.dronemoveup = tk.Button(self.root, text="Move Up", highlightbackground="lightslategrey",command=self.d_moveup)
		self.dronemoveup.config(width=10,font=('Arial',12,'bold'))
		self.dronemoveup.grid(row=4,column=10)

		self.droneforward = tk.Button(self.root, text="Forward", highlightbackground="lightslategrey",command=self.d_forward)
		self.droneforward.config(width=10,font=('Arial',12,'bold'))
		self.droneforward.grid(row=5, column=10)

		self.dronehover = tk.Button(self.root, text="Hover", highlightbackground="lightslategrey",command=self.d_hover)
		self.dronehover.config(width=10,font=('Arial',12,'bold'))
		self.dronehover.grid(row=6, column=10)

		self.droneback = tk.Button(self.root, text="Backward", highlightbackground="lightslategrey",command=self.d_backward)
		self.droneback.config(width=10,font=('Arial',12,'bold'))
		self.droneback.grid(row=7, column=10)

		self.dronemovedown = tk.Button(self.root, text="MoveDown", highlightbackground="lightslategrey",command=self.d_movedown)
		self.dronemovedown.config(width=10,font=('Arial',12,'bold'))
		self.dronemovedown.grid(row=8,column=10)

		####################################Control pad:Left/Right Controls####################################

		self.droneturnleft = tk.Button(self.root, text="TurnLeft ", highlightbackground="lightslategrey",command=self.d_turnleft)
		self.droneturnleft.config(width=9,font=('Arial',12,'bold'))
		self.droneturnleft.grid(row=6, column=6)

		self.droneleft = tk.Button(self.root, text="Left ", highlightbackground="lightslategrey",command=self.d_left)
		self.droneleft.config(width=4,font=('Arial',12,'bold'))
		self.droneleft.grid(row=6,column=7)

		self.droneright = tk.Button(self.root, text="Right", highlightbackground="lightslategrey",command=self.d_right)
		self.droneright.config(width=4,font=('Arial',12,'bold'))
		self.droneright.grid(row=6,column=11)

		self.droneturnright = tk.Button(self.root, text="TurnRight", highlightbackground="lightslategrey",command=self.d_turnright)
		self.droneturnright.config(width=9,font=('Arial',12,'bold'))
		self.droneturnright.grid(row=6, column=12)

	###################################GUI Drone button functions########################################
	def senActivate(self):
		self.battstat()
		self.altstat()
		self.velstat()
		self.gpsstat()

	def battstat(self):
		if str(self.drone.getBattery()[1]) != "OK":
			battDisplay = "Battery: "+str(self.drone.getBattery()[0])+ "% " + "\nState: " +str(self.drone.getBattery()[1])# Battery update variable
			self.battdis.config(text=battDisplay, fg='red')
			self.root.after(600, self.battstat)
		else:
			battDisplay = "Battery: "+str(self.drone.getBattery()[0])+ "% " + "\nState: " +str(self.drone.getBattery()[1])# Battery update variable
			self.battdis.config(text=battDisplay)
			self.root.after(600, self.battstat)

	def altstat(self):
		altDisplay = "Altitude: "+str(self.drone.NavData['altitude'][3]/10)+" cm"
		self.altdis.config(text=altDisplay)
		self.root.after(200, self.altstat)

	def velstat(self):
		velDisplay = "X Velocity: "+str(int(self.drone.NavData['demo'][4][0]/10)) + " cm/s" + "\nY Velocity: "+str(int(self.drone.NavData['demo'][4][1]/10)) + " cm/s"
		self.veldis.config(text=velDisplay)
		self.root.after(200, self.velstat)

	def gpsstat(self):
		gpsDisplay = "Latitude: "+str(self.drone.NavData['gps'][0]) + "\nLongitude: "+str(self.drone.NavData['gps'][1])
		self.gpsdis.config(text=gpsDisplay)
		self.root.after(200, self.gpsstat)

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


def fdrone__init(drone):
	drone.startup()
	drone.reset()

	drone.trim()
	drone.useDemoMode(False)
	drone.getNDpackage(["demo","altitude","gps"])
	time.sleep(2)

	NDC = drone.NavDataCount
	while NDC == drone.NavDataCount: time.sleep(0.0001)
	while drone.getBattery()[0] == -1: time.sleep(0.01)


def main():
	drone = ps_drone.Drone()		#Drone object
	fdrone__init(drone)
	root = tk.Tk()
	root.geometry("900x600")		#GUI window dimensions
	drone_GUI = DCMainApp(root, drone)
	#root.protocol("WM_DELETE_WINDOW", drone_GUI.quit)
	root.mainloop()


if __name__ == "__main__":
	main()
