from Tkinter import *
import ps_drone
import time
import sys
from itertools import islice
from subprocess import Popen, PIPE
from textwrap import dedent
import threading



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
	



'''
# Real-time battery tk.Label value update
class batteryThread(object):
	def __init__(self, drone):
		self.drone=drone
		
	def batteryDisplay(self):
		while True:
			bDrone
			time.sleep(0.5)
		#after timesleep over write old value

# Real-time altitude tk.Label value update	
class altitudeThread(threading.Thread):
	def __init__(self, *args, **kwargs):

		threading.Thread.__init__(self, *args, **kwargs)
		self.daemon = True
		self.start()

# Real-time velocity tk.Label value update
class velocityThread(threading.Thread):
	def __init__(self, *args, **kwargs):

		threading.Thread.__init__(self, *args, **kwargs)
		self.daemon = True
		self.start()

'''
class dcMainApp(object):

	#def hidebutton(event):
	#	event.widget.grid_remove()

	def __init__(self,root,drone):


		#########################################Drone Object##################################################
		self.drone = drone
		self.drone.startup()
		self.drone.reset()

		self.drone.useDemoMode(False)
		self.drone.getNDpackage(["demo"])
		time.sleep(0.5)

		bDisplay = "Battery: "+str(self.drone.getBattery()[0])+"%  \nState: "+str(self.drone.getBattery()[1]) # Battery initial variable

		#GUI Object instance 
		self.root = root
		self.root.title("D.F.C. - Drone Flight Controller")

		######################################Main GUI Window########################################
		self.startupside1 = tk.Frame(self.root)
		self.startupside1.grid(row=0, column=0, columnspan=3, rowspan=30)
		self.startupside1.config(width=200, height=600, background="lightslategrey")

		self.controllerside = tk.Frame(self.root, background="lightslategrey")
		self.controllerside.config(width=700, height=600)
		self.controllerside.grid(row=0, column=3, columnspan=30, rowspan=30)


		######################################Batt/Alt/Vel##################################################
	
		self.bbattery = tk.Button(self.root, text=bDisplay, highlightbackground="lightslategrey", command=self.battstat)
		self.bbattery.config(font=('Arial',12,'bold'), height=5)
		self.bbattery.grid(row=8, column=12)
		
		
		#self.AltDis = tk.Label(self.root, text="Altitude: "+str(self.drone.getBattery()[0])+"%  "+str(self.drone.getBattery()[1]))
		#self.AltDis.config(font=('Arial', 14, 'bold'), fg='gold2', bg='lightslategrey')
		#self.AltDis.grid(row=2, column=6)

		#self.VelDis = tk.Label(self.root, text="Velocity: "+str(self.drone.getBattery()[0])+"%  "+str(self.drone.getBattery()[1]))
		#self.VelDis.config(font=('Arial', 14, 'bold'), fg='gold2', bg='lightslategrey')
		#self.VelDis.grid(row=3, column=6)


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
	def battstat(self):	
		bDisplay2 = "Battery: "+str(self.drone.getBattery()[0])+"% \nState: "+str(self.drone.getBattery()[1])# Battery update variable
		self.bbattery.config(text=bDisplay2)
		self.root.after(1000, self.battstat)

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


		

	
