from Tkinter import *
import ps_drone
#from itertools import islice
#from subprocess import Popen, PIPE
#from textwrap import dedent
import math, select, sys, time
from ps_drone import Drone
from navigator import Navigator
from threading import Thread
from multiprocessing import Process
import numpy as np
np.seterr(divide='ignore', invalid='ignore')

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
        import Tkinter as tk            #python 2 support
        from Queue import Queue, Empty  #python 2 support
except ImportError:
        import tkinter as tk            #python 3 support
        from queue import Queue, Empty  #python 3 support

class DCMainApp(object):
    def __init__(self,root):
        # Modifiable Constants
        self.win_height = 600
        self.win_width  = 900
        self.button_width = 9
        self.button_text_color = "blue"
        self.button_text_face = "Arial"
        self.button_text_size = 15
        self.button_text_style = "bold"
        self.sensor_color_back = "lightgrey"
        self.control_color_back = "lightslategrey"
        self.sensor_width_per = 0.25
        self.control_width_per = 1.0 - self.sensor_width_per
        self.out_text = ""

        # Derivative Constants
        self.sensor_width = self.sensor_width_per * self.win_width
        self.control_width = self.control_width_per * self.win_width
        self.button_text = (
                self.button_text_face, self.button_text_size, self.button_text_style)


        # Argument fields
        self.drone = None              #Drone object
        self.navigator = None      #Navigator object
        self.root = root
        self.root.title("D.F.C. - Drone Flight Controller")

        # Live data
        self.startupside1 = tk.Frame(self.root)    #Mainwindow left
        self.startupside1.grid(row=0, column=0, columnspan=3, rowspan=30)
        self.startupside1.config(width=self.sensor_width,
                height=self.win_height, background=self.sensor_color_back)

        # Controller
        self.controllerside = tk.Frame(self.root)  #Mainwindow right
        self.controllerside.grid(row=0, column=3, columnspan=30, rowspan=30)
        self.controllerside.config(width=self.control_width,
                height=self.win_height, background=self.control_color_back)

        # TODO ADDED IN MERGE, TO BE CLEANED
        self.landing = False
        self.connect_proc = None


        ######################################Batt/Alt/Vel##################################################
        self.sensor_objs = []
        self.sensor_objs_names = ["battdis", "altdis", "veldis", "gpsdis"]
        self.sensor_label_text = [" ", " ", " ", " "]

        for i in range(len(self.sensor_objs_names)):
            self.sensor_objs.append(tk.Label(
                    self.root,
                    text=self.sensor_label_text[i],
                    fg=self.button_text_color,bg=self.sensor_color_back))
            self.sensor_objs[i].config(font=self.button_text)
            self.sensor_objs[i].grid(row=i+2, column=1)

        self.pushbat = tk.Button(self.root, text="Activate Sensors Display",
                highlightbackground=self.sensor_color_back, command=self.senActivate)
        self.pushbat.grid(row=1, column=1)

        # Special output label
        self.output = tk.Label(
                self.root,
                text=self.out_text,
                width=20,
                height=10,
                justify=LEFT,
                anchor="nw",
                wraplength=self.sensor_width,
                fg=self.button_text_color,
                bg=self.sensor_color_back)
        self.output.config(font=self.button_text)
        self.output.grid(row=25, column=1, rowspan=5, columnspan=2)

        ###################################Drone startup/shutdown##############################################

        self.state_objs = []
        self.state_objs_names = ["connect", "takeoff", "land", "shutdown", "quit"]
        self.state_label_text = ["Connect", "Launch", "Land", "Shutdown", "Quit GUI"]
        self.state_commands = [self.d_connect, self.take_off, self.d_land, self.shutdown, self.quit]
        self.state_rows = [25, 4,  4, 8, 26]
        self.state_cols = [ 6, 6, 12, 6, 12]

        for i in range(len(self.state_objs_names)):
            self.state_objs.append(tk.Button(
                    self.root,
                    text=self.state_label_text[i],
                    highlightbackground=self.control_color_back,
                    command=self.state_commands[i]
                    ))
            self.state_objs[i].config(width=self.button_width,font=self.button_text)
            self.state_objs[i].grid(row=self.state_rows[i],column=self.state_cols[i])

        ####################################Control pad:Forward/Backward Controls##############################

        self.xz_objs = []
        self.xz_objs_names = ["up", "forward", "hover", "backward", "down"]
        self.xz_label_text = ["Move Up", "Forward", "Hover", "Backward", "Move Down"]
        self.xz_commands = [self.d_moveup, self.d_forward, self.d_hover, self.d_backward, self.d_movedown]
        self.xz_rows = [4, 5, 6, 7, 8]

        self.dr_Control = tk.Label(self.root, text="Flight\nController",
                bg=self.control_color_back)
        self.dr_Control.config(font=('Arial',24,'bold'), fg='black')
        self.dr_Control.grid(row=1, column=10)

        for i in range(len(self.xz_objs_names)):
            self.xz_objs.append(tk.Button(
                    self.root,
                    text=self.xz_label_text[i],
                    highlightbackground=self.control_color_back,
                    command=self.xz_commands[i]
                    ))
            self.xz_objs[i].config(width=self.button_width,font=self.button_text)
            self.xz_objs[i].grid(row=self.xz_rows[i],column=10)

        ####################################Control pad:Left/Right Controls####################################

        self.y_objs = []
        self.y_objs_names = ["tleft", "left", "right", "tright"]
        self.y_label_text = ["Turn Left", "Left", "Right", "Turn Right"]
        self.y_commands = [self.d_turnleft, self.d_left, self.d_right, self.d_turnright]
        self.y_cols = [6, 7, 11, 12]

        for i in range(len(self.y_objs_names)):
            self.y_objs.append(tk.Button(
                    self.root,
                    text=self.y_label_text[i],
                    highlightbackground=self.control_color_back,
                    command=self.y_commands[i]
                    ))
            self.y_objs[i].config(width=self.button_width,font=self.button_text)
            self.y_objs[i].grid(row=6,column=self.y_cols[i])

    ###################################GUI Drone button functions########################################
    def senActivate(self):
        self.battstat()
        self.altstat()
        self.velstat()
        self.gpsstat()

    def battstat(self):
        battdis = self.sensor_objs_names.index("battdis")
        if str(self.drone.getBattery()[1]) != "OK":
            self.sensor_objs[battdis].config(text="Battery: "+str(self.drone.getBattery()[0])+ "% " + "\nState: " +str(self.drone.getBattery()[1]), fg='red')
            self.root.after(1000, self.battstat)
        else:
            self.sensor_objs[battdis].config(text="Battery: "+str(self.drone.getBattery()[0])+ "% " + "\nState: " +str(self.drone.getBattery()[1]))
            self.root.after(1000, self.battstat)

    def altstat(self):
        altdis = self.sensor_objs_names.index("altdis")
        altDisplay = "Altitude: "+str(self.drone.NavData['altitude'][3]/10)
        self.sensor_objs[altdis].config(text=altDisplay)
        self.root.after(600, self.altstat)

    def velstat(self):
        veldis = self.sensor_objs_names.index("veldis")
    	velDisplay = "Velocity: "+str(np.hypot(self.drone.NavData['demo'][4][0],self.drone.NavData['demo'][4][1]))
    	self.sensor_objs[veldis].config(text=velDisplay)
    	self.root.after(200, self.velstat)

    def gpsstat(self):
        gpsdis = self.sensor_objs_names.index("gpsdis")
    	gpsDisplay = "Latitude: "+str(self.drone.NavData['gps'][0]) + "\nLongitude: "+str(self.drone.NavData['gps'][1])
    	self.sensor_objs[gpsdis].config(text=gpsDisplay)
    	self.root.after(600, self.gpsstat)

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
        if self.drone != None: self.drone.shutdown     # Land drone and discard drone object
        self.root.destroy()     # Discard Main window object
        print "Exiting GUI"

    def weeble(self):
        # Test if movements must be 'cancelled out'
        # forward, stop, right, stop, up, stop, lturn, stop
        moves = [[ 0.0,  0.2,  0.0,  0.0],
                 [ 0.0, -0.2,  0.0,  0.0],
                 [ 0.2,  0.0,  0.0,  0.0],
                 [-0.2,  0.0,  0.0,  0.0],
                 [ 0.0,  0.0,  1.0,  0.0],
                 [ 0.0,  0.0, -1.0,  0.0],
                 [ 0.0,  0.0,  0.0,  1.0],
                 [ 0.0,  0.0,  0.0, -1.0],
                 ]
        self.drone.takeoff()
        time.sleep(3)
        for move in moves:
            print "moving {}".format(move)
            self.drone.move(*move)
            time.sleep(2)
        self.drone.hover()
        time.sleep(1)
        self.drone.land()


    def goto(self):
        # Maintain steady motion toward a GPS waypoint
        tar_threshold = 2.0

        while not self.landing:
            move = self.navigator.get_move()
            movement = move[0]
            tar_dist = move[1]
            print "dist: {}".format(tar_dist)

            if tar_dist < tar_threshold:
                print "landing"
                self.drone.hover()
                self.drone.land()
                self.landing = True
            else:
                print "moving: {}".format(movement)
                #self.drone.move(*movement)
                time.sleep(1.5)

    def smooth(self):
        # Use accelerometer to dynamically adjust x,y speed
        done, adj_spd, adj_ver = False, 0.03, [0, 0]
        test_time, lr_tol, max_spd = 10, 60, 250
        move_def = [ 0.00,  0.15,  0.00,  0.00]
        move_acc = [ 0.00,  0.15,  0.00,  0.00]

        # Begin test
        self.drone.takeoff()
        time.sleep(3)
        start_time = time.time()
        self.drone.move(*move_acc)
        print "self.drone.move({})".format(move_acc)

        # TODO
        # USE ADJ_VER TO KEEP TRACK OF UNDERGOING CORRECTIONS
        # IN CASE OF OVERCORRECTION, RESET TO DEFAULT MOVE

        # Begin corrections
        while not done:
            # Refresh velocity measurement
            vel = self.navigator.get_vel()

            # Correct left/right movement
            if   lr_tol <  vel[1]: move_acc[0] += -adj_spd
            elif vel[1] < -lr_tol: move_acc[0] +=  adj_spd
            else:                  move_acc[0]  =  move_def[0]

            # Maintain max_spd mm/s
            if   max_spd < vel[0]: move_acc[1] += -adj_spd
            elif vel[0] < max_spd: move_acc[1] +=  adj_spd
            else:                  move_acc[1]  =  move_def[1]

            # Perform movement
            self.drone.move(*move_acc)
            print "self.drone.move({})".format(move_acc)
            time.sleep(1)

            # Stop after test_time seconds
            if time.time() - start_time > test_time:
                done = True

        # Finish with a land
        self.drone.land()

    def d_shutdown_land(self):
        self.landing = True
        in_list.remove(in_list[0])
        self.drone.shutdown()

    # flight functions
    def d_smooth(self):
        moving = Thread(target=self.smooth, args=())
        moving.daemon = True
        moving.start()

    def d_weeble(self):
        moving = Thread(target=self.weeble, args=())
        moving.daemon = True
        moving.start()

    def d_goto(self):
        moving = Thread(target=self.goto, args=())
        moving.daemon = True
        moving.start()

    # debugging functions
    def d_take_off(self):
        self.drone.takeoff()

    def d_calibrate_all(self):
        self.navigator.calibrate_drone(True)

    def d_calibrate_simple(self):
        self.navigator.calibrate_drone()

    def d_res_nav(self):
        print "NoNavData: {}".format(self.drone.NoNavData)
        self.drone.reconnectNavData()

    # info printing for debugging
    def d_get_all(self):
        print self.navigator.get_all()

    # drone connection button
    def d_connect(self):
        gps_targets = [
                [25.758536, -80.374548], # south ecs parking lot
                [25.757582, -80.373888], # library entrance
                [25.758633, -80.372067], # physics lecture
                [25.759387, -80.376163], # roundabout
        ]

        # Initialize drone and navigator objs
        self.drone = Drone()
        self.drone.startup()
        self.drone.reset()
        self.navigator = Navigator(self.drone)
        time.sleep(0.5)
        self.navigator.set_waypoints(gps_targets)

def main():
    # Initialize GUI
    root = tk.Tk()
    root.geometry("900x600")                #GUI window dimensions
    drone_GUI = DCMainApp(root)
    #root.protocol("WM_DELETE_WINDOW", drone_GUI.quit)

    # Run GUI
    root.mainloop()


if __name__ == "__main__":
    main()
