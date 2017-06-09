from Tkinter import *
import ps_drone
#from itertools import islice
#from subprocess import Popen, PIPE
#from textwrap import dedent
import math, select, sys, time
from ps_drone import Drone
from navigator import Navigator
from threading import Thread
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

#TEST_HOME = [25.758995, -80.373743]
#gps_target = [25.758536, -80.374548] # south ecs parking lot
gps_target = [25.757582, -80.373888] # library entrance
#gps_target = [25.758633, -80.372067] # physics lecture
#gps_target = [25.759387, -80.376163] # roundabout



try:
        import Tkinter as tk            #python 2 support
        from Queue import Queue, Empty  #python 2 support
except ImportError:
        import tkinter as tk            #python 3 support
        from queue import Queue, Empty  #python 3 support

class DCMainApp(object):

        def __init__(self,root,drone,navigator):

                self.drone = drone              #Drone object
                self.navigator = navigator              #Navigator object
                self.root = root
                self.root.title("D.F.C. - Drone Flight Controller")

                self.startupside1 = tk.Frame(self.root)                                 #Mainwindow left
                self.startupside1.grid(row=0, column=0, columnspan=3, rowspan=30)
                self.startupside1.config(width=200, height=600, background="lightgrey")

                self.controllerside = tk.Frame(self.root, background="lightslategrey")  #Mainwindow right
                self.controllerside.config(width=700, height=600)
                self.controllerside.grid(row=0, column=3, columnspan=30, rowspan=30)

                # TODO ADDED IN MERGE, TO BE CLEANED
                self.landing = False


                ######################################Batt/Alt/Vel##################################################
                self.battdis = tk.Label(self.root, text=" ", fg='blue', bg='lightgrey')
                self.battdis.config(font=('Arial',15,'bold'))
                self.battdis.grid(row=2, column=1)

                self.altdis = tk.Label(self.root, text=" ", fg='blue', bg='lightgrey')
                self.altdis.config(font=('Arial',15,'bold'))
                self.altdis.grid(row=3, column=1)

                self.velDis = tk.Label(self.root, text=" ")
                self.velDis.config(font=('Arial', 15, 'bold'), fg='blue', bg='lightgrey')
                self.velDis.grid(row=4, column=1)

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

        def battstat(self):
                battDisplay = "Battery: "+str(self.drone.getBattery()[0])+ "% " + "\nState: " +str(self.drone.getBattery()[1])# Battery update variable
                self.battdis.config(text=battDisplay)
                self.root.after(600, self.battstat)

        def altstat(self):
                altDisplay = "Altitude: "+str(self.drone.NavData['altitude'][3]/10)
                self.altdis.config(text=altDisplay)
                self.root.after(10, self.altstat)


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
                self.drone.shutdown     # Land drone and discard drone object
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

def main():
    # Initialize drone and navigator objs
    drone = Drone()                #Drone object
    drone.startup()
    drone.reset()
    navigator = Navigator(drone)
    time.sleep(0.5)
    navigator.set_target(gps_target)

    # Print battery
    battery = drone.getBattery()
    while battery[0] == -1:
        time.sleep(0.1)
        battery = drone.getBattery()
    print battery

    # Initialize GUI
    root = tk.Tk()
    root.geometry("900x600")                #GUI window dimensions
    drone_GUI = DCMainApp(root, drone, navigator)
    #root.protocol("WM_DELETE_WINDOW", drone_GUI.quit)

    # Run GUI
    root.mainloop()


if __name__ == "__main__":
    main()
