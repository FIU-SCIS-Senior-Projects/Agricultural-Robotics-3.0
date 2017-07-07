import cv2, math, select, sys, time
from ps_drone import Drone
from navigator import Navigator
from viewer import Camera
from threading import Event, Thread
from decimal import Decimal
import Tkinter as tk
from PIL import Image, ImageTk
import numpy as np
np.seterr(divide='ignore', invalid='ignore')

'''
D.F.C.-Drone Flight Controller:

GUI that connects to Parrot AR 2.0 Drone and gives the user
control of the drone eliminating the need to run different
script through the CLI.
'''

class DCMainApp(object):
    def __init__(self,root,width,height):
        # Modifiable Constants
        self.map_width  = 640
        self.map_height = 400
        self.cam_width  = 640
        self.cam_height = 360
        self.button_width = 8
        self.button_text_color = "purple"
        self.button_text_bgrnd = "black"
        self.button_text_face = "Arial"
        self.button_text_size = 10
        self.label_text_size  = 9
        self.button_text_style = "bold"
        self.sensor_color_back = "lightgrey"
        self.control_color_back = "lightslategrey"
        self.sensor_width_per = 0.15
        self.stat_refresh = 200 # ms
        self.map_image   = Image.open("images/map.png")# updated variables
        self.drone_image = Image.open("images/drone.gif")
        self.drone_loc   = Image.open("images/marker.gif")
        self.bound_err   = Image.open("images/offscreen.gif")
        # Static map image display resolution is 640 x 400 with zoom level 19.
        # Future maps may be stored as a database of 640 x 400, zoom 19, map tiles
        # which the drone user may select.
        # Internal storage of map tiles will discard need for an active internet connection.
        self.LAT    =  25.759027    #Center latitude of staticmap image
        self.LON    = -80.374598    #Center longitude of staticmap image
        self.MINLAT =  25.759510    #Upper bound staticmap image latitude
        self.MINLON = -80.375419    #Lower bound staticmap image longitude
        self.MAXLAT =  25.758544    #Lower bound staticmap image latitude
        self.MAXLON = -80.373815    #Upper bound staticmap image longitude

        # Pixel map click function
        self.clk_arr = []

        # Map marker List
        self.mrkrs = []
        self.testarr = []
        self.lines = []

        # Radiobutton variable
        self.rte_selctn_var = tk.IntVar()

        # Derivative Constants
        self.win_height = height
        self.win_width  = width
        self.camera_width_per = 1.0 - self.sensor_width_per
        self.control_width_per = 1.0 - self.sensor_width_per
        self.sensor_width = self.sensor_width_per * self.win_width
        self.control_width = self.control_width_per * self.win_width
        self.camera_width = self.camera_width_per * self.win_width
        self.button_text = (
                self.button_text_face,
                self.button_text_size,
                self.button_text_style)
        self.labels_text = (
                self.button_text_face,
                self.label_text_size,
                self.button_text_style)

        # Object fields
        self.root      = root
        self.drone     = None
        self.navigator = None
        self.camera    = None
        self.camera_event      = Event()
        self.controller_manual = Event()

        # Controller
        self.controllerside = tk.Frame(self.root)
        self.controllerside.grid(rowspan=2, column=1, sticky="nsew")
        self.controllerside.config(width=self.sensor_width,
                height=self.win_height, background=self.control_color_back)

        # Camera
        self.cameraside = tk.Frame(self.root)
        self.cameraside.grid(row=0, column=2, sticky="nsew")
        self.cameraside.config(width=self.map_width,
                height=self.map_height, background=self.control_color_back)
        cam_img = np.zeros((self.cam_width, self.cam_height, 3), np.uint8)
        cam_img = Image.fromarray(cam_img)
        cam_img = ImageTk.PhotoImage(cam_img)
        self.panel_cam = tk.Label(
                self.cameraside,
                width=self.cam_width,
                height=self.cam_height,
                image = cam_img,
                bg='black')
        self.panel_cam.cam_img = cam_img
        self.panel_cam.grid(
                row=0, column=0,
                columnspan=30, rowspan=30,
                sticky=tk.W+tk.N)

        # Static GeoMap Container
        self.controllerside2 = tk.Frame(self.root)
        self.controllerside2.grid(row=1, column=2, stick="nsew")
        self.controllerside2.config(width=self.map_width,
                height=self.map_height, background=self.control_color_back)

        # Labels: Batt/Alt/Vel/GPS/Status
        self.sensor_objs = []
        self.sensor_label_text = []
        self.sensor_objs_names = ["battdis", "altdis", "veldis", "headis", "gpsdis", "stadis"]
        for name in self.sensor_objs_names: self.sensor_label_text.append(" ")
        self.sensor_cols = range(1, len(self.sensor_objs_names) + 1)

        for i in range(len(self.sensor_objs_names)):
            self.sensor_objs.append(tk.Label(
                    self.cameraside,
                    text=self.sensor_label_text[i],
                    fg=self.button_text_color, bg=self.button_text_bgrnd))
            self.sensor_objs[i].config(font=self.labels_text)
            self.sensor_objs[i].grid(row=0, column=self.sensor_cols[i])

        # Drone Startup / Shutdown / Connect / Quit / Launch / Land
        self.state_objs = []
        self.state_objs_names = ["connect", "takeoff", "land", "shutdown", "quit", "clear", "launch"]
        self.state_label_text = ["Connect", "Launch", "Land", "Shutdown", "Quit GUI", "Clear Sel.", "Start Route"]
        self.state_commands = [self.d_connect, self.take_off, self.d_land, self.shutdown, self.quit, self.clear_slctns, self.lnch_route]
        self.state_rows = [0, 6, 6, 1, 12, 12,11]
        self.state_cols = [0, 0, 2, 0, 0, 2, 2]

        for i in range(len(self.state_objs_names)):
            self.state_objs.append(tk.Button(
                    self.controllerside,
                    text=self.state_label_text[i],
                    highlightbackground=self.control_color_back,
                    command=self.state_commands[i]
                    ))
            self.state_objs[i].config(width=self.button_width,font=self.button_text)
            self.state_objs[i].grid(row=self.state_rows[i],column=self.state_cols[i])

        # Control pad: Forward/Backward/Altitude Controls
        self.xz_objs = []
        self.xz_objs_names = ["up", "forward", "hover", "backward", "down"]
        self.xz_label_text = ["Move Up", "Forward", "Hover", "Backward", "Move Down"]
        self.xz_commands = [self.d_moveup, self.d_forward, self.d_hover, self.d_backward, self.d_movedown]
        self.xz_rows = [2, 3, 4, 5, 6]


        for i in range(len(self.xz_objs_names)):
            self.xz_objs.append(tk.Button(
                    self.controllerside,
                    text=self.xz_label_text[i],
                    highlightbackground=self.control_color_back,
                    command=self.xz_commands[i]
                    ))
            self.xz_objs[i].config(width=self.button_width,font=self.button_text)
            self.xz_objs[i].grid(row=self.xz_rows[i],column=1)

        # Control pad: [Turn] Left/Right
        self.y_objs = []
        self.y_objs_names = ["tleft", "left", "right", "tright"]
        self.y_label_text = ["Turn Left", "Left", "Right", "Turn Right"]
        self.y_commands = [self.d_turnleft, self.d_left, self.d_right, self.d_turnright]
        self.y_cols = [0,0,2,2]
        self.x_rows = [3,4,3,4]

        for i in range(len(self.y_objs_names)):
            self.y_objs.append(tk.Button(
                    self.controllerside,
                    text=self.y_label_text[i],
                    highlightbackground=self.control_color_back,
                    command=self.y_commands[i]
                    ))
            self.y_objs[i].config(width=self.button_width,font=self.button_text)
            self.y_objs[i].grid(row=self.x_rows[i],column=self.y_cols[i])

        # Blue button
        self.blue_button = tk.Button(
                self.controllerside,
                text="Blue",
                highlightbackground=self.control_color_back,
                command=self.d_blue)
        self.blue_button.config(width=self.button_width,font=self.button_text)
        self.blue_button.grid(row=0, column=1)

        # Test button
        self.test_button = tk.Button(
                self.controllerside,
                text="Test",
                highlightbackground=self.control_color_back,
                command=self.d_test)
        self.test_button.config(width=self.button_width,font=self.button_text)
        self.test_button.grid(row=1, column=1)

        # Radio buttons
        self.rad_waypnts = tk.Radiobutton(
                self.controllerside,
                text="Waypoint",
                variable= self.rte_selctn_var,
                value=1,command=self.route_selctn)
        self.rad_waypnts.config(bg=self.control_color_back,state=tk.DISABLED)
        self.rad_waypnts.grid(row=11,column=0)

        self.rad_roi = tk.Radiobutton(
                self.controllerside,
                text="Rectangle",
                variable=self.rte_selctn_var,
                value=2,
                command=self.route_selctn)
        self.rad_roi.config(bg= self.control_color_back, state=tk.DISABLED)
        self.rad_roi.grid(row=11,column=1)

        # Drone map area
        self.maparea = tk.Canvas(
                self.controllerside2,
                bg='black',
                width=self.map_width,
                height=self.map_height)
        self.maparea.grid(row=0, column=0)

        self.droneimg = tk.Label(self.controllerside2)
        self.droneimg.grid(row=0,column=2)

        self.drone_mrkr = tk.Label(self.controllerside2)
        self.drone_mrkr.grid(row=0,column=2)

        self.err_img = tk.Label(self.controllerside2)
        self.err_img.grid(row=0,column=2)

        self.map_loc = ImageTk.PhotoImage(self.map_image)
        self.map_drone = ImageTk.PhotoImage(self.drone_image)
        self.map_drone_mrkr = ImageTk.PhotoImage(self.drone_loc)
        self.map_b_err = ImageTk.PhotoImage(self.bound_err)

        self.dr_img = self.maparea.create_image(0,0,image=self.map_drone,state=tk.HIDDEN)
        self.map_mrkrs = self.maparea.create_image(0,0,image=self.map_drone_mrkr,state=tk.HIDDEN)

    # Statistics Labels
    def senActivate(self):
        self.battstat()
        self.altstat()
        self.velstat()
        self.headingstat()
        self.gpsstat()
        self.camstat()
        self.stastat()

    def battstat(self):
        battdis = self.sensor_objs_names.index("battdis")
        if str(self.drone.getBattery()[1]) != "OK":
            self.sensor_objs[battdis].config(fg="red")
        else:
            self.sensor_objs[battdis].config(fg="purple")

        self.sensor_objs[battdis].config(text="bat: {}%".format(
            self.drone.getBattery()[0]))
        self.root.after(1000, self.battstat)

    def camstat(self):
        cam_img = self.camera.getFrame()
        cam_img = Image.fromarray(cam_img)
        cam_img = ImageTk.PhotoImage(cam_img)
        self.panel_cam.config(image = cam_img)
        self.panel_cam.cam_img = cam_img
        self.root.after(100, self.camstat)

    def altstat(self):
        altdis = self.sensor_objs_names.index("altdis")
        altDisplay = "alt: {}".format(
                Decimal(
                    self.navigator.get_nav()["alt"]
                    ).quantize(Decimal('0.001')))
        self.sensor_objs[altdis].config(text=altDisplay)
        self.root.after(self.stat_refresh, self.altstat)

    def stastat(self):
        stadis = self.sensor_objs_names.index("stadis")
        staDisplay = "{}".format(self.get_nav()["stus"])
    	self.sensor_objs[stadis].config(text=staDisplay)
    	self.root.after(self.stat_refresh, self.stastat)

    def velstat(self):
        veldis = self.sensor_objs_names.index("veldis")
        velDisplay = "vel: {}".format(
                Decimal(np.hypot(
                    self.navigator.get_nav()["vel"][0],
                    self.navigator.get_nav()["vel"][1])
                    ).quantize(Decimal('0.001')))
        self.sensor_objs[veldis].config(text=velDisplay)
        self.root.after(self.stat_refresh, self.velstat)

    def gpsstat(self):
        gpsdis = self.sensor_objs_names.index("gpsdis")
        gpsDisplay = "lat: {}  lon: {}".format(
                Decimal(
                    self.navigator.get_nav()["gps"][0]
                    ).quantize(Decimal('0.000001')),
                Decimal(
                    self.navigator.get_nav()["gps"][1]
                    ).quantize(Decimal('0.000001')))
        self.sensor_objs[gpsdis].config(text=gpsDisplay)
        self.root.after(self.stat_refresh, self.gpsstat)

    def headingstat(self):
        headis = self.sensor_objs_names.index("headis")
        heaDisplay = "hdg: {}".format(
                Decimal(
                    self.navigator.get_nav()["deg"]
                    ).quantize(Decimal('0.01')))
        self.sensor_objs[headis].config(text=heaDisplay)
        self.root.after(self.stat_refresh, self.headingstat)

    # Map drawing
    def get_p(self, lat, lon):
        """Convert given lat, lon to pixel location on map"""
        px = ((lon - self.MINLON) / (self.MAXLON - self.MINLON)) * self.map_width
        py = ((lat - self.MINLAT) / (self.MAXLAT - self.MINLAT)) * self.map_height
        return px, py

    def get_l(self, px, py):
        """Convert given pixel location to lat, lon on map"""
        lat = (py * (self.MAXLAT - self.MINLAT) / self.map_height) + self.MINLAT
        lon = (px * (self.MAXLON - self.MINLON) / self.map_width) + self.MINLON
        return lon, lat

    def render_map(self):
        cent_mapx = (self.map_width/2) + 3
        cent_mapy = (self.map_height/2) + 3
        self.loc = self.maparea.create_image(cent_mapx,cent_mapy,image=self.map_loc)

    def act_drone_loc(self):
        self.maparea.delete(self.dr_img)
        curr_lat = self.navigator.get_nav()["gps"][0]
        curr_lon = self.navigator.get_nav()["gps"][1]
        curr_px, curr_py = self.get_p(curr_lat, curr_lon)
        self.dr_img = self.maparea.create_image(
                curr_px,
                curr_py,
                image=self.map_drone,
                state=tk.NORMAL)
        self.root.after(900, self.act_drone_loc)

    def rend_mrkr(self, x, y):
        self.map_mrkrs = self.maparea.create_image(
                x, y - 14,
                image=self.map_drone_mrkr,
                state=tk.NORMAL) # Draw marker
        self.mrkrs.append(self.map_mrkrs)

    def rend_path(self):
        curr_lat = self.navigator.get_nav()["gps"][0]
        curr_lon = self.navigator.get_nav()["gps"][1]
        curr_px, curr_py = self.get_p(curr_lat, curr_lon)

        for point in self.navigator.waypoints:
            next_px, next_py = self.get_p(*point)
            line = self.maparea.create_line(
                    curr_px, curr_py,
                    next_px, next_py,
                    fill = 'green', width = 2)
            self.lines.append(line)
            curr_px, curr_py = next_px, next_py

    def select_rte(self,event):
        mode = self.rte_selctn_var.get()
        x = event.x                # Recent event variables
        y = event.y
        self.getlong, self.getlat = self.get_l(x, y)

        if mode == 1:   # single-waypoint
            self.navigator.mod_waypoints([[self.getlat, self.getlong]])
            self.rend_mrkr(x, y)
            self.rend_path()
        elif mode == 2: # rect-waypoint
            if(len(self.clk_arr) < 2):
                self.clk_arr.append([x, y]) # List of marker pixel locations

            if(len(self.clk_arr) == 2):
                a_lon, a_lat = self.get_l(*self.clk_arr[0])
                b_lon, b_lat = self.get_l(self.clk_arr[1][0], self.clk_arr[0][1])
                c_lon, c_lat = self.get_l(*self.clk_arr[1])
                d_lon, d_lat = self.get_l(self.clk_arr[0][0], self.clk_arr[1][1])
                a = [a_lat, a_lon]
                b = [b_lat, b_lon]
                c = [c_lat, c_lon]
                d = [d_lat, d_lon]

                self.navigator.gen_waypnts([a, b, c, d])
                self.rend_path()
                self.clk_arr = []

    # Determine selection mode
    def route_selctn(self):
        self.clear_slctns()
        self.maparea.bind("<Button-1>", self.select_rte)

    # Remove selections and clear navigator waypoints
    def clear_slctns(self):
        self.clk_arr = []
        self.testarr = []

        for mrkr in range(len(self.mrkrs)):
            self.maparea.delete(self.mrkrs[mrkr])
        self.mrkrs = []
        for line in self.lines:
            self.maparea.delete(line)
        self.lines = []

        self.navigator.waypoints.clear()
        self.navigator.next_tar()

    # flight functions
    def take_off(self):
        self.drone.takeoff()

    def d_land(self):
        self.controller_manual.set()
        self.drone.land()

    def shutdown(self):
        self.controller_manual.set()
        self.drone.shutdown()

    def d_hover(self):
        self.controller_manual.set()
        self.drone.hover()

    def d_forward(self):
        self.controller_manual.set()
        self.drone.moveForward()

    def d_backward(self):
        self.controller_manual.set()
        self.drone.moveBackward()

    def d_left(self):
        self.controller_manual.set()
        self.drone.moveLeft()

    def d_right(self):
        self.controller_manual.set()
        self.drone.moveRight()

    def d_turnright(self):
        self.controller_manual.set()
        xr = 0.50
        self.drone.turnRight(xr)

    def d_turnleft(self):
        self.controller_manual.set()
        xl = 0.50
        self.drone.turnLeft(xl)

    def d_moveup(self):
        self.controller_manual.set()
        self.drone.moveUp()

    def d_movedown(self):
        self.controller_manual.set()
        self.drone.moveDown()

    def lnch_route(self):
        if(self.rte_selctn_var.get() == 1):
            print ">>> Map Drone Waypoints Route"
        elif(self.rte_selctn_var.get()==2):
            print ">>> Map Drone ROI Route"
        print ">>> Drone Beginning Route"

    # Debugging button functions
    def d_blue(self):
        if self.camera != None: self.camera.tog_colors()

    def d_test(self):
        return None

    def d_calibrate_simple(self):
        self.navigator.calibrate_drone()

    def d_calibrate_all(self):
        self.navigator.calibrate_drone(True)

    def d_res_nav(self):
        print "NoNavData: {}".format(self.drone.NoNavData)
        self.drone.reconnectNavData()

    # Connection button
    def d_connect(self):
        # Initialize drone and navigator objs
        self.drone = Drone()
        self.drone.startup()
        self.drone.reset()
        self.navigator = Navigator(self.drone)
        self.camera = Camera(
                self.drone,
                self.cam_width,
                self.cam_height,
                self.camera_event)
        self.camera.start()
        self.senActivate()
        self.render_map()
        self.act_drone_loc()
        self.rad_waypnts.config(bg= self.control_color_back,state=tk.NORMAL)
        self.rad_roi.config(bg= self.control_color_back, state=tk.NORMAL)

    # GUI Quit button
    def quit(self):
        self.controller_manual.set()
        self.camera_event.set()
        if self.drone != None: self.drone.shutdown     # Land drone and discard drone object
        if self.camera != None: self.camera.cam_thread.join()
        if self.camera != None: self.camera.release()   # Shutdown camera
        self.root.destroy()     # Discard Main window object
        print "Exiting GUI"

