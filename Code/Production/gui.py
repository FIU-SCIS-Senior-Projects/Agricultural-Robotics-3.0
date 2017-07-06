from Tkinter import *
from PIL import Image, ImageTk
import cv2, math, select, sys, time
from ps_drone import Drone
from navigator import Navigator
from viewer import Camera
from threading import Event, Thread
from multiprocessing import Process
from decimal import Decimal
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
    def __init__(self,root,width,height):
        # Modifiable Constants
        self.win_height = height
        self.win_width  = width
        self.map_width  = 640
        self.map_height = 400
        self.cam_width  = 640
        self.cam_height = 360
        self.button_width = 8
        self.button_text_color = "purple"
        self.button_text_bgrnd = "black"
        self.button_text_face = "Arial"
        self.button_text_size = 10
        self.button_text_size2 = 9
        self.button_text_style = "bold"
        self.sensor_color_back = "lightgrey"
        self.control_color_back = "lightslategrey"
        self.sensor_width_per = 0.15
        self.camera_width_per = 1.0 - self.sensor_width_per
        self.control_width_per = 1.0 - self.sensor_width_per
        self.stat_refresh = 200 # ms

        self.map_image   = Image.open("staticmap_road.png")# updated variables
        self.drone_image = Image.open("AR3_0drone.gif")
        self.drone_loc   = Image.open("drone_mrkr.gif")
        self.bound_err   = Image.open("o_o_ran.gif")
        # Static map image display resolution is 640 x 400 with zoom level 19.
        # Future maps may be stored as a database of 640 x 400, zoom 19, map tiles
        # which the drone user may select.
        # Internal storage of map tiles will discard need for an active internet connection.
        self.LAT     = 25.759027         #Center latitude of staticmap image
        self.LONG    = -80.374598        #Center longitude of staticmap image
        self.MAXLAT  = 25.758544         #Lower bound staticmap image latitude
        self.MINLONG = -80.375419        #Lower bound staticmap image longitude
        self.MINLAT  = 25.759510         #Upper bound staticmap image latitude
        self.MAXLONG = -80.373815        #Upper bound staticmap image longitude

        # Pixel width calculation dx and dy
        self.pix_dx = abs(-80.374598 - -80.375381)/ abs(320 - 0)
        self.pix_dy = abs(25.759027 - 25.759510)/abs(200 - 0)

        # Pixel map click function
        self.clk_arr = []
        self.clk_pix_x = ''
        self.clk_pix_y = ''

        # Map marker List
        self.pix_gps_coor = []
        self.mrkr_list = []
        self.rect_line = []
        self.gps_vrtcs = []
        self.waypoints = []
        self.rect_path_line = []
        self.gen_waypnts_arr = []

        self.testarr = []

        # Radiobutton variable
        self.rte_selctn_var = IntVar()

        # Derivative Constants
        self.sensor_width = self.sensor_width_per * self.win_width
        self.control_width = self.control_width_per * self.win_width
        self.camera_width = self.camera_width_per * self.win_width
        self.button_text = (
                self.button_text_face, self.button_text_size, self.button_text_style)
        self.button_text2 = (
                self.button_text_face, self.button_text_size2, self.button_text_style)

        # Object fields
        self.drone = None              #Drone object
        self.navigator = None      #Navigator object
        self.camera = None            #Camera object
        self.root = root
        self.root.title("D.F.C. - Drone Flight Controller")

        # Controller
        self.controllerside = tk.Frame(self.root)  #Mainwindow right
        #self.controllerside.grid(row=0, column=3, columnspan=30, rowspan=30)
        self.controllerside.grid(rowspan=2, column=1, sticky="nsew")
        self.controllerside.config(width=self.sensor_width,
                height=self.win_height, background=self.control_color_back)
        self.controller_manual = Event()

        # Camera
        self.cameraside = tk.Frame(self.root)       #Mainwindow right
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
        self.camera_event = Event()

        # Static GeoMap Container
        self.controllerside2 = tk.Frame(self.root)
        self.controllerside2.grid(row=1, column=2, stick="nsew")
        self.controllerside2.config(width=self.map_width,
                height=self.map_height, background=self.control_color_back)

        # TODO ADDED IN MERGE, TO BE CLEANED
        self.landing = False
        self.threshold = 2.0

        # Blue button #
        self.blue_button = tk.Button(
                self.controllerside,
                text="Blue",
                highlightbackground=self.control_color_back,
                command=self.d_blue)
        self.blue_button.config(width=self.button_width,font=self.button_text)
        self.blue_button.grid(row=0, column=1)

        # Test button #
        self.test_button = tk.Button(
                self.controllerside,
                text="Test",
                highlightbackground=self.control_color_back,
                command=self.d_test)
        self.test_button.config(width=self.button_width,font=self.button_text)
        self.test_button.grid(row=1, column=1)

        ######################################Batt/Alt/Vel##################################################
        self.sensor_objs = []
        self.sensor_label_text = []
        self.sensor_objs_names = ["battdis", "altdis", "veldis", "headis", "gpsdis"]
        for name in self.sensor_objs_names: self.sensor_label_text.append(" ")
        self.sensor_cols = list(range(1,len(self.sensor_objs_names) + 1))

        for i in range(len(self.sensor_objs_names)):
            self.sensor_objs.append(tk.Label(
                    self.cameraside,
                    text=self.sensor_label_text[i],
                    fg=self.button_text_color, bg=self.button_text_bgrnd))
            self.sensor_objs[i].config(font=self.button_text2)
            self.sensor_objs[i].grid(row=0, column=self.sensor_cols[i])

        ###################################Drone startup/shutdown##############################################

        self.state_objs = []
        self.state_objs_names = ["connect", "takeoff", "land", "shutdown", "quit", "clear", "launch rte."]
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

        ####################################Control pad:Forward/Backward Controls##############################

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

        ####################################Control pad:Left/Right Controls####################################

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

    ####################################Drone map area##################################################

        self.maparea = tk.Canvas(self.controllerside2, bg='black',
            width=self.map_width, height=self.map_height)
        #self.maparea.bind("<Button-1>",self.capt_clicks)
        self.maparea.grid(row=0, column=0)

        self.rad_waypnts = tk.Radiobutton(self.controllerside,text="Waypoint"
                                                             ,variable= self.rte_selctn_var
                                                             ,value=1,command=self.route_selctn)
        self.rad_waypnts.config(bg= self.control_color_back,state=DISABLED)
        self.rad_waypnts.grid(row=11,column=0)

        self.rad_roi = tk.Radiobutton(self.controllerside,text="Rectangle"
                                                         ,variable=self.rte_selctn_var
                                                         ,value=2,command=self.route_selctn)
        self.rad_roi.config(bg= self.control_color_back, state=DISABLED)
        self.rad_roi.grid(row=11,column=1)

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

        self.dr_img = self.maparea.create_image(0,0,image=self.map_drone,state=HIDDEN)
        self.map_mrkrs = self.maparea.create_image(0,0,image=self.map_drone_mrkr,state=HIDDEN)

    ###################################GUI Drone button functions########################################
    def senActivate(self):
        self.battstat()
        self.altstat()
        self.velstat()
        self.headingstat()
        self.gpsstat()
        self.camstat()

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

    def render_map(self):
        cent_mapx = (self.map_width/2) + 3
        cent_mapy = (self.map_height/2) + 3
        self.loc = self.maparea.create_image(cent_mapx,cent_mapy,image=self.map_loc)

    def act_drone_loc(self):
        self.maparea.delete(self.dr_img)

        #capture coordinates
        CURRLONG = self.navigator.get_nav()["gps"][1]
        CURRLAT  = self.navigator.get_nav()["gps"][0]

        curr_px = ((CURRLONG - self.MINLONG)/(self.MAXLONG - self.MINLONG)) * (self.map_width - 0) + 0
        curr_py = ((CURRLAT - self.MINLAT)/(self.MAXLAT - self.MINLAT)) * (self.map_height - 0) + 0

        self.dr_img = self.maparea.create_image(curr_px,curr_py,image=self.map_drone,state=NORMAL)
        #redraw
        self.root.after(900, self.act_drone_loc)

    def rend_mrkrs(self):
        self.map_mrkrs = self.maparea.create_image(self.clk_pix_x,self.clk_pix_y-14
                                                                 ,image=self.map_drone_mrkr
                                                                 , state=NORMAL) # Draw marker

        self.getlat =  ((self.clk_pix_y * (self.MAXLAT - self.MINLAT))/(self.map_height-0)) + self.MINLAT
        self.getlong = ((self.clk_pix_x * (self.MAXLONG - self.MINLONG))/(self.map_width-0)) + self.MINLONG

        self.waypoints.append([self.getlat,self.getlong])
        self.navigator.mod_waypoints(self.waypoints)
        self.mrkr_list.append(self.map_mrkrs)
        self.waypoints = []

    def rend_wypnt_path(self):
        print self.navigator.waypoints
        curr_px = ((self.navigator.get_nav()["gps"][1] - self.MINLONG)/(self.MAXLONG - self.MINLONG)) * (self.map_width - 0) + 0
        curr_py = ((self.navigator.get_nav()["gps"][0] - self.MINLAT)/(self.MAXLAT - self.MINLAT)) * (self.map_height - 0) + 0
        for path in range(len(self.navigator.waypoints)):
            new_px = ((self.navigator.waypoints[path][1] - self.MINLONG)/(self.MAXLONG - self.MINLONG)) * (self.map_width - 0) + 0
            new_py = ((self.navigator.waypoints[path][0]  - self.MINLAT)/(self.MAXLAT - self.MINLAT)) * (self.map_height - 0) + 0

            if(curr_px != new_px and curr_py != new_py and self.navigator.waypoints[path-1]):
                self.line = self.maparea.create_line(curr_px
                                    ,curr_py
                                    ,new_px
                                    ,new_py
                                    ,fill='green'
                                    ,width=2)
                curr_px = new_px
                curr_py = new_py
                self.rect_line.append(self.line)

    def rend_rect_mrkrs(self):
        self.vrtx_x0_0   = self.clk_arr[0][0]
        self.vrtx_y0_0   = self.clk_arr[0][1]
        self.vrtx_x1_0   = self.clk_arr[1][0]
        self.vrtx_y1_0   = self.clk_arr[1][1]
        # set remaining vertices for ROI list
        self.vrtx_x0_1   = self.clk_arr[1][0]
        self.vrtx_y0_1   = self.clk_arr[0][1]
        self.vrtx_x1_1   = self.clk_arr[0][0]
        self.vrtx_y1_1   = self.clk_arr[1][1]

        self.testarr.append([self.vrtx_x0_0,self.vrtx_y0_0])
        self.testarr.append([self.vrtx_x0_1,self.vrtx_y0_1])
        self.testarr.append([self.vrtx_x1_0,self.vrtx_y1_0])
        self.testarr.append([self.vrtx_x1_1,self.vrtx_y1_1])
        # Render rectangle region of interest and markers
        for vertex in range(len(self.testarr)):
            if(vertex < len(self.testarr)-1):
                self.line = self.maparea.create_line(self.testarr[vertex][0]
                                                    ,self.testarr[vertex][1]
                                                    ,self.testarr[vertex+1][0]
                                                    ,self.testarr[vertex+1][1]
                                                    ,fill='red'
                                                    ,width=2) #(x0, y0, x1, y1, option, ...)
                self.map_mrkrs = self.maparea.create_image(self.testarr[vertex][0]
                                                          ,self.testarr[vertex][1] -14
                                                          ,image=self.map_drone_mrkr
                                                          ,state=NORMAL) # Draw marker
                self.mrkr_list.append(self.map_mrkrs)
                self.rect_line.append(self.line)
            elif(vertex == len(self.testarr)-1):
                self.line = self.maparea.create_line(self.testarr[vertex][0]
                                                    ,self.testarr[vertex][1]
                                                    ,self.testarr[vertex-(len(self.testarr)-1)][0]
                                                    ,self.testarr[vertex-(len(self.testarr)-1)][1]
                                                    ,fill='red'
                                                    ,width=2) #(x0, y0, x1, y1, option, ...)
                self.map_mrkrs = self.maparea.create_image(self.testarr[vertex][0]
                                                          ,self.testarr[vertex][1] -14
                                                          ,image=self.map_drone_mrkr
                                                          ,state=NORMAL) # Draw marker
                self.mrkr_list.append(self.map_mrkrs)
                self.rect_line.append(self.line)
                #self.rend_rect_path()

    def rend_rect_path(self):
        # Local variables
        self.range = 6
        self.rec_vrts_1 = self.testarr[0]   #rec_vrts_1[0] = longitude, rec_vrts_1[1] = latitude
        self.rec_vrts_2 = self.testarr[1]
        self.rec_vrts_3 = self.testarr[2]
        self.rec_vrts_4 = self.testarr[3]

        # East to West path orientation
        if(abs(self.rec_vrts_1[0] - self.rec_vrts_3[0]) > abs(self.rec_vrts_1[1]-self.rec_vrts_3[1])
                                and self.rec_vrts_1[0] > self.rec_vrts_3[0]):
            # Generate test waypoints longitudinally
            self.max_vrtcs_lon = abs(self.rec_vrts_1[0] - self.rec_vrts_3[0])/self.range
            self.temp_lon = self.rec_vrts_1[0]
            self.temp_lat = self.rec_vrts_1[1]

            self.gen_waypnts_arr.append([self.temp_lat,self.temp_lon])
            for vrtx in range(self.range):
                #start vertex -> shortest length -> longest length -> shortest length etc.
                self.new_tempvrtx = self.temp_lon - self.max_vrtcs_lon
                if(vrtx % 2 == 0 and vrtx < self.range - 1):
                    self.gen_waypnts_arr.append([self.temp_lat, self.new_tempvrtx])
                    self.temp_lat = self.rec_vrts_3[1]
                    self.gen_waypnts_arr.append([self.temp_lat, self.new_tempvrtx])
                    self.temp_lon = self.new_tempvrtx
                    self.new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx < self.range -1):
                    self.gen_waypnts_arr.append([self.temp_lat,self.new_tempvrtx])
                    self.temp_lat = self.rec_vrts_1[1]
                    self.gen_waypnts_arr.append([self.temp_lat, self.new_tempvrtx])
                    self.temp_lon = self.new_tempvrtx
                    self.new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx == self.range-1):
                    self.gen_waypnts_arr.append([self.temp_lat, self.new_tempvrtx])
                    self.temp_lat = self.rec_vrts_3[1]
                    self.gen_waypnts_arr.append([self.temp_lat, self.rec_vrts_3[0]])
                    self.new_tempvrtx = 0
                    break
                elif(vrtx % 2 == 0 and vrtx == self.range-1):
                    self.gen_waypnts_arr.append([self.temp_lat, self.new_tempvrtx])
                    self.temp_lat = self.rec_vrts_2[1]
                    self.gen_waypnts_arr.append([self.temp_lat, self.rec_vrts_2[0]])
                    self.new_tempvrtx = 0
                    break
        #West to East path orientation
        elif(abs(self.rec_vrts_1[0] - self.rec_vrts_3[0]) > abs(self.rec_vrts_1[1]-self.rec_vrts_3[1])
                                and self.rec_vrts_1[0] < self.rec_vrts_3[0]):
            # Generate test waypoints longitudinally
            self.max_vrtcs_lon = abs(self.rec_vrts_1[0] - self.rec_vrts_3[0])/self.range
            self.temp_lon = self.rec_vrts_1[0]
            self.temp_lat = self.rec_vrts_1[1]

            self.gen_waypnts_arr.append([self.temp_lat,self.temp_lon])
            for vrtx in range(self.range):
                #start vertex -> shortest length -> longest length -> shortest length etc.
                self.new_tempvrtx = self.temp_lon + self.max_vrtcs_lon
                if(vrtx % 2 == 0 and vrtx < self.range - 1):
                    self.gen_waypnts_arr.append([self.temp_lat, self.new_tempvrtx])
                    self.temp_lat = self.rec_vrts_3[1]
                    self.gen_waypnts_arr.append([self.temp_lat, self.new_tempvrtx])
                    self.temp_lon = self.new_tempvrtx
                    self.new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx < self.range -1):
                    self.gen_waypnts_arr.append([self.temp_lat,self.new_tempvrtx])
                    self.temp_lat = self.rec_vrts_1[1]
                    self.gen_waypnts_arr.append([self.temp_lat, self.new_tempvrtx])
                    self.temp_lon = self.new_tempvrtx
                    self.new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx == self.range-1):
                    self.gen_waypnts_arr.append([self.temp_lat, self.new_tempvrtx])
                    self.temp_lat = self.rec_vrts_3[1]
                    self.gen_waypnts_arr.append([self.temp_lat, self.rec_vrts_3[0]])
                    self.new_tempvrtx = 0
                    break
                elif(vrtx % 2 == 0 and vrtx == self.range-1):
                    self.gen_waypnts_arr.append([self.temp_lat, self.new_tempvrtx])
                    self.temp_lat = self.rec_vrts_2[1]
                    self.gen_waypnts_arr.append([self.temp_lat, self.rec_vrts_2[0]])
                    self.new_tempvrtx = 0
                    break
        # South to North path orientation
        elif(abs(self.rec_vrts_1[0] - self.rec_vrts_3[0]) < abs(self.rec_vrts_1[1]-self.rec_vrts_3[1])
                                and self.rec_vrts_1[1] > self.rec_vrts_3[1]):
            # Generate test waypoints longitudinally
            self.max_vrtcs_lat = abs(self.rec_vrts_1[1] - self.rec_vrts_3[1])/self.range
            self.temp_lon = self.rec_vrts_1[0]
            self.temp_lat = self.rec_vrts_1[1]

            self.gen_waypnts_arr.append([self.temp_lat,self.temp_lon])
            for vrtx in range(self.range):
                #start vertex -> shortest length -> longest length -> shortest length etc.
                self.new_tempvrtx = self.temp_lat - self.max_vrtcs_lat
                if(vrtx % 2 == 0 and vrtx < self.range - 1):
                    self.gen_waypnts_arr.append([self.new_tempvrtx, self.temp_lon])
                    self.temp_lon = self.rec_vrts_3[0]
                    self.gen_waypnts_arr.append([self.new_tempvrtx, self.temp_lon])
                    self.temp_lat = self.new_tempvrtx
                    self.new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx < self.range -1):
                    self.gen_waypnts_arr.append([self.new_tempvrtx,self.temp_lon])
                    self.temp_lon = self.rec_vrts_1[0]
                    self.gen_waypnts_arr.append([self.new_tempvrtx, self.temp_lon])
                    self.temp_lat = self.new_tempvrtx
                    self.new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx == self.range-1):
                    self.gen_waypnts_arr.append([self.new_tempvrtx, self.temp_lon])
                    self.temp_lon = self.rec_vrts_3[0]
                    self.gen_waypnts_arr.append([self.rec_vrts_3[1], self.temp_lon])
                    self.new_tempvrtx = 0
                    break
                elif(vrtx % 2 == 0 and vrtx == self.range-1):
                    self.gen_waypnts_arr.append([self.rec_vrts_3[1], self.temp_lon])
                    self.new_tempvrtx = 0
                    break
        # North to South path orientation
        elif(abs(self.rec_vrts_1[0] - self.rec_vrts_3[0]) < abs(self.rec_vrts_1[1]-self.rec_vrts_3[1])
                                and self.rec_vrts_1[1] < self.rec_vrts_3[1]):
            # Generate test waypoints longitudinally
            self.max_vrtcs_lat = abs(self.rec_vrts_1[1] - self.rec_vrts_3[1])/self.range
            self.temp_lon = self.rec_vrts_1[0]
            self.temp_lat = self.rec_vrts_1[1]

            self.gen_waypnts_arr.append([self.temp_lat,self.temp_lon])
            for vrtx in range(self.range):
                #start vertex -> shortest length -> longest length -> shortest length etc.
                self.new_tempvrtx = self.temp_lat + self.max_vrtcs_lat
                if(vrtx % 2 == 0 and vrtx < self.range - 1):
                    self.gen_waypnts_arr.append([self.new_tempvrtx, self.temp_lon])
                    self.temp_lon = self.rec_vrts_3[0]
                    self.gen_waypnts_arr.append([self.new_tempvrtx, self.temp_lon])
                    self.temp_lat = self.new_tempvrtx
                    self.new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx < self.range -1):
                    self.gen_waypnts_arr.append([self.new_tempvrtx,self.temp_lon])
                    self.temp_lon = self.rec_vrts_1[0]
                    self.gen_waypnts_arr.append([self.new_tempvrtx, self.temp_lon])
                    self.temp_lat = self.new_tempvrtx
                    self.new_tempvrtx = 0
                elif(vrtx % 2 == 1 and vrtx == self.range-1):
                    self.gen_waypnts_arr.append([self.new_tempvrtx, self.temp_lon])
                    self.temp_lon = self.rec_vrts_3[0]
                    self.gen_waypnts_arr.append([self.rec_vrts_3[1], self.temp_lon])
                    self.new_tempvrtx = 0
                    break
                elif(vrtx % 2 == 0 and vrtx == self.range-1):
                    self.gen_waypnts_arr.append([self.rec_vrts_3[1], self.temp_lon])
                    self.new_tempvrtx = 0
                    break
        # Path rendering
        for edge in range(len(self.gen_waypnts_arr)):
            if(edge < len(self.gen_waypnts_arr)-1):
                self.line = self.maparea.create_line(self.gen_waypnts_arr[edge][1]
                                                    ,self.gen_waypnts_arr[edge][0]
                                                    ,self.gen_waypnts_arr[edge + 1][1]
                                                    ,self.gen_waypnts_arr[edge + 1][0]
                                                    ,fill='green'
                                                    ,width=2)
                self.rect_path_line.append(self.line)

            elif(edge == len(self.gen_waypnts_arr)):
                self.line = self.maparea.create_line(self.gen_waypnts_arr[edge-1][1]
                                                    ,self.gen_waypnts_arr[edge-1][0]
                                                    ,self.gen_waypnts_arr[edge][1]
                                                    ,self.gen_waypnts_arr[edge][0]
                                                    ,fill='green'
                                                    ,width=2)
                self.rect_path_line.append(self.line)

    def waypoint_rte(self,event):
        self.clk_pix_x = event.x                # Recent event variables
        self.clk_pix_y = event.y

        self.getlat =  ((self.clk_pix_y * (self.MAXLAT - self.MINLAT))/(self.map_height-0)) + self.MINLAT
        self.getlong = ((self.clk_pix_x * (self.MAXLONG - self.MINLONG))/(self.map_width-0)) + self.MINLONG
        print "lat: {}\nlon: {}".format(self.getlat, self.getlong)

        self.clk_arr.append([event.x, event.y]) # List of marker pixel locations
        self.pix_gps_coor.append([self.getlat,self.getlong]) #List of GPS locations

        self.rend_mrkrs()

    def roi_rect_rte(self,event):
        self.clk_pix_x = event.x                # Recent event variables
        self.clk_pix_y = event.y

        self.getlat =  ((self.clk_pix_y * (self.MAXLAT - self.MINLAT))/(self.map_height-0)) + self.MINLAT
        self.getlong = ((self.clk_pix_x * (self.MAXLONG - self.MINLONG))/(self.map_width-0)) + self.MINLONG

        if(len(self.clk_arr) < 2):
            self.clk_arr.append([event.x, event.y]) # List of marker pixel locations
            self.pix_gps_coor.append([self.getlat,self.getlong]) #List of GPS locations
            if(len(self.clk_arr) == 2):
                self.vrtx_x0_0   = self.pix_gps_coor[0][0]
                self.vrtx_y0_0   = self.pix_gps_coor[0][1]
                self.vrtx_x1_0   = self.pix_gps_coor[1][0]
                self.vrtx_y1_0   = self.pix_gps_coor[1][1]
                # set remaining vertices for ROI list
                self.vrtx_x0_1   = self.pix_gps_coor[1][0]
                self.vrtx_y0_1   = self.pix_gps_coor[0][1]
                self.vrtx_x1_1   = self.pix_gps_coor[0][0]
                self.vrtx_y1_1   = self.pix_gps_coor[1][1]

                self.gps_vrtcs.append([self.vrtx_x0_0,self.vrtx_y0_0])
                self.gps_vrtcs.append([self.vrtx_x0_1,self.vrtx_y0_1])
                self.gps_vrtcs.append([self.vrtx_x1_0,self.vrtx_y1_0])
                self.gps_vrtcs.append([self.vrtx_x1_1,self.vrtx_y1_1])

                self.rend_rect_mrkrs()
                self.navigator.gen_waypnts(self.gps_vrtcs)

    def route_selctn(self):
        if(self.rte_selctn_var.get() == 1):
            self.maparea.bind("<Button-1>",self.waypoint_rte)
        elif(self.rte_selctn_var.get() == 2):
            self.maparea.bind("<Button-1>",self.roi_rect_rte)

    def lnch_route(self):
        if(self.rte_selctn_var.get() == 1):
            print ">>> Map Drone Waypoints Route"
            self.rend_wypnt_path()
        elif(self.rte_selctn_var.get()==2):
            print ">>> Map Drone ROI Route"
            self.rend_rect_path()
        print ">>> Drone Beginning Route"

    def clear_slctns(self):
        self.clk_pix_x = ''
        self.clk_pix_y = ''
        self.clk_arr = []
        self.pix_gps_coor = []
        self.gps_vrtcs = []
        self.gen_waypnts_arr = []
        self.testarr = []

        for mrkr in range(len(self.mrkr_list)):
            self.maparea.delete(self.mrkr_list[mrkr])
        self.mrkr_list = []
        for line in range(len(self.rect_line)):
            self.maparea.delete(self.rect_line[line])
        self.rect_line = []
        for path in range(len(self.rect_path_line)):
            self.maparea.delete(self.rect_path_line[path])
        for obj in range(len(self.navigator.waypoints)):
            self.navigator.waypoints.pop()

        self.gps_vrtcs = []
        self.navigator.gen_waypnts(self.gps_vrtcs)

        print ">>>Route removed"


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

    def quit(self):
        self.controller_manual.set()
        self.camera_event.set()
        if self.drone != None: self.drone.shutdown     # Land drone and discard drone object
        if self.camera != None: self.camera.cam_thread.join()
        if self.camera != None: self.camera.release()   # Shutdown camera
        self.root.destroy()     # Discard Main window object
        print "Exiting GUI"

    def weeble(self):
        # Test if movements must be 'cancelled out'
        # forward, stop, right, stop, up, stop, lturn, stop
        self.controller_manual.clear()
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
            if self.controller_manual.is_set(): return 0
            print "moving {}".format(move)
            self.drone.move(*move)
            time.sleep(2)
        self.drone.hover()
        time.sleep(1)
        self.drone.land()


    def goto(self):
        # Maintain steady motion toward a GPS waypoint

        self.controller_manual.clear()
        while not self.landing and not self.controller_manual.is_set():
            move = self.navigator.get_move()
            movement = move[0]
            tar_dist = move[1]
            print "dist: {}".format(tar_dist)

            if tar_dist < self.threshold:
                print "landing"
                self.drone.hover()
                self.drone.land()
                self.landing = True
            else:
                print "moving: {}".format(movement)
                self.drone.move(*movement)
                time.sleep(1.5)

    def smooth(self):
        # Use accelerometer to dynamically adjust x,y speed
        self.controller_manual.clear()
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
        while not done and not self.controller_manual.is_set():
            # Refresh velocity measurement
            vel = self.navigator.get_nav()["vel"]

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

    def nav_waypoints(self):
        thresh = 2.0
        self.navigator.next_tar()
        movement, dist = self.navigator.get_move_no_rot()
        
        # Begin test
        if dist != -1:
            self.controller_manual.clear()
            self.drone.takeoff()
            time.sleep(5)
        while (dist != -1) and not self.controller_manual.is_set():
            while (dist > thresh) and not self.controller_manual.is_set():
                self.drone.move(*movement)
                print "self.drone.move({})".format(movement)
                time.sleep(0.5)
                movement, dist = self.navigator.get_move_no_rot()
            self.navigator.next_tar()
            movement, dist = self.navigator.get_move_no_rot()
        self.controller_manual.set()
        self.drone.land()
        return True

    # flight buttons
    def lnch_route(self):
        print ">>>Drone Beginning Route"
        moving = Thread(target=self.nav_waypoints, args=())
        moving.daemon = True
        moving.start()

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
        self.rad_waypnts.config(bg= self.control_color_back,state=NORMAL)
        self.rad_roi.config(bg= self.control_color_back, state=NORMAL)



    def d_blue(self):
        self.camera.tog_colors()

    def d_test(self):
        self.d_smooth()
