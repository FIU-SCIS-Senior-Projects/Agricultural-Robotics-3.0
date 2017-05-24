import ps_drone, time

drone = ps_drone.Drone()
drone.startup()
drone.reset()
while drone.getBattery()[0] == -1: time.sleep(0.1)
time.sleep(0.5)
battery = drone.getBattery()
drone.printBlue("Battery: {}% {}".format(battery[0], battery[1]))

time.sleep(0.1)
#drone.setConfig("general:navdata_options", "65537")
drone.setConfig("general:navdata_options", "777060865")
drone.useDemoMode(False)

#time.sleep(0.1)
#CDC = drone.ConfigDataCount
#drone.getConfig()
#while CDC == drone.ConfigDataCount: time.sleep(0.001)
#CDC = drone.ConfigDataCount
#drone.getConfig()
#while CDC == drone.ConfigDataCount: time.sleep(0.001)
#for conf in drone.ConfigData:
#    if conf[0][:8] == "general:":
#        print conf

time.sleep(0.1)
NDC = drone.NavDataCount
drone.getNDpackage(["all"])
while drone.NavDataCount == NDC: time.sleep(0.001)
#print drone.NavData.keys()
print drone.NavData["gps"]




time.sleep(0.1)
drone.shutdown()
