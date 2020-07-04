from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time

# connect
# vehicle = connect('127.0.0.1:14550', wait_ready=True)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

# set the target altitude
targetAltitude = 30
# set home location
start_location = LocationGlobal(35.681841, 139.756737, 20.915894)
vehicle.home_location = start_location

# modes
guided = VehicleMode("GUIDED")
loiter = VehicleMode("LOITER")

vehicle.wait_for_armable(30) # wait for 30sec
print("Vehicle is armable")
vehicle.wait_for_mode(guided, timeout=5)
print("Mode is changed")
vehicle.arm(wait=True, timeout=5)
print("Armed state is {}".format(vehicle.armed))

# Take off (Altitude, error, timeout)
try:
    vehicle.wait_simple_takeoff(targetAltitude, 0.5, 30)
    print("Take off!!")
except TimeoutError as takeoffTimeout:
    print("Takeoff is timeout!!")
    vehicle.disarm()

# target locations
point02 = LocationGlobalRelative(35.68407150, 139.75523470, targetAltitude)
print("move to waypoint 02")
vehicle.simple_goto(point02)
time.sleep(65)

point03 = LocationGlobalRelative(35.68612810, 139.75536350, targetAltitude)
print("move to waypoint 03")
vehicle.simple_goto(point03)
time.sleep(50)

point04 = LocationGlobalRelative(35.68630240, 139.75665090, targetAltitude)
print("move to waypoint 04")
vehicle.simple_goto(point04)
time.sleep(30)

point05 = LocationGlobalRelative(35.68553550, 139.75703720, targetAltitude)
print("move to waypoint 05")
vehicle.simple_goto(point05)
time.sleep(25)

point06 = LocationGlobalRelative(35.68459440, 139.75647930, targetAltitude)
print("move to waypoint 06")
vehicle.simple_goto(point06)
time.sleep(30)

point07 = LocationGlobalRelative(35.68403670, 139.75574970, targetAltitude)
print("move to waypoint 07")
vehicle.simple_goto(point07)
time.sleep(25)

point08 = LocationGlobalRelative(35.68630240, 139.75845340, targetAltitude)
print("move to waypoint 08")
vehicle.simple_goto(point08)
time.sleep(75)

point09 = LocationGlobalRelative(35.68640700, 139.76124290, targetAltitude)
print("move to waypoint 09")
vehicle.simple_goto(point09)
time.sleep(60)

point10 = LocationGlobalRelative(35.68623270, 139.75978370, targetAltitude)
print("move to waypoint 10")
vehicle.simple_goto(point10)
time.sleep(30)

point11 = LocationGlobalRelative(35.68529150, 139.75978370, targetAltitude)
print("move to waypoint 11")
vehicle.simple_goto(point11)
time.sleep(25)

point12 = LocationGlobalRelative(35.68396700, 139.75991250, targetAltitude)
print("move to waypoint 12")
vehicle.simple_goto(point12)
time.sleep(35)

point13 = LocationGlobalRelative(35.68393210, 139.75871090, targetAltitude)
print("move to waypoint 13")
vehicle.simple_goto(point13)
time.sleep(25)

point14 = LocationGlobalRelative(35.68476870, 139.75811000, targetAltitude)
print("move to waypoint 14")
vehicle.simple_goto(point14)
time.sleep(25)

print("Return to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()