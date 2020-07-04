from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
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

# get command object
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

print(" Clear any existing commands")
cmds.clear() 

# Save the vehicle commands to a list
missionlist=[]
for cmd in cmds:
    missionlist.append(cmd)


print(" Define/add new commands.")
cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 35.68407150, 139.75523470, targetAltitude))
cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 35.68612810, 139.75536350, targetAltitude))
cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 35.68612810, 139.75536350, targetAltitude))

print(" Upload new commands to vehicle")
cmds.upload()

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

while True:
    nextwaypoint=vehicle.commands.next
    print("Next waypoint is {}".format(nextwaypoint))

    if nextwaypoint==3: #Dummy waypoint - as soon as we reach the waypoint, we exit.
        print("Exit 'standard' mission when start heading to final waypoint")
        break;
    time.sleep(1)


print("Return to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()