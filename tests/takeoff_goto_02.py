from dronekit import connect, VehicleMode
from pymavlink import mavutil # Needed for command message definitions
import time

# connect
# vehicle = connect('127.0.0.1:14550', wait_ready=True)
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

# set the target altitude
targetAltitude = 15

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
print("Take off!!")
vehicle.wait_simple_takeoff(targetAltitude, 0.5, 30)

###  copy from guided_set_speed_yaw.py ########################################################
def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    
# Set parameters
DURATION = 5 #Set duration for each segment.
NORTH = 2
SOUTH = -2
EAST = 2
WEST = -2
UP = -0.5
DOWN = 0.5

# Square path using velocity
print("zigzag path using SET_POSITION_TARGET_LOCAL_NED and velocity parameters")

print("Yaw 180 absolute (South)")
condition_yaw(180)

print("Velocity South & up")
send_ned_velocity(SOUTH,0,UP,DURATION)
send_ned_velocity(0,0,0,1)

print("Yaw 270 absolute (West)")
condition_yaw(270)

print("Velocity West & down")
send_ned_velocity(0,WEST,DOWN,DURATION)
send_ned_velocity(0,0,0,1)

print("Yaw 180 absolute (South)")
condition_yaw(180)

print("Velocity South & up")
send_ned_velocity(SOUTH,0,UP,DURATION)
send_ned_velocity(0,0,0,1)

print("Yaw 270 absolute (West)")
condition_yaw(270)

print("Velocity West & down")
send_ned_velocity(0,WEST,DOWN,DURATION)
send_ned_velocity(0,0,0,1)

print("Yaw 180 absolute (South)")
condition_yaw(180)

print("Velocity South & up")
send_ned_velocity(SOUTH,0,UP,DURATION)
send_ned_velocity(0,0,0,1)

print("Yaw 270 absolute (West)")
condition_yaw(270)

print("Velocity West & down")
send_ned_velocity(0,WEST,DOWN,DURATION)
send_ned_velocity(0,0,0,1)

##############################################################################

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()