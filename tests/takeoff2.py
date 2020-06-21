from dronekit import connect, VehicleMode
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

# change to LOITER mode
vehicle.wait_for_mode(loiter)

vehicle.close()