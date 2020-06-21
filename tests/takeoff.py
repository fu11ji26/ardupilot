from dronekit import connect, VehicleMode
import time

# connect
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# set the target altitude
targetAltitude = 15

# Arming excecute -> Guided mode, armed = True
print("Arming motors")
guided = VehicleMode("GUIDED")
vehicle.mode = guided
vehicle.armed = True

# Takeoff is enable?
while not vehicle.armed or vehicle.mode.name != 'GUIDED':
    print("Vehicle not ready...")
    time.sleep(1)

# Take off
print("Take off!!")
vehicle.simple_takeoff(targetAltitude)

# wait for reaching the target
while True:
    print("Altitude: {}".format(vehicle.location.global_relative_frame.alt))
    if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
        print("Reached target altitude")
        break
    time.sleep(1)

vehicle.mode = VehicleMode("LOITER")

vehicle.close()