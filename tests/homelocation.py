from dronekit import connect, VehicleMode
import time

# connect
vehicle = connect('127.0.0.1:14550', wait_ready=True)

while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    if not vehicle.home_location:
        print("Waiting for home location ...")

print("\n Home Location: {}".format(vehicle.home_location))
vehicle.close()