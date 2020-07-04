# encoding: utf-8
import time
import math
from pymavlink import mavutil
from dronekit import connect, Command, VehicleMode, LocationGlobal

import argparse
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

vehicle.wait_ready('autopilot_version')

# -------------------------------------------------------------
# 起動前に pip install requests
import requests

url="http://winggate.co.jp/mission_9th.waypoints"

def load_waypoint_from_url(url):
  try:
    r = requests.get(url)
    v = r.text.split("\n")
    if v[0].startswith('QGC'):
      v.pop(0)
    waypoints = []
    for line in v:
      waypoint = []
      splitted = line.split('\t')
      for i in range(0, 3):
        waypoint.append(int(splitted[i]))
      for i in range(4, 10):
        waypoint.append(float(splitted[i]))
      waypoint.append(int(splitted[11]))
      waypoints.append(waypoint)
    return waypoints
  except requests.exceptions.RequestException as err:
    print(err)

# test
# print(load_waypoint_from_url(url))

altitude = 30
lastWP = 15
# -------------------------------------------------------------

# Get Vehicle Home location - will be `None` until first set by autopilot
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print(" Waiting for home location ...")

# We have a home location, so print it!        
print("\n Home location: %s" % vehicle.home_location)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(altitude)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


# -------------------------------------------------------------
pointList = load_waypoint_from_url(url)
missionList = []

for item in pointList:
    cmd=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, item[7], item[8], altitude)
    missionList.append(cmd)

cmds = vehicle.commands
cmds.clear()

for command in missionList:
    cmds.add(command)
print(' Upload mission')
vehicle.commands.upload()

# start mission
vehicle.mode = VehicleMode("AUTO")
time.sleep(1)

while vehicle.commands.next < lastWP + 1:
    print(" Waiting mission complete %d" % vehicle.commands.next)
    time.sleep(1)

vehicle.mode = VehicleMode("RTL")
