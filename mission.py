#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
mission_basic.py: Example demonstrating basic mission operations including creating, clearing and monitoring missions.

Full documentation is provided at http://python.dronekit.io/examples/mission_basic.html
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

# Set up option parsing to get connection string
import argparse
import urllib

parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('-u', '--url', required=True)
args = parser.parse_args()

parser = argparse.ArgumentParser(
            prog='mission_runner.py',
            description='dronekit training mission runner',
            epilog='end',
            add_help=True,
            )

parser.add_argument('-u', '--url', required=True)

url = args.url
filename = "/tmp/downloaded_mission.txt"
urllib.urlretrieve(url, filename)

connection_string = "127.0.0.1:14551"
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    missionlist = []
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i == 0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray = line.split('\t')
                ln_index = int(linearray[0])
                print(ln_index)
                ln_currentwp = int(linearray[1])
                print(ln_currentwp)
                ln_frame = int(linearray[2])
                print(ln_frame)
                ln_command = int(linearray[3])
                print(ln_command)
                ln_param1 = float(linearray[4])
                print(ln_param1)
                ln_param2 = float(linearray[5])
                print(ln_param2)
                ln_param3 = float(linearray[6])
                print(ln_param3)
                ln_param4 = float(linearray[7])
                print(ln_param4)
                ln_param5 = float(linearray[8])
                print(ln_param5)
                ln_param6 = float(linearray[9])
                print(ln_param6)
                ln_param7 = float(linearray[10])
                print(ln_param7)
                ln_autocontinue = int(linearray[11].strip())
                print(ln_autocontinue)
                cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2,
                              ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    # Read mission from file
    missionlist = readmission(aFileName)

    print("\nUpload mission from a file: %s" % aFileName)
    # Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    # Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()


# def get_location_metres(original_location, dNorth, dEast):
#     """
#     Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
#     specified `original_location`. The returned Location has the same `alt` value
#     as `original_location`.
#
#     The function is useful when you want to move the vehicle around specifying locations relative to
#     the current vehicle position.
#     The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
#     For more information see:
#     http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
#     """
#     earth_radius = 6378137.0  # Radius of "spherical" earth
#     # Coordinate offsets in radians
#     dLat = dNorth / earth_radius
#     dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
#
#     # New position in decimal degrees
#     newlat = original_location.lat + (dLat * 180 / math.pi)
#     newlon = original_location.lon + (dLon * 180 / math.pi)
#     return LocationGlobal(newlat, newlon, original_location.alt)


# def get_distance_metres(aLocation1, aLocation2):
#     """
#     Returns the ground distance in metres between two LocationGlobal objects.
#
#     This method is an approximation, and will not be accurate over large distances and close to the
#     earth's poles. It comes from the ArduPilot test code:
#     https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
#     """
#     dlat = aLocation2.lat - aLocation1.lat
#     dlong = aLocation2.lon - aLocation1.lon
#     return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


# def distance_to_current_waypoint():
#     """
#     Gets distance in metres to the current waypoint.
#     It returns None for the first waypoint (Home location).
#     """
#     nextwaypoint = vehicle.commands.next
#     if nextwaypoint == 0:
#         return None
#     missionitem = vehicle.commands[nextwaypoint - 1]  # commands are zero indexed
#     lat = missionitem.x
#     lon = missionitem.y
#     alt = missionitem.z
#     targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
#     distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
#     return distancetopoint


# def download_mission():
#     """
#     Download the current mission from the vehicle.
#     """
#     cmds = vehicle.commands
#     cmds.download()
#     cmds.wait_ready()  # wait until download is complete.


# def adds_square_mission(aLocation, aSize):
#     """
#     Adds a takeoff command and four waypoint commands to the current mission.
#     The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
#
#     The function assumes vehicle.commands matches the vehicle mission state
#     (you must have called download at least once in the session and after clearing the mission)
#     """
#
#     cmds = vehicle.commands
#
#     print(" Clear any existing commands")
#     cmds.clear()
#
#     print(" Define/add new commands.")
#     # Add new commands. The meaning/order of the parameters is documented in the Command class.
#
#     # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
#     cmds.add(
#         Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
#                 0, 0, 0, 0, 10))
#
#     # Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
#     point1 = get_location_metres(aLocation, aSize, -aSize)
#     point2 = get_location_metres(aLocation, aSize, aSize)
#     point3 = get_location_metres(aLocation, -aSize, aSize)
#     point4 = get_location_metres(aLocation, -aSize, -aSize)
#     cmds.add(
#         Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
#                 0, 0, 0, point1.lat, point1.lon, 11))
#     cmds.add(
#         Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
#                 0, 0, 0, point2.lat, point2.lon, 12))
#     cmds.add(
#         Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
#                 0, 0, 0, point3.lat, point3.lon, 13))
#     cmds.add(
#         Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
#                 0, 0, 0, point4.lat, point4.lon, 14))
#     # add dummy waypoint "5" at point 4 (lets us know when have reached destination)
#     cmds.add(
#         Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0,
#                 0, 0, 0, point4.lat, point4.lon, 14))
#
#     print(" Upload new commands to vehicle")
#     cmds.upload()


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


print('Create a new mission (for current location)')
# adds_square_mission(vehicle.location.global_frame, 50)

# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(10)

print("Starting mission")
# Reset mission set to first (0) waypoint
# vehicle.commands.next = 0

# Set mode to AUTO to start mission

missionlist = readmission(filename)
print(missionlist)
cmds = vehicle.commands
cmds.clear()
for command in missionlist:
    cmds.add(command)

print(' Upload mission')
vehicle.commands.upload()

vehicle.mode = VehicleMode("AUTO")

# Monitor mission.
# Demonstrates getting and setting the command number
# Uses distance_to_current_waypoint(), a convenience function for finding the
#   distance to the next waypoint.

# while True:
#     nextwaypoint = vehicle.commands.next
#     print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
#
#     if nextwaypoint == 3:  # Skip to next waypoint
#         print('Skipping to Waypoint 5 when reach waypoint 3')
#         vehicle.commands.next = 5
#     if nextwaypoint == 5:  # Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
#         print("Exit 'standard' mission when start heading to final waypoint (5)")
#         break;
#     time.sleep(1)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
