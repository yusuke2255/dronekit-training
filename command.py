# -*- coding: utf-8 -*-
import re

class GoToCommand:
    azimuth = 0
    distance = 0
    def __init__( self , azimuth, distance ) :
        self.azimuth = azimuth
        self.distance = distance

class ArmCommand:
    def __init__(self):
        pass

class RtlCommand:
    def __init__(self):
        pass

class TakeOffCommand:
    altitude = 0
    def __init__( self , altitude ) :
        self.altitude = altitude

class UnknownCommand:
    message = ""
    def __init__( self , message ) :
        self.message = message

def resolve(message):
    if message == "ARM":
        return ArmCommand()

    if message == "RTL":
        return RtlCommand()

    elements = re.findall('^Take off to altitude (\d+)m', message)
    if len(elements) == 1:
        attitude = int(elements[0])
        return TakeOffCommand(attitude)

    elements = re.search('^Go to (.*) (\d+)(km|m)', message).groups()
    if len(elements) == 3:
        direction = elements[0]
        azimuth = to_azimuth(direction)
        if azimuth is None:
            return UnknownCommand(message)

        distance = elements[1]
        distance_unit = elements[2]

        meter = to_meter(distance, distance_unit)

        if meter is None:
            return UnknownCommand(message)

        return GoToCommand(azimuth, meter)

    return UnknownCommand(message)

def to_azimuth(direction):
    if direction == "north":
        return 0
    elif direction == "south":
        return 180
    elif direction == "east":
        return 90
    elif direction == "west":
        return 270
    else:
        return None

def to_meter(distance, unit):
    if unit == "m":
        return int(distance)
    elif unit == "km":
        return int(distance) * 1000

    return None