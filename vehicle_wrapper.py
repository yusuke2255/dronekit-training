# -*- coding: utf-8 -*-
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import vincenty
import distance

class GoToTimeout(Exception):
    pass

class VehicleWrapper:
    airspeed = 0
    vehicle = None
    def __init__( self , ip ) :
        self.vehicle = connect(ip, wait_ready=True)
        self.distance_calculator = distance.SphericalTrigonometry()

    def arm_and_wait( self ) :
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        # ARMED
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print "Vehicle is armed"

    def takeoff_and_wait(self, attr) :
        self.vehicle.simple_takeoff(attr)

        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= 10 * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def update_airspeed(self, airspeed):
        self.airspeed = airspeed
        self.vehicle.airspeed = airspeed

    def retrieve_current_altitude(self):
        return self.vehicle.location.global_relative_frame.alt

    '''
    指定した方位角の方向に進み、到達ポイントに到着するまで待つ
    :param dist: 距離(メートル)
    :param azimuth: 方位角
    :param alt 高度
    '''
    def go_and_wait(self, dist, azimuth, alt):
        current_lat = self.vehicle.location.global_frame.lat
        current_lon = self.vehicle.location.global_frame.lon
        destination = vincenty.direct(current_lat, current_lon, azimuth, dist, 1)
        destination_point = LocationGlobalRelative(destination["lat"], destination["lon"], alt)
        
        calculated_distance = self.distance_calculator.calculate(current_lat, current_lon, destination_point.lat, destination_point.lon)
        estimate_seconds = calculated_distance / self.airspeed
        print("estimate_seconds = distance(%f) / airspeed(%f) = %f" % (calculated_distance, self.airspeed, estimate_seconds))

        remaining_threshold = 0.8
        timeout = estimate_seconds * 2

        print("Going to target point.[ distance: %fm, estimate_time: %fs, airspeed: %fm/s, destination_point: [lat=%f, lot=%f] ]" % (calculated_distance, estimate_seconds, self.airspeed, destination_point.lat, destination_point.lon))
        elapsed_time = 0
        is_timeout = True
        c_lon = current_lat
        c_lat = current_lon

        self.vehicle.simple_goto(destination_point)

        while elapsed_time <= timeout:
            time.sleep(1)
            elapsed_time += 1
            c_lon = self.vehicle.location.global_frame.lon
            c_lat = self.vehicle.location.global_frame.lat
            remaining = self.distance_calculator.calculate(c_lat, c_lon, destination_point.lat, destination_point.lon)
            print('Going... [remaining %fm , current_position: [lat=%f, lon:%f], elapsed %ds/%f, airspeed=%f]' % (remaining, c_lat, c_lon, elapsed_time, timeout, self.vehicle.airspeed))
            if remaining <= remaining_threshold:
                is_timeout = False
                break

        if is_timeout:
            raise GoToTimeout("Can not reached to target point.[distance=%f, elapsed_time=%d, last_position: [lat=%f, lot: %f], destination: [lat=%f, lot: %f] ]" % (calculated_distance, elapsed_time, c_lat, c_lon, destination_point.lat, destination_point.lon))

    def rtl_and_close(self):
        print("Return to Launch")
        self.vehicle.mode = VehicleMode("RTL")
        self.vehicle.close()
