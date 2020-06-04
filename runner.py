# -*- coding: utf-8 -*-
import vehicle_wrapper
import argparse

parser = argparse.ArgumentParser(
            prog='runner.py',
            description='dronekit training',
            epilog='end',
            add_help=True,
            )

parser.add_argument('-i', '--ip', required=True)

args = parser.parse_args()

wrapper = vehicle_wrapper.VehicleWapper(args.ip)

# ARM
wrapper.arm_and_wait()

# 10m takeoff
wrapper.takeoff_and_wait(10)

# Set default/target airspeed to 3
wrapper.update_airspeed(3)

try:
    for i in range(5):
        # 北（方位角0度）の方向へ高度20mで20m進む
        wrapper.go_and_wait(20, 0, 20)
        # 東（方位角90度）の方向へ高度20mで100m進む
        wrapper.go_and_wait(100, 90, 20)
        # 北（方位角0度）の方向へ高度20mで20m進む
        wrapper.go_and_wait(20, 0, 20)
        # 西（方位角270度）の方向へ高度20mで100m進む
        wrapper.go_and_wait(100, 270, 20)
except Exception as e:
    print(e)
    wrapper.rtl_and_close()

wrapper.rtl_and_close()
