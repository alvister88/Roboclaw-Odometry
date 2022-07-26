from stringprep import in_table_a1
import time
import math
from tracemalloc import start, stop
from turtle import right
import numpy as np
from roboclaw_3 import Roboclaw
from multipledispatch import dispatch
from threading import Thread
import generate_bezier as gb
import pure_pursuit as pp
import api_controller as api

active_opmode = True
startup = True

# true position thread
def threaded_function():
    while active_opmode:
        left_tics, right_tics = api.read_encoders()
        left = api.inch_distance(left_tics)
        right = api.inch_distance(right_tics)
        theta = (right - left) / api.robot_width
        radius = (right / theta) - (api.robot_width / 2)
        
        rad_final_angle = (api.current_heading) * math.pi / 180 + theta
        rad_init_angle = api.current_heading * math.pi / 180
        
        # x position
        api.current_position[0] = radius * (math.cos(rad_final_angle) - math.cos(rad_init_angle))
        # y position
        api.current_position[1] = radius * (math.sin(rad_final_angle) - math.sin(rad_init_angle))
        time.sleep(0.01)
        print("running")


while startup:
    api.normalize_encoders()
    api.locations.append([0, 0, 0])
    startup = False

if active_opmode:
    true_position = Thread(target = threaded_function)
    true_position.start()
    
    time.sleep(1)
    
    active_opmode = False
    