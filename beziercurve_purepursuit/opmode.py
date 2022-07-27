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
run_thread = True




while startup:
    api.normalize_encoders()
    api.locations.append([0, 0, 0])
    startup = False

if active_opmode:    

    api.motor_speed(0, 0)
    
    print(api.current_position)
    print(api.current_heading)
    active_opmode = False


    