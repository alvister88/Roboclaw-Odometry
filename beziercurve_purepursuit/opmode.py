from stringprep import in_table_a1
import time
import math
from tracemalloc import start, stop
import numpy as np
from roboclaw_3 import Roboclaw
from multipledispatch import dispatch
import generate_bezier as gb
import pure_pursuit as pp
import api_controller as api

active_opmode = True
startup = True


while startup:
    api.normalize_encoders()
    api.locations.append([0, 0, 0])
    startup = False

if active_opmode:
    pathing = gb.generate_bezier_pathing(20, 20, 0, 11)
    
    active_opmode = False
    