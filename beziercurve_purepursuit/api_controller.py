from stringprep import in_table_a1
import time
import math
from tracemalloc import start, stop
import numpy as np
from roboclaw_3 import Roboclaw
from multipledispatch import dispatch


# Alvin's attempt at python localization x:)
# updated
#Linux comport name
rcL = Roboclaw("/dev/ttyACM0", 115200)
rcR = Roboclaw("/dev/ttyACM1", 115200)
rcL.Open() 
rcR.Open()

# global variables----------------------------------------------

tics_per_rev = 2442.96
wheel_circumference = 4 * math.pi
robot_width = 13.5
left_side = 0x81
right_side = 0x80
waypoints = []
locations = []

# use mid encoder normalize to prevent wrap around
MIN_QUADRATURE_VALUE: np.uint = 0
MAX_QUADRATURE_VALUE: np.uint = 500000
MID_QUADRATURE_VALUE = np.uint = int(MAX_QUADRATURE_VALUE / 2)

# absolute position and angle heading
current_position = [0.0,0.0]
# current heading in radians
current_heading = 0.0

# ----------------------------------------------------------------

def normalize_encoders():
    rcL.SetEncM1(left_side, MID_QUADRATURE_VALUE)
    rcL.SetEncM2(left_side, MID_QUADRATURE_VALUE)
    rcR.SetEncM1(right_side, MID_QUADRATURE_VALUE)
    rcR.SetEncM2(right_side, MID_QUADRATURE_VALUE)

def read_encoders():
    return float(rcL.ReadEncM2(left_side)[1]) - MID_QUADRATURE_VALUE, float(rcR.ReadEncM2(right_side)[1]) - MID_QUADRATURE_VALUE
    
def show_encoder():
    # left front
    encoderL1 = rcL.ReadEncM1(left_side)
    # left back
    encoderL2 = rcL.ReadEncM2(left_side)
    # right front
    encoderR1 = rcR.ReadEncM1(right_side) 
    # right back
    encoderR2 = rcR.ReadEncM2(right_side)
    
    print(tuple(np.subtract(encoderL1, (0, 250000, 0))))
    print(tuple(np.subtract(encoderL2, (0, 250000, 0))))
    print(tuple(np.subtract(encoderR1, (0, 250000, 0))))
    print(tuple(np.subtract(encoderR2, (0, 250000, 0))))

def tic_distance(inches):
    return int((inches / wheel_circumference) * tics_per_rev)

def inch_distance(tics):
    return int((tics * wheel_circumference) / tics_per_rev)

def normalize():
    global current_heading

    while current_heading > 180:
        current_heading -= 360
    while current_heading < -180:
        current_heading += 360

def normalize_radians():
    global current_heading

    while current_heading > math.pi:
        current_heading -= 2 * math.pi
    while current_heading < -math.pi:
        current_heading += 2 * math.pi
  
def to_radians(angle):      
    return angle * math.pi / 180

def motor_speed(speed_L, speed_R):
    rcL.SpeedM1M2(left_side, speed_L, speed_L)
    rcR.SpeedM1M2(right_side, speed_R, speed_R)


# (tics/s, tics/s, inches, inches, inches, inches, (0 or 1))
def move_motors(speed_L, speed_R, L_front, L_back, R_front, R_back):
    
    L_front = tic_distance(L_front) + int(rcL.ReadEncM1(left_side)[1])
    L_back = tic_distance(L_back) + int(rcL.ReadEncM2(left_side)[1])
    R_front = tic_distance(R_front) + int(rcR.ReadEncM1(right_side)[1])
    R_back = tic_distance(R_back) + int(rcR.ReadEncM2(right_side)[1])

    accel = 0
    deccel = 0
   
    rcL.SpeedAccelDeccelPositionM1M2(left_side, accel, speed_L, deccel, int(L_front), accel, speed_L, deccel, int(L_back), 0)
    rcR.SpeedAccelDeccelPositionM1M2(right_side, accel, speed_R, deccel, int(R_front), accel, speed_R, deccel, int(R_back), 0)

    # buffer 1 reading
    depth1 = np.uint8  
    # buffer 2 reading
    depth2 = np.uint8
    # track when command ends
    while  depth1 != (1, left_side, left_side) and depth2 != (1, right_side, right_side):
        depth1 = rcL.ReadBuffers(left_side)
        depth2 = rcR.ReadBuffers(right_side)
        # time.sleep(0.01)

    time.sleep(0.2)

   
def drive_straight(speed, distance):
    move_motors(speed, speed, distance, distance, distance, distance)

def drive_stop(interval):
    move_motors(0,0,0,0,0,0)
    time.sleep(interval)

# turn centered around back wheel axle
def turn_center(speed, angle):
    global current_heading
    global current_position
    # robot_length = 13.3
    # turn_radius_offset = robot_length / 2

    turn_circumference = math.pi * robot_width
    distance = (angle/360.0) * turn_circumference
    move_motors(speed, speed, -distance, -distance, distance, distance)
    current_heading += angle
    normalize()

# counter clockwise positve, 0 -> 180; 0 -> -180
def turn_heading(speed, new_heading):
    global current_heading
    turn_angle = new_heading - current_heading

    if turn_angle > 180:
        turn_angle -= 360
    elif turn_angle < -180:
        turn_angle += 360

    turn_center(speed, turn_angle)
    current_heading = new_heading
    normalize()



  


    
      

    
    
	
    
