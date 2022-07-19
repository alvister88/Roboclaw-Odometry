from threading import Thread
from stringprep import in_table_a1
import time
import math
from tracemalloc import stop
import numpy as np
from roboclaw_3 import Roboclaw


#Windows comport name
# rc = Roboclaw("COM11",115200)
#Linux comport name
rcL = Roboclaw("/dev/ttyACM0", 115200)
rcR = Roboclaw("/dev/ttyACM1", 115200)
rcL.Open()
rcR.Open()

# test encoder method
# testing

def normalize_encoders():
    rcL.SetEncM1(left_side, MID_QUADRATURE_VALUE)
    rcL.SetEncM2(left_side, MID_QUADRATURE_VALUE)
    rcR.SetEncM1(right_side, MID_QUADRATURE_VALUE)
    rcR.SetEncM2(right_side, MID_QUADRATURE_VALUE)

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
    return ((inches / wheel_circumference) * tics_per_rev) + MID_QUADRATURE_VALUE

# (tics/s, tics/s, inches, inches, inches, inches, (0 or 1))
def move_motors(speed1, speed2,m1,m2,m3,m4, buffer):
    accel = 1000
    deccel = 1000
    m1 = tic_distance(m1)
    m2 = tic_distance(m2)
    m3 = tic_distance(m3)
    m4 = tic_distance(m4)
   
    rcL.SpeedAccelDeccelPositionM1M2(left_side, accel, speed1, deccel, int(m1), accel, speed1, deccel, int(m2), buffer)
    rcR.SpeedAccelDeccelPositionM1M2(right_side, accel, speed2, deccel, int(m3), accel, speed2, deccel, int(m4), buffer)
    
    # buffer 1 reading
    depth1 = np.uint8  
    # buffer 2 reading
    depth2 = np.uint8
    # track when command ends
    while  depth1 != (1, left_side, left_side) and depth2 != (1, right_side, right_side):
        depth1 = rcL.ReadBuffers(left_side)
        depth2 = rcR.ReadBuffers(right_side)
        time.sleep(0.01)
    
    normalize_encoders()
    show_encoder()
    
def drive_straight(speed, distance, buffer):
    move_motors(speed, speed, distance, distance, distance, distance, buffer)

def drive_stop():
    move_motors(0,0,0,0,0,0,0)

# turn around back axle
def turn_center(speed, angle, buffer):
    global current_heading
    # inches
    robot_width = 14
    turn_circumference = math.pi * robot_width
    distance = (angle/360.0) * turn_circumference
    # move_motors(distance, distance, -1 * distance, -1 * distance)
    move_motors(speed, speed, -distance, -distance, distance, distance, buffer)
    # drive_stop
    current_heading += angle
    normalize()

# counter clockwise positve, 0 -> 180; 0 -> -180
def turn_heading(speed, new_heading, buffer):
    global current_heading
    turn_angle = new_heading - current_heading

    if turn_angle > 180:
        turn_angle -= 360
    elif turn_angle < -180:
        turn_angle += 360

    turn_center(speed, turn_angle, buffer)
    current_heading = new_heading
    normalize()

# normalize global heading
def normalize():
    global current_heading

    if current_heading > 180:
        current_heading % 360
    elif current_heading < -180:
        current_heading % 360
        current_heading *= -1

# drive relative to current position
def drive_to_position(speed, x_dist, y_dist, face_angle):
    global current_position
    global current_heading
    turn_angle = math.atan2(y_dist, x_dist) * (180/math.pi)
    print("turn angle: " + str(turn_angle))
    
    turn_center(speed, turn_angle, 0)
    
    target_distance = math.sqrt(math.pow(x_dist, 2) + math.pow(y_dist, 2))
    print("target distance: " + str(target_distance))
    
    drive_straight(speed, target_distance, 0)
    face_turn = face_angle - turn_angle
    
    if face_turn > 180:
        face_turn -= 360
    elif face_turn < -180:
        face_turn += 360

    print("new angle: " + str(face_turn))
    turn_center(speed, face_turn, 0)
    
    current_position[0] += x_dist
    current_position[1] += y_dist

    print("current position: " + str(current_position))
    print("current heading: " + str(current_heading))
    

# drive to absolute location; (x,y) plane
def drive_to_location(speed, x_pos, y_pos, face_angle):
    global current_position
    global current_heading
    # other tan acute angle
    target_heading = math.atan2(y_pos - current_position[1], x_pos - current_position[0]) * (180/math.pi)
    print("target heading: " + str(target_heading))
    
    turn_heading(speed, target_heading, 0)

    print("current heading: " + str(current_heading))
    target_distance = math.sqrt(math.pow(x_pos - current_position[0], 2) + math.pow(y_pos - current_position[1], 2))
    print("target distance: " + str(target_distance))
    
    drive_straight(speed, target_distance, 0)
    print("face angle: " + str(face_angle))
    turn_heading(speed, face_angle, 0)
    
    current_position[0] = x_pos
    current_position[1] = y_pos

    print("current position: " + str(current_position))
    print("current heading: " + str(current_heading))


    #(x (front back),y(left right)) 
def add_waypoint(speed, x_pos, y_pos, face_angle):
    waypoint = [speed, x_pos, y_pos, face_angle]
    waypoints.append(waypoint)

def localize():
    global waypoints
    global current_position
    global current_heading
    
    while(len(waypoints) != 0):
        target_location = waypoints[0]
        speed = target_location[0]
        x_pos = target_location[1]
        y_pos = target_location[2]
        face_angle = target_location[3]

        drive_to_location(speed, x_pos, y_pos, face_angle)
        
        print("next")
        waypoints.pop(0)

        time.sleep(1)

tics_per_rev = 2442.96
wheel_circumference = 4 * math.pi
left_side = 0x81
right_side = 0x80
position = []
waypoints = []
active_opmode = True
startup = True

# use mid encoder normalize to prevent wrap around
MIN_QUADRATURE_VALUE = np.uint
MAX_QUADRATURE_VALUE = np.uint
MID_QUADRATURE_VALUE = np.uint
MIN_QUADRATURE_VALUE = 0
MAX_QUADRATURE_VALUE = 500000
MID_QUADRATURE_VALUE = int(MAX_QUADRATURE_VALUE/2)

# absolute position and angle heading
current_position = [0,0]
current_heading = 0


while startup:
    if startup:
        normalize_encoders()
        startup = False

if active_opmode:
    # add_waypoint(2500, 20, -20, 90)
    # add_waypoint(2500, 30, -20, -100)
    # add_waypoint(2500, 0, 0, 0)
    # localize()
    drive_to_position(4000, 0, 0, 90)    
    # drive_to_position(3000, -10, 0, 0)
    
    show_encoder()
    # drive_straight(1000, 15, 0)
    # show_encoder()

    # turn_center(7000, 90, 0)
    
    active_opmode = False
      

    
    
	
    
