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
rcL = Roboclaw("/dev/ttyACM0",115200)
rcR = Roboclaw("/dev/ttyACM1",115200)
rcL.Open()
rcR.Open()

# test encoder method
def initencoder():
    if startup:
        rcL.ResetEncoders(left_side)
        rcR.ResetEncoders(right_side)
    time.sleep(2)

def show_encoder():
    # left front
    encoderL1 = rcL.ReadEncM1(left_side)
    # left back
    encoderL2 = rcL.ReadEncM2(left_side)
    # right front
    encoderR1 = rcR.ReadEncM1(right_side)
    # right back
    encoderR2 = rcR.ReadEncM2(right_side)
    print(encoderL1),
    print(encoderL2)
    print(encoderR1),
    print(encoderR2)
    time.sleep(1)

def tic_distance(inches):
    return (inches / wheel_circumference) * tics_per_rev

def move_motors(speed1, speed2,m1,m2,m3,m4, buffer):
    m1 = tic_distance(m1)
    m2 = tic_distance(m2)
    m3 = tic_distance(m3)
    m4 = tic_distance(m4)
    if m1 < 0 or m2 < 0:
        speed1 = -speed1
    if m3 < 0 or m4 < 0:
        speed2 = -speed2
    rcL.SpeedAccelDistanceM1M2(left_side, 1000, speed1, abs(int(m1)), speed1, abs(int(m2)), buffer)
    rcR.SpeedAccelDistanceM1M2(right_side, 1000, speed2, abs(int(m3)), speed2, abs(int(m4)), buffer)
    # buffer 1 reading
    depth1 = np.uint8  
    # buffer 2 reading
    depth2 = np.uint8
    # track when command ends
    while  depth1 != (1, left_side, left_side) and depth2 != (1, right_side, right_side):
        depth1 = rcL.ReadBuffers(left_side)
        depth2 = rcR.ReadBuffers(right_side)
        time.sleep(0.01)
    drive_stop
    time.sleep(1)       
    
def drive_straight(speed, distance, buffer):
    move_motors(speed, speed, distance, distance, distance, distance, buffer)

def drive_stop():
    move_motors(0,0,0,0,0,0,0)

# counter clockwise positve, 0 -> 180; 0 -> -180
def turn_center(speed, angle, buffer):
    # inches
    robot_width = 9.5
    turn_circumference = math.pi * robot_width
    distance = (angle/360.0) * turn_circumference
    # move_motors(distance, distance, -1 * distance, -1 * distance)
    move_motors(speed, speed, -distance, -distance, distance, distance, buffer)
    drive_stop

    #(x (front back),y(left right)) 
def add_waypoint(speed, x_pos, y_pos, face_angle):
    waypoint = [speed, x_pos, y_pos, face_angle]
    waypoints.append(waypoint)

def localize():
    global waypoints
    current_position = [0,0]
    current_angle = 0
    while(len(waypoints) != 0):
        target_position = waypoints[0]
        speed = target_position[0]
        x_pos = target_position[1]
        y_pos = target_position[2]
        face_angle = target_position[3]

        # other tan acute angle
        target_angle = math.atan2(y_pos - current_position[1], x_pos - current_position[0]) * (180/math.pi) - current_angle
        print("target angle: " + str(target_angle))
        
        if target_angle > 180:
            target_angle -= 360
        elif target_angle < -180:
            target_angle += 360
        turn_center(speed, target_angle, 0)

        new_angle = current_angle + target_angle
        print("new angle: " + str(new_angle))
        target_distance = math.sqrt(math.pow(x_pos - current_position[0], 2) + math.pow(y_pos - current_position[1], 2))
        print("target distance: " + str(target_distance))
       
        drive_straight(speed, target_distance, 0)
        print("face angle: " + str(face_angle - new_angle))
        turn_center(speed, face_angle - new_angle, 0)
        
        current_position[0] = x_pos
        current_position[1] = y_pos
        current_angle = face_angle
        
        print("next")
        waypoints.pop(0)

        time.sleep(1)



tics_per_rev = 2443
wheel_circumference = 4 * math.pi
left_side = 0x81
right_side = 0x80
position = []
waypoints = []
active_opmode = True
startup = True

while startup:
    if startup:
        initencoder()
        startup = False

if active_opmode:
    add_waypoint(2500, 20, -20, 90)
    add_waypoint(2500, 32, -20, 30)
    add_waypoint(2500, 0, 0, 0)
    
    # # add_waypoint(1000, 50, -20, 50)
    # # add_waypoint(700, 50, 0, 50)
    localize()
    # show_encoder()
    # drive_straight(1500, 15, 0)
    # show_encoder()

    # turn_center(7000, 90, 0)

    active_opmode = False
      

    
    
	
    
