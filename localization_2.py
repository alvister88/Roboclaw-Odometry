from stringprep import in_table_a1
import time
import math
from tracemalloc import stop
import numpy as np
from roboclaw_3 import Roboclaw
from multipledispatch import dispatch

# Alvin's attempt at python localization x:)

#Linux comport name
rcL = Roboclaw("/dev/ttyACM0", 115200)
rcR = Roboclaw("/dev/ttyACM1", 115200)
rcL.Open()
rcR.Open()


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
    return ((inches / wheel_circumference) * tics_per_rev)

# (tics/s, tics/s, inches, inches, inches, inches, (0 or 1))
def move_motors(speed_L, speed_R, L_front, L_back, R_front, R_back, interval):
    
    L_front = tic_distance(L_front) + int(rcL.ReadEncM1(left_side)[1])
    L_back = tic_distance(L_back) + int(rcL.ReadEncM2(left_side)[1])
    R_front = tic_distance(R_front) + int(rcR.ReadEncM1(right_side)[1])
    R_back = tic_distance(R_back) + int(rcR.ReadEncM2(right_side)[1])

    accel = 3000
    deccel = 3000
   
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

    time.sleep(0)
    
def drive_straight(speed, distance, interval):
    move_motors(speed, speed, distance, distance, distance, distance, interval)

def drive_stop():
    move_motors(0,0,0,0,0,0,0)

# turn centered around back wheel axle
def turn_center(speed, angle, interval):
    global current_heading
    global current_position
    # robot_length = 13.3
    # turn_radius_offset = robot_length / 2
    robot_width = 13.5

    turn_circumference = math.pi * robot_width
    distance = (angle/360.0) * turn_circumference
    move_motors(speed, speed, -distance, -distance, distance, distance, interval)
    current_heading += angle
    normalize()

    # account for position offset after turn
    # rad_final_angle = (current_heading + angle) * math.pi / 180
    # rad_init_angle = current_heading * math.pi / 180
    # y_offset = turn_radius_offset * (math.sin(rad_final_angle) - math.sin(rad_init_angle))
    # x_offset = turn_radius_offset * (math.cos(rad_final_angle) - math.cos(rad_init_angle))

    # current_position[0] += x_offset
    # current_position[1] += y_offset

# counter clockwise positve, 0 -> 180; 0 -> -180
def turn_heading(speed, new_heading, interval):
    global current_heading
    turn_angle = new_heading - current_heading

    if turn_angle > 180:
        turn_angle -= 360
    elif turn_angle < -180:
        turn_angle += 360

    turn_center(speed, turn_angle, interval)
    current_heading = new_heading
    normalize()

# normalize global heading
def normalize():
    global current_heading

    while current_heading > 180:
        current_heading -= 360
    while current_heading < -180:
        current_heading += 360

# drive relative to current position
@dispatch(int, int, int, int)
def drive_to_position(speed, x_dist, y_dist, face_angle):
    global current_position
    global current_heading
    global locations

    print("start drive to position")
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

    locations.append([current_position[0], current_position[1], face_angle])

    print("current position: " + str(current_position))
    print("current heading: " + str(current_heading) + "\n")

@dispatch(int, int, int)
def drive_to_position(speed, x_dist, y_dist):
    global current_position
    global current_heading
    global locations

    print("start drive to position")
    turn_angle = math.atan2(y_dist, x_dist) * (180/math.pi)
    print("turn angle: " + str(turn_angle))
    
    turn_center(speed, turn_angle, 0)
    
    target_distance = math.sqrt(math.pow(x_dist, 2) + math.pow(y_dist, 2))
    print("target distance: " + str(target_distance))
    
    drive_straight(speed, target_distance, 0)
    
    current_position[0] += x_dist
    current_position[1] += y_dist

    locations.append([current_position[0], current_position[1]])

    print("current position: " + str(current_position))
    print("current heading: " + str(current_heading) + "\n")
    

# drive to absolute location; (x,y) plane
@dispatch(int, int, int, int)
def drive_to_location(speed, x_pos, y_pos, face_angle):
    global current_position
    global current_heading

    print("start drive to location")
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

    locations.append([current_position[0], current_position[1], face_angle])

    print("current position: " + str(current_position))
    print("current heading: " + str(current_heading) + "\n")

@dispatch(int, int, int)
def drive_to_location(speed, x_pos, y_pos):
    global current_position
    global current_heading

    print("start drive to location")
    # other tan acute angle
    target_heading = math.atan2(y_pos - current_position[1], x_pos - current_position[0]) * (180/math.pi)
    print("target heading: " + str(target_heading))
    
    turn_heading(speed, target_heading, 0)

    print("current heading: " + str(current_heading))
    target_distance = math.sqrt(math.pow(x_pos - current_position[0], 2) + math.pow(y_pos - current_position[1], 2))
    print("target distance: " + str(target_distance))
    
    drive_straight(speed, target_distance, 0)
    
    current_position[0] = x_pos
    current_position[1] = y_pos

    print("current position: " + str(current_position))
    print("current heading: " + str(current_heading) + "\n")

# back track "movements" amount of locations
def backtrack(speed, movements):
    global locations
    locations_amount = len(locations)
    has_face_angle = True
    last_face_angle = 0

    if movements == "all":
        movements = locations_amount
        print(movements)
    
    while len(locations) > locations_amount - movements:
        print("backtrack motion")
        target_location = locations[len(locations)-1]
        x_pos = target_location[0]
        y_pos = target_location[1]
        try:
            last_face_angle = target_location[2]
            has_face_angle = True
        except IndexError:
            has_face_angle = False
        drive_to_location(speed, x_pos, y_pos)
        locations.pop(len(locations)-1)

    if has_face_angle:
        turn_heading(speed, last_face_angle, 0)

#(x (front back),y(left right)) 
@dispatch(int, int, int, int)
def add_waypoint(speed, x_pos, y_pos, face_angle):
    waypoint = [speed, x_pos, y_pos, face_angle]
    waypoints.append(waypoint)
@dispatch(int, int, int)
def add_waypoint(speed, x_pos, y_pos):
    waypoint = [speed, x_pos, y_pos]
    waypoints.append(waypoint)

def localize():
    global waypoints
    global current_position
    global current_heading
    
    while len(waypoints) != 0:
        target_location = waypoints[0]
        speed = target_location[0]
        x_pos = target_location[1]
        y_pos = target_location[2]

        try:
            face_angle = target_location[3]
            drive_to_location(speed, x_pos, y_pos, face_angle)
        except IndexError:
            drive_to_location(speed, x_pos, y_pos)

        waypoints.pop(0)

        # time.sleep(0.1)

tics_per_rev = 2442.96
wheel_circumference = 4 * math.pi
left_side = 0x81
right_side = 0x80
waypoints = []
locations = []
active_opmode = True
startup = True

# use mid encoder normalize to prevent wrap around
MIN_QUADRATURE_VALUE: np.uint = 0
MAX_QUADRATURE_VALUE: np.uint = 500000
MID_QUADRATURE_VALUE = np.uint = int(MAX_QUADRATURE_VALUE / 2)

# absolute position and angle heading
current_position = [0,0]
current_heading = 0


while startup:
    if startup:
        normalize_encoders()
        locations.append([0, 0, 0])
        startup = False

if active_opmode:
    
    # show_encoder()
    add_waypoint(7000, 20, -10, 120)
    add_waypoint(7000, 30, -20)
    localize()
    drive_to_position(5000, 10, 0, 90)
    drive_to_position(5000, 10, 10)
    backtrack(5000, "all")

    # add_waypoint(10000, 40, 40, 0) 
    # add_waypoint(10000, 60, 0, -90)
    # add_waypoint(10000, 20, -10, -180)
    # add_waypoint(10000, 0, 0, 0)
    # localize()
    # turn_center(4000, 360, 0)
    

    # turn_center(2000, 1000, 0)
    # print(current_heading)
    print("current position: " + str(current_position))
    print("current heading: " + str(current_heading))
    active_opmode = False
      

    
    
	
    
