import numpy as np
import math
import time
import api_controller as api
import generate_bezier as gb

# update----------------------
# odometry causes list out of bounds


# Parameters
k = 0.1  # look forward gain
Lfc = 0.5  # [inch] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle

start_time = time.time()
total_time = 0
prev_time = 0
elapsed_time = 0


def time_elapsed():
    global total_time
    global prev_time
    global elapsed_time
    total_time = time.time() - start_time
    elapsed_time = total_time - prev_time
    # prev_time = total_time


def status_update():
    global current_heading
    global current_position
    left_tics, right_tics = api.read_encoders()
    left = api.inch_distance(left_tics)
    right = api.inch_distance(right_tics)
    theta = (right - left) / api.robot_width
    center = (left + right) / 2

    rad_final_angle = (api.current_heading) + theta

    # x position
    api.current_position[0] = center * (math.cos(rad_final_angle))
    # y position
    api.current_position[1] = center * (math.sin(rad_final_angle))
    # true heading in radians
    api.current_heading = rad_final_angle
    api.normalize_radians()
    print("current position: " + str(api.current_position))
    print("current heading: " + str(api.current_heading))
    # print("left: " + str(left) + " tics: " + str(left_tics))
    # print("right: " + str(right) + " tics: " + str(right_tics))


class State:
    def __init__(self,
                 x=0.0,
                 y=0.0,
                 yaw=0.0,
                 v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        # self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        # self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, alpha):
        status_update()
        # self.x = api.current_position[0]
        # self.y = api.current_position[1]
        # self.yaw = api.current_heading
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(alpha) * dt
        self.v += a * dt
        # self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        # self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        omega = alpha / dt
        vel_R = self.v + (omega * 0.8 / 2)
        vel_L = self.v - (omega * 0.8 / 2)
        print("delta: " + str(alpha))
        print("omega: " + str(omega))
        print("velocity: " + str(self.v))
        print("velocity right: " + str(vel_R))
        print("velocity left: " + str(vel_L) + "\n")
        api.motor_speed(api.tic_distance(vel_L)*2, api.tic_distance(vel_R)*2)

    def calc_distance(self, point_x, point_y):
        # dx = self.rear_x - point_x
        # dy = self.rear_y - point_y
        dx = self.x - point_x
        dy = self.y - point_y
        # print("calc dist: " + str(point_x) + ", " + str(point_y))
        return math.hypot(dx, dy)


class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            # dx = [state.rear_x - icx for icx in self.cx]
            # dy = [state.rear_y - icy for icy in self.cy]
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(
                    self.cx[ind + 1], self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    # alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw


    # delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return alpha, ind


def run(x_list, y_list):
    global start_time
    #  target course
    cx = x_list
    cy = y_list

    target_speed = 10.0 / 1 # [m/s]

    # initial state
    state = State(x=-0.0, y=0.0, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    times = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while lastIndex > target_ind:
        global prev_time
        # Calc control input
        ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(state, target_course,
                                                    target_ind)

        state.update(ai, di)  # Control vehicle

        times += dt
        states.append(time, state)
        time.sleep(0.1)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"


def to_waypoint():
    # x_list, y_list = gb.generate_bezier_pathing(x_pos, y_pos, face_angle, curvature)
    x_list = [0, 
        2.167638483965014, 3.9941690962099137, 5.536443148688045,
        6.851311953352769, 7.995626822157433, 9.026239067055393, 10.0,
        10.973760932944607, 12.004373177842567, 13.14868804664723,
        14.463556851311953, 16.005830903790088, 17.832361516034982, 20
    ]
    y_list = [0, 
        0.29154518950437314, 1.1078717201166182, 2.3615160349854225,
        3.965014577259475, 5.830903790087463, 7.871720116618075, 10.0,
        12.128279883381925, 14.169096209912539, 16.034985422740526,
        17.63848396501458, 18.89212827988338, 19.708454810495624, 20
    ]
    run(x_list, y_list)
