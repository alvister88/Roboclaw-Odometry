import math
import time

def to_radians(angle):      
    return angle * math.pi / 180

def generate_relative_bezier_pathing(x_dist, y_dist, angle, curve_radius):
    x_pos = 0 + x_dist
    y_pos = 0 + y_dist
    face_angle = 0 + angle
  
    generate_bezier_pathing(x_pos, y_pos, face_angle, curve_radius)

def generate_bezier_pathing(x_pos, y_pos, face_angle, curvature):
    # generated waypoints for curve
    
    x_waypoints = []
    y_waypoints = []
    end_point = (float(x_pos), float(y_pos))
    end_angle = to_radians(face_angle)

    theta1 = to_radians(0)
    theta2 = end_angle

    p1 = [0,0]
    # projected along the start angle vector
    p2 = (0 + curvature * math.cos(theta1),
          0 + curvature * math.sin(theta1))
    # projected opposite of the end angle vector
    p3 = (end_point[0] - curvature * math.cos(theta2),
          end_point[1] - curvature * math.sin(theta2))
    p4 = end_point

    def bezier(t):
        not_t = 1 - t
        x_val = not_t**3 * p1[0]
        x_val += 3 * not_t**2 * t * p2[0]
        x_val += 3 * not_t * t**2 * p3[0]
        x_val += t**3 * p4[0]

        y_val = not_t**3 * p1[1]
        y_val += 3 * not_t**2 * t * p2[1]
        y_val += 3 * not_t * t**2 * p3[1]
        y_val += t**3 * p4[1]

        return (x_val, y_val)

    def get_curve_length(end, end_angle, curvature):
        intervals = 750
        dist = 0
        for i in range(intervals):
            p69 = bezier(i / intervals)
            p70 = bezier((i + 1) / intervals)
            dx = p70[0] - p69[0]
            dy = p70[1] - p69[1]
            dist += math.sqrt(dx * dx + dy * dy)

        print(dist)
        return dist

    curve_length = get_curve_length(end_point, end_angle, curvature)
    # ideal inches between 2 waypoints
    point_interval = 2.0
    # prevent uneven distance between 2 waypoints
    true_point_interval = curve_length / int(curve_length / point_interval)
    amt_waypoints = int(curve_length / true_point_interval)
    print(true_point_interval)
    print(amt_waypoints)
  
    def generate_waypoints():
        for t in range(amt_waypoints):
            x = bezier(t / (amt_waypoints))[0]
            y = bezier(t / (amt_waypoints))[1]
            x_waypoints.append(x)
            y_waypoints.append(y)
            print("(" + str(x) + ", " + str(y) + ")")
            # print(x, end = ", ")
            # print(y, end = ", ")
        x_waypoints.append(end_point[0])
        y_waypoints.append(end_point[1])
        print(end_point)

    generate_waypoints()

    return x_waypoints, y_waypoints
  