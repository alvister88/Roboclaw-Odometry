import math
import time
import api_controller as api


def generate_relative_bezier_pathing(x_dist, y_dist, angle, curve_radius):
    global current_position
    global current_heading
    x_pos = current_position[0] + x_dist
    y_pos = current_position[1] + y_dist
    face_angle = current_heading + angle
  
    generate_bezier_pathing(x_pos, y_pos, face_angle, curve_radius)

def generate_bezier_pathing(x_pos, y_pos, face_angle, curvature):
    global current_position
    global current_heading
    # generated waypoints for curve
    
    waypoints = []
    end_point = (float(x_pos), float(y_pos))
    end_angle = api.to_radians(face_angle)

    theta1 = api.to_radians(current_heading)
    theta2 = end_angle

    p1 = current_position
    # projected along the start angle vector
    p2 = (current_position[0] + curvature * math.cos(theta1),
          current_position[1] + curvature * math.sin(theta1))
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

    current_position = [x_pos, y_pos]
    current_heading = face_angle
    api.normalize()
  
    def generate_waypoints():
        for t in range(1, amt_waypoints):
            waypoint = bezier(t / (amt_waypoints))
            waypoints.append(waypoint)
            print(waypoint)
        waypoints.append(end_point)
        print(end_point)

    generate_waypoints()

    return waypoints
  