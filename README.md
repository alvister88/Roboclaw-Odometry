# Bezier Curve Pure Pursuit

Code utilizes the roboclaw motor controller to control robot movement. A four point bezier curve is used to generate a path enabling a robot with two degrees of freedom to account for three degrees of freedom. In this case, a differential drive robot can take in three parameters: x<sub>position</sub>, y<sub>position</sub>, end orientation.


## Desmos calcuation and graphs for 4-point bezier curve
The heading and positions match that of a robot if the graph is rotated 90°

https://www.desmos.com/calculator/fhjjndlpno

## Simulation
Source code for simulation in bpp_simulation. Simulation adapted to follow waypoints from generate_bezier algorithm.
https://replit.com/@AlvinZhu3/pure-pursuit#main.py

# NOTES
- api_controller is a handler for the roboclaw_3.py api
  - adapted methods from the base api so that it's usable
- generate_bezier.py fully working pathfinding algorithm creates waypoints based on defined interval and returns a list of x coordinates and list of y coordinates
  - amount of waypoints adapted to distance interval between each waypoint
- localization_2.py is a fully working simple odometry code with drive to coordinates, drive to position, and backtracking
- pure_pursuit.py only works for one velocity, may need to make another path following algorithm

