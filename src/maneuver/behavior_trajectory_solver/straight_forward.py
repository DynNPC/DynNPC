import random
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '/mnt/sda/AdvFuzz/src/maneuver')))
from utils import bezier_derivative, raycast_to_ground, bezier_point, calculate_angle
import lgsvl
import numpy as np
import math


def straight_forward(sim, npc_status, forward, right, num_points, n):
    """
    Generate straight path points .
    npc_status = [npc.state.position, speed]
    """
    waypoints = []
    speed = npc_status[1]  # Adjust speed as necessary, especially to decrease during the turn
    # Set Bezier control points for a right turn
    
    P0 = npc_status[0]
    P1 = P0 + forward * (n / 3) # Move forward
    P2 = P1 + forward * (n / 3)  # Start turning right
    P3 = P2 + forward * (n / 3)   # Finish the turn

    # Calculate initial and final angles
    # start_angle, end_angle = calculate_initial_final_angles(forward, right)
    #angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    # Generate waypoints using the Bezier curve and interpolate angles
    for i in range(num_points + 1):
        t = i / num_points
        current_point = bezier_point(t, P0, P1, P2, P3)
        ground_point = raycast_to_ground(sim, current_point)
        
        # print(interpolated_angle)
        # Create a waypoint at this position with the interpolated angle
        wp = lgsvl.DriveWaypoint(
            ground_point, speed, angle=angle)
        waypoints.append(wp)

    return waypoints


