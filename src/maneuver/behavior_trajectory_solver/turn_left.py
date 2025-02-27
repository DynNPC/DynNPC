import random
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '/mnt/sda/AdvFuzz/src/maneuver')))
from utils import bezier_derivative, raycast_to_ground, bezier_point, calculate_angle
import lgsvl
import numpy as np
import math


def turn_left(sim, npc_status, forward, right, num_points, n, m):
    """
    Generate left turn path points with interpolated angles.
    npc_status = [npc.state.position, speed]
    """
    waypoints = []
    speed = npc_status[1]  # Adjust speed as necessary, especially to decrease during the turn
    k = np.random.uniform(0, m)
    # Set Bezier control points for a right turn
    P0 = npc_status[0]
    P1 = P0 + forward * n  # Move forward
    P2 = P1 - right * k  # Start turning right
    P3 = P2 - right * (m-k)   # Finish the turn

    # Calculate initial and final angles
    start_angle, end_angle = calculate_initial_final_angles(forward, right)

    # Generate waypoints using the Bezier curve and interpolate angles
    for i in range(num_points + 1):
        t = i / num_points
        current_point = bezier_point(t, P0, P1, P2, P3)
        ground_point = raycast_to_ground(sim, current_point)
        interpolated_angle = linear_interpolate_angle(
            start_angle, end_angle, t)
        # print(interpolated_angle)
        # Create a waypoint at this position with the interpolated angle
        wp = lgsvl.DriveWaypoint(
            ground_point, speed, angle=lgsvl.Vector(0, interpolated_angle, 0))
        waypoints.append(wp)

    return waypoints



def linear_interpolate_angle(start_angle, end_angle, t):
    """Linearly interpolate between two angles."""
    return start_angle + (end_angle - start_angle) * t


def calculate_initial_final_angles(forward, right):
    """Calculate the initial and final angles based on direction vectors."""
    start_angle = np.degrees(np.arctan2(forward.x, forward.z))
    # Turn 90° to the left, counterclockwise-
    end_angle = start_angle-90
    # If the end angle is less than the initial angle, it means that a positive rotation is required
    return start_angle, end_angle

