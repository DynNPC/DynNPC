import math
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '/mnt/sda/AdvFuzz/src/maneuver')))
from generate_behavior_trajectory import generate_behavior_trajectory
import numpy as np
import random

from loguru import logger
import lgsvl
from utils import raycast_to_ground

def smoothSpeed(sim, npc, waypoints, forward):

    npc_speed = npc.state.speed
    npc_position = npc.state.position
    npc_forward = lgsvl.utils.transform_to_forward(npc.transform)
    start_speed = waypoints[0].speed
    before_waypoints = []
    angle=npc.state.transform.rotation
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
 
    acc = start_speed - npc_speed 
    print("acc is :", acc)
    point_num = 12 if npc_speed > 6 else 4 
    for i in range(point_num):
        new_position = npc_position + npc_forward * (i+1)
        ground_point = sim.map_point_on_lane(raycast_to_ground(sim, new_position)).position
        new_speed = npc_speed + acc * (i+1) / point_num
        before_waypoints.append(lgsvl.DriveWaypoint(ground_point, new_speed, angle=angle))
    
 
    for i in range(len(waypoints)):
        waypoints[i].position = waypoints[i].position + before_waypoints[-1].position - npc_position
        
    before_waypoints.extend(waypoints)
    

    return before_waypoints
    
        
def keepSpeed(sim, npc, waypoints, forward):

    keep_speed = waypoints[-1].speed
    start_position = waypoints[-1].position
 
    angle = waypoints[-1].angle
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
  
    keep_waypoints = []
    for i in range(30):
        new_position = start_position + forward * (i+1) * 2
        ground_point = sim.map_point_on_lane(raycast_to_ground(sim, new_position)).position
        keep_waypoints.append(lgsvl.DriveWaypoint(ground_point, keep_speed, angle=angle))
    return keep_waypoints
    

def on_waypoint(agent, index, waypoints, sim, npc, forward):
  
    total_waypoints = len(waypoints)
    

    if index == total_waypoints - 1:

        after_waypoints = keepSpeed(sim, npc, waypoints, forward)
  
        agent.follow(after_waypoints, loop=False)


def on_waypoint_acc(agent, index, waypoints, sim, npc, ego, forward):

    total_waypoints = len(waypoints)

    if index == total_waypoints - 1:
        ego_speed = ego.state.speed
        last_waypoints = []
        start_position = npc.state.position
   
        angle=npc.state.transform.rotation
        angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
        for i in range(10):
            new_position = start_position + forward * (i + 1) * 2
            ground_point = sim.map_point_on_lane(raycast_to_ground(sim, new_position)).position
            last_waypoints.append(lgsvl.DriveWaypoint(ground_point, ego_speed, angle=angle))
        all_waypoints = smoothSpeed(sim, npc, last_waypoints, forward)
        agent.follow(all_waypoints, loop=False)
            

def handle_zone_L1(self, npc, forward, right, index, ninegrid_flags, waypoints_flags, action_change_freq):
    if random.random() < 0.5:
  
        if waypoints_flags[index]:
            waypoints = generate_behavior_trajectory(
                self.sim, self.ego, npc, forward, right, 'cut_in_left', num_points=20, n=5, m=5, k=5)
            logger.info(f"{npc.name} is performing cut_in_left.")
            waypoints_flags[index] = False
            ninegrid_flags[index] = False
            waypoints = smoothSpeed(self.sim, npc, waypoints, forward)
            npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                agent, index, waypoints, self.sim, npc, forward))
            npc.follow(waypoints, loop=False)
    else:
        waypoints_flags[index] = False
        ninegrid_flags[index] = False
        print("random as follow")
        npc_start_position = npc.state.position
        
     
      
        npc_speed = npc.state.speed
     
        angle=npc.state.transform.rotation
        angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
        waypoints = []
        for i in range(20):
            new_position = npc_start_position + forward * (i + 1) * 2
            ground_point = self.sim.map_point_on_lane(raycast_to_ground(self.sim, new_position)).position
            waypoints.append(lgsvl.DriveWaypoint(ground_point, npc_speed, angle=angle))
     
        npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                agent, index, waypoints, self.sim, npc, forward))
        npc.follow(waypoints, loop=False)
   


def handle_zone_N1(self, npc, forward, right, index, ninegrid_flags, waypoints_flags, action_change_freq):
    a=1


def handle_zone_R2(self, npc, forward, right, index, ninegrid_flags, waypoints_flags, action_change_freq):

    if waypoints_flags[index]:
        waypoints = generate_behavior_trajectory(
            self.sim, self.ego, npc, forward, right, 'cut_in_right', num_points=20, n=10, m=5, k=5)
        logger.info(f"{npc.name} is performing cut_in_right.")
        waypoints_flags[index] = False
        ninegrid_flags[index] = False
        waypoints = smoothSpeed(self.sim, npc, waypoints, forward)
        npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                agent, index, waypoints, self.sim, npc, forward))
        npc.follow(waypoints, loop=False)


def handle_zone_L2(self, npc, forward, right, index, ninegrid_flags, waypoints_flags, action_change_freq):

    if waypoints_flags[index]:
        waypoints = generate_behavior_trajectory(
            self.sim, self.ego, npc, forward, right, 'cut_in_left', num_points=20, n=10, m=5, k=5)
        logger.info(f"{npc.name} is performing cut_in_left.")
        waypoints_flags[index] = False
        ninegrid_flags[index] = False
        waypoints = smoothSpeed(self.sim, npc, waypoints, forward)
        npc.on_waypoint_reached(lambda agent, index: on_waypoint(
           agent, index, waypoints, self.sim, npc, forward))
        npc.follow(waypoints, loop=False)


def handle_zone_R1(self, npc, forward, right, index, ninegrid_flags, waypoints_flags, action_change_freq):
    if random.random() < 0.5:

        if waypoints_flags[index]:
            waypoints = generate_behavior_trajectory(
                self.sim, self.ego, npc, forward, right, 'cut_in_right', num_points=20, n=5, m=5, k=5)
            logger.info(f"{npc.name} is performing cut_in_right.")
            
            waypoints = smoothSpeed(self.sim, npc, waypoints, forward)
            npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                agent, index, waypoints, self.sim, npc, forward))
            npc.follow(waypoints, loop=False)
    else:
        waypoints_flags[index] = False
        ninegrid_flags[index] = False
        npc_start_position = npc.state.position
        
 
        npc_speed = npc.state.speed
        
        angle=npc.state.transform.rotation
        angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
        waypoints = []
        for i in range(20):
            new_position = npc_start_position + forward * (i + 1) * 2
            ground_point = self.sim.map_point_on_lane(raycast_to_ground(self.sim, new_position)).position
            waypoints.append(lgsvl.DriveWaypoint(ground_point, npc_speed, angle=angle))
       
        npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                agent, index, waypoints, self.sim, npc, forward))
        npc.follow(waypoints, loop=False)
       


def handle_zone_L3_R3(self, npc, forward, index, ninegrid_flags, waypoints_flags):

    ninegrid_flags[index] = False
    waypoints_flags[index] = False
    
    npc_speed = npc.state.speed

    npc_start_position = npc.state.position
    angle=npc.state.transform.rotation
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    logger.info(f"{npc.name} is performing acceleration.")
    waypoints = []
    for i in range(20):
        new_position = npc_start_position + forward * 2 * i
        speed = min(npc_speed + 1 * (i+1), 10)
        ground_point = self.sim.map_point_on_lane(raycast_to_ground(self.sim, new_position)).position
        waypoints.append(lgsvl.DriveWaypoint(ground_point, speed, angle=angle))
   
    npc.on_waypoint_reached(lambda agent, index: on_waypoint_acc(
                agent, index, waypoints, self.sim, npc, self.ego, forward))
    npc.follow(waypoints, loop=False)


def handle_zone_F1orN1(self, npc, forward, index, ninegrid_flags, waypoints_flags, region):
    ninegrid_flags[index] = False
    waypoints_flags[index] = False
        
    npc_start_position = npc.state.position
    npc_speed = npc.state.speed
    npc_forward = lgsvl.utils.transform_to_forward(npc.transform)
    angle=npc.state.transform.rotation
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    waypoints = []
    flag = 0
    if region == 0:

        npc_position = npc.state.position
        ego_position = self.ego.state.position
        relative_position = lgsvl.Vector(ego_position.x - npc_position.x,
                                          ego_position.y - npc_position.y,
                                          ego_position.z - npc_position.z)
        dot = npc_forward.x * relative_position.x + npc_forward.y * relative_position.y + npc_forward.z * relative_position.z
        if dot > 0:
            flag = 1
        else:
            flag = -1 
    elif region == 2:
        flag = -1
    elif region == 7:
        flag = 1
    
    if flag == 1:
       
        logger.info(f"{npc.name} is keeping speed.")
     
        for i in range(20):
            new_position = npc_start_position + forward * (i + 1) * 2
            ground_point = self.sim.map_point_on_lane(raycast_to_ground(self.sim, new_position)).position
            waypoints.append(lgsvl.DriveWaypoint(ground_point, npc_speed, angle=angle))
        npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                agent, index, waypoints, self.sim, npc, forward))
        npc.follow(waypoints, loop=False)
    elif flag == -1:
       
        logger.info(f"{npc.name} is following lane until stop.")
        acc = npc_speed / 20
        for i in range(20):
            new_position = npc_start_position + forward * (i + 1) * 2
            ground_point = self.sim.map_point_on_lane(raycast_to_ground(self.sim, new_position)).position
            new_speed = max(npc_speed - acc * (i + 1), 0)
            waypoints.append(lgsvl.DriveWaypoint(ground_point, new_speed, angle=angle))
        npc.follow(waypoints, loop=False)


def handle_zone_none(self, npc, forward, index, ninegrid_flags, waypoints_flags):
    ninegrid_flags[index] = False
    waypoints_flags[index] = False
    npc_speed = npc.state.speed
    if npc_speed == 0:
        logger.info(f"{npc.name} is following closest lane.")
        npc.follow_closest_lane(True, random.random()*7 + 3)
    else:
        logger.info(f"{npc.name} is keeping speed.")
        waypoints = []
        npc_start_position = npc.state.position
       
        angle=npc.state.transform.rotation
        angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
        print(angle)
        for i in range(20):
            new_position = npc_start_position + forward * (i + 1) * 2
           
            ground_point = self.sim.map_point_on_lane(raycast_to_ground(self.sim, new_position)).position
           
            waypoints.append(lgsvl.DriveWaypoint(ground_point, npc_speed, angle=angle))
       
        npc.on_waypoint_reached(lambda agent, index: on_waypoint(
            agent, index, waypoints, self.sim, npc, forward))
        
        npc.follow(waypoints, loop=False)
       