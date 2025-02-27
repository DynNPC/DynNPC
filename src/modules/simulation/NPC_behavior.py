import random
import sys
import os

import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '/mnt/sda/AdvFuzz/src/maneuver')))
from loguru import logger
from generate_behavior_trajectory import generate_behavior_trajectory
import lgsvl
from utils import raycast_to_ground
import os
#from NineGrid import NineGrid
import math

# TODO acc zone
def smoothSpeed(sim, npc, waypoints, forward):
    #   ====8m/3m====
    npc_speed = npc.state.speed
    npc_position = npc.state.position
 
    start_speed = waypoints[0].speed
    before_waypoints = []
    
 
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    
    acc = start_speed - npc_speed 
    point_num = 8 if npc_speed > 6 else 3 
    for i in range(point_num):
        new_position = npc_position + forward * (i+1)
        new_position = raycast_to_ground(sim, new_position)
   
        ground_point = sim.map_point_on_lane(new_position).position
        new_speed = npc_speed + acc * (i+1) / point_num
        before_waypoints.append(lgsvl.DriveWaypoint(ground_point, new_speed, angle=angle))
    
  
    for i in range(len(waypoints)):
        waypoints[i].position = waypoints[i].position + before_waypoints[-1].position - npc_position
    before_waypoints.extend(waypoints)
    
    return before_waypoints


def keepSpeed(sim, npc, forward, waypoints):
  
    keep_speed = waypoints[-1].speed
    start_position = waypoints[-1].position
    # npc_forward = lgsvl.utils.transform_to_forward(npc.state.transform)
    # angle = waypoints[-1].angle
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    # 再生成一段保持速度和方向的路径点
    keep_waypoints = []
    for i in range(2):
        new_position = start_position + forward * (i+1)
        new_position = raycast_to_ground(sim, new_position)
        # if new_position is  None:
        #     return True
        ground_point = sim.map_point_on_lane(new_position).position
        keep_waypoints.append(lgsvl.DriveWaypoint(ground_point, keep_speed, angle=angle))
        
    waypoints.extend(keep_waypoints)
    return waypoints



# delete random as follow to make more interactive actions, So as L2
def handle_zone_L1(sim, ego, npc, forward, right, speed_record):
    
       
        waypoints = generate_behavior_trajectory(
            sim, ego, npc, forward, right, 'cut_in_left', num_points=20, n=5, m=5, k=5)
        logger.info(f"{npc.name} is performing cut_in_left.")
        return waypoints


def handle_zone_L2(sim, ego, npc, forward, right, speed_record):
  
      
        waypoints = generate_behavior_trajectory(
            sim, ego, npc, forward, right, 'cut_in_left', num_points=20, n=10, m=5, k=5)
        logger.info(f"{npc.name} is performing cut_in_left.")
        return waypoints
        

# delete random as follow to make more interactive actions, So as R2
def handle_zone_R1(sim, ego, npc, forward, right, speed_record):
    
        
        waypoints = generate_behavior_trajectory(
            sim, ego, npc, forward, right, 'cut_in_right', num_points=20, n=5, m=5, k=5)
        logger.info(f"{npc.name} is performing cut_in_right.")
        return waypoints


def handle_zone_R2(sim, ego, npc, forward, right, speed_record):
    
     
        waypoints = generate_behavior_trajectory(
            sim, ego, npc, forward, right, 'cut_in_right', num_points=20, n=10, m=5, k=5)
        logger.info(f"{npc.name} is performing cut_in_right.")
        return waypoints
    

def handle_zone_L3_R3(sim, npc, forward, speed_record): 
    
   
    npc_speed = speed_record[npc.name]
    
    npc_start_position = npc.state.position
   
    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    logger.info(f"{npc.name} is performing acceleration.")
    waypoints = []
    
    for i in range(10):
        new_position = npc_start_position + forward * (i+1)
        speed = min(npc_speed + 1 * (i+1), 12)
        new_position = raycast_to_ground(sim, new_position)
      
        ground_point = sim.map_point_on_lane(new_position).position
        waypoints.append(lgsvl.DriveWaypoint(ground_point, speed, angle=angle))
    return waypoints


def handle_zone_F1orN1(sim, ego, npc, forward, region, speed_record, timer):
    npc_start_position = npc.state.position
    npc_name = npc.name
    npc_forward = lgsvl.utils.transform_to_forward(npc.state.transform)

    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    waypoints = []
  
    flag = 0
    if region == 0:
        
        npc_position = npc_start_position
        ego_position = ego.state.position
        relative_position = lgsvl.Vector(ego_position.x - npc_position.x,
                                          ego_position.y - npc_position.y,
                                          ego_position.z - npc_position.z)
        dot = npc_forward.x * relative_position.x + npc_forward.y * relative_position.y + npc_forward.z * relative_position.z
        if dot > 0:
            flag = 1 # region = 7，F1
        else:
            flag = -1 # region = 2，N1
    elif region == 2:
        flag = -1
    elif region == 7:
        flag = 1
    
    if flag == 1:
       
        
        logger.info(f"{npc_name} is following keeping speed.")
   
        npc_speed = speed_record[npc_name]
        for i in range(5):
            new_position = npc_start_position + forward * (i + 1)
     
            new_position = raycast_to_ground(sim, new_position)
    
            ground_point = sim.map_point_on_lane(new_position).position
            waypoints.append(lgsvl.DriveWaypoint(ground_point, npc_speed, angle=angle))
    elif flag == -1:
    
        timer[npc.name] = 2
        sL = np.random.uniform(1, 2)
        sM = np.random.uniform(2, 4)
        sH = np.random.uniform(4, 6)
        seed = random.random() * 3
        if seed < 1:
            if seed < 0.5:
                speed1 = sM
                speed2 = sH
                strategy = "medium -> high"
            else:
                speed2 = sM
                speed1 = sH
                strategy = "high -> medium"
        elif seed < 2:
            if seed < 1.5:
                speed1 = sL
                speed2 = sH
                strategy = "low -> high"
            else:
                speed1 = sH
                speed2 = sL
                strategy = "high -> low"
        else:
            if seed < 2.5:
                speed1 = sL
                speed2 = sM
                strategy = "low -> medium"
            else:
                speed1 = sM
                speed2 = sL
                strategy = "medium -> low"
        speed1 = sL
        speed2 = sL
        acc = speed2 - speed1
        
        logger.info(f"{npc_name} is following lane until stop. Strategy: {strategy}")
        for i in range(20):
            new_position = npc_start_position + forward * (i + 1)
            new_position = raycast_to_ground(sim, new_position)
            ground_point = sim.map_point_on_lane(new_position).position

            if i < 8:
                new_speed = speed1
            elif i < 10:
                new_speed = speed1 + acc / 2 * (i - 7)
            elif i < 18:
                new_speed = speed2
            else:
                new_speed = speed2 - speed2 / 3 * (i - 17)
            waypoints.append(lgsvl.DriveWaypoint(ground_point, new_speed, angle=angle))

        
    return waypoints


def handle_zone_none(sim, npc, forward, speed_record):
    
    npc_name = npc.name
    npc_speed = speed_record[npc_name]
    waypoints = []
    npc_start_position = npc.state.position

    angle = lgsvl.Vector(0, math.degrees(math.atan2(forward.x, forward.z)), 0)
    waypoints.append(lgsvl.DriveWaypoint(npc_start_position, npc_speed, angle=angle)) # ensure not None
    if npc_speed == 0: 

        logger.info(f"{npc_name} is starting.")
        target_speed = random.random() * 2 + 1
        new_position = raycast_to_ground(sim, npc_start_position)
   
        ground_point = sim.map_point_on_lane(new_position).position
        
        waypoints.append(lgsvl.DriveWaypoint(ground_point, target_speed, angle=angle))
        
    else:
        logger.info(f"{npc_name} is keeping speed away from Ego.")
        for i in range(10):
            new_position = npc_start_position + forward * (i + 1)
            new_position = raycast_to_ground(sim, new_position)
         
            new_position = sim.map_point_on_lane(new_position)
            ground_point = new_position.position
            waypoints.append(lgsvl.DriveWaypoint(ground_point, npc_speed, angle=angle))
    return waypoints
