import random
import sys
import os

import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '/mnt/sda/AdvFuzz/src/maneuver')))
from loguru import logger
from generate_behavior_trajectory import generate_behavior_trajectory
import lgsvl
from utils import raycast_to_ground


import math

# According to Ego's target planning route

# TODO forward / right

ego_zone_list = [1, 2]
ego_des_list = [[8, 12], [7, 3]]
npc_zone_list = [1, 2, 5, 6, 9, 10, 13, 14]
junction_bound_start = {
    1 : [-392.839660644531,  375.433532714844], #1
    2 : [-390.367065429688, 378.021026611328],  #2
    3 : [-391.234008789063, 395.448150634766],  #3
    4 : [-394.138824462891, 398.221557617188],  #4
    5 : [-396.731628417969, 400.7001953125],    #5
    6 : [-399.511901855469, 403.340667724609],  #6
    7 : [-415.670593261719, 402.202819824219],  #7
    8 : [-418.143218994141, 399.615264892578],  #8
    9 : [-421.072235107422, 396.549499511719],  #9
    10 : [-423.6767578125, 393.823822021484],    #10
    11 : [-423.046569824219, 378.823577880859], #11
    12 : [-420.241058349609, 376.14501953125],   #12
    13 : [-417.415374755859, 373.430694580078],  #13
    14 : [-414.593963623047, 370.736999511719],   #14
    15 : [-398.37353515625, 369.642242431641],  #15
    16 : [-395.768859863281, 372.367950439453]   #16
}
    
bubble_bound_end = {
    3 : [-370.503509521484, 417.133361816406],
    4 : [-373.434234619141, 419.931732177734],
    7 : [-437.359832763672, 422.929077148438],
    8 : [-439.832458496094, 420.341522216797],
    11 : [-443.683532714844, 357.049682617188],
    12 : [-441.024810791016, 354.510986328125],
    13 : [-438.050659179688, 351.6552734375],
    14 : [-435.229675292969, 348.961944580078],
    15 : [-376.684020996094, 348.916046142578],
    16 : [-374.079437255859, 351.641784667969]
}


'''
args:
    ======NPC======
    npc itself  /   npc zone
    ======EGO======
    ego.state   /   ego zone    /   target destination(zone)

zone:
    ======EGO======
    ego_zone = random.randrange(0, 2)  ---> initial zone    /   init status
    {
        0 : zone1
        1 : zone2
    }
    ego_zone_scope = scenario_env["agents"]["ego"]["position"][ego_zone]
    ego_position = lgsvl.Vector(ego_position_scope["x"], ego_position_scope["y"], ego_position_scope["z"])

    
    action_flag = random.randrange(0, 2)  ---> 0: straight 1: turn
    {
        00 : zone8
        01 : zone12
        10 : zone7
        11 : zone3
    }
    ego_des_scope = scenario_env["agents"]["ego"]["destination"][status][action_flag]
    ego_des = lgsvl.Vector(ego_des_scope["v1"], ego_des_scope["v2"], ego_des_scope["v3"])
    ======NPC======
    npc_zone = random.randrange(0, 8)
    {
        0 : zone1
        1 : zone2
        2 : zone5
        3 : zone6
        4 : zone9
        5 : zone10
        6 : zone13
        7 : zone14
    }
    # TODO 
    {
        if ego_zone is zone1:
            if action_flag is straight: ---> 8
                NPC: 1 --- random to 8 / 12
                     2 --- X
                     5 --- X
                     6 --- X
                     9 --- turn left to 4
                     10 --- X 
                     13 --- X
                     14 --- X
            elif action_flag is left: ---> 12
                NPC: 1 --- random to 8 / 12
                     2 --- X
                     5 --- X
                     6 --- X
                     9 --- random to 4 / 16
                     10 --- straight to 15
                     13 --- X
                     14 --- X
        elif ego_zone is zone2:
            if action_flag is straight: ---> 7
                NPC: 1 --- X
                     2 --- random to 3 / 7
                     5 --- X
                     6 --- turn right to 7
                     9 --- turn left to 4
                     10 --- X 
                     13 --- X
                     14 --- X
            elif action_flag is right: ---> 3
                NPC: 1 --- X
                     2 --- random to 3 / 7 (green) | turn right to 3 (red)
                     5 --- X
                     6 --- X
                     9 --- X
                     10 --- X
                     13 --- X
                     14 --- straight to 3(red)
            

    }
    npc_pos_scope = bubble[npc_zone][random.randrange(0, 2)]

'''                       
# according to (x,y,z) define npc_zone, but why not record======init_env======                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            

# npc position is often generated from bubble['scope'][_], 
def actionJudge(ego_loc, action_flag, npc_loc, traffic_signal):
    #logger.info(f"Traffic sign ==={traffic_signal}===")
    # ego_zone, action_flag, npc_zone as args
    ego_zone = ego_zone_list[ego_loc]
    ego_des_zone = ego_des_list[ego_zone - 1][action_flag]
    npc_zone = npc_zone_list[npc_loc]
    npc_des = None
    '''
    if ego_des_zone == 8:
        if npc_zone == 9:
            npc_des = 4
    elif ego_des_zone == 12:
        if npc_zone == 9:
            npc_des = 4 if random.random() < 0.5 else 16
        elif npc_zone == 10:
            npc_des = 15
    elif ego_des_zone == 7:
        if npc_zone == 6:
            npc_des = 7
        elif npc_zone == 9:
            npc_des = 4
    elif ego_des_zone == 3:
        if npc_zone == 14:
            if traffic_signal == 'red':
                npc_des = 3
    '''

    if npc_zone == 1:
        if traffic_signal == 'green':
            npc_des = 8 if random.random() < 0.5 else 12

    elif npc_zone == 2:
        npc_des = (3 if random.random() < 0.5 else 7) if traffic_signal == 'green' else 3

    elif npc_zone == 5:
        if traffic_signal == 'red':
            npc_des = 12 if random.random() < 0.5 else 16

    elif npc_zone == 6:
        if ego_des_zone == 7:
            npc_des = 7
        elif traffic_signal == 'red':
            npc_des = 11 if random.random() < 0.5 else 7

    elif npc_zone == 9:
        if traffic_signal == 'green':
            if ego_des_zone == 12:
                npc_des = 16
            elif ego_des_zone == 8 or ego_des_zone == 7:
                npc_des = 4
            else:
                npc_des = 4 if random.random() < 0.5 else 16
        
    elif npc_zone == 10:
        if traffic_signal == 'green':
            if ego_des_zone == 12:
                npc_des = 15
            else:
                npc_des = 11 if random.random() < 0.5 else 15
        else:
            npc_des = 11
    
    elif npc_zone == 13:
        if traffic_signal == 'red':
            npc_des = 8 if random.random() < 0.5 else 4

    elif npc_zone == 14:
        if traffic_signal == 'red':
            npc_des = 3
        else:
            npc_des = 15

    return npc_zone, npc_des, ego_des_zone


def zoneJudge(npc_loc):
    return npc_zone_list[npc_loc]
'''
Action: 
    1 -> 8 / 12
    6 -> 7
    9 -> 16 / 4
    10 -> 15
    2 -> 3 / 7
    14 -> 3

    5 -> 12 / 16
    6 -> 11
    10 -> 11
    13 -> 8 / 4
    14 -> 15
'''


# if forward --- X, distance X
def calculate_distance(final_point, now_point, forward):
    final_point = np.array([final_point[0], final_point[1]])
    now_point = np.array([now_point.x, now_point.z])
    forward = np.array([forward.x, forward.z])
    direction_vector = final_point - now_point
    dot_product = np.dot(direction_vector, forward)
    forward_magnitude = np.linalg.norm(forward)

    direction_vector_magnitude = np.linalg.norm(direction_vector)
    #return direction_vector_magnitude
    return dot_product / forward_magnitude


def straight_forward(distance, npc_position, des):
    npc_x = npc_position.x
    npc_y = npc_position.z
    des_x = des[0]
    des_y = des[1]

    x = des_x - npc_x
    y = des_y - npc_y
    sqr = (x ** 2 + y ** 2) ** 0.5
    sin = y / sqr
    cos = x / sqr

    waypoints = []
    for i in range(math.ceil(distance)):
        waypoints.append(lgsvl.Vector(npc_x + (i+1) * cos, 10, npc_y + (i+1) * sin))
    return waypoints

'''
    *init_env --ego_zone, ego_des_zone, npc_zone--- all as .json index   ---traffic_signal, init_speed---
    *run_Simulation:
        speed_recoed = {}
        acc_flag = {}
        action_flag = {}
        if intersecton:
            for ... :
                def move_to_junction():
                {
                    npc_zone, npc_des = actonjudge()   ---as actual zone name
                    to_junction_waypoints = toJunction()
                    way.extend()
                }
                def nextAction():
                {
                    way.extend(wayGenerate())
                }
                def on_waypoint(agent, index, way[]):
                {
                    if index = len(way[agent.name] - 1):
                        action_flag[agent.name].set()
                }
                for npc in npc_list:
                    if action_flag:
                        action_flag[npc.name].clear()
                        way = []
                        p = threadings.Thread(target = nextAction(), args = ())
                        p.start()
                        p.join()
                        if way is not empty:
                            npc.follow(way)
                        else:
                            npc.follow_closest_lane(0, loop=False)
                    elif acc_flag:
                        acc_flag[npc.name].clear()
                        way = []
                        p = threadings.Thread(target = move_to_junction(), args = ())
                        p.start()
                        p.join()
                        speed_record[] = way[-1].speed = init_speed
                        npc.on_waypoint_reached()
                        npc.follow(way)
                sim.run


'''

def toJunction(sim, npc, npc_zone, init_speed):
    npc_name = npc.name
    
    waypoints = []
    
    junction_bound  = junction_bound_start[npc_zone]
    #print(f"{npc_name} 1")
    forward = lgsvl.utils.transform_to_forward(npc.state.transform)
    #print(f"{npc_name} 2")
    #print(f"{npc_name} 3")
    npc_position = npc.state.position
    #print(f"{npc_name} 4")
    # distance < 0
    distance_to_junction = calculate_distance(junction_bound, npc_position, forward)

    waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward=forward
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance_to_junction)
    #print(f"{npc_name} 7")
    return waypoints

# npc.rotation <--> ego(world).rotation

# forward = forward, right = right
def from1(sim, ego, npc, forward, right, npc_des, init_speed):
    # move at {init_speed} m/s, turn or straight at random speed
    waypoints = []
    if npc_des == 8:
        #straight
        npc_position = npc.state.position
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, npc_position, forward) 
        waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward=forward
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance)
    elif npc_des == 12:
        #turn left
        turnL_waypoints = []
        npc_position = npc.state.position
        des = junction_bound_start[npc_des]
        distance_forward = calculate_distance(des, npc_position, forward)
        distance_left = calculate_distance(des, npc_position, -right)
        turnL_waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward=lgsvl.utils.transform_to_forward(npc.state.transform)
                                                       , right=lgsvl.utils.transform_to_right(npc.state.transform), 
                                                       behavior_type='turn_left', num_points=20, n=distance_forward, m=distance_left)
        waypoints.extend(turnL_waypoints)
        #straight
        des = bubble_bound_end[npc_des]
        start_pos = waypoints[-1].position
        distance = calculate_distance(des, start_pos, -right)
        waypoints.extend(generate_behavior_trajectory(sim, None, [start_pos, init_speed], 
                                                       forward= -right
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance))
    return waypoints

# forward = forward, right = right
def from2(sim, ego, npc, forward, right, npc_des, init_speed):
    waypoints = []
    if npc_des == 7:
        #straight
        npc_position = npc.state.position
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, npc_position, forward) 
        waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward=forward
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance)
    elif npc_des == 3:
        #turn right
        turnR_waypoints = []
        npc_position = npc.state.position
        des = junction_bound_start[npc_des]
        distance_forward = calculate_distance(des, npc_position, forward)
        distance_right = calculate_distance(des, npc_position, right)
        turnR_waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward=lgsvl.utils.transform_to_forward(npc.state.transform)
                                                       , right=lgsvl.utils.transform_to_right(npc.state.transform), 
                                                       behavior_type='turn_right', num_points=20, n=distance_forward, m=distance_right)
        waypoints.extend(turnR_waypoints)
        #straight
        des = bubble_bound_end[npc_des]
        start_pos = waypoints[-1].position
        distance = calculate_distance(des, start_pos, right)
        waypoints.extend(generate_behavior_trajectory(sim, None, [start_pos, init_speed], 
                                                       forward=right
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance))
    return waypoints
    
# foward = -right, right = forward
def from5(sim, ego, npc, forward, right, npc_des, init_speed):
    waypoints = []
    if npc_des == 12:
        #straight
        npc_position = npc.state.position
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, npc_position, -right) 
        waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward= -right
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance)
    elif npc_des == 16:
        #turn left
        turnL_waypoints = []
        npc_position = npc.state.position
        des = junction_bound_start[npc_des]
        distance_forward = calculate_distance(des, npc_position, -right)
        distance_right = calculate_distance(des, npc_position, -forward)
        turnL_waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward=lgsvl.utils.transform_to_forward(npc.state.transform)
                                                       , right=lgsvl.utils.transform_to_right(npc.state.transform), 
                                                       behavior_type='turn_left', num_points=20, n=distance_forward, m=distance_right)
        waypoints.extend(turnL_waypoints)
        #straight
        des = bubble_bound_end[npc_des]
        start_pos = waypoints[-1].position
        distance = calculate_distance(des, start_pos, -forward)
        waypoints.extend(generate_behavior_trajectory(sim, None, [start_pos, init_speed], 
                                                       forward= -forward
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance))
    return waypoints

# forward = -right, right = forward
def from6(sim, ego, npc, forward, right, npc_des, init_speed):
    if npc_des == 7:
        waypoints = []
        #turn right
        turnR_waypoints = []
        junction_bound  = junction_bound_start[npc_des]
        npc_position = npc.state.position
        distance_forward = calculate_distance(junction_bound, npc_position, -right)
        distance_right = calculate_distance(junction_bound, npc_position, forward)
        # TODO ego / npc -> speed control
        turnR_waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                        forward=lgsvl.utils.transform_to_forward(npc.state.transform)
                                                        , right=lgsvl.utils.transform_to_right(npc.state.transform), 
                                                        behavior_type='turn_right', num_points=20, n=distance_forward, m=distance_right)
        
        speed = randomSpeed(ego, npc, turnR_waypoints)
        if speed > 0:
            for i in range(len(turnR_waypoints)):
                turnR_waypoints[i].speed = speed

        waypoints.extend(turnR_waypoints)
        #straight
        start_pos = waypoints[-1].position
        speed = waypoints[-1].speed
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, start_pos, forward)
        waypoints.extend(generate_behavior_trajectory(sim, None, [start_pos, speed], 
                                                        forward=forward
                                                        , right=None, 
                                                        behavior_type='straight_forward', num_points=20, n=distance))
    elif npc_des == 11:
        #straight
        npc_position = npc.state.position
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, npc_position, -right) 
        waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward= -right
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance)
    return waypoints

# forward = -forward, right = -right
def from9(sim, ego, npc, forward, right, npc_des, init_speed):
    waypoints = []
    if npc_des == 16:
        #straight
        npc_position = npc.state.position
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, npc_position, -forward) 
        # TODO 
        waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward= -forward
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance)
        speed = randomSpeed(ego, npc, waypoints)
        if speed > 0:
            for i in range(len(waypoints)):
                waypoints[i].speed = speed

    elif npc_des == 4:
        #turn left
        # TODO
        turnL_waypoints = []
        npc_position = npc.state.position
        des = junction_bound_start[npc_des]
        distance_forward = calculate_distance(des, npc_position, -forward)
        distance_left = calculate_distance(des, npc_position, right)
        turnL_waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward=lgsvl.utils.transform_to_forward(npc.state.transform)
                                                       , right=lgsvl.utils.transform_to_right(npc.state.transform), 
                                                       behavior_type='turn_left', num_points=20, n=distance_forward, m=distance_left)
        
        speed = randomSpeed(ego, npc, turnL_waypoints)
        if speed > 0:
            for i in range(len(turnL_waypoints)):
                turnL_waypoints[i].speed = speed

        waypoints.extend(turnL_waypoints)
        #straight
        des = bubble_bound_end[npc_des]
        start_pos = waypoints[-1].position
        distance = calculate_distance(des, start_pos, right)
        waypoints.extend(generate_behavior_trajectory(sim, None, [start_pos, 2], 
                                                       forward=right
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance))
    return waypoints

# forward = -forward, right = -right
def from10(sim, ego, npc, forward, right, npc_des, init_speed):
    if npc_des == 15:
        waypoints = []
        #straight
        npc_position = npc.state.position
        # TODO ego / npc -> speed control
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, npc_position, -forward)
        waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                        forward=-forward
                                                        , right=None, 
                                                        behavior_type='straight_forward', num_points=20, n=distance)
        speed = randomSpeed(ego, npc, waypoints)
        if speed > 0:
            for i in range(len(waypoints)):
                waypoints[i].speed = speed
    elif npc_des == 11:
        waypoints = []
        #turn right
        turnR_waypoints = []
        junction_bound  = junction_bound_start[npc_des]
        npc_position = npc.state.position
        distance_forward = calculate_distance(junction_bound, npc_position, -forward)
        distance_right = calculate_distance(junction_bound, npc_position, -right)
        turnR_waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                        forward=lgsvl.utils.transform_to_forward(npc.state.transform)
                                                        , right=lgsvl.utils.transform_to_right(npc.state.transform), 
                                                        behavior_type='turn_right', num_points=20, n=distance_forward, m=distance_right)
        waypoints.extend(turnR_waypoints)
        #straight
        start_pos = waypoints[-1].position
        speed = waypoints[-1].speed
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, start_pos, -right)
        waypoints.extend(generate_behavior_trajectory(sim, None, [start_pos, speed], 
                                                        forward=-right
                                                        , right=None, 
                                                        behavior_type='straight_forward', num_points=20, n=distance))
    return waypoints

# forward = right, right = -forward
def from13(sim, ego, npc, forward, right, npc_des, init_speed):
    waypoints = []
    if npc_des == 4:
        #straight
        npc_position = npc.state.position
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, npc_position, right) 
        waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward= right
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance)
    elif npc_des == 8:
        #turn left
        turnL_waypoints = []
        npc_position = npc.state.position
        des = junction_bound_start[npc_des]
        distance_forward = calculate_distance(des, npc_position, right)
        distance_right = calculate_distance(des, npc_position, forward)
        turnL_waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                       forward=lgsvl.utils.transform_to_forward(npc.state.transform)
                                                       , right=lgsvl.utils.transform_to_right(npc.state.transform), 
                                                       behavior_type='turn_left', num_points=20, n=distance_forward, m=distance_right)
        waypoints.extend(turnL_waypoints)
        #straight
        des = bubble_bound_end[npc_des]
        start_pos = waypoints[-1].position
        distance = calculate_distance(des, start_pos, forward)
        waypoints.extend(generate_behavior_trajectory(sim, None, [start_pos, init_speed], 
                                                       forward= forward
                                                       , right=None, 
                                                       behavior_type='straight_forward', num_points=20, n=distance))
    return waypoints

# forward = right, right = -forward
def from14(sim, ego, npc, forward, right, npc_des, init_speed):
    if npc_des == 3:
        waypoints = []
        #straight
        npc_position = npc.state.position
        # TODO ego / npc -> speed control
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, npc_position, right)
        waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                        forward=right
                                                        , right=None, 
                                                        behavior_type='straight_forward', num_points=20, n=distance)
        speed = randomSpeed(ego, npc, waypoints)
        if speed > 0:
            for i in range(len(waypoints)):
                waypoints[i].speed = speed
    elif npc_des == 15:
        waypoints = []
        #turn right
        turnR_waypoints = []
        junction_bound  = junction_bound_start[npc_des]
        npc_position = npc.state.position
        distance_forward = calculate_distance(junction_bound, npc_position, right)
        distance_right = calculate_distance(junction_bound, npc_position, -forward)
        turnR_waypoints = generate_behavior_trajectory(sim, None, [npc_position, init_speed], 
                                                        forward=lgsvl.utils.transform_to_forward(npc.state.transform)
                                                        , right=lgsvl.utils.transform_to_right(npc.state.transform), 
                                                        behavior_type='turn_right', num_points=20, n=distance_forward, m=distance_right)
        waypoints.extend(turnR_waypoints)
        #straight
        start_pos = waypoints[-1].position
        speed = waypoints[-1].speed
        des = bubble_bound_end[npc_des]
        distance = calculate_distance(des, start_pos, -forward)
        waypoints.extend(generate_behavior_trajectory(sim, None, [start_pos, speed], 
                                                        forward=-forward
                                                        , right=None, 
                                                        behavior_type='straight_forward', num_points=20, n=distance))
    return waypoints


def randomSpeed(ego, npc, waypoints):
    ego_speed = ego.state.speed
    ego_position = np.array([ego.state.position.x, ego.state.position.z])
    # target = math.ceil(len(waypoints) / 2)
    

    # collision_point = np.array([waypoints[target].position.x, waypoints[target].position.z])
    
    # distance_vector = collision_point - ego_position
    forward = lgsvl.utils.transform_to_forward(ego.state.transform)
    #right = lgsvl.utils.transform_to_right(ego.state.transform)
    forward = np.array([forward.x, forward.z])
    forward_magnitude = np.linalg.norm(forward)
    #right = np.array([right.x, right.z])

    min_dot = 99999 # ego/npc vector distance
    min_index = None
    min_dis = None # npc route distance

    max_dot = 0.000001
    max_index = None
    max_dis = None

    for i in range(len(waypoints)):
        now_point = waypoints[i].position
        now_point = np.array([now_point.x, now_point.z])
        distance_vector = now_point - ego_position
        
        dot = np.dot(distance_vector, forward)
        if dot >= 0:
            dis = np.linalg.norm(distance_vector)
            if dis < min_dot:
                min_dot = dis
                min_index = i
                
            if dis > max_dot:
                max_dot = dis
                max_index = i

    ego_speed = ego_speed if ego_speed != 0 else 1
    tmin = min_dot / ego_speed
    tmax = max_dot / ego_speed

    distance = 0
    for i in range(len(waypoints) - 1):
        distance += np.linalg.norm(np.array([waypoints[i+1].position.x, waypoints[i+1].position.z]) - np.array([waypoints[i].position.x, waypoints[i].position.z]))
        if i + 1 == min_index:
            min_dis = distance
        if i + 1 == max_index:
            max_dis = distance
    
    vmin = (min_dis / tmin) if min_dis is not None else 0
    vmax = (max_dis / tmax) if max_dis is not None else 0
    v1 = min(vmin, vmax)
    v2 = max(vmin, vmax)

    seed = 3 / 4
    if v1 == 0 and v2 == 0:
        speed = -1
        strategy = 'keep speed'
    elif v1 == v2:
        if seed < 1/3:
            speed =  np.random.uniform(0, v1)
            strategy = 'yield'
        elif seed < 2/3:
            speed =  v1
            strategy = 'interactive'
        else:
            speed =  np.random.uniform(v1, v1+2)
            strategy = 'overtake'
        #speed =  np.random.uniform(0, v1) if seed < 0.5 else np.random.uniform(v1, v1+2)
    else:
        if v1 == 0:
            if seed < 1/3:
                speed =  np.random.uniform(0, v2)
                strategy = 'yield'
            elif seed < 2/3:
                speed =  v2
                strategy = 'interactive'
            else:
                speed =  np.random.uniform(v2, v2+2)
                strategy = 'overtake'
            #speed =  np.random.uniform(0, v2) if seed < 0.5 else np.random.uniform(v2, v2+2)
        if v2 == 0:
            if seed < 1/3:
                speed =  np.random.uniform(0, v1)
                strategy = 'yield'
            elif seed < 2/3:
                speed =  v1
                strategy = 'interactive'
            else:
                speed =  np.random.uniform(v1, v1+2)
                strategy = 'overtake'
            #speed =  np.random.uniform(0, v1) if seed < 0.5 else np.random.uniform(v1, v1+2)
        else:
            if seed < 1/3:
                speed =  np.random.uniform(0, v1)
                strategy = 'yield'
            elif seed < 2/3:
                speed =  np.random.uniform(v1, v2)
                strategy = 'interactive'
            else:
                speed =  np.random.uniform(v2, v2+2)
                strategy = 'overtake'
    if strategy != 'keep speed':
        speed = 2 if speed < 2 else speed
        speed = 8 if speed > 4 else speed
    logger.info(f"{npc.name} perform {strategy}")
    return speed
  

def wayGenerate(sim, ego, npc, forward, right, npc_zone, npc_des, ego_des, init_speed):
    waypoints = []
    if npc_des is None:
        logger.info(f"{npc.name} has no des because of traffic sign")
        return None
    
    logger.info(f"{npc.name} from {npc_zone} to {npc_des}")
    if npc_zone == 1:
        waypoints = from1(sim, ego, npc, forward, right, npc_des, init_speed)
    elif npc_zone == 2:
        waypoints = from2(sim, ego, npc, forward, right, npc_des, init_speed)
    elif npc_zone == 5:
        waypoints = from5(sim, ego, npc, forward, right, npc_des, init_speed)
    elif npc_zone == 6:
        waypoints = from6(sim, ego, npc, forward, right, npc_des, init_speed)
    elif npc_zone == 9:
        waypoints = from9(sim, ego, npc, forward, right, npc_des, init_speed)
    elif npc_zone == 10:
        waypoints = from10(sim, ego, npc, forward, right, npc_des, init_speed)
    elif npc_zone == 13:
        waypoints = from13(sim, ego, npc, forward, right, npc_des, init_speed)
    elif npc_zone == 14:
        waypoints = from14(sim, ego, npc, forward, right, npc_des, init_speed)

    return waypoints