import time
import lgsvl
from loguru import logger
import numpy as np
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '/mnt/sda/AdvFuzz/src/maneuver')))
#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '/mnt/sda/AdvFuzz/src/modules')))
from field_generation import generate_random_position
from junction_behavior import from1
from utils import raycast_to_ground
from environs import Env
import os
import random
import json


#from handle_zones1 import handle_zone_L1, handle_zone_N1, handle_zone_R1, handle_zone_L2, handle_zone_R2, handle_zone_L3_R3, handle_zone_F1orN1, handle_zone_none
from NineGrid import NineGrid


def connect_svl():
    env = Env()

    SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST",
                             lgsvl.wise.SimulatorSettings.simulator_host)
    SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT",
                             lgsvl.wise.SimulatorSettings.simulator_port)
    logger.info("Connecting to the Simulator")
    # Connects to the simulator instance at the ip defined by LGSVL__SIMULATOR_HOST, default is localhost or 127.0.0.1

    sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)

    logger.info("Connect to simulator successfully, Version =", sim.version)
    return sim

def initial_scene(sim):
    if sim.current_scene == lgsvl.wise.DefaultAssets.map_sanfrancisco_correct1:
        sim.reset()
    else:
        sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco_correct1)
    # logger.info("Loading recording {} from dataset {}", recording, config["dataset"])
    logger.info("Load map successfully!")
    return sim

def set_ego(sim, spawns, start_pos):
    ego_state = lgsvl.AgentState()
    forward = lgsvl.utils.transform_to_forward(ego_state.transform)
    ego_state.transform.position = start_pos
    ego_state.transform = sim.map_point_on_lane(start_pos)
    ego_state.velocity = 5 * forward
    ego = sim.add_agent(os.environ.get(
        "LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, ego_state)
    return ego


def set_npc(sim, spawns, name, start_pos, is_light):
    npc_state = lgsvl.AgentState()
    
    forward = lgsvl.utils.transform_to_forward(npc_state.transform)
    # npc_state.transform = spawns[0]
    npc_state.transform.position = start_pos
    npc_state.transform = sim.map_point_on_lane(start_pos)
    # npc_state.velocity = 5 * forward
    npc = sim.add_agent(name, lgsvl.AgentType.NPC, npc_state)
    return npc


def set_apollo(sim, spawns, start_pos):
    ego_state = lgsvl.AgentState()
    ego_state.transform = spawns[0]
    ego_state.transform.position = start_pos
    ego = sim.add_agent(os.environ.get(
        "LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, ego_state)
    logger.info("Set ego at start_pos.")
    return ego


    
class simulator():
    def __init__(self, sim, ego):
        self.sim = sim
        self.ego = ego

if __name__ == '__main__':
    sim = connect_svl()
    initial_scene(sim)
    spawns = sim.get_spawn()
    
    # json_file = open("F:\software_lab\Mystique\src\settings\scenario_env.json", 'r')
    json_file = open("/mnt/sda/AdvFuzz/src/settings/scenario_junction.json", 'r')
    scenario_env = json.load(json_file)
    
    sim.set_time_of_day(6)
    bubble = scenario_env["bubble"]["scope"]
    """
    lane_index = random.randrange(0, 4)
    forward_index = random.uniform(0, 150)
    npc_position = generate_random_position(bubble, lane_index, forward_index, sim, forward)
    print(npc_position)
    """
    
    # ego_position_scope = bubble["scope"][lane_index]
    
    # ego_position_scope = bubble["scope"][1]
    # ego_position=lgsvl.Vector(ego_position_scope[0][0], 0, ego_position_scope[0][1])
    
    # # ego_position = lgsvl.Vector(593115, 0, 4134899)
    # ego_position = lgsvl.Vector(-390.367065429688, 10.1996097564697, 378.021026611328)
    
    ego_zone = random.randrange(0, 2)
    if ego_zone == 0:
        ego_position_scope = scenario_env["agents"]["ego"]["position"][0]
        status = "left"
    else:
        ego_position_scope = scenario_env["agents"]["ego"]["position"][1]
        status = "right"
    ego_position = lgsvl.Vector(ego_position_scope["x"], ego_position_scope["y"], ego_position_scope["z"])
    action_flag = random.randrange(0, 2) # 0直行、1转弯
    ego_des_scope = scenario_env["agents"]["ego"]["destination"][status][action_flag]
    ego_des = lgsvl.Vector(ego_des_scope["v1"], ego_des_scope["v2"], ego_des_scope["v3"])
    print(f"ego_position: {ego_position}, ego_des: {ego_des}")
    

    npc_zone  = random.randrange(0, 8)
    npc_pos_scope = bubble[npc_zone]
    npc_pos_scope = bubble[0]

    def randomPostioin(npc_pos_scope):
        bubble_bound = npc_pos_scope[0]
        junction_bound = npc_pos_scope[1]

        xmin = bubble_bound[0]
        xmax = junction_bound[0]
        ymin = bubble_bound[1]
        ymax = junction_bound[1]

        x = xmax - xmin
        y = ymax - ymin
        k = y / x
        random_pos = np.random.uniform(0, x)
        return lgsvl.Vector(xmin + random_pos, 0, ymin + k * random_pos)

    #npc_position = randomPostioin(npc_pos_scope)

    #print(f"npc_position: {npc_position}")
    
    ego_position = lgsvl.Vector(-390.367065429688, 10.1996097564697, 378.021026611328)

    ego_position_ground = raycast_to_ground(sim, ego_position)
    ego = set_ego(sim, spawns, ego_position_ground)
    forward = lgsvl.utils.transform_to_forward(ego.state.transform)
    right = lgsvl.utils.transform_to_right(ego.state.transform)
    print("egp start pos:", ego_position_ground)
    # npc_position = raycast_to_ground(sim, ego_position_ground - 3.5 * 1 * right + 35 * 1 * forward)
    # npc_position = raycast_to_ground(sim, ego_position_ground - 3.5 * right)
    # npc_position = raycast_to_ground(sim, ego_position_ground + 95 * forward)
    # npc_position = raycast_to_ground(sim, ego_position_ground - 3.5 * right * 5 + 20 * forward)
    #print("npc start pos:", npc_position)
    
    npc_position = lgsvl.Vector(-421.072235107422, 10.1996097564697, 396.549499511719)

    ego_signal = sim.get_controllable(ego_position,'signal')
    npc_signal = sim.get_controllable(npc_position, 'signal')
    npc_signal.control("green")

    print(npc_signal.control_policy)

    print(f"ego/npc signal: {ego_signal} / {npc_signal}")
    #print(sim.get_controllable(ego_position,'signal').control_policy)
    npc = set_npc(sim, spawns, "SUV", npc_position, True)
    a = simulator(sim, ego)
    ninegrid_flag = [True]
    waypoints_flag = [True]
    action_change_freq = 5
    action_timer = 5.0
    i = 1
    #waypoints = from1(a.sim, a.ego, npc, forward, right, 12, 3)
    #npc.follow(waypoints, loop=False)
    while action_timer >= 0:
           
        # nine_grid = NineGrid(npc, forward, right)
        # if ninegrid_flag[0]:
        #     region = nine_grid.get_ego_region(ego.state.position)
        #     print("region is:", region)
        #     if region == 5:  # ====如果ego在npc的左后方====
        #         handle_zone_L1(a, npc, forward, right, 0, ninegrid_flag, waypoints_flag, action_change_freq)
        #     # elif region == 2: # ego在npc的正后方
        #     #     handle_zone_N1(a, npc, forward, right, 0, ninegrid_flag, waypoints_flag, action_change_freq)
        #     elif region == 1: # ====ego在npc的右后方====
        #         handle_zone_R1(a, npc, forward, right, 0, ninegrid_flag, waypoints_flag, action_change_freq)
        #     elif region == 4: # ====ego在npc的左测====
        #         handle_zone_L2(a, npc, forward, right, 0, ninegrid_flag, waypoints_flag, action_change_freq)
        #     elif region == 3: # ====ego在npc的右侧====
        #         handle_zone_R2(a, npc, forward, right, 0, ninegrid_flag, waypoints_flag, action_change_freq)    
        #     elif region == 6: # ====ego在npc侧前方====
        #         handle_zone_L3_R3(a, npc, 0, ninegrid_flag, waypoints_flag)
        #     elif region == 7 or region == 0 or region == 2: # ====ego在npc同一条lane上====
        #         handle_zone_F1orN1(a, npc, 0, ninegrid_flag, waypoints_flag, region)
        #     else:   # ego在九宫格外
        #         handle_zone_none(a, npc, 0, ninegrid_flag, waypoints_flag)
            
        # else:
        #             # npc每5s内至多执行一次变道行为
                    
        #     action_timer -= 0.1
        #     if action_timer <= 0:
        #         print("next action")
        #         ninegrid_flag[0] = True
        #         waypoints_flag[0] = True
        #         action_timer= 5.0  
        a.sim.run(0.1)
        print(i,".npc.state.position is:", npc.state.position)
        print(i, ".ego=====", ego.state.position)
        '''
        with open("src\modules\simulation\speed_data.txt", 'a') as file:
            print(i,".npc.state.speed is:", npc.state.speed)
            file.write(f"{npc.state.speed}\n")
        '''
        # print(ego.state.position)
            
        i+=1


