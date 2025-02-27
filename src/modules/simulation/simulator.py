import os
import pickle
import threading
import lgsvl
import time
import json
from environs import Env
import random
import math

import numpy as np

from modules.simulation.NPC_behavior import handle_zone_F1orN1, handle_zone_L1, handle_zone_L2, handle_zone_L3_R3, handle_zone_R1, handle_zone_R2, handle_zone_none, keepSpeed, smoothSpeed
from modules.simulation.junction_behavior import actionJudge, toJunction, wayGenerate, zoneJudge
import modules.simulation.utils as util
import modules.simulation.liability as liability

from datetime import datetime
from loguru import logger

from modules.simulation.NineGrid import NineGrid
# from modules.simulation.handle_zones import handle_zone_F1orN1, handle_zone_L1, handle_zone_L3_R3, handle_zone_N1, handle_zone_R1, handle_zone_L2, handle_zone_R2, handle_zone_none
class Simulator(object):

    def __init__(self, scenarioType, default_record_folder, target_record_folder, total_sim_time, data_prime, target_status_folder, target_collision_folder, lgsvl_map = 'Highway101GLE', apollo_map = 'Highway101GLE'):
        
        self.scenarioType = scenarioType
        self.default_record_folder = default_record_folder
        self.target_record_folder = target_record_folder
        self.target_status_foldef = target_status_folder
        self.target_collision_foldef = target_collision_folder
        ################################################################
        self.total_sim_time = total_sim_time
        self.destination = None
        ################################################################
        self.sim = None
        self.data_prime = data_prime
        self.dv = None
        self.lgsvl_map = lgsvl_map
        self.apollo_map = apollo_map
        self.ego = None
        self.mutated_npc_list = [] # The list contains all the npc added
        self.fixed_npc_list = []
        self.yellow_lines = None
        self.cross_lines = None
        self.edge_lines = None

        # self.ego_des_zone = None
        self.npc_init_zone = {}
        self.npc_init_speed = {}
        self.traffic_signal = None
        self.stragety = None

        self.connect_lgsvl()
        self.load_map(self.lgsvl_map)
        self.isEgoFault = False
        self.isHit = False
        
        
        
        # self.maxint = 130
        # self.egoFaultDeltaD = 0

        self.modules = [
            'Localization',
            'Transform',
            'Routing',
            'Prediction',
            'Planning',
            'Control',
            'Perception',
        ]
        self.dy_modules = [
            'Recorder',
        ]
        # self.init_environment()

    def connect_lgsvl(self):
        env = Env()
        logger.info("Connecting to the Simulator")
        SIMULATOR_HOST = os.environ.get("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host)
        SIMULATOR_PORT = os.environ.get("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port)
        try:
            sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT) 
            self.sim = sim
        except Exception as e:
            logger.error('Connect LGSVL wrong: ' + '127.0.0.1:8181')
            logger.error(e.message)
        logger.info('Connected LGSVL 127.0.0.1:8181')

    def load_map(self, mapName="SanFrancisco_correct"):
        if self.sim.current_scene == mapName:
           self.sim.reset()
        else:
           self.sim.load(self.lgsvl_map)
        logger.info('Loaded map: ' + mapName)

    def set_ego(self, sim, start_pos):
        ego_state = lgsvl.AgentState()
        # ego_state.transform = spawns[0]
        # ego_state.velocity = 10 * forward
        start_pos = self.raycast_to_ground( start_pos)
        ego_state.transform.position = start_pos
        ego_state.transform = sim.map_point_on_lane(start_pos)
        ego = sim.add_agent(os.environ.get(
            "LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, ego_state)
        return ego    

    def set_npc(self, name, start_pos, is_light):
        
        npc_state = lgsvl.AgentState()
        start_pos = self.raycast_to_ground(start_pos)
        npc_state.transform.position = start_pos
        npc_state.transform = self.sim.map_point_on_lane(start_pos)
        npc = self.sim.add_agent(name, lgsvl.AgentType.NPC, npc_state)
        logger.info("Set {} at start_pos.", name)
        if is_light:
            logger.info("Set npc lights on because of night.")
            c = lgsvl.NPCControl()
            c.headlights = 2
            npc.apply_control(c)
        return npc
    
    def raycast_to_ground(self, position):
        start_height = 100
        start_point = lgsvl.Vector(
            position.x, position.y + start_height, position.z)
        hit = self.sim.raycast(start_point, lgsvl.Vector(0, -1, 0), layer_mask=1 << 0)
        if hit:
            return hit.point
        else:
            return position



    def init_environment(self, scenario_obj):
        
        self.isEgoFault = False
        self.isHit = False

        #self.stragety = self.data_prime["strategy"]

        self.mutated_npc_list = []
        self.fixed_npc_list = []
        if self.scenarioType == 'roadway':
            # load environments 
            rain_rate = scenario_obj[2][0]
            fog_rate = scenario_obj[2][1]
            wetness_rate = scenario_obj[2][2]
            cloudiness_rate = scenario_obj[2][3]
            self.sim.weather = lgsvl.WeatherState(
                rain=rain_rate,
                fog=fog_rate,
                wetness=wetness_rate,
                cloudiness=cloudiness_rate,
            )
            daytime = scenario_obj[2][4]
            night = True if daytime > 17 else False
            self.sim.set_time_of_day(daytime)

            # load ego start pos
            ego_data = self.data_prime['agents']['ego']
            ego_start_pos = scenario_obj[0][0]
            ego_position = ego_data['position'][ego_start_pos]
            ego_pos_vector = lgsvl.Vector(x=ego_position['x'], y=ego_position['y'], z=ego_position['z'])
            ego_state = lgsvl.AgentState()
            ego_state.transform = self.sim.map_point_on_lane(ego_pos_vector)
            self.forward = lgsvl.utils.transform_to_forward(ego_state.transform)
            self.right = lgsvl.utils.transform_to_right(ego_state.transform)
            self.ego = self.sim.add_agent(os.environ.get("LGSVL__VEHICLE_0",lgsvl.wise.DefaultAssets.ego_test), lgsvl.AgentType.EGO, ego_state)
            logger.info("Set ego at start_pos.")
            
            ## load ego destination
            ego_end_pos = scenario_obj[0][1]
            des_method = ego_data['destination']['method']
            if des_method == 'xyz':
                x = ego_data['destination']['value'][ego_end_pos]['v1']
                y = ego_data['destination']['value'][ego_end_pos]['v2']
                z = ego_data['destination']['value'][ego_end_pos]['v3']
                self.destination = lgsvl.Vector(x, y, z)
            else:
                raise RuntimeError('Unmatched destination method')

            # load mutated npc
            npcs = self.data_prime['agents']['npcs']
            bubble = self.data_prime['bubble']['scope']
            index = 0
            for m_npc in npcs:
                lane_num = scenario_obj[1][index][0]
                npc_type = m_npc['type']
                npc_goal = m_npc['goal']
                forward_num = scenario_obj[1][index][1]
                npc_pos_x = bubble[lane_num][0][0]
                npc_pos_y = 0
                npc_pos_z = bubble[lane_num][0][1]
                npc_pos = lgsvl.Vector(x=npc_pos_x, y=npc_pos_y, z=npc_pos_z)
                npc_state = lgsvl.AgentState()
                npc_state.transform.position = npc_pos
                npc_pos = npc_state.transform.position + forward_num * self.forward
                npc = self.set_npc(npc_type, npc_pos, is_light=night)
                if npc_goal == 'fixed':
                    self.fixed_npc_list.append(npc)
                elif npc_goal == 'mutated':
                    self.mutated_npc_list.append(npc)
                else:
                    raise RuntimeError('Wrong npc goal. Only support fixed or mutated.')
                index += 1
                
        elif self.scenarioType == 'intersection':
            # load environments 
            rain_rate = scenario_obj[2][0]
            fog_rate = scenario_obj[2][1]
            wetness_rate = scenario_obj[2][2]
            cloudiness_rate = scenario_obj[2][3]
            
            self.traffic_signal = scenario_obj[3][0]
            #print(self.traffic_signal)
            # environment = self.data_prime["environment"]
            # signal_control = scenario_obj[2][5]
            # signal_status = environment["signal"][signal_control]
            # traffic_signal_loc = environment["traffic_signal_loc"]
            # print(signal_status)
            # self.sim.get_controllable(lgsvl.Vector(traffic_signal_loc[0], traffic_signal_loc[1], traffic_signal_loc[2]), 'signal').control(signal_status)

            self.sim.weather = lgsvl.WeatherState(
                rain=rain_rate,
                fog=fog_rate,
                wetness=wetness_rate,
                cloudiness=cloudiness_rate,
            )
            daytime = scenario_obj[2][4]
            night = True if daytime > 17 else False
            self.sim.set_time_of_day(daytime)
            
            # load ego start pos
            
            ego_data = self.data_prime['agents']['ego']
            ego_start_pos = scenario_obj[0][0]
            ego_position = ego_data['position'][ego_start_pos]
            ego_pos_vector = lgsvl.Vector(x=ego_position['x'], y=ego_position['y'], z=ego_position['z'])
            ego_state = lgsvl.AgentState()
            ego_state.transform = self.sim.map_point_on_lane(ego_pos_vector)
            self.forward = lgsvl.utils.transform_to_forward(ego_state.transform)
            self.right = lgsvl.utils.transform_to_right(ego_state.transform)
            self.ego = self.sim.add_agent(os.environ.get("LGSVL__VEHICLE_0",lgsvl.wise.DefaultAssets.ego_test), lgsvl.AgentType.EGO, ego_state)
            logger.info("Set ego at start_pos.")

            ## load ego destination
            
            ego_end_pos = scenario_obj[0][1]
            if ego_start_pos == 0:
                ego_des = ego_data['destination']['left'][ego_end_pos]
            elif ego_start_pos == 1:
                ego_des = ego_data['destination']['right'][ego_end_pos]
            
            x = ego_des['v1']
            y = ego_des['v2']
            z = ego_des['v3']
            self.destination = lgsvl.Vector(x, y, z)

            # load mutated npc
            
            npcs = self.data_prime['agents']['npcs']
            bubble = self.data_prime['bubble']['scope']
            index = 0
            for m_npc in npcs:
                lane_num = scenario_obj[1][index][0] # bubble index, not equal to actual lane id
                npc_type = m_npc['type']
                npc_goal = m_npc['goal']
                forward_num = scenario_obj[1][index][1]
                npc_pos_x = bubble[lane_num][0][0]
                npc_pos_y = 0
                npc_pos_z = bubble[lane_num][0][1]
                #npc_pos = lgsvl.Vector(x=npc_pos_x, y=npc_pos_y, z=npc_pos_z)
                # print(f"{npc_type}: start pos {npc_pos}")
                npc_state = lgsvl.AgentState()
                #npc_state.transform.position = npc_pos
                npc_pos_scope = bubble[lane_num]

                def initPostioin(npc_pos_scope, forward_num):
                    bubble_bound = npc_pos_scope[0]
                    junction_bound = npc_pos_scope[1]
                    
                    xmin = bubble_bound[0]
                    xmax = junction_bound[0]
                    ymin = bubble_bound[1]
                    ymax = junction_bound[1]
                    
                    x = xmax - xmin
                    y = ymax - ymin
                    sqr = (x ** 2 + y ** 2) ** 0.5
                    sin = y / sqr
                    cos = x / sqr
                    
                    return lgsvl.Vector(xmin + forward_num * cos, 0, ymin + forward_num * sin)
                
                npc_pos = initPostioin(npc_pos_scope, forward_num)
                #print(f"{npc_type}: start pos {npc_pos}")
                npc = self.set_npc(npc_type, npc_pos, is_light=night)

                self.npc_init_zone[npc.name] = lane_num
                self.npc_init_speed[npc.name] = scenario_obj[1][index][2]

                if npc_goal == 'fixed':
                    self.fixed_npc_list.append(npc)
                elif npc_goal == 'mutated':
                    self.mutated_npc_list.append(npc)
                else:
                    raise RuntimeError('Wrong npc goal. Only support fixed or mutated.')
                index += 1
            
            


        # load lines
        # yellow line
        self.yellow_lines = self.data_prime['lines']['yellow_lines']
        self.cross_lines = self.data_prime['lines']['cross_lines']
        self.edge_lines = self.data_prime['lines']['edge_lines']



# TODO 修改run的逻辑
    def runSimulation(self, scenario_obj, json_file, case_id):

        #exit_handler()
        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
        logger.info(' === Simulation Start:  ['  + date_time + '] ===')
        self.sim.reset()
        self.init_environment(scenario_obj)

        ego_zone = scenario_obj[0][0]
        ego_action_flag = scenario_obj[0][1]

        mutated_npc_num = len(scenario_obj[1])

        assert mutated_npc_num == len(self.mutated_npc_list)

        # simulation info
        simulation_recording = {
            'bbox': {
                'ego' : self.ego.bounding_box
            },
            'frames': {

            }
        }
        for npc_i in range(mutated_npc_num):
            simulation_recording['bbox']['npc_' + str(npc_i)] = self.mutated_npc_list[npc_i].bounding_box

        global collision_info
        global accident_happen
        global time_index
        
        collision_info = None
        accident_happen = False
        # TODO 其他判断方法
        def on_collision(agent1, agent2, contact):
            global accident_happen
            global collision_info
            global time_index

            accident_happen = True
            collision_info = {}

            try:
                agent1_info = [agent1.state, agent1.bounding_box]
            except KeyError:
                print("Error: The 'transform' key is missing in the agent's state.1")

            if agent2 is None:
                # Calculate distances from all NPCs to ego and find the nearest one
                min_distance = float('inf')
                nearest_npc = None
                ego_position = agent1.state.position if agent1 else self.ego.state.position
                for npc in self.mutated_npc_list:
                    distance = math.sqrt(
                        (ego_position.x - npc.state.position.x) ** 2 +
                        (ego_position.z - npc.state.position.z) ** 2
                    )
                    if distance < min_distance: 
                        min_distance = distance
                        nearest_npc = npc
                try:
                    agent2_info = [nearest_npc.state, nearest_npc.bounding_box] 
                except KeyError:
                    print("Error: The 'transform' key is missing in the agent's state.2")
                name2 = nearest_npc.name if nearest_npc else "Unknown NPC"
            else:
                agent2_info = [agent2.state, agent2.bounding_box]
                name2 = agent2.name

            name1 = "STATIC OBSTACLE" if agent1 is None else agent1.name
            if contact == None:
                contact = self.ego.state.position
                contact_loc = [contact.x, contact.y, contact.z]
                collision_info['contact'] = contact_loc
                
            logger.error(f"{name1} collided with {name2} at {str(contact)}")

            if contact:
                contact_loc = [contact.x, contact.y, contact.z]
                collision_info['contact'] = contact_loc
            
            collision_info['time'] = time_index
            collision_info['ego'] = agent1_info
            collision_info['npc'] = agent2_info

            print("sim should stop")
            self.isHit = True
                    
        # INIT apollo      
        BRIDGE_HOST = os.environ.get("BRIDGE_HOST", "127.0.0.1")
        BRIDGE_PORT = int(os.environ.get("BRIDGE_PORT", 9090))
        self.ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT) #address, port
        self.ego.on_collision(on_collision)
        
        times = 0
        success = False
        while times < 3:
            try:
                dv = lgsvl.dreamview.Connection(self.sim, self.ego, os.environ.get("BRIDGE_HOST", "127.0.0.1"))
                dv.set_hd_map(self.apollo_map)
                dv.set_vehicle('Lincoln2017MKZ_LGSVL')
                dv.setup_apollo(self.destination.x, self.destination.z, self.modules, default_timeout=30)
                success = True
                break
            except:
                logger.warning('Fail to connect with apollo, try again!')
                times += 1
        if not success:
            raise RuntimeError('Fail to connect with apollo')

        if self.default_record_folder:
            util.disnable_modules(dv, self.dy_modules)
            time.sleep(1)
            util.enable_modules(dv, self.dy_modules)
        
        dv.set_destination(self.destination.x, self.destination.z)
        logger.info(' --- Set ego_destination: ' + str(self.destination.x) + ',' + str(self.destination.z))
        
        delay_t = 3
        time.sleep(delay_t)

        # TODO

        for npc in self.mutated_npc_list:
            npc.follow_closest_lane(True, 0)


        time_index = 0

        # record start
        simulation_recording['frames'][time_index] = {
            'ego': self.ego.state
        }

        for npc_i in range(mutated_npc_num):
            simulation_recording['frames'][time_index]['npc_' + str(npc_i)] = self.mutated_npc_list[npc_i].state
            
        if self.scenarioType == 'roadway':
            logger.info('---Scenario Type: ' + self.scenarioType)
        #TODO 执行npc行为
            forward = self.forward
            right = self.right

            # Frequency of action change of NPCs
            total_sim_time = self.total_sim_time 
            # action_change_freq = 5 


           
  
            action_flag = {}
            speed_record = {}
            timer = {}
            way = {}
            #N1_cnt = {} # if not N1, reset cnt = 2; if N1 cnt -= 1. if cnt = 0 reset cnt =2
            #skip = {}
            for npc in self.mutated_npc_list:
                action_flag[npc.name] = threading.Event()
                action_flag[npc.name].set()
                way[npc.name] = []
                speed_record[npc.name] = 0
                timer[npc.name] = 2
                #N1_cnt[npc.name] = 2
                #skip[npc.name] = False
            
            threads = []
           
            for j in range(total_sim_time * 10):    # 0.1s * 50
                if self.isHit:
                    time_index += 1
                    try:
                        simulation_recording['frames'][time_index] = {
                            'ego': self.ego.state
                        }
                        for npc_i in range(len(self.mutated_npc_list)):
                            simulation_recording['frames'][time_index]['npc_' + str(npc_i)] = self.mutated_npc_list[npc_i].state
                    except KeyError:
                        print("Error: The 'transform' key is missing in the agent's state.3")
                    break
                

                try:
                    simulation_recording['frames'][time_index] = {
                        'ego': self.ego.state
                    }
                    for npc_i in range(len(self.mutated_npc_list)):
                        simulation_recording['frames'][time_index]['npc_' + str(npc_i)] = self.mutated_npc_list[npc_i].state


                    #for npc in self.mutated_npc_list:
                    #    skip[npc.name] = False

                    for i, npc in enumerate(self.mutated_npc_list):  
                        # 线程目标函数：计算下一个动作的waypoints
                        def npc_behavior(sim_status, ego_status, npc, way, speed_record, region):
            
                            #forward = lgsvl.utils.transform_to_forward(ego_status.state.transform)
                            #right = lgsvl.utils.transform_to_right(ego_status.state.transform)
                            
                            waypoints = []
                                
                            if region == 5: 
                                waypoints = handle_zone_L1(sim_status, ego_status, npc, forward, right, speed_record)
                            elif region == 1: 
                                waypoints = handle_zone_R1(sim_status, ego_status, npc, forward, right, speed_record)
                            elif region == 4: 
                                waypoints = handle_zone_L2(sim_status, ego_status, npc, forward, right, speed_record)
                            elif region == 3: 
                                waypoints = handle_zone_R2(sim_status, ego_status, npc, forward, right, speed_record)    
                            elif region == 6: 
                                waypoints = handle_zone_L3_R3(sim_status, npc, forward, speed_record)
                            elif region == 7 or region == 0 or region == 2: 
                                waypoints = handle_zone_F1orN1(sim_status, ego_status, npc, forward, region, speed_record, timer)
                            else:   
                                waypoints = handle_zone_none(sim_status, npc, forward, speed_record)
                               
                                

                         
                            waypoints = keepSpeed(sim_status, npc, forward, smoothSpeed(sim_status, npc, waypoints, forward))
                                
                            way.extend(waypoints) 
                            
                        
                        def on_waypoint(agent, index, waypoints, action_flag):
                            
                            '''
                            for i in range(len(waypoints)):
                                print(f"{agent.name}:waypoint{i}----speed{waypoints[i].speed}")
                            print(f"{agent.name} at index{index} of {len(waypoints)-1}")
                            '''
                            if index == len(waypoints) - 1 :
                                action_flag[agent.name].set()
                            else:
                                action_flag[agent.name].clear()
                        

                        npc_status = {}
                        
                        if action_flag[npc.name].is_set():
                            
                            action_flag[npc.name].clear()
                            way[npc.name] = []
                
                            ego_status = self.ego
                            sim_status = self.sim
                            nine_grid = NineGrid(npc, forward, right)
                            region = nine_grid.get_ego_region(ego_status.state.position)
                            
                            p = threading.Thread(target=npc_behavior, args=(sim_status, ego_status, npc, way[npc.name], speed_record, region))
                            p.start()
                            p.join()
                           
                            speed_record[npc.name] = way[npc.name][-1].speed
                            waypoints = way[npc.name]
                            npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                                agent, index, waypoints, action_flag))  
                            
                            
                            npc.follow(way[npc.name], loop=False)
                    
                    for npc in self.mutated_npc_list:
                        
                        timer[npc.name] -= 0.1
                            
                        if timer[npc.name] < 0:
                            timer[npc.name] = 2
                            if npc.state.speed == 0:
                                action_flag[npc.name].set()
                                speed_record[npc.name] = random.random() * 2 + 1
                                logger.info(f"{npc.name} reset action_flag----speed{speed_record[npc.name]}")
                            

                    
            
                    
                    module_status_mark = True
                    while module_status_mark:
                        module_status_mark = False
                        module_status = dv.get_module_status()
                        for module, status in module_status.items():
                            if (not status) and (module in self.modules):
                                logger.warning('$$Simulator$$ Module is closed: ' + module + ' ==> restart')
                                dv.enable_module(module)
                                time.sleep(0.5)
                                module_status_mark = True
                    time_index += 1
                    self.sim.run(0.1)
                except (KeyError,TypeError):
                    print("Error: The 'transform' key is missing in the agent's state.4")
                    self.sim.run(0.1)

               
            
           

        elif self.scenarioType == 'intersection':
            logger.info('---Scenario Type: ' + self.scenarioType)
            #TODO 执行npc行为
            forward = self.forward
            right = self.right
            # Frequency of action change of NPCs
            total_sim_time = self.total_sim_time 

            signal_change = self.traffic_signal
            traffic_signal = "red"
            signal = self.sim.get_controllable(lgsvl.Vector(-421.072235107422, 10.1996097564697, 396.549499511719), 'signal')
            signal.control(traffic_signal)


            acc_flag = {}
            action_flag = {}
            npc_loc = {}
            ego_des = None
            way = {}
            stop_flag = {}
            for npc in self.mutated_npc_list:
                acc_flag[npc.name] = threading.Event()
                acc_flag[npc.name].set()
                action_flag[npc.name] = threading.Event()
                action_flag[npc.name].clear()
                stop_flag[npc.name] = False
                way[npc.name] = []

            #print(self.npc_init_speed)
            for j in range(total_sim_time * 10):    # 0.1s * 50
                if self.isHit:
                    time_index += 1
                    try:
                        simulation_recording['frames'][time_index] = {
                            'ego': self.ego.state
                        }
                        for npc_i in range(len(self.mutated_npc_list)):
                            simulation_recording['frames'][time_index]['npc_' + str(npc_i)] = self.mutated_npc_list[npc_i].state
                    except KeyError:
                        print("Error: The 'transform' key is missing in the agent's state.3")
                    break
                

                try:
                    simulation_recording['frames'][time_index] = {
                        'ego': self.ego.state
                    }
                    for npc_i in range(len(self.mutated_npc_list)):
                        simulation_recording['frames'][time_index]['npc_' + str(npc_i)] = self.mutated_npc_list[npc_i].state


                    for i, npc in enumerate(self.mutated_npc_list):  
                
                        def move_to_junction(sim_status, npc, way):
                            
                            
                            
                            npc_zone = zoneJudge(self.npc_init_zone[npc.name])

                            
                            to_junction_waypoints = toJunction(sim_status, npc, npc_zone, self.npc_init_speed[npc.name])
                            way[npc.name].extend(to_junction_waypoints)
                        
                        def nextAction(sim_status, ego_status, npc, way):
                            npc_zone, npc_des, ego_des = actionJudge(ego_zone, ego_action_flag, self.npc_init_zone[npc.name], traffic_signal)

                            next_waypoints = (wayGenerate(sim_status, ego_status, npc, forward, right, 
                                                   npc_zone, npc_des, ego_des, self.npc_init_speed[npc.name]))
                            if next_waypoints is None:
                                way[npc.name].append(None)
                            else:
                                way[npc.name].extend(next_waypoints)

                        def on_waypoint(agent, index, way, action_flag):
                            
                            if index == len(way) - 1:
                                logger.info(f"{agent.name} set flag")
                                action_flag[agent.name].set()
                            else:
                                action_flag[agent.name].clear()
                        
                        def on_waypoint_next(agent, index, action_flag):
                            
                            action_flag[agent.name].clear()
                            
    
                        if acc_flag[npc.name].is_set():
                            
                            sim_status = self.sim
                            ego_status = self.ego
                            logger.info(f"{npc.name} move to junction")
                            acc_flag[npc.name].clear()
                            way[npc.name] = []
                            p = threading.Thread(target = move_to_junction, args = (sim_status, npc, way))
                            p.start()
                            p.join()

                            

                            
                            waypoints = way[npc.name]
                            
                            npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                                agent, index, waypoints, action_flag)) 
                            npc.follow(way[npc.name], loop=False)   
                        elif action_flag[npc.name].is_set():
                            
                            sim_status = self.sim
                            ego_status = self.ego

                            npc_zone, npc_des, ego_des = actionJudge(ego_zone, ego_action_flag, self.npc_init_zone[npc.name], traffic_signal)
                            if npc_des is not None and stop_flag[npc.name] is False:
                                logger.info(f"{npc.name} next action")
                                action_flag[npc.name].clear()
                                stop_flag[npc.name] = True
                                way[npc.name] = []
                                p = threading.Thread(target = nextAction, args = (sim_status, ego_status, npc, way))
                                p.start()
                                p.join()

                                
                                if way[npc.name][0] is not None:
                                    npc.follow(way[npc.name], loop=False)
                                else:
                                    logger.info(f"{npc.name} stop")
                                    npc.follow_closest_lane(0, loop=False)
                            

                        
                    
                    
                    module_status_mark = True
                    while module_status_mark:
                        module_status_mark = False
                        module_status = dv.get_module_status()
                        for module, status in module_status.items():
                            if (not status) and (module in self.modules):
                                logger.warning('$$Simulator$$ Module is closed: ' + module + ' ==> restart')
                                dv.enable_module(module)
                                time.sleep(0.5)
                                module_status_mark = True
                    time_index += 1
                    self.sim.run(0.1)              
                
                except (KeyError,TypeError):
                    print("Error: The 'transform' key is missing in the agent's state.4")
                    self.sim.run(0.1)
                
                signal_change -= 0.1
                if signal_change <= 0 and traffic_signal == "red":
                    logger.info("====== Traffic Signal Change ======")
                    traffic_signal = "green"
                    signal.control(traffic_signal)

                

        
        
        if self.default_record_folder:
            util.disnable_modules(dv, self.dy_modules)
            time.sleep(0.5)

        # check new folder and move -> save folderm
        if self.default_record_folder:
            util.check_rename_record(self.default_record_folder, self.target_record_folder, case_id)

        # compute fitness score & check other bugs such as line cross or else
        '''
        

        global collision_info
        global accident_happen
        
        collision_info = None
        accident_happen = False
        
        '''
        # Step 1 obtain time
        simulation_slices = max(simulation_recording['frames'].keys())

        '''
        simulation_recording[frames][time_index] = {
                    'ego': self.ego.transform,
                    'npc': []
                }
        '''
        fault = []
        max_fitness = -1111
        # Step 2 compute distance and check line error and filter npc_fault
        for t in range(simulation_slices):
            try:
                simulation_frame = simulation_recording['frames'][t]
                ego_info = {
                    'state': simulation_frame['ego'],
                    'bbox': simulation_recording['bbox']['ego']
                }            
                # compute distance
                for npc_i in range(len(self.mutated_npc_list)):
                    npc_id = 'npc_' + str(npc_i)
                    npc_info = {
                        'state': simulation_frame[npc_id],
                        'bbox': simulation_recording['bbox'][npc_id]
                    }
                    
                    npc_ego_fitness = liability.compute_danger_fitness(ego_info, npc_info, False)
                    '''
                    
                    ego_position = ego_info['state'].position
                    npc_position = npc_info['state'].position
                    npc_ego_fitness = 1 /   ((ego_position.x - npc_position.x) ** 2,
                                            (ego_position.y - npc_position.y) ** 2,
                                            (ego_position.z - npc_position.z) ** 2)
                    '''
                    if npc_ego_fitness > max_fitness:
                        max_fitness = npc_ego_fitness
                
                # check line
                for yellow_line in self.yellow_lines:
                    hit_yellow_line = liability.ego_yellow_line_fault(ego_info, yellow_line)
                    if hit_yellow_line:
                        fault.append('hit_yellow_line')
                
                for edge_line in self.edge_lines:
                    hit_edge_line = liability.ego_edge_line_fault(ego_info, edge_line)
                    if hit_edge_line:
                        fault.append('hit_edge_line')
            except KeyError as e:
                continue
        # Step 3 if collision, check is npc fault
        '''
        agent1_info = [agent1.transform, agent1.state]
                        
            if not agent2:
                agent2_info = [None, None]
            else:
                agent2_info = [agent2.transform, agent2.state]
            
            if contact:
                contact_loc = [contact.x, contact.y, contact.z]
            
            collision_info['time'] = time_index
            collision_info['ego'] = agent1_info
            collision_info['npc'] = agent2_info
            collision_info['contact'] = contact_loc

        '''
        if collision_info is not None:
            ego_info = {
                'state': collision_info['ego'][0],
                'bbox': collision_info['ego'][1]
            }
            
            # if collision_info['npc'] is not None:
            if collision_info.get('npc') is not None:
                npc_info = {
                    'state': collision_info['npc'][0],
                    'bbox': collision_info['npc'][1]
                }
            
                ego_fault = liability.ego_collision_fault(ego_info, npc_info, self.cross_lines)
                if ego_fault:
                    fault.append('ego_fault')
                else:
                    fault.append('npc_fault')
                
                fitness = liability.compute_danger_fitness(ego_info, npc_info, True)
                # if fitness < max_fitness:
                if fitness <= max_fitness:
                    logger.error('Please increase K in liability.compute_danger_fitness: Collision - ' + str(fitness) + 'No Collision - ' + str(max_fitness))
                    raise RuntimeError('liability.compute_danger_fitness parameter setting is not right.')
                else:
                    max_fitness = fitness

        if len(fault) == 0:
            fault.append('normal')
        
        #fitness_score = self.findFitness(deltaDList, dList, self.isHit, hit_time)
        
        result_dict = {}
        result_dict['fitness'] = max_fitness
        #(fitness_score + self.maxint) / float(len(self.mutated_npc_list) - 1 ) # Try to make sure it is positive
        
        
                # save simulation info
        simulation_file = os.path.join(self.target_status_foldef, case_id + '.obj')
        if os.path.isfile(simulation_file):
            os.system("rm " + simulation_file)
        with open(simulation_file, 'wb') as f_f:
            pickle.dump(simulation_recording, f_f)

        collision_file = os.path.join(self.target_collision_foldef, case_id + '.obj')
        if os.path.isfile(collision_file):
            os.system("rm " + collision_file)
        with open(collision_file, 'wb') as f_f1:
            pickle.dump(collision_info, f_f1)
            
        result_dict['fault'] = fault
        
        logger.info(' === Simulation End === ')

        return result_dict