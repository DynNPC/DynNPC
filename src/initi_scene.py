import os
import lgsvl
from environs import Env
from pathlib import Path
import time
from loguru import logger
import json
from connect_simulator import connect_svl
import hdmap
from hdmap import MapParser


def initial_scene(sim):
    if sim.current_scene == lgsvl.wise.DefaultAssets.map_sanfrancisco:
        sim.reset()
    else:
        sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco)
    # logger.info("Loading recording {} from dataset {}", recording, config["dataset"])
    logger.info("Load map successfully!")
    return sim


def set_weather(sim, weatherList):
    daytime = weatherList["daytime"]
    rain_num = weatherList["rain"]
    fog_num = weatherList["fog"]
    cloudiness_num = weatherList["cloudiness"]

    sim.set_time_of_day(daytime)

    sim.weather = lgsvl.WeatherState(
        rain=rain_num, fog=fog_num, wetness=0, cloudiness=cloudiness_num, damage=0)
    logger.info("Current daytime: {}; Current weather: {}",
                sim.time_of_day, sim.weather)

    return sim

def set_ego(sim, start_pos):
    ego_state = lgsvl.AgentState()
    start_pos = raycast_to_ground(sim, start_pos)
    ego_state.transform = sim.map_point_on_lane(start_pos)
    
    # ego_state.transform.position = start_pos
    # ego_state.transform = sim.map_point_on_lane(start_pos)
    
    # ds = ego_state.transform.position + 175 * forward
    # ds = raycast_to_ground(sim, ds)
    # ego_state.transform = sim.map_point_on_lane(ds)
    
    # ego_state.transform = spawns[0]
    
    # ds = ego_state.transform.position -  4* 3.5 * right 
    # ds = raycast_to_ground(sim, ds)
    # ego_state.transform = sim.map_point_on_lane(ds)
    
    ego = sim.add_agent(os.environ.get(
        "LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, ego_state)
    return ego


def set_apollo(sim, spawns, start_pos):
    ego_state = lgsvl.AgentState()
    ego_state.transform = spawns[0]
    ego_state.transform.position = start_pos
    ego = sim.add_agent(os.environ.get(
        "LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, ego_state)
    logger.info("Set ego at start_pos.")
    return ego


def set_npc(sim, spawns, name, start_pos, is_light):
    npc_state = lgsvl.AgentState()
    npc_state.transform = spawns[0]
    npc_state.transform.position = start_pos
    npc = sim.add_agent(name, lgsvl.AgentType.NPC, npc_state)
    logger.info("Set {} at start_pos.", name)
    if is_light:
        logger.info("Set npc lights on because of night.")
        c = lgsvl.NPCControl()
        c.headlights = 2
        npc.apply_control(c)
    return npc


def raycast_to_ground(sim, position):
    start_height = 5
    start_point = lgsvl.Vector(
        position.x, position.y + start_height, position.z)
    hit = sim.raycast(start_point, lgsvl.Vector(0, -1, 0))
    if hit:
        return hit.point
    else:
        return position


def get_weather_setting(setting_path):
    weather_params_path = os.path.join(setting_path, 'weather.json')
    if not os.path.exists(weather_params_path):
        logger.error("Could not find init weather parameters in {}",
                     weather_params_path)
    with open(weather_params_path, 'r') as file:
        data = json.load(file)
        logger.info("Load init wether parameters successfully.")
    return data


def get_map_setting(setting_path):
    map_params_path = os.path.join(setting_path, 'map.json')
    if not os.path.exists(map_params_path):
        logger.error("Could not find init map parameters in {}",
                     map_params_path)
    with open(map_params_path, 'r') as file:
        data = json.load(file)
        logger.info("Load init map parameters successfully.")
    return data


if __name__ == '__main__':

    # ma = MapParser('src\maps\Highway101GLE/base_map.bin')

    current_file_path = os.path.abspath(__file__)
    current_directory = os.path.dirname(current_file_path)
    settingPath = os.path.join(current_directory, 'settings')
    mapList = get_map_setting(settingPath)

    sim = connect_svl()

    sim = initial_scene(sim)
    sim.set_time_of_day(12)


    start_pos = mapList["ego_start_pos"][0]
    start_pos = lgsvl.Vector(start_pos[0], start_pos[1], start_pos[2])
    ego_state = lgsvl.AgentState()
    start_pos = raycast_to_ground(sim, start_pos)
    ego_state.transform = sim.map_point_on_lane(start_pos)
    forward = lgsvl.utils.transform_to_forward(ego_state.transform)
    right = lgsvl.utils.transform_to_right(ego_state.transform)
    
    ds = ego_state.transform.position + 175 * forward
    ds = raycast_to_ground(sim, ds)
    ego_state.transform = sim.map_point_on_lane(ds)

    

    

    
    ego_state.velocity = 10 * forward

    
    ego = sim.add_agent(os.environ.get(
        "LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, ego_state)

    print('forward',forward)
    print(ego.state)
    n = 0
    while n< 30:
        
        sim.run(1)
        pos = ego.state.transform.position
        print(f"位置: {pos}")  
        n+=1
    

    
    




# lane4 (460.177185058594, 2.0722827911377, 1317.20336914063)


# (413.147979736328, 0, 1182.49719238281)
# (474.225463867188, 0, 1357.41284179688)