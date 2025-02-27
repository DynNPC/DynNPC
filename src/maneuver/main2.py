from environment import initial_scene, set_weather, set_ego, set_npc
from generate_behavior_trajectory import generate_behavior_trajectory
from utils import raycast_to_ground
import lgsvl
from connect_simulator import connect_svl
from loguru import logger
import sys
import os
current_dir = os.path.dirname(__file__)
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from modules.simulation.liability import get_bbox2, get_distance_ego_npc2
from modules.simulation.NineGrid import NineGrid


def on_waypoint(agent, index, waypoints):
    total_waypoints = len(waypoints)
    # 检查是否为路径点列表中的最后一个点
    if index == total_waypoints - 1:
        # 以末速度继续行驶
        agent.follow_closest_lane(follow=True, max_speed=waypoints[-1].speed)

def on_collision(agent1, agent2, contact):
    global accident_happen
    global collision_info
    global time_index

    accident_happen = True
    collision_info = {}

    name1 = "STATIC OBSTACLE" if agent1 is None else agent1.name
    name2 = "STATIC OBSTACLE" if agent2 is None else agent2.name
    logger.error(str(name1) + " collided with " + str(name2) + " at " + str(contact))

    agent1_info = [agent1.state, agent1.bounding_box]
                    
    if not agent2:
        agent2_info = [None, None]
    else:
        agent2_info = [agent2.state, agent2.bounding_box]
    
    if contact:
        contact_loc = [contact.x, contact.y, contact.z]
    
    # collision_info['time'] = time_index
    collision_info['ego'] = agent1_info
    collision_info['npc'] = agent2_info
    # collision_info['contact'] = contact_loc

    # sim.stop()

def handle_waypoints1(sim, npc, forward, right):
    waypoints = generate_behavior_trajectory(sim, ego, npc, forward, right, 'follow_lane', num_points=10, action_change_freq=5)
    npc.follow(waypoints, loop=False)
    npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                agent, index, waypoints))
    
def handle_waypoints2(sim, npc, forward, right):
    waypoints = generate_behavior_trajectory(sim, ego, npc, forward, right,  'cut_in_right', num_points=10, n=3, m=3, k=3)
    npc.follow(waypoints, loop=False)
    npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                agent, index, waypoints))

def handle_waypoints3(sim, npc, forward, right):
    waypoints = generate_behavior_trajectory(sim, ego, npc, forward, right,  'cut_in_left', num_points=10, n=3, m=3, k=3)
    npc.follow(waypoints, loop=False)
    npc.on_waypoint_reached(lambda agent, index: on_waypoint(
                agent, index, waypoints))


if __name__ == '__main__':
    sim = connect_svl()
    initial_scene(sim)
    set_weather(sim)
    # sim.set_time_of_day(0)
    spawns = sim.get_spawn()
    forward = lgsvl.utils.transform_to_forward(spawns[0])
    right = lgsvl.utils.transform_to_right(spawns[0])
    # "x": 272.908563232422, "y": 1.09990131855011,"z": 780.539184570313
    start_pos_1 = lgsvl.Vector(
        272.908563232422, 1.09990131855011, 780.539184570313)
    start_pos = raycast_to_ground(sim, start_pos_1)
    print(f"start_pos: {start_pos}")
    npc_pos = start_pos  # NPC is 3.5m to the left of the EGO
    npc_pos = npc_pos + 100 * forward
    ego = set_ego(sim, spawns, start_pos, forward)
    npc1 = set_npc(sim, spawns, "Sedan", npc_pos, True)
    npc_pos2 = npc_pos - 3.5 * right + 10 * forward
    npc2 = set_npc(sim, spawns, "SUV", npc_pos2, True)
    npc_pos3 = npc_pos + 3.5 * right + 20 * forward
    npc3 = set_npc(sim, spawns, "SUV", npc_pos3, True)
    npc_pos4 = npc_pos + 7 * right + 30 * forward
    npc4 = set_npc(sim, spawns, "SUV", npc_pos4, True)
    npc_list = [npc1, npc2, npc3, npc4]
    flag_list = [True, True, True, True]
    for i in range(2000):
        for index,npc in enumerate(npc_list):
            nine_grid = NineGrid(npc, forward, right)
            region = nine_grid.get_ego_region(ego.state.position)
            print(f"npc{index+1} region: {region}")
            if flag_list[index] and region == 2:
                handle_waypoints1(sim, npc, forward, right)
                flag_list[index] = False
            elif flag_list[index] and region == 1:
                handle_waypoints2(sim, npc, forward, right)
                flag_list[index] = False
            elif flag_list[index] and region == 5:
                handle_waypoints3(sim, npc, forward, right)
                flag_list[index] = False
            elif flag_list[index] and region == 0:
                npc.follow_closest_lane(True, npc.state.speed)
                flag_list[index] = False
            elif flag_list[index] and region == None:
                npc.follow_closest_lane(True, 0)
        sim.run(0.1)
        ego.on_collision(on_collision)
        # # 输出npc1速度
        # print(f"npc1 speed: {npc1.state.speed}")
        # # 输出npc1位置
        # print(f"npc1 position: {npc1.state.transform.position}")
    input("Press Enter to continue...")
    sim.run(2)
