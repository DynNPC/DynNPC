import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '/mnt/sda/AdvFuzz/src/maneuver')))
import numpy as np
from utils import raycast_to_ground, bezier_point, bezier_derivative
import lgsvl
import random
from loguru import logger

def cut_in_right(sim,ego,npc, forward, right,num_points,n,m,k):
    """
    生成向右变道路径点。
    n: npc从start_pos直行n个forward到达P0开始变道
    m: P0需要m个forward到达P1控制点
    k: P2需要k个forward到达P3结束点
    """
    waypoints = []
    # 定义贝塞尔曲线的控制点
    P0 = npc.state.position + forward * n
    P1 = P0 + forward * m
    P2 = P1 + right * 3.5
    P3 = P2 + forward * k
    points = [P0, P1, P2, P3]
    last_point = None
    total_distance = n
    # 生成贝塞尔曲线上的路径点
    for i in range(1, num_points + 1):
        t = i / num_points
        current_point = bezier_point(t, *points)
        ground_point = raycast_to_ground(sim, current_point)
        if last_point is not None:
            total_distance += np.linalg.norm(np.array([ground_point.x, ground_point.z]) - np.array([last_point.x, last_point.z]))
        last_point = ground_point
        if i == num_points:
            # P3
            mapped_point = sim.map_point_on_lane(ground_point)           
            n_forward = calculate_distance(mapped_point, ego, forward)
            # P2
            mapped_point_P2 = sim.map_point_on_lane(raycast_to_ground(sim, current_point - forward * k))
            n_forward_P2 = calculate_distance(mapped_point_P2, ego, forward)
            ground_point = mapped_point.position
        # 计算切线方向
        tangent = bezier_derivative(t, *points)
        angle = np.degrees(np.arctan2(tangent.x, tangent.z))
        waypoints.append(lgsvl.DriveWaypoint(ground_point, 0, angle=lgsvl.Vector(0, angle, 0)))  # 初始化速度为0
            # 计算速度并赋值给所有路径点
            
    #speed = calculate_velocity(ego, npc, n_forward, total_distance)
    strategy, speed = randomSpeed(n_forward, n_forward_P2, npc, ego, total_distance)
    logger.info(f"{npc.name} executives strategy: {strategy}")
    for waypoint in waypoints:
        waypoint.speed = speed
    return waypoints
    
def calculate_distance(final_point, ego, forward):
    final_point = np.array([final_point.position.x, final_point.position.z])
    ego_position = np.array([ego.state.position.x, ego.state.position.z])
    forward = np.array([forward.x, forward.z])
    direction_vector = final_point - ego_position
    dot_product = np.dot(direction_vector, forward)
    forward_magnitude = np.linalg.norm(forward)
    return dot_product / forward_magnitude

def calculate_velocity(ego, npc, n_forward, distance):
    is_npc_ahead = npc.state.position.x < ego.state.position.x

    npc_length = npc.bounding_box.max.x - npc.bounding_box.min.x
    t0 = (n_forward - (npc_length / 2)) / max(ego.state.speed, 10)
    t1 = (n_forward + (npc_length / 2)) / max(ego.state.speed, 10)
    v0 = distance / max(t0, 1)
    v1 = distance / max(t1, 1)
    velocity = np.random.uniform(v0, v1)

    if not is_npc_ahead:
        velocity = min(velocity, ego.state.speed)
    print(f"velocity:{velocity}")
    return velocity

def randomSpeed(n_forward, n_forward_P2, npc, ego, total_distance):
    max_forward = n_forward
    min_forward = n_forward_P2
    
    #npc_length = npc_status['bbox'].max.x - npc_status['bbox'].min.x
    npc_length = npc.bounding_box.max.x - npc.bounding_box.min.x
    #设一个最低速度，防止速度太小，时间太长，算的npc太小
    tmin = (min_forward - (npc_length / 2)) / max(ego.state.speed, 10)
    tmax = (max_forward + (npc_length / 2)) / max(ego.state.speed, 10)
    
    vmax = total_distance / tmin
    vmin = total_distance / tmax
    
    seed = random.random()
    # 引入一个与npc速度比较的模块？
    if seed < 1/3:
        speed = np.random.uniform(0, vmin)
        strategy = "yield"
    elif seed < 2/3:
        speed = np.random.uniform(vmin, vmax)
        strategy = "interactivate"
    else:
        speed = 15 if vmax > 15 else np.random.uniform(vmax, 15)
        strategy = "overtake"
    speed = 1 if speed < 1 else speed
    speed = 15 if speed > 15 else speed
    return strategy, speed