import random
import lgsvl
from utils import raycast_to_ground

def generate_random_position(bubble, lane_index, forward_index, sim, forward):
    """
    lane_index: 0, 1, 2, 3
    region_range: 0-150的范围内取值
    """
    # 从bubble中获取车道的范围
    lane_scope = bubble["scope"][lane_index]
    
    # 每个车道划分为三个区域
    x_min = lane_scope[0][0]
    x_max = lane_scope[1][0]
    z_min = lane_scope[0][1]
    z_max = lane_scope[1][1]
    
    # 车道宽度为 3.5 米
    lane_width = 3.5
    # 每个车道划分为三个区域

    position_start = lgsvl.Vector(x_min, 0, z_min)
    
    # 起始位置+多少forward
    position_end = position_start + forward_index * forward
    
    ground_position = raycast_to_ground(sim, position_end)
    
    return ground_position

    