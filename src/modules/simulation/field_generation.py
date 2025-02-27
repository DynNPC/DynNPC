import random
import lgsvl
from utils import raycast_to_ground

def generate_random_position(bubble, lane_index, forward_index, sim, forward):
    """
    lane_index: 0, 1, 2, 3
    region_range: 0-150
    """
    
    lane_scope = bubble["scope"][lane_index]
    
    
    x_min = lane_scope[0][0]
    x_max = lane_scope[1][0]
    z_min = lane_scope[0][1]
    z_max = lane_scope[1][1]
    
   
    lane_width = 3.5
    

    position_start = lgsvl.Vector(x_min, 0, z_min)
    
  
    position_end = position_start + forward_index * forward
    
    ground_position = raycast_to_ground(sim, position_end)
    
    return ground_position

    