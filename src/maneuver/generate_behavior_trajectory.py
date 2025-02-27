
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '/mnt/sda/AdvFuzz/src/maneuver')))
from behavior_trajectory_solver.cut_in_right import cut_in_right
from behavior_trajectory_solver.cut_in_left import cut_in_left
from behavior_trajectory_solver.follow_lane import follow_lane
from behavior_trajectory_solver.turn_left import turn_left
from behavior_trajectory_solver.turn_right import turn_right
from behavior_trajectory_solver.straight_forward import straight_forward


def generate_behavior_trajectory(sim,ego, npc_status, forward, right, behavior_type, num_points, **kwargs):

    # =============Only edit cut_*=================
    if behavior_type == 'follow_lane':
        action_change_freq = kwargs.get('action_change_freq')
        return follow_lane(sim, ego,npc_status, forward, num_points,action_change_freq)
    elif behavior_type == 'straight_forward':
        n = kwargs.get('n')
        return straight_forward(sim, npc_status, forward, right, num_points, n)
    elif behavior_type == 'turn_right':
        n = kwargs.get('n')
        m = kwargs.get('m')
        return turn_right(sim, npc_status, forward, right, num_points, n, m)
    elif behavior_type == 'turn_left':
        n = kwargs.get('n')
        m = kwargs.get('m')
        return turn_left(sim, npc_status, forward, right, num_points, n, m)
    elif behavior_type == 'cut_in_right':
        n = kwargs.get('n')
        m = kwargs.get('m')
        k = kwargs.get('k')
        return cut_in_right(sim,ego, npc_status,forward, right, num_points,n,m,k)
    elif behavior_type == 'cut_in_left':
        n = kwargs.get('n')
        m = kwargs.get('m')
        k = kwargs.get('k')
        return cut_in_left(sim,ego, npc_status,forward, right, num_points,n,m,k)
    else:
        raise ValueError(f"Unsupported behavior type: {behavior_type}")
