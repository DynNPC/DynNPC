from environment import initial_scene, set_weather, set_ego, set_npc
import lgsvl
from connect_simulator import connect_svl
import os
import numpy as np
import time

class NineGrid:
    def __init__(self, npc_position, npc_forward, npc_right, grid_width=3.5, grid_height=10):
        self.npc_position = npc_position
        self.npc_forward = npc_forward
        self.npc_right = npc_right
        self.grid_width = grid_width
        self.grid_height = grid_height

    def get_ego_region(self, ego_position):
        # 计算相对位置
        relative_position = lgsvl.Vector(ego_position.x - self.npc_position.x,
                                          ego_position.y - self.npc_position.y,
                                          ego_position.z - self.npc_position.z)

        # 创建矩阵A和向量b
        A = np.array([
            [self.npc_forward.x, self.npc_right.x],
            [self.npc_forward.z, self.npc_right.z]
        ])
        b = np.array([relative_position.x, relative_position.z])

        # 解线性方程组来找到m和n
        try:
            m, n = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            return None  # 如果A不可逆，则返回None

        # 根据m和n的值判断九宫格位置
        if abs(m) <= self.grid_height * 1.5 and abs(n) <= self.grid_width * 1.5:
            # 判断列
            if abs(n) <= self.grid_width * 0.5:
                col = 2  # 中
            elif n > self.grid_width * 0.5:
                col = 3  # 右
            else:
                col = 1  # 左

            # 判断行
            if abs(m) <= self.grid_height * 0.5:
                row = 2  # 中
            elif m > self.grid_height * 0.5:
                row = 1  # 上
            else:
                row = 3  # 下

            # 根据行列确定区域编号
            if row == 3:
                if col == 1:
                    return 5
                elif col == 2:
                    return 2
                elif col == 3:
                    return 1
            elif row == 2:
                if col == 1:
                    return 4
                elif col == 3:
                    return 3
                elif col == 2:
                    return 0    # 中间npc位置
            elif row == 1:
                if col == 2:
                    return 6  # 表示在上半范围
        return None  # 如果m和n的值不在预期范围内，则返回None


if __name__ == '__main__':
    sim = connect_svl()
    initial_scene(sim)
    set_weather(sim)
    spawns = sim.get_spawn()
    forward = lgsvl.utils.transform_to_forward(spawns[0])
    right = lgsvl.utils.transform_to_right(spawns[0])
    start_pos = lgsvl.Vector(
        263.174041748047, 1.0931624174118, 762.89306640625)
    npc_pos = start_pos   # NPC is 3.5m to the left of the EGO
    npc_pos_new = npc_pos + 10 * forward
    ego = set_ego(sim, spawns, start_pos, forward)
    npc1 = set_npc(sim, spawns, "SUV", npc_pos_new, True)
    npc1.follow_closest_lane(follow=True, max_speed=0, isLaneChange=False)
    print(ego.state.transform.position)
    print(npc1.state.transform.position)
    while True:
        NineGrid1 = NineGrid(npc1.state.transform.position, forward, right)
        # 统计时间
        start = time.time()
        print(NineGrid1.get_ego_region(ego.state.transform.position))
        end = time.time()
        print("Time: ", end - start)
        sim.run(2)
    # # 获取车道信息
    # ma = MapParser('src\maps\Highway101GLE/base_map.bin')
    # ego_state = ego.state
    # agent_x, agent_y = hdmap.LGSVL2MapPos(sim, ego_state.transform)
    # ego_lane = ma.get_lane_by_id(hdmap.findLane(ma, agent_x, agent_y))
    # print(ego_lane)

    # npc1_state = npc1.state
    # agent_x, agent_y = hdmap.LGSVL2MapPos(sim, npc1_state.transform)
    # npc_lane = ma.get_lane_by_id(hdmap.findLane(ma, agent_x, agent_y))
    # print(npc_lane)
