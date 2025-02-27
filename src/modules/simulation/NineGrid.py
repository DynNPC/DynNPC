import numpy as np
import lgsvl

class NineGrid:
    def __init__(self, npc, forward, right, grid_width=4.5, grid_height=30):
        self.npc = npc
        self.forward = forward
        self.right = right
        self.grid_width = grid_width
        self.grid_height = grid_height

    def get_ego_region(self, ego_position):
        # 使用npc的当前位置和方向
        npc_position = self.npc.state.position

        # 计算相对位置
        relative_position = lgsvl.Vector(ego_position.x - npc_position.x,
                                          ego_position.y - npc_position.y,
                                          ego_position.z - npc_position.z)

        # 创建矩阵A和向量b
        A = np.array([
            [self.forward.x, self.right.x],
            [self.forward.z, self.right.z]
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
                    return 5  # Zone L1
                elif col == 2:
                    return 2 # Zone N1
                elif col == 3:
                    return 1 # Zone R1
            elif row == 2:
                if col == 1:
                    return 4    # Zone L2
                elif col == 3:
                    return 3    # Zone R2
                elif col == 2:
                    return 0    # 中间npc位置
            elif row == 1:
                if col == 1 or col == 3:
                    return 6  # Zone L3/R3
                if col == 2:
                    return 7
        return None  # 如果m和n的值不在预期范围内，则返回None

