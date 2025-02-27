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
     
        npc_position = self.npc.state.position

       
        relative_position = lgsvl.Vector(ego_position.x - npc_position.x,
                                          ego_position.y - npc_position.y,
                                          ego_position.z - npc_position.z)

      
        A = np.array([
            [self.forward.x, self.right.x],
            [self.forward.z, self.right.z]
        ])
        b = np.array([relative_position.x, relative_position.z])

      
        try:
            m, n = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            return None  

       
        if abs(m) <= self.grid_height * 1.5 and abs(n) <= self.grid_width * 1.5:
           
            if abs(n) <= self.grid_width * 0.5:
                col = 2  
            elif n > self.grid_width * 0.5:
                col = 3  
            else:
                col = 1  

            
            if abs(m) <= self.grid_height * 0.5:
                row = 2  
            elif m > self.grid_height * 0.5:
                row = 1  
            else:
                row = 3  

          
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
                    return 0    
            elif row == 1:
                if col == 1 or col == 3:
                    return 6  # Zone L3/R3
                if col == 2:
                    return 7
        return None  

