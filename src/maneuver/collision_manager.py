# collision_manager.py
import lgsvl

collision_detected = False

def stop_ego_vehicle(ego):

    state = ego.state
    state.velocity = lgsvl.Vector(0, 0, 0)
    ego.state = state
 
    ego.brake = 1

def stop_npc_vehicle(agent):
 
    state = agent.state
    state.velocity = lgsvl.Vector(0, 0, 0)
    agent.state = state

    agent.brake = 1


def on_collision(agent1, agent2, contact):
    global collision_detected
    collision_detected = True
    # [<lgsvl.agent.EgoVehicle object at 0x000001A8CF862F40>, <lgsvl.agent.NpcVehicle object at 0x000001A8CF862C70>]
  
    # if isinstance(agent1, lgsvl.EgoVehicle):
    #     stop_ego_vehicle(agent1)
    # elif isinstance(agent2, lgsvl.EgoVehicle):
    #     stop_ego_vehicle(agent2)
   
    if isinstance(agent1, lgsvl.NpcVehicle):
        stop_npc_vehicle(agent1)
    elif isinstance(agent2, lgsvl.NpcVehicle):
        stop_npc_vehicle(agent2)

 
    print(agent1,agent2,contact)
    # if isinstance(agent1, lgsvl.agent.EgoVehicle) and isinstance(agent2, lgsvl.agent.NpcVehicle):
    #     print("Collision detected between EGO and NPC.")
    # elif isinstance(agent1, lgsvl.agent.NpcVehicle) and isinstance(agent2, lgsvl.agent.EgoVehicle):
    #     print("Collision detected between NPC and EGO.")
    # else:
    #     print("Collision detected between two NPCs.")

def empty_callback(agent1, agent2, contact):
    pass

def reset_collision():
    global collision_detected
    collision_detected = False

def set_collision_callback(ego):
    ego.on_collision(on_collision)

def remove_collision_callback(ego):
    ego.on_collision(empty_callback)

def is_collision_detected():
    global collision_detected
    return collision_detected
