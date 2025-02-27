from environment import initial_scene, set_weather, set_ego, set_npc
from behaviors import BehaviorFactory
import lgsvl
from connect_simulator import connect_svl


def on_waypoint(agent, index, total_waypoints):
    print("Waypoint {} reached.".format(index))
    # 检查是否为路径点列表中的最后一个点
    if index == total_waypoints - 1:
        print("Last waypoint reached, switching to follow closest lane.")
        agent.follow_closest_lane(follow=True, max_speed=6, isLaneChange=False)


if __name__ == '__main__':
    sim = connect_svl()
    initial_scene(sim)
    set_weather(sim)
    spawns = sim.get_spawn()
    forward = lgsvl.utils.transform_to_forward(spawns[0])
    right = lgsvl.utils.transform_to_right(spawns[0])
    start_pos = lgsvl.Vector(
        263.174041748047, 1.0931624174118, 762.89306640625)
    npc_pos = start_pos - 3.5 * right  # NPC is 3.5m to the left of the EGO
    ego = set_ego(sim, spawns, start_pos, forward)
    npc1 = set_npc(sim, spawns, "SUV", npc_pos, True)
    print(ego.state.transform.position)
    print(npc1.state.transform.position)
    npc1.follow_closest_lane(follow=True, max_speed=6, isLaneChange=False)
    npcChangedLanes = False
    sim.run(2)
    print("Press Enter to start simulation and make the NPC change lanes.")
    input()

    while True:
        sim.run(0.5)
        # 输出npc1速度
        print(f"npc1 speed: {npc1.state.speed}")
        # 输出npc1的角度
        # print(f"npc1 angle: {npc1.state.rotation}")
        # If the NPC has not already changed lanes then the distance between the NPC and EGO is calculated
        if not npcChangedLanes:
            egoCurrentState = ego.state
            npcCurrentState = npc1.state

            separationDistance = (
                egoCurrentState.position - npcCurrentState.position).magnitude()

            # If the EGO and NPC are within 15m, then NPC will change lanes to the right (in front of the EGO)
            if separationDistance <= 15:
                # npc1.change_lane(False)
                factory = BehaviorFactory()
                change_lane_behavior = factory.get_behavior(
                    "change_lane", sim, npc1.state.position, forward, right, turn_direction='right', num_points=10)
                waypoints = change_lane_behavior.create_waypoints()
                for wp in waypoints:
                    print(wp.position)
                npc1.follow(waypoints, loop=False)
                # npc1.on_lane_change(on_lane_change)
                npcChangedLanes = True
                # After the last waypoint is reached, continue following the lane
                total_waypoints = len(waypoints)
                npc1.on_waypoint_reached(lambda agent, index: on_waypoint(
                    agent, index, total_waypoints))
