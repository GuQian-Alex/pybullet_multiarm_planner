from planners.rrt_toy import RRTPlanner as toy_planner
from planners.rrt_basic import RRTPlanner as basic_planner
from simulation.pybullet_world import PyBulletWorld
from robots.robot_group import RobotGroup
from robots.robot import Robot

def main():
    world = PyBulletWorld(render=True)
    world.connect()
    world.load_plane()

    # world.load_obstacles(
    #     [
    #         {"type":"box",
    #          "position":[0,0,0.5],
    #          "size":[0.2,0.2,1.0],
    #          "color":[1,0,0,1],
    #          }
    #     ]
    # )

    urdf_path="assets/ur5/ur5.urdf"
    initial_poses = [
        [[-0.4,0,0.5],[0,0,0,1]],
         [[0.4,0,0.5],[0,0,0,1]]
    ]

    initial_configs = [
        [0,-1.0,1.0,0,0,0],
        [0,-1.0,1.0,0,0,0]
    ]
    urdf_paths = [urdf_path,urdf_path]

    robot1 = Robot(urdf_path,initial_poses[0][0],initial_poses[0][1],initial_configs[0])
    # robot2 = Robot(urdf_path,initial_poses[1][0],initial_poses[1][1],initial_configs[1])

    group = RobotGroup([robot1])
    collision_fn = world.make_collision_fn(group)

    start_config = group.get_joint_positions()
    goal_config = [1.0,-1.2,1.0,0.3,0.2,0.0]

    #toy_planner -----> rrt_toy.py
    # planner = toy_planner(
    #     max_iterations=2000,
    #     goal_sample_rate=0.2,
    #     goal_tolerance=0.05,
    # )

    # path = planner.plan(
    #     start_config=start_config,
    #     goal_config=goal_config,
    #     sample_fn=group.sample,
    #     distance_fn = group.distance,
    #     extend_fn=group.get_extend_fn(),
    #     collision_fn=collision_fn,
    # )

    # basic_planner ----->rrt_basic.py
    planner = basic_planner(
        max_iterations=2000,
        goal_sample_rate=0.2,
        goal_tolerance=0.05,
    )

    path = planner.plan(
        start=start_config,
        goal_sample=goal_config,
        distance=group.distance,
        sample=group.sample,
        extend=group.get_extend_fn(),
        collision=collision_fn,
        goal_test=lambda q: False,
        iterations= planner.max_iteration,
        goal_probability=planner.goal_sample_rate,
        greedy=True,
        visualize=False,
        fk=None,
        group=False,
    )

    if path is None:
        print("Planning failed")
        return

    print("planning succeeded")
    print("Path length",len(path))

    while True:
        for q in path:
            group.set_joint_positions(q)
            world.step(steps=10,sleep=0.01)

if __name__ == "__main__":
    main()


