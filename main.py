import json
import time

from simulation.pybullet_world import PyBulletWorld
from robots.robot_group import RobotGroup
from robots.robot import Robot
from planners.rrt_basic import RRTPlanner


def load_world_config(path):
    with open(path,"r") as f:    #with保证任务完成后自动关闭文件
        return json.load(f)

def create_robots(robot_configs):
    robots=[]

    for robot_config in robot_configs:
        robot = Robot(
            urdf_path=robot_config["urdf"],
            base_position=robot_config["base_position"],
            base_oritation=robot_config["base_orientation"],
            initial_joints=robot_config["start"],
        )
        robots.append(robot)

    return robots


def main():
    config = load_world_config("config/demo_config.json")

    world = PyBulletWorld(render=True)
    world.connect()
    world.load_plane()

    world.load_obstacles(config.get("obstacles",[])) #如果不存在就返回空列表[]
    robots = create_robots(config["robots"])
    group = RobotGroup(robots)
    print("Group create finished")

    start_config = group.get_joint_positions()
    goal_config = group.merge_configs([
        robot_config["goal"]
        for robot_config in config["robots"]
    ])

    collision_fn = world.make_collision_fn(group)

    planner = RRTPlanner(
        max_iterations=3000,
        goal_sample_rate=0.2,
        goal_tolerance=0.05,
    )

    print("start planning")
    path = planner.plan(
        start=start_config,
        goal_sample=goal_config,
        distance=group.distance,
        sample=group.sample,
        extend=group.get_extend_fn(),
        collision=collision_fn,
        goal_test=lambda q: False,
        iterations=planner.max_iteration,
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
            group.control_joints_positions(q)
            world.step()
            print("step finished")

# 按装订区域中的绿色按钮以运行脚本。
if __name__ == '__main__':
    main()

# 访问 https://www.jetbrains.com/help/pycharm/ 获取 PyCharm 帮助
