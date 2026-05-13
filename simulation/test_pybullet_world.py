from simulation.pybullet_world import PyBulletWorld
from robots.robot_group import RobotGroup
from robots.robot import Robot

def main():
    world = PyBulletWorld(render=True)
    world.connect()
    world.load_plane()

    world.load_obstacles(
        [
            {"type":"box",
             "position":[0,0,0.5],
             "size":[0.2,0.2,1.0],
             "color":[1,0,0,1],
             }
        ]
    )

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
    robot2 = Robot(urdf_path,initial_poses[1][0],initial_poses[1][1],initial_configs[1])

    group = RobotGroup([robot1, robot2])
    collision_fn = world.make_collision_fn(group)

    q=group.get_joint_positions()

    print("current q:",q)
    print("collision:",collision_fn(q))

    print("RobotGroup basic test passed!")

if __name__ == "__main__":
    main()
