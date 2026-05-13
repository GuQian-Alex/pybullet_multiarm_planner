#一个阶段性的小功能集成测试
import time
from robot import Robot
from robot_group import RobotGroup
import pybullet as p
import pybullet_data

def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,9.8)

    p.loadURDF("plane.urdf")
    urdf_path = "assets/ur5/ur5.urdf"

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

    group = RobotGroup([robot1,robot2])

    print("num robots",group.num_robots)
    print("total dof",group.total_dof)

    assert group.num_robots ==2
    assert group.total_dof == 12

    current_q = group.get_joint_positions()
    print("current q is ",current_q)

    assert len(current_q) == 12

    target_q = [
        0.5,-1.2,1.1,0.2,0.1,0.1,
        -0.5,-1.1,1.0,-0.2,-0.1,0.0
    ]

    group.set_joint_positions(target_q)
    time.sleep(1)

    new_q = group.get_joint_positions()
    print("new q is :",new_q)

    assert len(new_q) == 12

    sample_q = group.sample()
    print("sample q is :",sample_q)

    assert len(sample_q) == 12

    dist = group.distance(current_q,target_q)
    print("distance is :",dist)

    assert dist >= 0

    extend_fn = group.get_extend_fn()
    waypoints = extend_fn(current_q,target_q)

    print("num waypoints: ", len(waypoints))

    assert len(waypoints)>0
    assert len(waypoints[0]) == 12

    for q in waypoints:
        group.set_joint_positions(q)
        time.sleep(0.05)

    poses = group.forward_kinematics(target_q)
    print("eef poses:",poses)
    assert len(poses) == 2
    print("RobotGroup basic test passed!")

    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
