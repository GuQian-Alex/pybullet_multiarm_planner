import numpy as np
import utils.pybullet_utils as pu
from itertools import combinations,product

RRT_EDGE_CHECK_RESOLUTION = 0.04
#这个类不负责具体的算法，他负责把多台机械臂包装成一台“虚拟大机械臂”
class RobotGroup:
    def __init__(self,robots):
        self.robots = robots
        self.num_robots = len(robots)
        self.dofs = [robot.dof for robot in robots]
        self.total_dof = sum(self.dofs)
        self.robot_ids = [robot.id for robot in robots]

    def split_config(self,group_config):
        if len(group_config) != self.total_dof:
            raise ValueError(
                f"Expected config length {self.total_dof}, got {len(group_config)}"
            )
        result = []
        start = 0
        for dof in self.dofs:
            end = start + dof
            result.append(group_config[start:end])
            start = end
        return result

    def merge_configs(self,robot_configs):
        if len(robot_configs) != self.num_robots:
            raise ValueError(
                f"Expected config length {self.num_robots}, got {len(robot_configs)}"
            )
        group_config = []
        for config in robot_configs:
            group_config.extend(config)
        return group_config

    def set_joint_positions(self,group_config):
        robot_configs = self.split_config(group_config)
        for robot,config in zip(self.robots,robot_configs):
            robot.set_arm_joints(config)

    def get_joint_positions(self):
        robot_configs = []
        for robot in self.robots:
            robot_configs.append(robot.get_arm_joint_values())
        return self.merge_configs(robot_configs)

    def sample(self):
        robot_configs = []
        for robot in self.robots:
            robot_configs.append(robot.sample_fn())
        return self.merge_configs(robot_configs)

    def difference_fn(self, q1, q2):
        difference = []
        split_q1 = self.split_config(q1)
        split_q2 = self.split_config(q2)
        for robot, q1_, q2_ in zip(self.robots, split_q1, split_q2):
            difference += robot.difference_fn(q1_, q2_)
        return difference

    def distance(self, q1, q2):
        diff = np.array(self.difference_fn(q2, q1))
        return np.sqrt(np.dot(diff, diff))

    def get_extend_fn(self,resolutions = None):
        if resolutions is None:
            resolutions = 0.05 * np.ones(self.total_dof)

        def fn(q1,q2):
                steps = np.abs(np.divide(self.difference_fn(q2,q1),resolutions))
                num_steps = max(1, int(np.ceil(np.max(steps))))
                waypoints = []
                q1_array = np.array(q1)
                diffs = np.array(self.difference_fn(q2,q1))
                for i in range(num_steps):
                    ratio = float(i + 1) / float(num_steps)
                    q = q1_array + ratio * diffs
                    waypoints.append(list(q))
                return waypoints

        return fn

    def forward_kinematics(self,group_config):
        robot_configs = self.split_config(group_config)
        poses = []
        for robot,config in zip(self.robots,robot_configs):
            pose = robot.forward_kinemactics(config)
            print("pose is :",pose)
            poses.append(pose)
        return poses

    def get_collision_fn(self, obstacles, attachments, self_collisions, disabled_collisions,resolution=RRT_EDGE_CHECK_RESOLUTION):
        # check_link_pairs is a 2d list
        check_link_pairs = []
        for i in range(self.num_robots):
            check_link_pairs.append(
                pu.get_self_link_pairs(self.robots[i].id, self.robots[i].joints, disabled_collisions)
                if self_collisions else [])
        moving_bodies = self.robot_ids + [attachment.child for attachment in attachments]
        if obstacles is None:
            obstacles = list(set(pu.get_bodies()) - set(moving_bodies))
        check_body_pairs = list(product(moving_bodies, obstacles)) + list(combinations(moving_bodies, 2))

        def collision_fn(q):
            split_q = self.split_config(q)
            for i, q_ in zip(range(self.num_robots), split_q):
                if pu.violates_limits(self.robots[i].id, self.robots[i].joints, q_):
                    return True
            self.set_joint_positions(q)
            for attachment in attachments:
                attachment.assign()
            for i, pairs in enumerate(check_link_pairs):
                for link1, link2 in pairs:
                    if pu.pairwise_link_collision(self.robots[i].id, link1, self.robots[i].id, link2):
                        return True
            return any(pu.pairwise_collision(*pair) for pair in check_body_pairs)

        def edge_collision_fn(q1,q2):
            dist = self.distance(q1, q2)
            steps = max(2, int(dist / resolution))
            for k in range(steps + 1):
                s = k / steps
                q = [q1[j] + s * (q2[j] - q1[j]) for j in range(len(q1))]
                if not collision_fn(q):
                    return False
            return True
        return collision_fn,edge_collision_fn