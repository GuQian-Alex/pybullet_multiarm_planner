import os
import pybullet as p
import numpy as np
import utils.pybullet_utils as pu
from math import pi
from pathlib import Path

# 默认数值都是UR5的数值，也可以导入自定义机械臂模型


class Robot:
    def __init__(self,urdf_path,base_position,base_oritation,initial_joints):
        self.base_position = base_position
        self.base_oritation = base_oritation
        self.initial_joints = initial_joints
        self.urdf_path = urdf_path


        PROJECT_ROOT = Path(__file__).resolve().parents[1]
        p.setAdditionalSearchPath(str(PROJECT_ROOT))

        if urdf_path and os.path.exists(self.urdf_path):
            print(f"加载自定义URDF: {urdf_path}")
            self.id = p.loadURDF(self.urdf_path,
                                 basePosition=self.base_position,
                                 baseOrientation=self.base_oritation,
                                 flags=p.URDF_USE_SELF_COLLISION
                                 )

        else:
            print("加载默认UR5模型")
            self.id = p.loadURDF("assets/ur5/ur5.urdf",
                                 basePosition=self.base_position,
                                 baseOrientation=self.base_oritation,
                                 flags=p.URDF_USE_SELF_COLLISION
                                 )

        # pu.dump_body(self.id)
        self.joints = pu.get_revolute_joints(self.id) #这里默认机械臂是不带有末端夹爪的，否这这里会超
        self.dof = len(self.joints)
        self.end_effector = self.joints[-1]
        self.lower_limits, self.upper_limits = pu.get_joints_limits(self.id, self.joints)

        self.difference_fn = pu.get_difference_fn(self.id,self.joints)
        self.distance_fn = pu.get_distance_fn(self.id,self.joints)
        self.sample_fn = pu.get_sample_fn(self.id,self.joints)
        self.extend_fn = pu.get_extend_fn(self.id,self.joints)



    def set_arm_joints(self,joint_values):
        pu.set_joint_positions(self.id, self.joints, joint_values)
        pu.control_joints(self.id,self.joints,joint_values)

    def control_arm_joint(self,joint_values):
        pu.control_joints(self.id,self.joints,joint_values)

    def get_arm_joint_values(self):
        return pu.get_joint_positions(self.id,self.joints)

    def get_eef_pose(self):
        return pu.get_link_pose(self.id,self.end_effector)

    def inverse_kinematics(self,position,orientation=None):
        return pu.inverse_kinematics(self.id,self.joints,position,orientation)

    def forward_kinemactics(self,joint_values):
        return pu.forward_kinematics(self.id,self.joints,joint_values,self.end_effector)

    def reset(self):
        self.set_arm_joints(self.initial_joints)






