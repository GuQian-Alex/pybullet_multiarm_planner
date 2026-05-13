import pybullet as p
import pybullet_data
import time
import numpy as np
import os
from pathlib import Path

class UR5Viewer:
    def __init__(self, urdf_path=None, gui=True):
        """
        初始化UR5观察器
        
        Args:
            urdf_path: URDF文件路径，如果为None则使用默认UR5
            gui: 是否显示GUI界面
        """
        # 连接物理引擎
        if gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        # 设置搜索路径
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 设置重力
        p.setGravity(0, 0, -9.8)
        
        # 设置相机参数
        self.setup_camera()
        
        # 加载地面
        self.plane_id = p.loadURDF("plane.urdf")
        
        # 加载UR5机械臂

        PROJECT_ROOT = Path(__file__).resolve().parents[1]
        p.setAdditionalSearchPath(str(PROJECT_ROOT))

        if urdf_path and os.path.exists(urdf_path):
            print(f"加载自定义URDF: {urdf_path}")
            self.ur5_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.5])
        else:
            print("加载默认UR5模型")
            self.ur5_id = p.loadURDF("assets/ur5/ur5.urdf", basePosition=[0, 0, 0.5])
        
        # 获取关节信息
        self.num_joints = p.getNumJoints(self.ur5_id)
        self.joint_info = self.get_joint_info()
        
        # 重置机械臂到初始位置
        self.reset_to_initial_pose()
        
        print(f"UR5加载完成，共有 {self.num_joints} 个关节")
        print("使用鼠标和键盘控制视角:")
        # print(" - 鼠标左键: 旋转视角")
        # print(" - 鼠标中键: 平移视角")
        print(" - 鼠标滚轮: 缩放")
        # print(" - WASD: 移动相机")
        # print(" - Q/E: 上升/下降相机")
        print(" - R: 重置机械臂姿势")
        print(" - 空格: 随机机械臂姿势")
    
    def setup_camera(self):
        """设置相机初始位置和参数"""
        # 设置相机距离和角度
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.5]
        )
        
        # 设置调试可视化参数
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
    
    def get_joint_info(self):
        """获取关节信息"""
        joint_info = {}
        for i in range(self.num_joints):
            info = p.getJointInfo(self.ur5_id, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]
            joint_info[joint_name] = {
                'index': i,
                'type': joint_type,
                'lower_limit': info[8],
                'upper_limit': info[9],
                'max_force': info[10],
                'max_velocity': info[11]
            }
            print(f"关节 {i}: {joint_name}, 类型: {joint_type}")
        return joint_info
    
    def reset_to_initial_pose(self):
        """重置机械臂到初始姿势"""
        # UR5的初始关节角度（弧度）
        initial_positions = [0, -1.57, 1.57, -1.57, -1.57, 0]  # 近似零位姿势
        
        for i in range(6):  # UR5有6个自由度
            p.resetJointState(self.ur5_id, i, initial_positions[i])
    
    def set_random_pose(self):
        """设置随机关节姿势"""
        for i in range(6):
            joint_name = f"shoulder_pan_joint" if i == 0 else \
                        f"shoulder_lift_joint" if i == 1 else \
                        f"elbow_joint" if i == 2 else \
                        f"wrist_1_joint" if i == 3 else \
                        f"wrist_2_joint" if i == 4 else \
                        f"wrist_3_joint"
            
            joint_info = self.joint_info.get(joint_name, {})
            lower = joint_info.get('lower_limit', -3.14)
            upper = joint_info.get('upper_limit', 3.14)
            
            random_angle = np.random.uniform(lower, upper)
            p.resetJointState(self.ur5_id, i, random_angle)
    
    def get_end_effector_position(self):
        """获取末端执行器位置"""
        # 假设末端执行器是最后一个链接
        end_effector_state = p.getLinkState(self.ur5_id, self.num_joints - 1)
        return end_effector_state[0]  # 世界坐标系中的位置
    
    def add_debug_objects(self):
        """添加调试物体帮助观察"""
        # 在基座标系添加坐标轴
        p.addUserDebugLine([0, 0, 0], [0.2, 0, 0], [1, 0, 0], parentObjectUniqueId=self.ur5_id, parentLinkIndex=-1)
        p.addUserDebugLine([0, 0, 0], [0, 0.2, 0], [0, 1, 0], parentObjectUniqueId=self.ur5_id, parentLinkIndex=-1)
        p.addUserDebugLine([0, 0, 0], [0, 0, 0.2], [0, 0, 1], parentObjectUniqueId=self.ur5_id, parentLinkIndex=-1)
        
        # 在末端执行器添加标记
        end_effector_pos = self.get_end_effector_position()
        p.addUserDebugPoints([end_effector_pos], [[1, 0, 0]], pointSize=10, lifeTime=0)
    
    def interactive_control(self):
        """交互式控制循环"""
        print("开始交互式控制...")
        
        while True:
            # 处理键盘输入
            keys = p.getKeyboardEvents()
            
            if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                print("重置机械臂姿势")
                self.reset_to_initial_pose()
            
            if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
                print("设置随机姿势")
                self.set_random_pose()
            
            # 更新调试信息
            self.add_debug_objects()
            
            # 步进模拟
            p.stepSimulation()
            time.sleep(1./240.)
    
    def run(self):
        """运行观察器"""
        try:
            self.interactive_control()
        except KeyboardInterrupt:
            print("程序被用户中断")
        finally:
            p.disconnect()

def main():
    """主函数"""
    # 可以选择使用自定义URDF文件
    
    # custom_urdf = None  # 设置为None使用默认UR5
    # custom_urdf = "assets/ur5/ur5_with_gripper.urdf"  # 取消注释使用自定义URDF
    # custom_urdf = "assets/ur5/ur5_training.urdf"  # 取消注释使用自定义URDF
    custom_urdf = "assets/ur5/ur5.urdf"
    # 创建观察器
    viewer = UR5Viewer(urdf_path=custom_urdf, gui=True)
    
    # 运行观察器
    viewer.run()

if __name__ == "__main__":
    main()