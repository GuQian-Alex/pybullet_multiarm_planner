

import time
import math
import random

import pybullet as p
import pybullet_data as pd


DT = 1.0 / 240.0

# ===== 控制参数 =====
MAX_FORCE = 600
POSITION_GAIN = 0.6
VELOCITY_GAIN = 0.35
MAX_VELOCITY = 2.0

# ===== RRT 参数 =====
RRT_MAX_ITER = 4000
RRT_STEP_SIZE = 0.18
RRT_GOAL_SAMPLE_RATE = 0.15
RRT_GOAL_THRESHOLD = 0.18
RRT_EDGE_CHECK_RESOLUTION = 0.04
PATH_EXECUTION_THRESHOLD = 0.08


class RRTNode:
    def __init__(self, q, parent=None):
        self.q = q
        self.parent = parent


def get_revolute_joints(robot):
    joints = []

    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        if info[2] == p.JOINT_REVOLUTE:
            joints.append(i)

    return joints


def get_joint_limits(robot, joints):
    lower_limits = []
    upper_limits = []

    for j in joints:
        info = p.getJointInfo(robot, j)
        lower = info[8]
        upper = info[9]

        if lower >= upper:
            lower = -math.pi
            upper = math.pi

        lower_limits.append(lower)
        upper_limits.append(upper)

    return lower_limits, upper_limits


def get_current_joint_positions(robot, joints):
    states = p.getJointStates(robot, joints)
    return [s[0] for s in states]


def set_joint_positions_direct(robot, joints, q):
    for j, qj in zip(joints, q):
        p.resetJointState(
            bodyUniqueId=robot,
            jointIndex=j,
            targetValue=qj,
            targetVelocity=0.0
        )


def command_joint_positions(robot, joints, q_cmd):
    for j, qj in zip(joints, q_cmd):
        p.setJointMotorControl2(
            bodyIndex=robot,
            jointIndex=j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=qj,
            targetVelocity=0.0,
            force=MAX_FORCE,
            positionGain=POSITION_GAIN,
            velocityGain=VELOCITY_GAIN,
            maxVelocity=MAX_VELOCITY
        )


def joint_distance(q1, q2):
    return math.sqrt(
        sum((a - b) ** 2 for a, b in zip(q1, q2))
    )


def sample_joint_config(lower_limits, upper_limits):
    return [
        random.uniform(lower_limits[i], upper_limits[i])
        for i in range(len(lower_limits))
    ]


def nearest_node(tree, q_rand):
    best = tree[0]
    best_dist = joint_distance(tree[0].q, q_rand)

    for node in tree[1:]:
        d = joint_distance(node.q, q_rand)
        if d < best_dist:
            best = node
            best_dist = d

    return best


def steer(q_from, q_to, step_size):
    d = joint_distance(q_from, q_to)

    if d <= step_size:
        return list(q_to)

    ratio = step_size / d

    return [
        q_from[i] + ratio * (q_to[i] - q_from[i])
        for i in range(len(q_from))
    ]


def is_within_limits(q, lower_limits, upper_limits):
    for qi, low, high in zip(q, lower_limits, upper_limits):
        if qi < low or qi > high:
            return False
    return True


def is_collision_free(robot, joints, q, obstacle_ids, lower_limits, upper_limits):
    if not is_within_limits(q, lower_limits, upper_limits):
        return False

    set_joint_positions_direct(robot, joints, q)

    #更新碰撞检测数据
    p.performCollisionDetection()

    for obs in obstacle_ids:
        #既可以用于距离查询，也可以用于碰撞/穿透查询
        contacts = p.getClosestPoints(
            bodyA=robot,
            bodyB=obs,
            distance=0.0
        )

        if len(contacts) > 0:
            return False

    return True


def is_edge_collision_free(
    robot,
    joints,
    q_from,
    q_to,
    obstacle_ids,
    lower_limits,
    upper_limits,
    resolution=RRT_EDGE_CHECK_RESOLUTION
):
    dist = joint_distance(q_from, q_to)
    steps = max(2, int(dist / resolution))

    for k in range(steps + 1):
        s = k / steps

        q = [
            q_from[i] + s * (q_to[i] - q_from[i])
            for i in range(len(q_from))
        ]

        if not is_collision_free(
            robot,
            joints,
            q,
            obstacle_ids,
            lower_limits,
            upper_limits
        ):
            return False

    return True


def extract_path(goal_node):
    path = []
    node = goal_node

    while node is not None:
        path.append(node.q)
        node = node.parent

    path.reverse()
    return path


def interpolate_path(path, max_step=0.04):
    dense_path = []

    for i in range(len(path) - 1):
        q0 = path[i]
        q1 = path[i + 1]

        dense_path.append(q0)

        dist = joint_distance(q0, q1)
        steps = max(1, int(dist / max_step))

        for k in range(1, steps):
            s = k / steps
            q = [
                q0[d] + s * (q1[d] - q0[d])
                for d in range(len(q0))
            ]
            dense_path.append(q)

    dense_path.append(path[-1])
    return dense_path


def rrt_plan(
    robot,
    joints,
    q_start,
    q_goal,
    obstacle_ids,
    lower_limits,
    upper_limits
):
    if not is_collision_free(
        robot,
        joints,
        q_start,
        obstacle_ids,
        lower_limits,
        upper_limits
    ):
        print("Start configuration is in collision.")
        return None

    if not is_collision_free(
        robot,
        joints,
        q_goal,
        obstacle_ids,
        lower_limits,
        upper_limits
    ):
        print("Goal configuration is in collision.")
        return None

    tree = [RRTNode(q_start)]

    for it in range(RRT_MAX_ITER):
        if random.random() < RRT_GOAL_SAMPLE_RATE:
            q_rand = q_goal
        else:
            q_rand = sample_joint_config(lower_limits, upper_limits)

        nearest = nearest_node(tree, q_rand)
        q_new = steer(nearest.q, q_rand, RRT_STEP_SIZE)

        if not is_edge_collision_free(
            robot,
            joints,
            nearest.q,
            q_new,
            obstacle_ids,
            lower_limits,
            upper_limits
        ):
            continue

        new_node = RRTNode(q_new, parent=nearest)
        tree.append(new_node)

        if joint_distance(q_new, q_goal) < RRT_GOAL_THRESHOLD:
            if is_edge_collision_free(
                robot,
                joints,
                q_new,
                q_goal,
                obstacle_ids,
                lower_limits,
                upper_limits
            ):
                goal_node = RRTNode(q_goal, parent=new_node)
                path = extract_path(goal_node)
                path = interpolate_path(path, max_step=0.035)

                print(f"RRT success. iter={it}, nodes={len(tree)}, path points={len(path)}")
                return path

        if it % 500 == 0:
            print(f"RRT iter={it}, nodes={len(tree)}")

    print("RRT failed.")
    return None


def create_box_obstacle(position, half_extents, color):
    collision_id = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=half_extents
    )

    visual_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=color
    )

    body_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_id,
        baseVisualShapeIndex=visual_id,
        basePosition=position
    )

    return body_id


def draw_end_effector_path(robot, joints, end_effector, path):
    if path is None or len(path) < 2:
        return

    original_q = get_current_joint_positions(robot, joints)

    ee_positions = []

    for q in path:
        set_joint_positions_direct(robot, joints, q)
        ee_pos = p.getLinkState(robot, end_effector)[0]
        ee_positions.append(ee_pos)

    set_joint_positions_direct(robot, joints, original_q)

    for i in range(len(ee_positions) - 1):
        p.addUserDebugLine(
            lineFromXYZ=ee_positions[i],
            lineToXYZ=ee_positions[i + 1],
            lineColorRGB=[0, 0, 1],
            lineWidth=2,
            lifeTime=0
        )


def execute_path(robot, joints, path):
    if path is None:
        return

    index = 0
    counter = 0

    while index < len(path):

        q_cmd = path[index]

        command_joint_positions(robot, joints, q_cmd)
        p.stepSimulation()

        q_now = get_current_joint_positions(robot, joints)
        err = joint_distance(q_now, q_cmd)

        if counter % 240 == 0:
            print(f"index={index}/{len(path)}, err={err:.4f}")


        if err < PATH_EXECUTION_THRESHOLD:
            index += 1
            print("path point ",index," finished")

        time.sleep(DT)


def main():
    # ===== 1. 初始化仿真 =====
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())

    p.setGravity(0, 0, -9.81)
    p.setTimeStep(DT)
    p.setRealTimeSimulation(0)

    p.loadURDF("plane.urdf")

    robot = p.loadURDF(
        "/home/qian/Documents/UR5_RL_Planning_Pybullet/assets/ur5/ur5.urdf",
        useFixedBase=True
    )

    joints = get_revolute_joints(robot)[:6] #只获取前6个就行了，后面的关节往往是夹爪的关节
    end_effector = joints[-1]

    lower_limits, upper_limits = get_joint_limits(robot, joints)

    print("joints:", joints)
    print("lower_limits:", lower_limits)
    print("upper_limits:", upper_limits)

    # ===== 2. 添加障碍物 =====
    obstacle_ids = []

    obstacle_ids.append(
        create_box_obstacle(
            position=[0.45, 0.00, 0.45],
            half_extents=[0.10, 0.10, 0.10],
            color=[1, 0, 0, 0.6]
        )
    )

    obstacle_ids.append(
        create_box_obstacle(
            position=[0.15, -0.45, 0.35],
            half_extents=[0.10, 0.10, 0.10],
            color=[1, 0.5, 0, 0.6]
        )
    )

    # ===== 3. 设置起点和目标点 =====
    q_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    q_goal = [
        1.2,
        -1.1,
        1.4,
        -1.5,
        1.0,
        0.6
    ]

    # 重置关节角度，设定为起始值
    set_joint_positions_direct(robot, joints, q_start)

    #给一点可视化的时间
    time.sleep(1.0)

    # ===== 4. RRT 路径规划 =====

    #关闭渲染
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)


    path = rrt_plan(
        robot=robot,
        joints=joints,
        q_start=q_start,
        q_goal=q_goal,
        obstacle_ids=obstacle_ids,
        lower_limits=lower_limits,
        upper_limits=upper_limits
    )

    set_joint_positions_direct(robot, joints, q_start)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    if path is None:
        print("No path found.")
        time.sleep(5.0)
        p.disconnect()
        return

    # ===== 5. 绘制末端路径 =====
    draw_end_effector_path(
        robot=robot,
        joints=joints,
        end_effector=end_effector,
        path=path
    )

    # ===== 6. 执行路径 =====
    print("Executing path...")
    execute_path(robot, joints, path)
    print("Path execution finished.")

    # ===== 7. 保持窗口 =====
    while True:
        p.stepSimulation()
        time.sleep(DT)


if __name__ == "__main__":
    main()