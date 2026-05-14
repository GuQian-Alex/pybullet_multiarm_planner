from ompl import base as ob
from ompl import geometric as og

# 调用OMPL的5个步骤
# 什么是状态；
# 状态的范围是多少；
# 什么状态是合法的；
# 起点和终点是什么；
# 用什么规划算法。

# 自定义的合法性检查器
class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si):
        super().__init__(si)

    def isValid(self, state):
        x = state[0]
        y = state[1]

        # 圆形障碍物：圆心 (0, 0)，半径 0.25
        return x * x + y * y > 0.25 * 0.25


def main():

    # 定义实向量状态空间
    space = ob.RealVectorStateSpace(2)
    # OMPL本身不关心这些变量物理上代表什么。它只知道这是一个数学空间。

    # 设置状态空间的边界
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1.0)
    bounds.setHigh(1.0)
    space.setBounds(bounds)

    # Space Information 中记录关于这个状态空间的一切规则
    si = ob.SpaceInformation(space)

    validity_checker = ValidityChecker(si)
    # 把合法性检查其传入SpaceInformation
    si.setStateValidityChecker(validity_checker)

    # 检查从一个点移动到另一个点的“线段”是否合法
    # 这里其实相当于rrt_basice中的从nearest_node向new_node扩展的过程
    si.setStateValidityCheckingResolution(0.01)
    # 配置完毕，可以使用
    si.setup()

    # 从这个状态空间中分配一个状态对象
    start = space.allocState()
    start[0] = -0.8
    start[1] = -0.8

    goal = space.allocState()
    goal[0] = 0.8
    goal[1] = 0.8

    # ProblemDefinition就是“我要解决的问题”
    pdef = ob.ProblemDefinition(si)

    # 设置起点和目标状态，并且允许目标容差为0.05
    pdef.setStartAndGoalStates(start, goal, 0.05)

    planner = og.RRTConnect(si)

    planner.setProblemDefinition(pdef)

    planner.setup()

    # 最多规划2s
    solved = planner.solve(2.0)

    if solved:
        print("Solved!")

        path = pdef.getSolutionPath()
        path.interpolate(20)

        print("Path states:")
        for i in range(path.getStateCount()):
            state = path.getState(i)
            print(f"{i:02d}: x={state[0]:.3f}, y={state[1]:.3f}")
    else:
        print("No solution found.")


if __name__ == "__main__":
    main()