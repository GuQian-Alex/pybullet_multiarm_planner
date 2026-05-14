from ompl import base as ob
from ompl import geometric as og


class OMPLPlanner:
    def __init__(
            self,
            planner_name="rrt_connect",
            solve_time=5.0,
            interpolate_count=100,
            simpilify=True,
    ):
        self.planner_name = planner_name
        self.solve_time = solve_time
        self.interpolate_count =interpolate_count
        self.simpilify = simpilify

    def plan(
            self,
            start_config,
            goal_config,
            lower_limits,
            upper_limits,
            collision_fn,
    ):
        dim = len(start_config)
        space = ob.RealVectorStateSpace(dim)
        bounds = ob.RealVectorBounds(dim)

        for i in range(dim):
            bounds.setLow(i,lower_limits[i])
            bounds.setHigh(i,upper_limits[i])
        space.setBounds(bounds)

        setup = og.SimpleSetup(space)

        def is_state_valid(state):  #这里为什么要多次一举？
            q = [state[i] for i in range(dim)]
            return collision_fn(q)

        validity_checker = ValidityChecker(
            setup.getSpaceInformation(),
            dim,
            is_state_valid,
        )

        setup.setStateValidityChecker(validity_checker)

        start = space.allocState()
        goal = space.allocState()

        for i in range(dim):
            start[i] = start_config[i]
            goal[i] = goal_config[i]

        setup.setStartAndGoalStates(start,goal)

        planner = self._make_planner(
            self.planner_name,
            setup.getSpaceInformation(),
        )

        setup.setPlanner(planner)

        solved = setup.solve(self.solve_time)

        if not solved:
            return None

        if self.simpilify:
            setup.simplifySolution()

        path = setup.getSolutionPath()

        if self.interpolate_count is not None:
            path.interpolate(self.interpolate_count)

        return self._path_to_list(path,dim)

    def _make_planner(self,planner_name,space_information):
        if planner_name == "rrt":
            return og.RRT(space_information)

        if planner_name == "rrt_connect":
            return og.RRTConnect(space_information)

        if planner_name == "rrt_star":
            return og.RRTstar(space_information)

        if planner_name == "prm":
            return og.PRM(space_information)

        raise ValueError(f"Unsupported OMPL planner:{planner_name}")


    def _path_to_list(self,path,dim):
        result = []
        for state in path.getStates():
            q = [state[i] for i in range(dim)]
            result.append(q)

        return result



class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si, dim, collision_fn):
        super().__init__(si)
        self.dim = dim
        self.collision_fn = collision_fn

    def isValid(self, state):
        q = [state[i] for i in range(self.dim)]
        return not self.collision_fn(q)


