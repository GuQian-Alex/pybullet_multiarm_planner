import random



class TreeNode:
    def __init__(self,config,parent=None):
        self.config = config
        self.parent = parent

    def retrace_path(self):
        path = []
        node = self

        while node is not None:
            path.append(node.config)
            node = node.parent

        path.reverse()
        return path

class RRTPlanner:
    def __init__(
            self,
            max_iterations=2000,
            goal_sample_rate=0.2,
            goal_tolerance=0.05,
    ):
        self.max_iteration=max_iterations
        self.goal_sample_rate=goal_sample_rate
        self.goal_tolerance=goal_tolerance

    def plan(
            self,
            start_config,
            goal_config,
            sample_fn,
            distance_fn,
            extend_fn,
            collision_fn,
    ):
        if collision_fn(start_config):
            print("RRT failed: start config is in collision")
            return None

        if collision_fn(goal_config):
            print("RRT failed: goal config is in collision")
            return None

        tree = [TreeNode(start_config)]

        for i in range(self.max_iteration):
            if random.random()<self.goal_sample_rate:
                sample_config=goal_config
            else:
                sample_config=sample_fn()

            nearest_node = self.find_nearst_node(
                tree,
                sample_config,
                distance_fn
            )
            new_node = self.extend_tree(
                nearest_node,
                sample_config,
                extend_fn,
                collision_fn,
            )
            if new_node is None:
                continue
            tree.append(new_node)
            if distance_fn(new_node.config,goal_config)<self.goal_tolerance:
                goal_node = TreeNode(goal_config,parent=new_node)
                return goal_node.retrace_path()

            if i % 100 == 0:
                print(f"RRT iteration {i}, tree size: {len(tree)}")
        print("RRT failed: reached max interations")
        return None

    def find_nearst_node(self,tree,target_config,distance_fn):
        nearest_node=tree[0]
        nearest_distance=distance_fn(nearest_node.config,target_config)

        for node in tree[1:]:
            distance = distance_fn(node.config,target_config)

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_node = node
        return nearest_node

    def extend_tree(
            self,
            nearest_node,
            target_config,
            extend_fn,
            collision_fn,
    ):
        last_valid_node = nearest_node
        for config in extend_fn(nearest_node.config,target_config):
            if collision_fn(config):
                break
            last_valid_node = TreeNode(config,parent=last_valid_node)

        if last_valid_node is nearest_node:
            return None

        return last_valid_node


