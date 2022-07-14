from typing import Tuple, List
import random


class Node:
    """
    A node in the RRT tree.
    """

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent = None

    def set_parent(self, parent):
        self.parent = parent


class RRTSimplified:
    def __init__(self,
                 plan_ox: List[float],
                 plan_oy: List[float],
                 plan_grid_size: float,
                 plan_robot_radius: float):
        self.plan_ox = plan_ox
        self.plan_oy = plan_oy
        self.plan_grid_size = plan_grid_size
        self.plan_robot_radius = plan_robot_radius

        self.min_x = -10.0
        self.max_x = 10.0
        self.min_y = -10.0
        self.max_y = 10.0

        self.step = 0.5
        self.max_iteration_times = 10000

        self.nodes: List[Node] = []

    def sample(self) -> Node:
        return Node(random.uniform(self.min_x, self.max_x),
                    random.uniform(self.min_y, self.max_y))

    @staticmethod
    def distance(node1: Node, node2: Node) -> float:
        return ((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2) ** 0.5

    def nearest(self, node: Node) -> Node:
        """
        Find the nearest node to the given node.
        """
        min_distance = float('inf')
        min_node = None
        for exist_node in self.nodes:
            distance = RRTSimplified.distance(node, exist_node)
            if distance < min_distance:
                min_distance = distance
                min_node = exist_node
        return min_node

    def steer(self, from_node: Node, to_node: Node) -> Node:
        """
        Steer from the given node to the target node.
        """
        new_node = Node(from_node.x, from_node.y)
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = ((dx ** 2) + (dy ** 2)) ** 0.5
        new_node.x += self.step * dx / d
        new_node.y += self.step * dy / d

        return new_node

    def is_route_no_collision(self, start_node: Node, end_node: Node) -> bool:
        """
        Check whether the given route has no collision.
        """
        for ox, oy in zip(self.plan_ox, self.plan_oy):
            cross = (end_node.x - start_node.x) * (ox - start_node.x) + \
                (end_node.y - start_node.y) * (oy - start_node.y)
            if cross <= 0:
                distance = self.distance(start_node, Node(ox, oy))
            else:
                d2 = (end_node.x - start_node.x) ** 2 + \
                    (end_node.y - start_node.y) ** 2
                if cross >= d2:
                    distance = self.distance(end_node, Node(ox, oy))
                else:
                    r = cross / d2
                    px = start_node.x + (end_node.x - start_node.x) * r
                    py = start_node.y + (end_node.y - start_node.y) * r
                    distance = self.distance(Node(ox, py), Node(px, oy))

            if distance < self.plan_robot_radius:
                return False

        return True

    def plan(self,
             plan_sx: float,
             plan_sy: float,
             plan_gx: float,
             plan_gy: float) -> Tuple[List[float], List[float]]:

        start_node = Node(plan_sx, plan_sy)
        goal_node = Node(plan_gx, plan_gy)

        self.nodes.append(start_node)

        success = False
        for i in range(self.max_iteration_times):
            random_node = self.sample()
            nearest_node = self.nearest(random_node)

            new_node = self.steer(nearest_node, random_node)

            if self.is_route_no_collision(nearest_node, new_node):
                new_node.set_parent(nearest_node)
                self.nodes.append(new_node)

                if RRTSimplified.distance(new_node, goal_node) < self.plan_robot_radius:
                    goal_node.set_parent(new_node)
                    self.nodes.append(goal_node)
                    success = True
                    print(f"Iteration time: {i}")

                    # Simplify the path
                    node1 = self.nodes[-1]
                    if node1.parent is not None:
                        node2: Node = node1.parent
                    else:
                        break

                    while True:
                        if node2.parent is not None and self.is_route_no_collision(node1, node2.parent):
                            node1.set_parent(node2.parent)
                            node1: Node = node2.parent

                            if node1.parent is not None:
                                node2 = node1.parent
                            else:
                                break
                        else:
                            if node2.parent is not None:
                                node2 = node2.parent
                            else:
                                break

                    break

        if success:
            path: List[Node] = []
            node = self.nodes[-1]
            while node.parent is not None:
                path.append(node)
                node = node.parent
            path.append(self.nodes[0])

            path.reverse()

            # Simplify the path
            simplified_path: List[Node] = []
            simplified_path.append(path[0])

            i = 0
            while i < len(path) - 1:
                j = len(path) - 1
                while j != i:
                    if self.is_route_no_collision(path[i], path[j]):
                        simplified_path.append(path[j])
                        i = j - 1
                        break

                    j -= 1

                if (j == i):
                    simplified_path.append(path[i + 1])

                i += 1

            return ([node.x for node in simplified_path], [node.y for node in simplified_path])
        else:
            return ([], [])

    def test(self) -> Tuple[List[float], List[float]]:
                
        plan_sx = 9.303865722269596e-05
        plan_sy = -0.00016220068199821012
        plan_gx = -0.9932613372802734
        plan_gy = -2.0161428451538086

        random.seed(0)

        import time

        time_list = []
        path_distance_list = []
        number_list = []

        for i in range(10):
            # Calc time of self.plan()
            start_time = time.time()
            path = self.plan(plan_sx, plan_sy, plan_gx, plan_gy)
            end_time = time.time()
            time_list.append(end_time - start_time)

            # Calc total distance of path
            total_distance = 0
            for i in range(len(path[0]) - 1):
                total_distance += RRTSimplified.distance(Node(path[0][i], path[1][i]), Node(path[0][i + 1], path[1][i + 1]))

            path_distance_list.append(total_distance)
            number_list.append(len(path[0]))
        
        # Average
        average_time = sum(time_list) / len(time_list)
        average_path_distance = sum(path_distance_list) / len(path_distance_list)
        average_number = sum(number_list) / len(number_list)

        print("Test plan over!")
        print(f"Time: {average_time}")
        print(f"Total distance: {average_path_distance}")
        print(f"Path number: {average_number}")
