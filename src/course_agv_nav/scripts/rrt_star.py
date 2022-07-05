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


class RRTStar:
    def __init__(self,
                 plan_ox: List[float],
                 plan_oy: List[float],
                 plan_grid_size: float,
                 plan_robot_radius: float):
        self.plan_ox = plan_ox
        self.plan_oy = plan_oy
        self.plan_grid_size = plan_grid_size
        self.plan_robot_radius = plan_robot_radius

        print(f"{plan_ox} {plan_oy} {plan_grid_size} {plan_robot_radius}")

        self.min_x = -10.0
        self.max_x = 10.0
        self.min_y = -10.0
        self.max_y = 10.0

        self.step = 0.5
        self.max_iteration_times = 10000
        self.area_radius = 1

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
            distance = RRTStar.distance(node, exist_node)
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

    def find_neighbors(self, center_node: Node) -> List[Node]:
        neighbors = []
        for node in self.nodes:
            if RRTStar.distance(node, center_node) <= self.area_radius:
                neighbors.append(node)
        return neighbors

    def distance_from_start(self, cur_node: Node):
        total_distance = 0
        while cur_node.parent is not None:
            total_distance += RRTStar.distance(cur_node, cur_node.parent)
            cur_node = cur_node.parent
        return total_distance

    def plan(self,
             plan_sx: float,
             plan_sy: float,
             plan_gx: float,
             plan_gy: float) -> Tuple[List[float], List[float]]:

        print(f"{plan_sx} {plan_sy} {plan_gx} {plan_gy}")

        start_node = Node(plan_sx, plan_sy)
        goal_node = Node(plan_gx, plan_gy)

        self.nodes.append(start_node)

        success = False
        for i in range(self.max_iteration_times):
            print('.')

            random_node = self.sample()
            nearest_node = self.nearest(random_node)

            new_node = self.steer(nearest_node, random_node)

            if self.is_route_no_collision(nearest_node, new_node):
                neighbors = self.find_neighbors(new_node)
                min_distance = float('inf')
                best_neighbor = neighbors[0]
                for neighbor_node in neighbors:
                    new_node.set_parent(neighbor_node)
                    distance = self.distance_from_start(new_node)
                    if distance < min_distance:
                        min_distance = distance
                        best_neighbor = neighbor_node
                new_node.set_parent(best_neighbor)
                self.nodes.append(new_node)
                neighbors.remove(best_neighbor)

                for neighbor_node in neighbors:
                    if neighbor_node.parent is not None:
                        cur_parent = neighbor_node.parent
                        cur_distance = self.distance_from_start(neighbor_node)
                        neighbor_node.set_parent(new_node)
                        new_distance = self.distance_from_start(neighbor_node)
                        if cur_distance <= new_distance:
                            neighbor_node.set_parent(cur_parent)

                if RRTStar.distance(new_node, goal_node) < self.plan_robot_radius:
                    goal_node.set_parent(new_node)
                    self.nodes.append(goal_node)
                    success = True
                    print(f"Iteration time: {i}")
                    break

        if success:
            path: List[Node] = []
            node = self.nodes[-1]
            while node.parent is not None:
                path.append(node)
                node = node.parent
            path.append(self.nodes[0])

            path.reverse()

            return ([node.x for node in path], [node.y for node in path])
        else:
            return ([], [])
