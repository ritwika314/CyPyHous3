"""
Path Planning Sample Code with RRT*

Author: AtsushiSakai(@Atsushi_twi)

Modifications for use in CyPhyHouse made by Amelia Gosse (gossea)
"""

# TODO: revisit documentation.
import copy
import math
import random
from typing import Union

import numpy as np

from src.motion.planner import Planner
from src.motion.pos import Pos, Node, to_node


class RRT(Planner):
    """
    Class for RRT* Planning
    """

    def __init__(self, rand_area: list = None, expand_dis: float = 0.1, goal_sample_rate: int = 15,
                 max_iter: int = 500):
        super(RRT, self).__init__()
        if rand_area is None:
            rand_area = [-12.5, 12.5]
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter

    def find_path(self, start: Pos, end: Pos, obstacle_list: Union[list, None] = None,
                  search_until_max_iter: bool = False) -> \
            Union[list, None]:
        """
        RRT* Path Planning
        search_until_max_iter: Search until max iteration for path improving or not
        """
        if obstacle_list is None:
            obstacle_list = []
        start = to_node(start)
        end = to_node(end)
        node_list = [start]
        for i in range(self.max_iter):
            rnd = self.get_random_point(end)
            nind = get_nearest_list_index(node_list, rnd)

            new_node = self.steer(node_list, rnd, nind)

            if self.__collision_check(new_node, obstacle_list):
                nearinds = find_near_nodes(node_list, new_node)
                new_node = self.choose_parent(node_list, obstacle_list, new_node, nearinds)
                node_list.append(new_node)
                self.rewire(node_list, obstacle_list, new_node, nearinds)

            # generate course
            if not search_until_max_iter:
                last_index = self.get_best_last_index(node_list, end)
                if last_index:
                    path = gen_final_course(node_list, start, end, last_index)
                    path = path[::2]
                    return path[::-1]

        print("Reached max iteration")

        last_index = self.get_best_last_index(node_list, end)
        if last_index:
            path = gen_final_course(node_list, start, end, last_index)
            path = path[::2]
            return path[::-1]

        return None

    def choose_parent(self, node_list: list, obstacle_list: list, new_node: Node, nearinds: list) -> Node:
        """
        choose the parent for each node.
        :param node_list:
        :param obstacle_list:
        :param new_node:
        :param nearinds:
        :return:
        """

        if not nearinds:
            return new_node

        dlist = []
        for i in nearinds:
            dx = new_node.x - node_list[i].x
            dy = new_node.y - node_list[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(obstacle_list, node_list[i], theta, d):
                dlist.append(node_list[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return new_node

        new_node.cost = mincost
        new_node.parent = minind

        return new_node

    def steer(self, node_list: list, rnd: list, nind: int) -> Node:
        """
        steer vehicle.
        :param node_list:
        :param rnd:
        :param nind:
        :return:
        """

        # expand tree
        nearest_node = node_list[nind]
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node = Node(rnd[0], rnd[1], 0)
        current_distance = math.sqrt(
            (rnd[1] - nearest_node.y) ** 2 + (rnd[0] - nearest_node.x) ** 2)
        # Find a point within expand_dis of nind, and closest to rnd
        if current_distance <= self.expand_dis:
            pass
        else:
            new_node.x = nearest_node.x + self.expand_dis * math.cos(theta)
            new_node.y = nearest_node.y + self.expand_dis * math.sin(theta)
        new_node.cost = float("inf")
        new_node.parent = None
        return new_node

    def get_random_point(self, end: Pos) -> list:
        """
        function to get a random point near the end
        :param end:
        :return:
        """

        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(self.min_rand, self.max_rand),
                   random.uniform(self.min_rand, self.max_rand), 0]
        else:  # goal point sampling
            rnd = [end.x, end.y, 0]

        return rnd

    def get_best_last_index(self, node_list: list, end: Node) -> Union[int, None]:
        """
        function to get best last index.
        :param node_list:
        :param end:
        :return:
        """

        disglist = [calc_dist_to_goal(end,
                                      node.x, node.y) for node in node_list]
        goal_inds = [disglist.index(i) for i in disglist if i <= self.expand_dis]

        if not goal_inds:
            return None

        mincost = min([node_list[i].cost for i in goal_inds])
        for i in goal_inds:
            if node_list[i].cost == mincost:
                return i

        return None

    def rewire(self, node_list: list, obstacle_list: list, new_node: Node, nearinds: list) -> None:
        """
        rewiring function.
        :param node_list:
        :param obstacle_list:
        :param new_node:
        :param nearinds:
        :return:
        """
        nnode = len(node_list)
        for i in nearinds:
            near_node = node_list[i]

            dx = new_node.x - near_node.x
            dy = new_node.y - near_node.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = new_node.cost + d

            if near_node.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(obstacle_list, near_node, theta, d):
                    near_node.parent = nnode - 1
                    near_node.cost = scost

    def check_collision_extend(self, obstacle_list: list, near_node: Node, theta: float, d: float):
        """
        extended check collision
        :param obstacle_list:
        :param near_node:
        :param theta:
        :param d:
        :return:
        """
        tmp_node = copy.deepcopy(near_node)

        for i in range(int(d / self.expand_dis)):
            tmp_node.x += self.expand_dis * math.cos(theta)
            tmp_node.y += self.expand_dis * math.sin(theta)
            if not self.__collision_check(tmp_node, obstacle_list):
                return False
        return True

    def __collision_check(self, node: Node, obstacle_list: list) -> bool:
        """
        check collision callback
        :param node:
        :param obstacle_list:
        :return:
        """

        for obs in obstacle_list:
            try:
                dx = obs.x - node.x
                dy = obs.y - node.y
                d = dx * dx + dy * dy
                if d <= obs.radius ** 2:
                    return False  # collision
            except AttributeError:
                print("obstacle might not be correctly formatted")

        return True  # safe


def get_nearest_list_index(node_list: list, rnd: list) -> int:
    """
    function for returning the list index of the nearest point
    :param node_list:
    :param rnd:
    :return:
    """
    dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
    minind = dlist.index(min(dlist))
    return minind


def find_near_nodes(node_list: list, new_node: Node) -> list:
    """
    TODO: add documentation for find_near_nodes
    :param node_list:
    :param new_node:
    :return:
    """
    nnode = len(node_list)
    r = 50.0 * math.sqrt((math.log(nnode) / nnode))
    dlist = [(node.x - new_node.x) ** 2 +
             (node.y - new_node.y) ** 2 for node in node_list]
    nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
    return nearinds


def gen_final_course(node_list: list, start: Node, end: Node, goal_ind: int) -> list:
    """
    generate the final path
    :param node_list:
    :param start:
    :param end:
    :param goal_ind:
    :return:
    """
    path = [Pos(np.array([end.x, end.y, 0]))]
    while node_list[goal_ind].parent is not None:
        node = node_list[goal_ind]
        path.append(Pos(np.array([node.x, node.y, 0])))
        goal_ind = node.parent
    path.append(Pos(np.array([start.x, start.y, 0])))
    return path


def calc_dist_to_goal(end: Node, x: float, y: float) -> float:
    """
    calculate the distance to goal
    :param end:
    :param x:
    :param y:
    :return:
    """
    return np.linalg.norm([x - end.x, y - end.y])


'''
a = RRT()
p1 = Pos(np.array([0, 0, 0]))
p2 = Pos(np.array([1, 1, 0]))

c = a.find_path(p1, p2, [])
print(c)
'''
