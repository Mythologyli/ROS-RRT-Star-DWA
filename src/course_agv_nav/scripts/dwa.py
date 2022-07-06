#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from enum import Enum

import numpy as np


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.7  # [m/s]
        self.min_speed = -0.7  # [m/s]
        self.max_yawrate = 200.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.4  # [m/ss]
        self.max_dyawrate = 200.0 * math.pi / 180.0  # [rad/ss]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.v_reso = self.max_accel*self.dt/10.0  # [m/s]
        self.yawrate_reso = self.max_dyawrate*self.dt/10.0  # [rad/s]
        self.predict_time = 2  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 0.1
        self.obstacle_cost_gain = 1.0
        self.robot_type = RobotType.rectangle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.4  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.4  # [m] for collision check
        self.robot_length = 0.6  # [m] for collision check

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


class DWA:
    def __init__(self, config: Config):
        self.config = config
        pass

    def plan(self, x, goal, ob: np.ndarray):
        """
        Dynamic Window Approach control
        """
        point_x = x[0]
        point_y = x[1]
        point_yaw = x[2]
        point_v = x[3]
        point_w = x[4]
        goal_x = goal[0]
        goal_y = goal[1]

        v_min, v_max = self.get_v_limit(point_v)
        w_min, w_max = self.get_w_limit(point_w)
        min_cost = float('inf')
        best_u = (v_max, w_max)

        print(f"v_min: {v_min}, v_max: {v_max}, w_min: {w_min}, w_max: {w_max}, v_reso: {self.config.v_reso}, w_reso: {self.config.yawrate_reso}")

        for v in np.arange(v_min, v_max, self.config.v_reso):
            for w in np.arange(w_min, w_max, self.config.yawrate_reso):
                print('.')
                new_point_x, new_point_y, new_point_yaw = self.get_next_point(
                    point_x, point_y, point_yaw, v, w)
                cost = self.get_cost(
                    v, w, new_point_x, new_point_y, new_point_yaw, goal_x, goal_y, ob)
                if cost < min_cost:
                    min_cost = cost
                    best_u = (v, w)
        trajectory = None
        best_u = np.array(list(best_u))

        print('return')

        return best_u, trajectory

    def get_v_limit(self, pre_v):
        v_min = pre_v - self.config.max_accel * self.config.dt
        v_max = pre_v + self.config.max_accel * self.config.dt
        v_min = max(v_min, self.config.min_speed)
        v_max = min(v_max, self.config.max_speed)
        return v_min, v_max

    def get_w_limit(self, pre_w):
        w_min = pre_w - self.config.max_dyawrate * self.config.dt
        w_max = pre_w + self.config.max_dyawrate * self.config.dt
        w_min = max(w_min, -self.config.max_yawrate)
        w_max = min(w_max, self.config.max_yawrate)
        return w_min, w_max

    def get_next_point(self, point_x, point_y, point_yaw, v, w):
        r = v / w
        syaw = math.sin(point_yaw)
        cyaw = math.cos(point_yaw)
        new_point_yaw = point_yaw + w * self.config.dt

        # calculate new position
        new_point_x = point_x - r * syaw + r * math.sin(new_point_yaw)
        new_point_y = point_y + r * cyaw - r * math.cos(new_point_yaw)

        return new_point_x, new_point_y, new_point_yaw

    def get_cost(self, v, w, new_point_x, new_point_y, new_point_yaw, goal_x, goal_y, ob: np.ndarray):
        # cost1 = self.config.to_goal_cost_gain * \
        #     math.atan2(goal_y - new_point_y, goal_x -
        #                new_point_x) - new_point_yaw
        cost1_angle = math.atan2(goal_y - new_point_y,
                                 goal_x - new_point_x) - new_point_yaw

        print(f"cost1_angle 1: {cost1_angle} new_point_yaw: {new_point_yaw} goal_x: {goal_x} goal_y: {goal_y} new_point_x: {new_point_x} new_point_y: {new_point_y}")

        cost1_angle = abs(math.atan2(
            math.sin(cost1_angle), math.cos(cost1_angle)))

        print(f"cost1_angle 2: {cost1_angle}")

        cost1 = self.config.to_goal_cost_gain * cost1_angle

        if ob is None:
            cost2 = 0.0
        else:
            dist = float('inf')
            for o in ob:
                cur_dist = math.hypot(o[0] - new_point_x, o[1] - new_point_y)
                if cur_dist < dist:
                    cur_dist = dist
            dist = cur_dist - self.config.robot_radius

            assert dist != float('inf')

            cost2 = self.config.obstacle_cost_gain / dist

        cost3 = self.config.speed_cost_gain * (self.config.max_speed - abs(v))

        print(
            f"cost1: {cost1} cost2: {cost2} cost3: {cost3}  cost: {abs(cost1) + abs(cost2) + abs(cost3)}")

        return abs(cost1) / (2.0 * math.pi) + \
            abs(cost2) * (20.0 * math.sqrt(2.0)) + \
            abs(cost3) / self.config.max_speed
