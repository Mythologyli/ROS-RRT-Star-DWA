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
    def __init__(self,config):
        self.config = config
        pass

    def plan(self, x, goal, ob):
        """
        Dynamic Window Approach control
        """
        ##TODO

        return u, trajectory
