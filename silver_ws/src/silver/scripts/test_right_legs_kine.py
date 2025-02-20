#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from leg_math import LegMath

import numpy as np

leg = LegMath()
p1 = leg.inv_kine([0.3, 0.3, -0.3], True)

print("p1: ", p1)
