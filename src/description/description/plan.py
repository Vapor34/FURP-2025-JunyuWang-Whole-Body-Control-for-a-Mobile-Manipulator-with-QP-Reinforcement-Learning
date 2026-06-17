import os
import time
import threading
import numpy as np
from pathlib import Path

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState