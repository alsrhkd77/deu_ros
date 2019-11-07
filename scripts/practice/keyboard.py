#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

key_mapping = {'w': [0, 1], 'x': [0, -1], 'a': [-1, 0], 'd': [1, 0], 's': [0, 0]}
g_last_twist = None

