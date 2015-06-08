#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 6/8/15

@author: sampfeiffer

test_plugin.py contains...
"""
__author__ = 'sampfeiffer'

import rospy
from std_msgs.msg import Float32

SUMMIT_BAT = '/summit_xl_controller/battery'
MULTIMEDIA_BAT = '/multimedia/battery'

rospy.init_node('test_rqt_battery')
summit_pub = rospy.Publisher(SUMMIT_BAT, Float32)
multimedia_pub = rospy.Publisher(MULTIMEDIA_BAT, Float32)

battery_status = [i for i in range(1, 100, 10)]
battery_status.reverse()

hz = 50.0

# Publish battery levels going down simulating a robot
while not rospy.is_shutdown():
    for status in battery_status:
        rospy.loginfo("Publishing status: " + str(status))
        for i in range(int(hz)): # One second with every status at hz
            summit_pub.publish(status)
            multimedia_pub.publish(status)
            rospy.sleep(1.0/hz)
