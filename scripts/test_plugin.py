#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 6/8/15

@author: sampfeiffer

test_plugin.py contains...
"""
__author__ = 'sampfeiffer'

import rospy
from std_msgs.msg import Float32, Bool
import random


# rosparam set /batteries_dashboard/batteries "{'battery1': {'percentage_topic': '/percentage1', 'charging_topic': '/charging1', 'tooltip_name': 'BATT1'}}"
# rosparam set /batteries_dashboard/batteries "{'battery2': {'percentage_topic': '/percentage2', 'charging_topic': '/charging2', 'tooltip_name': 'BATT2'}}"

NUM_BATTERIES = 4

rospy.init_node('test_rqt_battery')
batteries_list = []
for i in range(NUM_BATTERIES):
    batteries_list.append({'battery'+str(i) : {'percentage_topic': '/percentage'+str(i), 'charging_topic': '/charging'+str(i), 'tooltip_name': 'BATT'+str(i)}})

rospy.loginfo("Setting params.")
rospy.set_param('/batteries_dashboard/batteries', batteries_list)
own_params = rospy.get_param('/batteries_dashboard')
rospy.loginfo("We set the params:\n" + str(own_params))

for battery_list_elem in batteries_list:
    for battery in battery_list_elem.keys():
        battery_list_elem[battery].update({'percentage_pub': rospy.Publisher(battery_list_elem[battery]['percentage_topic'], Float32)})
        battery_list_elem[battery].update({'charging_pub': rospy.Publisher(battery_list_elem[battery]['charging_topic'], Bool)})

print "batteries_list looks like:\n" + str(batteries_list)

battery_status = [i for i in range(1, 100, 10)]
battery_status.reverse()

hz = 50.0

# Publish battery levels going down simulating a robot
while not rospy.is_shutdown():
    for status in battery_status:
        rospy.loginfo("Publishing status: " + str(status))
        for i in range(int(hz)): # One second with every status at hz
            for battery_list_elem in batteries_list:
                for battery in battery_list_elem.keys():
                    battery_list_elem[battery]['percentage_pub'].publish(status)
                    battery_list_elem[battery]['charging_pub'].publish(random.choice([True, False]))
            rospy.sleep(1.0/hz)
