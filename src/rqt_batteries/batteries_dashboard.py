#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# rqt_batteries: batteries_dashboard.py
#
# Copyright (c) 2015 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
#   * Sammy Pfeiffer

import rospy

from rqt_robot_dashboard.dashboard import Dashboard

from python_qt_binding.QtCore import QSize

from std_msgs.msg import Float32, Bool
from .wrap_battery import WrappedBattery


class BatteriesDashboard(Dashboard):
    """
    Dashboard for Batteries

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    def setup(self, context):
        self.name = 'Batteries Dashboard'
        self.max_icon_size = QSize(50, 30)

        self._last_dashboard_message_time = rospy.Time.now()
        self._widget_initialized = False

        NAMESPACE = '/batteries_dashboard'
        if rospy.has_param(NAMESPACE):
            # rosparam set /batteries_dashboard/batteries "{'battery1': {'percentage_topic': '/percentage1', 'charging_topic': '/charging1', 'tooltip_name': 'BATT1'}}"
            # rosparam set /batteries_dashboard/batteries "{'battery2': {'percentage_topic': '/percentage2', 'charging_topic': '/charging2', 'tooltip_name': 'BATT2'}}"
            if rospy.has_param(NAMESPACE + '/batteries'):
                self._batteries_list = rospy.get_param(NAMESPACE + '/batteries')
            else:
                rospy.logwarn("No batteries to monitor found in param server under " + NAMESPACE + "/batteries")
            # Looks like:
            # {'battery1': {'battery_name': 'BATT1',
            #  'charging_topic': '/charging1',
            #  'percentage_topic': '/percentage1'},
            # 'battery2': {'battery_name': 'BATT2',
            #  'charging_topic': '/charging2',
            #  'percentage_topic': '/percentage2'}}

        else:
            rospy.logerr("You must set /batteries_dashboard parameters to use this plugin. e.g.:\n" +
                         "rosparam set /batteries_dashboard/batteries \"{'battery1': {'percentage_topic': '/percentage1', 'charging_topic': '/charging1', 'battery_name': 'BATT1'}}\"")
            exit(-1)

        for battery_elem in self._batteries_list: # list of all batteries to monitor
            for battery_name in battery_elem.keys(): # there is only one key which is the name
                percentage_topic = battery_elem[battery_name].get('percentage_topic', None)
                charging_topic = battery_elem[battery_name].get('charging_topic', None)
                tooltip_name = battery_elem[battery_name].get('tooltip_name', None)
                rospy.loginfo("Battery: " + str(battery_name) + " has percentage topic: " +
                    str(percentage_topic)  +  " and charging topic: " + str(charging_topic) +
                              " and has tooltip name: " + str(tooltip_name))

                battery_elem[battery_name].update({'current_percentage': 0.0})
                battery_elem[battery_name].update({'percentage_sub': rospy.Subscriber(percentage_topic,
                                                                                Float32,
                                                                                self.dashboard_callback,
                                                                                callback_args={'battery': battery_name},
                                                                                queue_size=1)})

                battery_elem[battery_name].update({'charging_status': False})
                if charging_topic:
                    battery_elem[battery_name].update({'charging_sub': rospy.Subscriber(charging_topic,
                                                                                    Bool,
                                                                                    self.dashboard_callback,
                                                                                    callback_args={'battery': battery_name},
                                                                                    queue_size=1)})

                # Setup the widget
                battery_elem[battery_name].update({'battery_widget': WrappedBattery(self.context, name=tooltip_name)})

        self._widget_initialized = True


    def get_widgets(self):
        widgets_list = []
        for battery_elem in self._batteries_list:
            for battery_name in battery_elem.keys():
                widgets_list.append([battery_elem[battery_name]['battery_widget']])
        return widgets_list

    def dashboard_callback(self, msg, cb_args):
        """
        callback to process messages

        :param msg:
        :type msg: Float32 or Bool
        :param cb_args:
        :type cb_args: dictionary
        """
        if not self._widget_initialized:
            return

        if cb_args.has_key('battery'):
            battery_name = cb_args['battery']
            if type(msg) == Bool:
                for battery_elem in self._batteries_list:
                    if battery_elem.has_key(battery_name):
                        battery_elem[battery_name].update({'charging_status': msg.data})

            if type(msg) == Float32:
                for battery_elem in self._batteries_list:
                    if battery_elem.has_key(battery_name):
                        battery_elem[battery_name].update({'current_percentage': msg.data})


        # Throttling to 1Hz the update of the widget whatever the rate of the topics is, maybe make it configurable?
        if (rospy.Time.now() - self._last_dashboard_message_time) < rospy.Duration(1.0):
            return
        self._last_dashboard_message_time = rospy.Time.now()

        # Update all widgets
        for battery_elem in self._batteries_list:
            for battery_name in battery_elem.keys():
                battery_elem[battery_name]['battery_widget'].set_power_state_perc(
                     battery_elem[battery_name]['current_percentage'], battery_elem[battery_name]['charging_status'])
                rospy.logdebug("Updated " + str(battery_name) + " with "
                              + str(round(battery_elem[battery_name]['current_percentage']))
                              + "% battery and is "
                              + ("charging." if battery_elem[battery_name].get('charging_status') else "not charging."))


    def shutdown_dashboard(self):
        for battery_elem in self._batteries_list:
            for battery_name in battery_elem.keys():
                if battery_elem[battery_name]['percentage_sub']:
                    battery_elem[battery_name]['percentage_sub'].unregister()
                if battery_elem[battery_name]['charging_sub']:
                    battery_elem[battery_name]['charging_sub'].unregister()
