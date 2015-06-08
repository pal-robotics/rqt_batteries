import rospy

from rqt_robot_dashboard.dashboard import Dashboard

from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QMessageBox

from std_msgs.msg import Float32, Bool
from .wrap_battery import WrappedBattery


class SurveillanceDashboard(Dashboard):
    """
    Dashboard for Batteries

    :param context: the plugin context
    :type context: qt_gui.plugin.Plugin
    """
    def setup(self, context):
        self.name = 'Batteries Dashboard'
        self.max_icon_size = QSize(50, 30)

        self._last_dashboard_message_time = rospy.Time.now()
        self._batteries = []

        NAMESPACE = '/batteries_dashboard'
        if rospy.has_param(NAMESPACE):
            # rosparam set /batteries_dashboard/batteries "{'battery1': {'percentage_topic': '/percentage1', 'charging_topic': '/charging1', 'tooltip_name': 'BATT1'}}"
            # rosparam set /batteries_dashboard/batteries "{'battery2': {'percentage_topic': '/percentage2', 'charging_topic': '/charging2', 'tooltip_name': 'BATT2'}}"
            self._batteries_dict = rospy.get_param(NAMESPACE + '/batteries')
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

        for battery in self._batteries_dict.keys():
            batt_dict_name = battery
            percentage_topic = self._batteries_dict[battery].get('percentage_topic', None)
            charging_topic = self._batteries_dict[battery].get('charging_topic', None)
            tooltip_name = self._batteries_dict[battery].get('tooltip_name', None)
            rospy.loginfo("Battery: " + str(batt_dict_name) + " has percentage topic: " +
                str(percentage_topic)  +  " and charging topic: " + str(charging_topic) +
                          " and has tooltip name: " + str(tooltip_name))

            self._batteries_dict[battery].update({'current_percentage': 0.0})
            self._batteries_dict[battery].update({'percentage_sub': rospy.Subscriber(percentage_topic,
                                                                            Float32,
                                                                            self.dashboard_callback,
                                                                            callback_args=batt_dict_name,
                                                                            queue_size=1)})

            self._batteries_dict[battery].update({'charging_status': False})
            if charging_topic:
                self._batteries_dict[battery].update({'charging_topic': rospy.Subscriber(charging_topic,
                                                                                Bool,
                                                                                self.dashboard_callback,
                                                                                callback_args=batt_dict_name,
                                                                                queue_size=1)})

            # Setup the widget
            self._batteries.append(WrappedBattery(self.context, name=tooltip_name))


    def get_widgets(self):
        return [self._batteries]

    def dashboard_callback(self, msg, cb_args):
        """
        callback to process messages

        :param msg:
        :type msg: Float32 or Bool
        :param cb_args:
        :type cb_args: str
        """
        # The type makes us know if the callback was fired up
        # from the percentage subscriber or the charging status topic
        if type(msg) == Bool:
            # The cb_args has the name of the battery in the dictionary
            self._batteries_dict[cb_args].update({'charging_status': msg.data})

        if type(msg) == Float32:
            self._batteries_dict[cb_args].update({'current_percentage': msg.data})


        if (rospy.Time.now() - self._last_dashboard_message_time) < rospy.Duration(1.0): # Let's throttle to 1hz
            return
        self._last_dashboard_message_time = rospy.Time.now()

        # Update all widgets
        for battery in self._batteries_dict.keys():
            # TODO: UPDATE FOR EACH, AND FIGURE OUT THE CHARGING THINGY
        self._batteries[0].set_power_state_perc(self._last_batteries_values[0], False)

        rospy.loginfo("Updated battery values with: " + str(self._last_batteries_values) )


    def shutdown_dashboard(self):
        self._battery_summit_topic_sub.unregister()
        self._battery_multimedia_topic_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # self._console.save_settings(plugin_settings, instance_settings)
        # self._monitor.save_settings(plugin_settings, instance_settings)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # self._console.restore_settings(plugin_settings, instance_settings)
        # self._monitor.restore_settings(plugin_settings, instance_settings)
        pass
