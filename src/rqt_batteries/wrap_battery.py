

from python_qt_binding.QtCore import QSize
from rqt_robot_dashboard.widgets import BatteryDashWidget
import rospkg
import os

class WrappedBattery(BatteryDashWidget):
    """
    Dashboard widget to display batteries state.
    """
    #TODO When nonbutton Dashboard objects are available rebase this widget
    def __init__(self, context, name):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        icon_names = ['ic-battery-0.svg', 'ic-battery-20.svg', 'ic-battery-40.svg',
                 'ic-battery-60-green.svg', 'ic-battery-80-green.svg', 'ic-battery-100-green.svg']
        icons = []
        for icon in icon_names:
            icons.append([icon])

        charge_icon_names = ['ic-battery-charge-0.svg', 'ic-battery-charge-20.svg', 'ic-battery-charge-40.svg',
                        'ic-battery-charge-60-green.svg', 'ic-battery-charge-80-green.svg', 'ic-battery-charge-100-green.svg']
        charge_icons = []
        for charge_icon in charge_icon_names:
            charge_icons.append([charge_icon])

        icon_paths = []
        icon_paths.append(['rqt_batteries', 'images'])
        self._wrapped_battery_name = name
        super(WrappedBattery, self).__init__(name=name,
                                                  icons=icons, charge_icons=charge_icons,
                                                  icon_paths=icon_paths,
                                                  suppress_overlays=True)


        self.update_perc(0)

    def set_power_state_perc(self, percentage, charging):
        """
        """
        print self._wrapped_battery_name + " updating with values: " + str(percentage) + "% and charging: " + str(charging)
        self.update_perc(percentage)
        self.update_time(percentage) # + remaining
        self.set_charging(charging)


