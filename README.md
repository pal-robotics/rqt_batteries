rqt plugin dashboard for showing battery status.

![Screenshot of the batteries plugin working](https://raw.githubusercontent.com/pal-robotics/rqt_batteries/master/images/rqt_batteries_ss.png)

You must set in the parameter server
which topics to listen to and it will
do the magic for you.

The necessary parameters are:

    Topic(s) with Float32 data showing the percentage of battery left.
    Topic(s) with Bool data showing if charging the battery.

Example parameter server configuration can be found in the
 config folder. Looks like:

    batteries_dashboard:
        batteries:
            # Must be an array (it's parsed like that) so the "- " must be added. 
            # This is the name of the battery internally in the node, won't be shown
            - battery1:
                # topic of type std_msgs/Float32 publishing a number between 0.0 and 100.0 representing the battery left
                percentage_topic: /percentage1
                # topic of type std_msgs/Bool representing if the battery is charging (True) or not
                charging_topic: /charging1
                # name of the battery for the tooltip when mousing over
                tooltip_name: BATT1
            - battery2:
                percentage_topic: /percentage2
                charging_topic: /charging2
                tooltip_name: BATT2