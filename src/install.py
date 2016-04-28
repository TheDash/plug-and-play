#!/usr/bin/env python

""" the install command will:

        - add the module to the "installed" modules database
        - ping the ip address given if it is an ip device to confirm its existence (you must have the device connected to install it to the robot)
        - prompt for x,y,z positioning and the frame the offset is relative to (default: base_link)
        - add a launch file to spawn the node on boot with correct parameters. The launch file launches the ROS driver and the static transform publisher

"""


import rospy
