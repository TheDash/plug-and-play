#!/usr/bin/env python

""" the install command will:

        - add the module to the "installed" modules database
        - ping the ip address given if it is an ip device to confirm its existence (you must have the device connected to install it to the robot)
        - prompt for x,y,z positioning and the frame the offset is relative to (default: base_link)
        - add a launch file to spawn the node on boot with correct parameters. The launch file launches the ROS driver and the static transform publisher

    Usage:

        For ethernet devices:

            rosrun plug-and-play install [module_name] usb [ip_addr] [x, y, z]

        For USB devices:

            rosrun plug-and-play install [module_name] usb [mac_address] [x, y, z]

"""


import rospy
import argparse


def install(args):


if __name__ == '__main__':
    try:

        args = sys.argv[1:]


        install(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
