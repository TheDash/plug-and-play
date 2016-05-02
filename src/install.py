#!/usr/bin/env python

""" the install command will:

        - add the module to the "installed" modules database
        - ping the ip address given if it is an ip device to confirm its existence (you must have the device connected to install it to the robot)
        - prompt for x,y,z positioning and the frame the offset is relative to (default: base_link)
        - add a launch file to spawn the node on boot with correct parameters. The launch file launches the ROS driver and the static transform publisher

    Usage:

        For ethernet devices:

            rosrun plug-and-play install [module_name] eth [ip_addr] [x, y, z]

        For USB devices:

            rosrun plug-and-play install [module_name] usb [mac_address] [x, y, z]

"""


#import rospy
import argparse


#def install(args):

def add_eth_module(module_name, ip_addr, position):
    print "Adding eth module " + module_name
    return 0

def add_usb_module(module_name, mac_address, position):
    print "Adding usb module " + module_name
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Installs a module to your robot to be used for plug-and-play")
    parser.add_argument("--module", dest="module", help="[REQUIRED] the name of the module. e.g velodynehdle32, kinect2, kinect1")
    parser.add_argument("--position", dest="position", help="[REQUIRED] the position of the module relative to the frame name -f, e.g [10, 12, 13]")
    parser.add_argument("--frame", dest="frame", help="[REQUIRED] the frame to which the static transform will be relative to for the position of the sensor")
    parser.add_argument("--type", dest="type", help="[REQUIRED] the type of module to install, e.g eth for ethernet, usb for usb, bluetooth for bluetooth")
    parser.add_argument("--ip_addr", dest="ip_addr", help="the ip address of the module if eth is the type")
    parser.add_argument("--mac_addr", dest="mac_addr", help="the mac address of the device if using usb as the type")

    args = parser.parse_args()
    
#    args = sys.argv[1:]
#    install(sys.argv[1:])
    #except rospy.ROSInterruptException:
    #    pass
