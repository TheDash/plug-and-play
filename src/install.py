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

def add_eth_module(module_name, ip_addr, position, frame):
    print "Adding eth module " + module_name
    return 0

def add_usb_module(module_name, mac_address, position, frame):
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
    parser.add_argument("--list-available", dest="list", help="List all of the available modules to install, e.g to use as a module name")
     

    args = parser.parse_args()

    if not args.module:
        print "The argument --module is not set. Please pick a module to install"
        return 0

    if not args.position:
        print "The argument --position is not set. Please set the position [x, y, z]"    
        return 0
   
    if not args.frame:
        print "The argument --frame is not set. Please pick a relative frame to set for the static transform"
        return 0   

    if not args.type:
        print "The type of the module to install is not set. Please pick a type"
        return 0

    if args.type == "eth" && not args.ip_addr:
        print "Ethernet module selected, but no IP address of ethernet device is known. Please find out the IP address of your device"
        return 0

    if args.type == "usb" && not args.mac-addr:
        print "USB module selected, but no mac address of device is known. Please find out the mac address of your device"
        return 0

    if args.type == "usb":
        add_usb_module(args.module, args.mac_addr, args.position, args.frame)
        return 0

    if args.type =="eth":
        add_eth_module(args.module, args.ip_addr, args.position, args.frame)
        return 0

#    args = sys.argv[1:]
#    install(sys.argv[1:])
    #except rospy.ROSInterruptException:
    #    pass
