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
import os.path
import pickle

LINUX_STORAGE_PATH="/var/lib/ros-plug-and-play/"
INSTALLED_MODULES_FILE="installed_modules.pkl"
AVAILABLE_MODULES_FILE="available_modules"


def add_eth_module(module_name, ip_addr, position, frame):
    print "Adding eth module " + module_name
    return 0

def add_usb_module(module_name, mac_address, position, frame):
    print "Adding usb module " + module_name
    return 0

def get_default_args(module):
    print "Default arguments selected"
    return 0

def add_module_to_repo(parser):
    print "Adding module %s to online repository" % module
    return 0

def has_modules_list():
    return os.path.isfile(LINUX_STORAGE_PATH + INSTALLED_MODULES_FILE)

def has_available_list():
    return os.path.isfile(LINUX_STORAGE_PATH + AVAILABLE_MODULES_FILE)

def create_pickle_file(module, position, frame, type, ip_addr, mac_addr, driver):
    pickle_list = [module, position, frame, type, ip_addr, mac_addr, driver]
    pkl_file = open(LINUX_STORAGE_PATH + INSTALLED_MODULES_FILE, 'rb')
    pickle.dump(pickle_list, pkl_file)
    pkl_file.close()
    print "Created pickle file"
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
    parser.add_argument("--driver", dest="driver", help="Specify a driver to load, if no driver is specified the default settings will be used for the driver. if --auto is selected, this is ignored")
    parser.add_argument("--auto", dest="auto", help="If this parameter is specified, only the --module parameter will be read, and the default settings taken from the plug-and-play github page will be used for the driver name")
    parser.add_argument("--add", dest="add", help="If using this parameter, it will add the module to the online repository if it is not already there. This will make a pull request to a github page with the driver details and its arguments, preferably, you will set it up for default use by other users. This will allow them to add it using just the module name and --auto parameter")
 

    args = parser.parse_args()

    if args.auto:
        default_args = get_default_args(args.module) 
        parser.position = default_args.position
        parser.frame = default_args.frame
        parser.type = default_args.type
        parser.ip_addr = default_args.ip_addr
        parser.mac_addr = default_args.mac_addr
        parser.driver = default_args.driver 

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

    if not args.driver:
        print "The driver.py, driver.launch, or driver executable filename is not known. Please pass the filename in as a parameter in one of those types"
        return 0

    if args.type == "eth" and not args.ip_addr:
        print "Ethernet module selected, but no IP address of ethernet device is known. Please find out the IP address of your device"
        return 0

    if args.type == "usb" and not args.mac-addr:
        print "USB module selected, but no mac address of device is known. Please find out the mac address of your device"
        return 0

    if args.type == "usb":
        add_usb_module(args.module, args.mac_addr, args.position, args.frame, args.driver)

    if args.type == "eth":
        add_eth_module(args.module, args.ip_addr, args.position, args.frame, args.driver)

    if args.add:
        uinput = input("Do you wish to make a pull request for your module parameters? This will allow other users to use the --auto option, so they can easily use this module on their robot. [y\n]?")
        if uinput == 'y':
            add_module_to_repo(parser)
        if uinput == 'n':
            print "Thanks!"

#    args = sys.argv[1:]
#    install(sys.argv[1:])
    #except rospy.ROSInterruptException:
    #    pass
