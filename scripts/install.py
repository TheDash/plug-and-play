#!/usr/bin/env python
# Author: Devon Ash, April 2016.

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

    Example:
        rosrun install.py --module velodynehdle32 --position "[1, 1, 1]" --frame base_link --driver driver.launch

        

"""

# IMPORTS
import argparse
import os.path
import pickle
import sys
import pyudev

# CONSTANTS
LINUX_STORAGE_PATH="/var/lib/ros-plug-and-play/"
LINUX_MODULES_PATH="/var/lib/ros-plug-and-play/modules/"
INSTALLED_MODULES_FILE="installed_modules.pkl"
AVAILABLE_MODULES_FILE="available_modules"
LAUNCH_SUBDIR="/launch"
RULES_SUBDIR="/rules.d"

# Pickle file position naming
MODULE_NAME=0
POSITION=1
FRAME=2
TYPE=3
IP_ADDR=4
DRIVER=5

def add_module(module_name, position, frame, driver, ip_addr):
    if not is_installed(module_name):
        print "Adding module " + module_name
        create_module_directories(module_name)
        create_udev_rule(module_name)

        #add_module_to_database(module_name, position, frame, type, ip_addr, driver)
    return 0

def create_udev_rule(module_name):
    print "Starting udev rule creation... " + module_name
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    uinput = input("Please unplug and plug in your device you wish to use as a plug and play robotics module. Press Enter after you have done so. One module at a time.")    
    device = monitor.poll(timeout=3)
    if device:
        print('{0.action}: {0}'.format(device))
    if uinput == "":
        print "Enter detected. Creating udev rule"
        # Monitor syslog, udevadm monitor, for hints on new devices connected to comp. 
        
    return 0

def get_default_args(module):
    print "Default arguments selected"
    return 0

def add_module_to_repo(parser):
    print "Adding module %s to online repository" % module
    return 0

def create_module_directories(module_name):
    if os.path.exists(LINUX_STORAGE_PATH):
        if not os.path.exists(LINUX_MODULES_PATH):
            os.makedir(LINUX_MODULES_PATH)
        if os.path.exists(LINUX_MODULES_PATH + module_name):
            print "Module directory is already in the database, returning."
            return 0
        os.makedir(LINUX_MODULES_PATH + module_name)
        print "Creating directory " + LINUX_MODULES_PATH + module_name
        os.makedir(LINUX_MODULES_PATH + module_name + LAUNCH_SUBDIR)
        print "Creating directory " + LINUX_MODULES_PATH + module_name + LAUNCH_SUBDIR
        os.makedir(LINUX_MODULES_PATH + module_name + RULES_SUBDIR)
        print "Creating directory " + LINUX_MODULES_PATH + module_name + RULES_SUBDIR


def is_installed(module_name):
    print "Checking if " + module_name " is installed"
    if has_modules_list():
        pkl_file = open(LINUX_STORAGE_PATH + INSTALLED_MODULES_FILE, 'rb')
        modules = pickle.load(pkl_file)
        for module in modules:
            if module[MODULE_NAME] is module_name:
                print module_name + " is already installed"
                return True
            else:
                print module_name + " is not installed"
                return False

def has_modules_list():
    return os.path.isfile(LINUX_STORAGE_PATH + INSTALLED_MODULES_FILE)

def has_available_list():
    return os.path.isfile(LINUX_STORAGE_PATH + AVAILABLE_MODULES_FILE)

def add_module_to_database(module, position, frame, type, ip_addr, driver):
    pickle_list = [module, position, frame, type, ip_addr, driver]
    pkl_file = open(LINUX_STORAGE_PATH + INSTALLED_MODULES_FILE, 'rb')
    pickle.dump(pickle_list, pkl_file)
    pkl_file.close()
    print "Added module to database"
    return 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Installs a module to your robot to be used for plug-and-play")
    parser.add_argument("--module", dest="module", help="[REQUIRED] the name of the module. e.g velodynehdle32, kinect2, kinect1")
    parser.add_argument("--position", dest="position", help="[REQUIRED] the position of the module relative to the frame name -f, e.g [10, 12, 13]")
    parser.add_argument("--frame", dest="frame", help="[REQUIRED] the frame to which the static transform will be relative to for the position of the sensor")
    parser.add_argument("--ip_addr", dest="ip_addr", help="the ip address of the module if eth is the type")
    parser.add_argument("--list-available", dest="list", help="List all of the available modules to install, e.g to use as a module name")
    parser.add_argument("--driver", dest="driver", help="Specify a driver to load, if no driver is specified the default settings will be used for the driver. if --auto is selected, this is ignored")
    parser.add_argument("--auto", dest="auto", help="If this parameter is specified, only the --module parameter will be read, and the default settings taken from the plug-and-play github page will be used for the driver name")
    parser.add_argument("--add", dest="add", help="If using this parameter, it will add the module to the online repository if it is not already there. This will make a pull request to a github page with the driver details and its arguments, preferably, you will set it up for default use by other users. This will allow them to add it using just the module name and --auto parameter")

    args = parser.parse_args()

    if args.auto:
        default_args = get_default_args(args.module) 
        parser.position = default_args.position
        parser.frame = default_args.frame
        parser.ip_addr = default_args.ip_addr
        parser.driver = default_args.driver 

    if not args.module:
        print "The argument --module is not set. Please pick a module to install"
        sys.exit(0)

    if not args.position:
        print "The argument --position is not set. Please set the position [x, y, z]"    
        sys.exit(0)

    if not args.frame:
        print "The argument --frame is not set. Please pick a relative frame to set for the static transform"
        sys.exit(0)

    if not args.driver:
        print "The driver.py, driver.launch, or driver executable filename is not known. Please pass the filename in as a parameter in one of those types"
        sys.exit(0)

    add_module(args.module, args.position, args.frame, args.driver, args.ip_addr)

    if args.add:
        uinput = input("Do you wish to make a pull request for your module parameters? This will allow other users to use the --auto option, so they can easily use this module on their robot. [y\n]?")
        if uinput == 'y':
            add_module_to_repo(parser)
        if uinput == 'n':
            print "Thanks!"

