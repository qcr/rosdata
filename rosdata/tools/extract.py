#!/usr/bin/env python


###############
### MODULES ###
###############

import yaml
import pathlib

import rosbag

from rosdata.core import ROSBagExtractor
from rosdata.utils import yes_or_no, rmdir


###################
### ENTRY POINT ###
###################


def extract_main(args):
    """Extracts the data from a ROSBag to a specified directory given a extraction config.

    Args:
        args: the command line arguments
    """
    
    # Get arguments
    bag_path = pathlib.Path(args.rosbag)
    root_output_dir = pathlib.Path(args.root_output_dir)
    extraction_config_path = pathlib.Path(args.extraction_config)


    # Check to see if root output directory exists - ask if wish to remove it
    if root_output_dir.exists():
        if yes_or_no("\nData already exists at %s. Would you like to overwrite existing data"%(root_output_dir)):
            rmdir(root_output_dir)
        else:
            print("\tUsing existing data where possible")
    print("")

    # create directory
    root_output_dir.mkdir(parents=True, exist_ok=True)
    

    # Open the ROS bag file
    print("Opening ROS Bag")
    bag = rosbag.Bag(bag_path, 'r')

    # Load the config YAML
    print("Loading Extraction Config")
    with open(extraction_config_path) as file:
        extraction_config = yaml.load(file, Loader=yaml.FullLoader)

    # Initialise rosbag extractor object and extract data
    print("Running Data Extraction\n")
    extractor = ROSBagExtractor(bag, extraction_config, root_output_dir)
    extractor.extract_data()


#################
### FUNCTIONS ###
#################