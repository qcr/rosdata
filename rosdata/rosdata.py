#!/usr/bin/env python3

### IMPORT MODULES ###
import sys
import yaml
import rosbag
import pathlib
from .rosbag_extractor import ROSBagExtractor


### FUNCTIONS ###
def extract_rosbag_data(bag_path : pathlib.Path, extraction_config_path : pathlib.Path, root_output_dir : pathlib.Path):
    """Extracts the data from a ROSBag to a specified directory given a extraction config.

    Args:
        bag_path (pathlib.Path): the path to the ROSBag
        extraction_config_path (pathlib.Path): the path to the extraction config file
        root_output_dir (pathlib.Path): the root output directory for the extracted data
    """
    
    # Check to see if root output directory exists - ask if wish to remove it
    if root_output_dir.exists():
        if yes_or_no("\nData already exists at %s. Would you like to overwrite existing data"%(root_output_dir)):
            rmdir(root_output_dir)
        else:
            print("\tUsing existing data where possible")
        #     print("Exiting program.")
        #     sys.exit(0)
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




### HELPER FUNCTIONS ###
def yes_or_no(question):
    """Yes or no question from gist
        https://gist.github.com/garrettdreyfus/8153571

    Args:
        question (str): Question to ask

    Returns:
        bool: Returns true or false to question
    """
    reply = str(input(question+' (y/n): ')).lower().strip() #Just look at first char
    if len(reply):
        if reply[0] == 'y':
            return True
        if reply[0] == 'n':
            return False
        else:
            return yes_or_no("Uhhhh... please enter ") #Invalid input
    else:   
        return yes_or_no("Uhhhh... please enter ") #Zero length


def rmdir(dir : pathlib.Path):
    """Removes a directory and any sub-directories

    Args:
        dir (Path): directory to be removed
    """
    for item in dir.iterdir():
        if item.is_dir():
            rmdir(item)
        else:
            item.unlink()
    dir.rmdir()