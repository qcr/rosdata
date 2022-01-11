#!/usr/bin/env python3

### IMPORT MODULES ###
# import sys
import csv
from numpy.lib.npyio import save
import yaml
import rosbag
import pathlib
import numpy as np
import matplotlib.pyplot as plt

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


def visualise_data(csv_file : pathlib.Path, save_file : pathlib.Path = None):
    # variables
    pose_data = []

    # Open and read in csv file
    with open(str(csv_file), newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for idx, row in enumerate(reader):
            if idx == 0:
                # first line is header, use to get start position of pose data
                # assume order is [..., "pos_x", "pos_y", "pos_z", "quat_x", "quat_y", "quat_z", "quat_w", ...]
                start_pose_idx = row.index("pos_x")
            else:
                if not 'None' in row[start_pose_idx:start_pose_idx+7]:
                    pose_data.append(row[start_pose_idx:start_pose_idx+7])

    # Convert to numpy array
    pose_data = np.array(pose_data, dtype=np.float)

    # Visualise Data
    # fig, (ax1, ax2) = plt.subplots(ncols=2)
    ax1 = plt.subplot(121)
    ax2 = plt.subplot(122, projection='3d')
    cval = np.arange(0, pose_data.shape[0]) # scatter plot colour value is based on order

    # 2D Plot
    ax1.plot(pose_data[:,0], pose_data[:,1], marker=None, color='#C0C0C0')
    ax1.scatter(pose_data[:,0], pose_data[:,1], marker='x', c=cval)
    ax1.set_aspect('equal', adjustable='box')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('2D Pose Plot')

    # 3D Plot
    ax2.plot(pose_data[:,0], pose_data[:,1], pose_data[:,2], marker=None, color='#C0C0C0')
    ax2.scatter(pose_data[:,0], pose_data[:,1], pose_data[:,2], marker='x', c=cval)
    # ax2.set_aspect('equal', adjustable='box')
    ax2.set_xlabel('X Position')
    ax2.set_ylabel('Y Position')
    ax2.set_zlabel('Z Position')
    ax2.set_title('3D Pose Plot')

    if save_file is not None:
        plt.savefig(str(save_file))
    plt.show()


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