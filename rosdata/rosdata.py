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
from .rosbag_transformer import Status as RBTStatus


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


def visualise_pose_data(csv_file : pathlib.Path, save_file : pathlib.Path = None):
    # variables
    data = []

    # Open and read in csv file
    with open(str(csv_file), newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for idx, row in enumerate(reader):
            if idx == 0:
                # first line is header, use to get start position of pose data
                # assume order is [..., "pos_x", "pos_y", "pos_z", "quat_x", "quat_y", "quat_z", "quat_w", ...]
                try:
                    start_idx = row.index("pos_x")
                except ValueError:
                    print("Could not find the index for the \'pos_x\' data in the passed CSV file.")
                    return
            else:
                if not 'None' in row[start_idx:start_idx+7]:
                    data.append(row[start_idx:start_idx+7])

    # Convert to numpy array
    data = np.array(data, dtype=np.float)

    # Visualise Data
    ax1 = plt.subplot(121)
    ax2 = plt.subplot(122, projection='3d')
    cval = np.arange(0, data.shape[0]) # scatter plot colour value is based on order

    # 2D Plot
    ax1.plot(data[:,0], data[:,1], marker=None, color='#C0C0C0')
    ax1.scatter(data[:,0], data[:,1], marker='x', c=cval)
    ax1.set_aspect('equal', adjustable='box')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('2D Pose Plot')

    # 3D Plot
    ax2.plot(data[:,0], data[:,1], data[:,2], marker=None, color='#C0C0C0')
    ax2.scatter(data[:,0], data[:,1], data[:,2], marker='x', c=cval)
    # ax2.set_aspect('equal', adjustable='box')
    ax2.set_xlabel('X Position')
    ax2.set_ylabel('Y Position')
    ax2.set_zlabel('Z Position')
    ax2.set_title('3D Pose Plot')

    # Show and save plot
    if save_file is not None:
        plt.savefig(str(save_file))
    plt.show()


def visualise_chain_differential_data(csv_file : pathlib.Path, save_file : pathlib.Path = None):
    # variables
    success_data = []
    status_data = []

    # Open and read in csv file
    with open(str(csv_file), newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for idx, row in enumerate(reader):
            if idx == 0:
                # first line is header, use to get position of chain differential data
                try:
                    chain_diff_idx = row.index("chain_differential")
                except ValueError:
                    print("Could not find the index for the \'chain_differential\' data in the passed CSV file.")
                    return
                
                # see if this transform CSV file has a status component, if not ignore reporting
                try:
                    status_idx = row.index("status")
                except ValueError:
                    status_idx = None
            else:
                if row[chain_diff_idx] != 'None':
                    success_data.append(float(row[chain_diff_idx]))
                
                if status_idx != None:
                    status_data.append(row[status_idx])

    # Convert to numpy array
    success_data = np.array(success_data, dtype=np.object)

    # Visualise Data
    _, (ax1, ax2) = plt.subplots(ncols=2)

    # Histogram
    ax1.hist(success_data, bins='auto', edgecolor='black')
    ax1.set_title("Chain Differential Histogram\n(Only Includes Success Status)")
    ax1.set_xlabel("Chain Differential (seconds)")
    ax1.set_ylabel("Count")

    # Cumulative Histogram
    ax2.hist(success_data, bins='auto', cumulative=True, edgecolor='black')
    ax2.set_title("Chain Differential Cumulative Histogram\n(Only Includes Success Status)")
    ax2.set_xlabel("Chain Differential (seconds)")
    ax2.set_ylabel("Cumulative Count")

    # Status count
    if status_idx != None:
        print("\nROSBagTransform Status Counts:")
        for status in RBTStatus:
            print("\t%s: %d"%(status.name.replace('_', ' ').title(), status_data.count(status.name)))
        print("")

    # Show and save plot
    plt.tight_layout()
    if save_file is not None:
        plt.savefig(str(save_file))
    plt.show()


def visualise_lookup_differential_data(csv_file : pathlib.Path, save_file : pathlib.Path = None):
    # variables
    data = []

    # Open and read in csv file
    with open(str(csv_file), newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for idx, row in enumerate(reader):
            if idx == 0:
                # first line is header, use to get position of frame_timestamp and transform_timestamp data
                try:
                    frame_timestamp_idx = row.index("frame_timestamp")
                    transform_timestamp_idx = row.index("transform_timestamp")
                except ValueError:
                    print("Could not find the index for the \'frame_timestamp\' or \'transform_timestamp\' data in the passed CSV file.")
                    return
            else:
                if row[frame_timestamp_idx] != 'None' and row[transform_timestamp_idx] != 'None':
                    data.append(float(row[transform_timestamp_idx]) - float(row[frame_timestamp_idx]))

    # Convert to numpy array
    data = np.array(data, dtype=np.float)

    # Visualise Data
    plt.hist(data, bins='auto', edgecolor='black')
    plt.title("Lookup Differential Histogram")
    plt.xlabel("Lookup Differential (seconds)")
    plt.ylabel("Count")

    # Show and save plot
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