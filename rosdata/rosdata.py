#!/usr/bin/env python3

### IMPORT MODULES ###
# import sys
import csv
import yaml
import rosbag
import pathlib
import numpy as np
from tqdm import tqdm
from tabulate import tabulate 
import matplotlib.pyplot as plt

import cv2
import imutils
import spatialmath as sm

from .rosbag_extractor import ROSBagExtractor
from .rosbag_transformer import ROSBagTransformer
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


def show_info(bag_path : pathlib.Path, **kwargs):

    # Check to see if showing all information
    show_all = all(x == False for x in kwargs.values())

    # Open the ROS bag file
    print("Opening ROS Bag")
    bag = rosbag.Bag(bag_path, 'r')

    # Builds and shows the transform tree
    if show_all or ('transform_tree' in kwargs and kwargs['transform_tree']):
        print("Processing the Transforms... Building the transform tree")
        bag_transformer = ROSBagTransformer()
        bag_transformer.build_transform_tree(bag)
        print("\nBuilt the following transform tree")
        bag_transformer.show_tree()

    # Report information for topics
    if show_all or ('topic_info' in kwargs and kwargs['topic_info']):
        print("\nExtracting Topic Information")

        # Get topic info, contains tuple so create data structure to hold info
        # which will be added to
        topic_data = bag.get_type_and_topic_info()[1]
        data = {x: {'type': topic_data[x][0], 'count': topic_data[x][1],
            'connections': topic_data[x][2], 'frequency': topic_data[x][3],
            'frames': []} for x in topic_data}
        topic_count = 0
        for topic in topic_data:
            topic_count += data[topic]['count']

        # Get frame if present for each topic
        for topic, msg, _ in tqdm(bag.read_messages(), total=topic_count):
            # add frame id to this topic if the message has a header and not already present in the list
            # for this topic
            if hasattr(msg, 'header') and msg.header.frame_id not in data[topic]['frames']:
                data[topic]['frames'].append(msg.header.frame_id)

            # could break once all topics have gone through once, however that assumes that
            # all topics are only associated with one transform frame. That assumption should
            # in a correct ROS system, but what about in a poorly coded ROS system and should we
            # try and catch that error?

        # Change data type for tabulate
        data_table = [['Topic', 'Message Type', 'Message Count', 'Connections', 'Frequency', 'Transform Frames']]
        for topic in data:
            row = [topic] + list(data[topic].values())
            data_table.append(row)
        
        print("\nTopic Information")
        print(tabulate(data_table, headers='firstrow'))#, tablefmt='grid'))
        print("")


def visualise_pose_data(csv_file : pathlib.Path, save_file : pathlib.Path = None):
    # variables
    data = []

    # Open and read in csv file
    with open(str(csv_file), newline='') as f:
        reader = csv.reader(f, delimiter=',')
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
        
        # close file
        f.close()

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
    with open(str(csv_file), newline='') as f:
        reader = csv.reader(f, delimiter=',')
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
        
        # close file
        f.close()

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
    with open(str(csv_file), newline='') as f:
        reader = csv.reader(f, delimiter=',')
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
        
        # close file
        f.close()

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


def rotate_images(data : pathlib.Path, rot_amount : float, csv_file : pathlib.Path = None, rot_trans : list = None, overwrite : bool = False):

    # Get filenames from directory
    filenames = sorted([f.name for f in data.iterdir() if f.is_file()])
    
    # Open csv file if provided
    csv_data = []
    if csv_file is not None:
        with open(str(csv_file), newline='') as f:
            reader = csv.reader(f, delimiter=',')
            for row in reader:
                csv_data.append(row)
        f.close()

        # Check same filenames exists
        csv_filenames = [f[0] for f in csv_data[1:]]
        if set(filenames) != set(csv_filenames):
            print("ERROR! The filenames in the provided data folder and those in the CSV file are not identical.")
            return 

    # Get rotated image output directory and run check
    output_dir = data.parent / (data.name + "_rotated")
    if overwrite:
        output_dir = data

    if output_dir.exists() and overwrite == False:
        if yes_or_no("The folder \'%s\' already exists. These images have most likely been rotated previously. Would you like to remove this folder and rotate the images"%(output_dir)):
            rmdir(output_dir)
        else:
            print("Aborting operation")
            return
    
    if not output_dir.exists():
        output_dir.mkdir(parents=True)

    # Rotate each image
    print("\nRotating Images")
    for filename in tqdm(filenames):
        # Open image and rotate
        img = cv2.imread(str(data / filename))
        img_rot = imutils.rotate(img, rot_amount)

        # Save image
        cv2.imwrite(str(output_dir / filename), img_rot)

    # No need to continue if csv_file is none
    if csv_file == None:
        return

    # Get output csv file
    output_csv = csv_file.parent / (csv_file.stem + "_rotated.csv")
    if overwrite:
        output_csv = csv_file
    
    # Apply transform to CSV Data
    apply_transform = sm.SE3.Eul(rot_trans, unit='deg')
    pos_x_idx = csv_data[0].index("pos_x") # starting position of pose data
    with open(output_csv, 'w', newline='') as f: 
        # create csvwriter object and write header
        csvwriter = csv.writer(f, delimiter=',')
        csvwriter.writerow(csv_data[0])

        # loop through data
        for data in csv_data[1:]:
            # Get current transform data
            transform_data = data[pos_x_idx:pos_x_idx+7]
            if 'None' in transform_data:
                # write out same as before, and go onto next
                csvwriter.writerow(data)
                continue
            transform_data = [float(x) for x in transform_data]
            
            # create SE3 for current pose
            curr_pose = sm.SE3(transform_data[0:3])
            curr_pose.A[:3, :3] = sm.base.q2r(transform_data[3:])
            
            # Apply transform and get position and quat
            new_pose = curr_pose * apply_transform
            pos = new_pose.A[:3, -1]
            quat = sm.base.r2q(new_pose.A[:3,:3])

            # Copy new pose into data and save
            data[pos_x_idx:pos_x_idx+7] = list(pos) + list(quat)
            csvwriter.writerow(data)

        # close file
        f.close()


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