#!/usr/bin/env python


###############
### MODULES ###
###############

import csv
import pathlib
import numpy as np
import matplotlib.pyplot as plt

from rosdata.utils import CSVROSData
from rosdata.core import TransformStatus


###################
### ENTRY POINT ###
###################


def visualize_main(args):
    """Used to visualize data generated using the ROSData tools.

    Args:
        args: the command line arguments
    """

    # Get CSV file path
    csv_fpath = pathlib.Path(args.csv_file)

    # Determine if saving is required
    save_fpath = None
    if args.save == True or isinstance(args.save, str):
        save_dir = pathlib.Path(args.csv_file).parent
        if args.dir is not None:
            save_dir = pathlib.Path(args.dir)
        save_dir.mkdir(parents=True, exist_ok=True)
        
        save_fpath = save_dir / pathlib.Path(args.csv_file).stem
        if isinstance(args.save, str):
            save_fpath = save_dir / args.save

    # Read in CSV ROSData file
    csv_rosdata = CSVROSData(csv_fpath)

    # Determine visualization to show
    if args.type.lower() == 'pose':
        visualise_pose_data(csv_rosdata, save_fpath)
    elif args.type.lower() == "chain_diff":
            visualise_chain_differential_data(csv_rosdata, save_fpath)
    elif args.type.lower() == "lookup_diff":
        visualise_lookup_differential_data(csv_rosdata, save_fpath)


#################
### FUNCTIONS ###
#################



def visualise_pose_data(csv_rosdata: CSVROSData, save_file: pathlib.Path = None):
    # Make sure pose data exists
    if not csv_rosdata.pose_data_exists():
        print("No pose data exists in the provided CSV file. Exiting program.")
        exit(0)

    # Get pose data as float numpy array
    data = np.asarray(csv_rosdata.get_data(fields=['pos_x', 'pos_y', 'pos_z']), dtype=np.float)

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


def visualise_chain_differential_data(csv_rosdata: CSVROSData, save_file: pathlib.Path = None):
    # Make sure chain differential is present in the data
    if not csv_rosdata.field_exists('chain_differential'):
        print("No chain differential data exists in the provided CSV file. Exiting program.")
        exit(0)

    # get transform chain differential link and status
    chain_data = csv_rosdata.get_data(fields='chain_differential')
    chain_data = np.asarray([x for x in chain_data if x != None], dtype=np.float)
    status_data = [x for x in csv_rosdata.get_data(fields='status')]

    # Visualise Data
    _, (ax1, ax2) = plt.subplots(ncols=2)

    # Histogram
    ax1.hist(chain_data, bins='auto', edgecolor='black')
    ax1.set_title("Chain Differential Histogram\n(Only Includes Success Status)")
    ax1.set_xlabel("Chain Differential (seconds)")
    ax1.set_ylabel("Count")

    # Cumulative Histogram
    ax2.hist(chain_data, bins='auto', cumulative=True, edgecolor='black')
    ax2.set_title("Chain Differential Cumulative Histogram\n(Only Includes Success Status)")
    ax2.set_xlabel("Chain Differential (seconds)")
    ax2.set_ylabel("Cumulative Count")

    # Status count
    if status_data != []:
        print("\nROSBagTransform Status Counts:")
        for status in TransformStatus:
            print("\t%s: %d"%(status.name.replace('_', ' ').title(), status_data.count(status.name)))
        print("")

    # Show and save plot
    plt.tight_layout()
    if save_file is not None:
        plt.savefig(str(save_file))
    plt.show()


def visualise_lookup_differential_data(csv_rosdata: CSVROSData, save_file: pathlib.Path = None):
    # Make sure chain differential is present in the data
    if not csv_rosdata.field_exists('frame_timestamp') or not csv_rosdata.field_exists('transform_timestamp'):
        print("No frame or transform timestamp data exists in the provided CSV file. Exiting program.")
        exit(0)

    # Get transform timestamp as numpy float array
    lookup_diff_data = np.asarray([x[1]-x[0] for x in csv_rosdata.get_data(fields=['frame_timestamp', 'transform_timestamp']) if None not in x], dtype=np.float)

    # Visualise Data
    plt.hist(lookup_diff_data, bins='auto', edgecolor='black')
    plt.title("Lookup Differential Histogram")
    plt.xlabel("Lookup Differential (seconds)")
    plt.ylabel("Count")

    # Show and save plot
    if save_file is not None:
        plt.savefig(str(save_file))
    plt.show()

