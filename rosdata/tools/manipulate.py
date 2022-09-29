#!/usr/bin/env python


###############
### MODULES ###
###############

import cv2
import csv
import imutils
import pathlib
from tqdm import tqdm

import spatialmath as sm

from rosdata.utils import yes_or_no, rmdir


###################
### ENTRY POINT ###
###################


def manipulate_main(args):
    """Manipulation operations for data generated using the ROSData tools.

    Args:
        args: the command line arguments
    """
    
    
    if args.action.lower() == "rot_images":
        if args.csv_file == None:
            rotate_images(pathlib.Path(args.data), args.rot, overwrite=args.overwrite)
        else:
            csv_file = pathlib.Path(args.csv_file[0])
            rot_trans = [float(x) for x in args.csv_file[1:]]
            rotate_images(pathlib.Path(args.data), args.rot, csv_file, rot_trans, overwrite=args.overwrite)


#################
### FUNCTIONS ###
#################




def rotate_images(data: pathlib.Path, rot_amount: float, csv_file: pathlib.Path = None, rot_trans: list = None, overwrite: bool = False):

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
    apply_transform = sm.SE3.Rx(rot_trans[0], unit='deg') * sm.SE3.Ry(rot_trans[1], unit='deg') * sm.SE3.Rz(rot_trans[2], unit='deg')
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


